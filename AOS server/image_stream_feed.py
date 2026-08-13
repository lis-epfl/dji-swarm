"""
LIS_Swarm image stream publisher
================================
Publishes each drone's live camera frame into the "DroneFeedSharedMemory"
mapping consumed by the Unity VR sim's ImageSharing.cs, which displays the
feeds and re-publishes the selected views into the stitcher's separate
"BlockSharedMemory" (StitcherThreading.py) — the 3 body-yaw-selected ones under
STABSTITCH, every fresh feed under PLANAR. That section is a fixed 24-slot
array with no slot count to configure on either side; the two maps are sized
independently, so nothing here changes when it does.

This replaces running image_stream.py as a separate process. That was never
safe: the ds_wrapper shared-memory protocol busy-waits on a single status byte
per drone slot, so a second process polling getImageAndTelemetryData starves
the flight controller's command/telemetry loops (they collapsed to ~1 Hz).

Instead, this publisher is EMBEDDED in the controller (swarm_flocking.py
--image-stream) and adds ZERO ds_wrapper calls: the controller's telemetry
threads already fetch the full image+telemetry array at 20 Hz and discard the
pixels. Each DroneController takes frame consumers via add_frame_sink() (there
may be others — clip_recorder.py registers one too); ours copies the raw YUV
slice out of the (aliased, soon-overwritten) wrapper buffer into a latest-wins
mailbox, and a per-drone worker thread does the heavy lifting
(YUV->BGR convert, resize, shared-memory handshake) off the control path.
The cv2 calls release the GIL, and imageSharingUtil.write_memory's consumer
handshake (flag polling + pacing, 0.04 s here) can block for up to ~1 s,
which is exactly why it must live on its own thread. Frame rate therefore
tops out at the telemetry rate (20 Hz per drone).

Block layout (must match ImageSharing.cs / imageSharingUtil.write_memory):
    int32 flag | int32 droneId (ZERO-based) | float32 heading
      | float32 camPos[3] | float32 camRot[4] xyzw | float32 captureTime
      | int32 poseStatus | 800x450x3 BGR
    block = 48 + 1080000 bytes; mapping = MAX_DRONES * block (fixed capacity so
    the size never depends on fleet size or creation order), indexed by
    (drone_id - 1). ImageSharing.cs marks blocks it has consumed (and blocks
    never written) with droneId = -1 and skips them; every write here restores
    droneId, which is how Unity detects a genuinely new frame.

    The pose fields are what the sim's PLANAR stitcher needs and a heading alone
    cannot give: one scalar is no position at all and one of three rotation
    degrees of freedom. Supply a `pose_solver` to fill them; without one they are
    zeroed with poseStatus 0, which costs PLANAR (it drops unposed views) and
    nothing else.

Real-drone mode note: keep the Unity scene's PyUniSharingFast component with
enableImageWriting DISABLED — in the DJI scene ImageSharing.cs is the sole
producer of the stitcher's BlockSharedMemory.

No ds_wrapper import — decode mode and frames are passed in by the controller.
"""

import mmap
import threading
import time

import cv2
import numpy as np

import utils.imageSharingUtil as imageSharingUtil


# Wrapper array geometry (see CLAUDE.md): YUV 1920x1080 image in [0:3110400].
RAW_IMAGE_BYTES = 3110400
RAW_ROWS = 1080 * 3 // 2   # 1620 (YUV420 planar / NV12)
RAW_COLS = 1920

BLOCK_MAP_NAME = "DroneFeedSharedMemory"
# Re-exported rather than redefined: utils.imageSharingUtil owns the block layout
# because it is what writes the bytes -- and, unlike this file, it is parsed by
# check_wire_layout.py, so a constant stated there is checked against the C# and a
# constant stated here is not. image_feed_test.py and clip_replay.py import these.
BLOCK_HEADER_BYTES = imageSharingUtil.BLOCK_HEADER_BYTES
MAX_DRONES = imageSharingUtil.FEED_MAX_DRONES


class ImageStreamPublisher:
    """Per-drone worker threads that push frames to DroneFeedSharedMemory."""

    def __init__(self, drones, hw_decode, width=800, height=450, pose_solver=None):
        """
        Args:
            drones: {drone_id (1-based int): DroneController} — each controller
                    must expose add_frame_sink()/remove_frame_sink(), which
                    register a callable its telemetry thread invokes as
                    sink(data, telem).
            hw_decode: ds_wrapper.isHWDecoderEnabled() result — 1 selects the
                    NV12 (hardware) colour conversion, anything else the
                    planar YUV420 (software) one, matching image_stream.py.
            width/height: output frame size; 800x450 is the fixed size the
                    Unity consumer reads (ImageSharing.cs ImageWidth/Height
                    consts must match).
            pose_solver: optional dji_camera_pose.CameraPoseSolver. Given one,
                    each block carries the camera pose the sim's PLANAR stitcher
                    needs; without one the blocks carry poseStatus 0 and PLANAR
                    falls back to the individual feeds (STABSTITCH is unaffected
                    either way). ONE solver for the whole fleet, never one per
                    drone — they must share a latched origin or their positions
                    are not in a common frame, which is the entire point of it.
        """
        self._drones = dict(drones)
        self._pose_solver = pose_solver
        # Capture timestamps are seconds since the publisher started, NOT a wall
        # clock: the wire field is float32, in which time.time() (~1.75e9) has
        # about 128 s of resolution. The consumer only ever reads differences.
        self._t0 = time.perf_counter()
        bad_ids = [did for did in self._drones if not 1 <= did <= MAX_DRONES]
        if bad_ids:
            raise ValueError(
                "drone ids {} outside mapping capacity 1..{}".format(
                    bad_ids, MAX_DRONES))
        self._cvt = (cv2.COLOR_YUV2BGR_NV12 if hw_decode == 1
                     else cv2.COLOR_YUV420p2RGB)
        self._size = (int(width), int(height))
        self._image_bytes = self._size[0] * self._size[1] * 3
        self._block_bytes = BLOCK_HEADER_BYTES + self._image_bytes
        # Per-worker memory maps: each drone's worker owns its OWN mmap object
        # over the same named section. Multiple mmap views of one Windows named
        # mapping share the physical pages but keep INDEPENDENT file positions.
        # A SINGLE shared mmap has one file position, and write_memory drives it
        # through seek()->write() sequences that are NOT GIL-atomic, so with 2+
        # worker threads one drone's write can resume at another drone's offset:
        # frames land in the wrong block (torn/garbled) and a mis-placed flag
        # reset wedges a block at flag=1 forever (that feed freezes). Separate
        # mmap objects give each worker its own position, so disjoint per-drone
        # blocks can never cross-write and no lock is needed.
        # Fixed capacity: every map is MAX_DRONES blocks (plus the scene-plane
        # trailer) so its size matches what ImageSharing.cs creates regardless of
        # fleet size or of which process creates the named mapping first.
        #
        # Through open_feed_map rather than mmap.mmap directly, because this runs in
        # swarm_flocking.main() BEFORE the aircraft are armed: an unhandled OSError
        # here would not degrade the mosaic, it would take down the flight
        # controller. open_feed_map falls back to the pre-trailer size instead.
        self._mmfs = {}
        self._trailer_ok = True
        for did in self._drones:
            self._mmfs[did], ok = imageSharingUtil.open_feed_map(BLOCK_MAP_NAME)
            self._trailer_ok = self._trailer_ok and ok
        # The trailer's own view. NOT one of the per-drone ones: those belong to
        # worker threads, and the whole reason there is one each (see above) is that
        # write_memory's seek()->write() is not GIL-atomic across a shared file
        # position. This one is only ever touched by the caller's thread.
        self._plane_mmf, ok = imageSharingUtil.open_feed_map(BLOCK_MAP_NAME)
        self._trailer_ok = self._trailer_ok and ok
        self._plane_seq = 0
        self._plane_beat = 0
        self._plane_next_due = 0.0
        # Per-drone latest-wins mailbox: {id: (yuv_copy, heading)} + an event
        # the worker sleeps on. A slow worker just drops frames, never queues.
        self._latest = {}
        self._locks = {did: threading.Lock() for did in self._drones}
        self._events = {did: threading.Event() for did in self._drones}
        # The sink object registered per drone, kept so stop() can unregister
        # the IDENTICAL object. Never clear a controller's sinks wholesale —
        # other consumers (clip_recorder) register their own.
        self._sinks = {}
        self._running = False
        self._threads = []

    # Cadence of the scene-plane trailer. 10 Hz rather than 1-2 Hz because the
    # standoff tracks the formation: at a 2 m/s closing speed half a second of lag
    # is a metre, which is the entire plane-error budget at a 34 m standoff.
    PLANE_PERIOD_S = 0.1

    def set_standoff(self, result, now=None):
        """
        Publish the PLANAR scene-plane standoff for Unity to pick up.

        `result` is a clip_scene_plane.pick_facade() result dict, or None when no
        facade could be picked (which publishes the NO_FACADE status rather than
        going silent -- Unity has to be able to tell "no wall" from "no producer").

        Called straight from the control loop, deliberately, rather than from a
        thread of its own. There is no flag handshake on the trailer -- nothing else
        writes those bytes -- so this is ~48 bytes of struct.pack_into with no retry
        and no sleep, and it cannot block. Being on the control loop is also what
        makes the heartbeat honest: a wedged loop stops it and Unity falls back to
        its inspector value, which is exactly what a heartbeat is for. A thread would
        keep republishing a stale plane through a stall.
        """
        if not self._trailer_ok:
            return
        now = time.monotonic() if now is None else now
        if now < self._plane_next_due:
            return
        self._plane_next_due = now + self.PLANE_PERIOD_S
        self._plane_beat += 1
        if result is None:
            self._plane_seq = imageSharingUtil.write_standoff_trailer(
                self._plane_mmf, self._plane_seq, self._plane_beat, 0.0,
                imageSharingUtil.FEED_TR_STATUS_NO_FACADE)
            return
        status = (imageSharingUtil.FEED_TR_STATUS_DWELLING
                  if result.get("state") == "dwelling"
                  else imageSharingUtil.FEED_TR_STATUS_LOCKED)
        self._plane_seq = imageSharingUtil.write_standoff_trailer(
            self._plane_mmf, self._plane_seq, self._plane_beat,
            result.get("standoff_m") or 0.0, status,
            facade_id=result.get("facade_id", -1),
            look_off_deg=result.get("look_off_deg") or 0.0,
            spread_m=result.get("spread_m") or 0.0,
            px_per_m=result.get("px_per_m") or 0.0,
            tilt_deg=result.get("tilt_deg") or 0.0,
            view_count=result.get("view_count") or 0)

    def clear_standoff(self):
        """Publish "no facade", unconditionally and off the rate limit.

        Used on Stop and at close(): the operator has stopped supplying a plane and
        Unity should return to its inspector value now, not after the heartbeat
        happens to age out.
        """
        if not self._trailer_ok:
            return
        self._plane_beat += 1
        self._plane_next_due = 0.0
        self._plane_seq = imageSharingUtil.write_standoff_trailer(
            self._plane_mmf, self._plane_seq, self._plane_beat, 0.0,
            imageSharingUtil.FEED_TR_STATUS_NO_FACADE)

    def _make_sink(self, drone_id):
        """Build the frame_sink callable run on drone_id's telemetry thread.

        Must stay cheap (one ~3 MB memcpy) and must copy immediately: `data`
        aliases the wrapper's shared memory and is overwritten by the next
        ds_wrapper call.
        """
        def sink(data, telem):
            try:
                yuv = np.array(data[:RAW_IMAGE_BYTES], copy=True)
                heading = float(telem.get("heading", 0.0))

                # Solved HERE, not on the worker thread, and not from a later
                # telemetry sample: `data` and `telem` came out of the same
                # ds_wrapper fetch, so this is the tightest pose/frame pairing
                # available anywhere in the system. Pose/video skew is a
                # first-order error term for the planar mosaic — at the 40 deg/s
                # yaw clamp, 300 ms of it is ~55 px — so pairing them one call
                # later would give away the one part of that budget we control.
                pose = None
                if self._pose_solver is not None:
                    pos, quat, status = self._pose_solver.pose_for(telem)
                    if status:
                        pose = (pos, quat)
                capture_time = time.perf_counter() - self._t0

                with self._locks[drone_id]:
                    self._latest[drone_id] = (yuv, heading, pose, capture_time)
                self._events[drone_id].set()
            except Exception:
                # Streaming is non-critical to flight; never break the
                # telemetry loop.
                pass
        return sink

    def _worker(self, drone_id):
        event = self._events[drone_id]
        mmf = self._mmfs[drone_id]
        while self._running:
            if not event.wait(timeout=0.5):
                continue
            event.clear()
            with self._locks[drone_id]:
                frame = self._latest.pop(drone_id, None)
            if frame is None:
                continue
            yuv, heading, pose, capture_time = frame
            try:
                img = cv2.cvtColor(yuv.reshape(RAW_ROWS, RAW_COLS), self._cvt)
                img = cv2.resize(img, self._size)
                imageSharingUtil.write_memory(
                    mmf, (drone_id - 1) * self._block_bytes,
                    self._image_bytes, img, drone_id - 1, heading,
                    pace_s=0.04, pose=pose, capture_time=capture_time)
            except Exception as e:
                print("[image-stream {}] frame error: {}".format(drone_id, e),
                      flush=True)

    def start(self):
        if self._running:
            return
        self._running = True
        for did, drone in self._drones.items():
            sink = self._make_sink(did)
            self._sinks[did] = sink
            drone.add_frame_sink(sink)
            t = threading.Thread(target=self._worker, args=(did,),
                                 daemon=True, name="ImgStream_{}".format(did))
            self._threads.append(t)
            t.start()

    def stop(self):
        self._running = False
        for did, drone in self._drones.items():
            drone.remove_frame_sink(self._sinks.pop(did, None))
        for event in self._events.values():
            event.set()   # wake workers so they see _running == False
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
        # Retire the scene plane BEFORE dropping the view: closing the handle does
        # not clear the section, so a last "no facade" write is what tells Unity to
        # go back to its inspector value rather than leaving it to time out on the
        # heartbeat.
        try:
            self.clear_standoff()
        except Exception:
            pass
        # Close every per-worker mmap (workers no longer touch them once
        # _running is False and their threads have joined), then the trailer's.
        for mmf in list(self._mmfs.values()) + [self._plane_mmf]:
            try:
                mmf.close()
            except Exception:
                pass
