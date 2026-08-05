"""
LIS_Swarm image stream publisher
================================
Publishes each drone's live camera frame into the "DroneFeedSharedMemory"
mapping consumed by the Unity VR sim's ImageSharing.cs, which displays the
feeds and re-publishes the 3 body-yaw-selected views into the stitcher's
separate 3-slot "BlockSharedMemory" (StitcherThreading.py).

This replaces running image_stream.py as a separate process. That was never
safe: the ds_wrapper shared-memory protocol busy-waits on a single status byte
per drone slot, so a second process polling getImageAndTelemetryData starves
the flight controller's command/telemetry loops (they collapsed to ~1 Hz).

Instead, this publisher is EMBEDDED in the controller (swarm_flocking.py
--image-stream) and adds ZERO ds_wrapper calls: the controller's telemetry
threads already fetch the full image+telemetry array at 20 Hz and discard the
pixels. Each DroneController exposes a `frame_sink` hook; ours copies the raw
YUV slice out of the (aliased, soon-overwritten) wrapper buffer into a
latest-wins mailbox, and a per-drone worker thread does the heavy lifting
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
# because it is what writes the bytes. image_feed_test.py imports this name.
BLOCK_HEADER_BYTES = imageSharingUtil.BLOCK_HEADER_BYTES
MAX_DRONES = 10            # fixed mapping capacity (must match ImageSharing.cs)


class ImageStreamPublisher:
    """Per-drone worker threads that push frames to DroneFeedSharedMemory."""

    def __init__(self, drones, hw_decode, width=800, height=450, pose_solver=None):
        """
        Args:
            drones: {drone_id (1-based int): DroneController} — each controller
                    must expose a settable `frame_sink` attribute called by its
                    telemetry thread as frame_sink(data, telem).
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
        # Fixed capacity: every map is MAX_DRONES blocks so its size matches
        # what ImageSharing.cs creates regardless of fleet size or of which
        # process creates the named mapping first.
        self._mmfs = {
            did: mmap.mmap(-1, MAX_DRONES * self._block_bytes, BLOCK_MAP_NAME)
            for did in self._drones
        }
        # Per-drone latest-wins mailbox: {id: (yuv_copy, heading)} + an event
        # the worker sleeps on. A slow worker just drops frames, never queues.
        self._latest = {}
        self._locks = {did: threading.Lock() for did in self._drones}
        self._events = {did: threading.Event() for did in self._drones}
        self._running = False
        self._threads = []

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
            drone.frame_sink = self._make_sink(did)
            t = threading.Thread(target=self._worker, args=(did,),
                                 daemon=True, name="ImgStream_{}".format(did))
            self._threads.append(t)
            t.start()

    def stop(self):
        self._running = False
        for drone in self._drones.values():
            drone.frame_sink = None
        for event in self._events.values():
            event.set()   # wake workers so they see _running == False
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
        # Close every per-worker mmap (workers no longer touch them once
        # _running is False and their threads have joined).
        for mmf in self._mmfs.values():
            try:
                mmf.close()
            except Exception:
                pass
