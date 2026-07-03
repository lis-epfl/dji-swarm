"""
LIS_Swarm image stream publisher
================================
Publishes each drone's live camera frame into the "BlockSharedMemory" mapping
consumed by the stitcher pipeline (StitcherThreading.py / the Unity VR sim's
PyUniSharingFast.cs / ImageSharing.cs).

This replaces running image_stream.py as a separate process. That was never
safe: the ds_wrapper shared-memory protocol busy-waits on a single status byte
per drone slot, so a second process polling getImageAndTelemetryData starves
the flight controller's command/telemetry loops (they collapsed to ~1 Hz).

Instead, this publisher is EMBEDDED in the controller (swarm_flocking.py
--image-stream) and adds ZERO ds_wrapper calls: the controller's telemetry
threads already fetch the full image+telemetry array at 10 Hz and discard the
pixels. Each DroneController exposes a `frame_sink` hook; ours copies the raw
YUV slice out of the (aliased, soon-overwritten) wrapper buffer into a
latest-wins mailbox, and a per-drone worker thread does the heavy lifting
(YUV->BGR convert, resize, shared-memory handshake) off the control path.
The cv2 calls release the GIL, and imageSharingUtil.write_memory's consumer
handshake (flag polling + 0.06 s pacing) can block for up to ~1 s, which is
exactly why it must live on its own thread. Frame rate therefore tops out at
the telemetry rate (10 Hz per drone); the consumer paces itself to ~16 Hz max
anyway.

Block layout (must match PyUniSharingFast.cs / imageSharingUtil.write_memory):
    int32 flag | int32 droneId (ZERO-based) | float32 heading | 640x360x3 BGR
    block = 12 + 691200 bytes; mapping = num_drones * block, indexed by
    (drone_id - 1). Whoever creates the named mapping first fixes its size.

Real-drone mode note: the Unity sim's PyUniSharingFast component must have
enableImageWriting DISABLED, otherwise its writer fights this one for the same
blocks.

No ds_wrapper import — decode mode and frames are passed in by the controller.
"""

import mmap
import threading

import cv2
import numpy as np

import utils.imageSharingUtil as imageSharingUtil


# Wrapper array geometry (see CLAUDE.md): YUV 1920x1080 image in [0:3110400].
RAW_IMAGE_BYTES = 3110400
RAW_ROWS = 1080 * 3 // 2   # 1620 (YUV420 planar / NV12)
RAW_COLS = 1920

BLOCK_MAP_NAME = "BlockSharedMemory"
BLOCK_HEADER_BYTES = 12    # int32 flag + int32 droneId + float32 heading


class ImageStreamPublisher:
    """Per-drone worker threads that push frames to BlockSharedMemory."""

    def __init__(self, drones, hw_decode, width=640, height=360):
        """
        Args:
            drones: {drone_id (1-based int): DroneController} — each controller
                    must expose a settable `frame_sink` attribute called by its
                    telemetry thread as frame_sink(data, telem).
            hw_decode: ds_wrapper.isHWDecoderEnabled() result — 1 selects the
                    NV12 (hardware) colour conversion, anything else the
                    planar YUV420 (software) one, matching image_stream.py.
            width/height: output frame size; 640x360 is the fixed size the
                    stitcher/Unity consumers read.
        """
        self._drones = dict(drones)
        self._cvt = (cv2.COLOR_YUV2BGR_NV12 if hw_decode == 1
                     else cv2.COLOR_YUV420p2RGB)
        self._size = (int(width), int(height))
        self._image_bytes = self._size[0] * self._size[1] * 3
        self._block_bytes = BLOCK_HEADER_BYTES + self._image_bytes
        self._mmf = mmap.mmap(-1, len(self._drones) * self._block_bytes,
                              BLOCK_MAP_NAME)
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
                with self._locks[drone_id]:
                    self._latest[drone_id] = (yuv, heading)
                self._events[drone_id].set()
            except Exception:
                # Streaming is non-critical to flight; never break the
                # telemetry loop.
                pass
        return sink

    def _worker(self, drone_id):
        event = self._events[drone_id]
        while self._running:
            if not event.wait(timeout=0.5):
                continue
            event.clear()
            with self._locks[drone_id]:
                frame = self._latest.pop(drone_id, None)
            if frame is None:
                continue
            yuv, heading = frame
            try:
                img = cv2.cvtColor(yuv.reshape(RAW_ROWS, RAW_COLS), self._cvt)
                img = cv2.resize(img, self._size)
                imageSharingUtil.write_memory(
                    self._mmf, (drone_id - 1) * self._block_bytes,
                    self._image_bytes, img, drone_id - 1, heading)
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
        try:
            self._mmf.close()
        except Exception:
            pass
