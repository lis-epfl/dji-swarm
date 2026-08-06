"""
LIS_Swarm clip recorder
=======================
Records a SHORT, time-limited section of a flight with pictures attached: one
MP4 per drone plus a per-frame index, and the flight data for exactly that
window, all in their own folder under a separate recordings root:

    recordings/clip_YYYYMMDD_HHMMSS/
        drone1.mp4   drone1_frames.csv     drone2.mp4  drone2_frames.csv  ...
        user_commands.csv  drone_commands.csv  telemetry.csv  swarm_debug.csv
        session.json

Started and stopped from the browser GUI's "Record clip" button; auto-stops at
`max_seconds` so a forgotten recording cannot fill the disk. The continuous
flight log (flight_logs/) is untouched and keeps running throughout.

Where the frames come from
--------------------------
Nowhere new: the controller's telemetry threads already fetch the full
image+telemetry array at 20 Hz and throw the pixels away. This module registers
a DroneController.add_frame_sink() consumer, exactly like image_stream_feed.py
(both can be active at once), so recording adds ZERO ds_wrapper calls and cannot
slow the command/telemetry rates. Frame rate therefore tops out at the telemetry
rate, and a drone whose telemetry never parses contributes no frames at all —
the sink is only called when it does.

The sink does one ~3 MB copy onto a small bounded queue and returns; a per-drone
worker thread does the YUV->BGR convert and the encode. The queue is 4 deep
(~200 ms of slack, ~12 MB/drone): enough to ride out a slow disk write, small
enough that memory cannot run away. Unlike the live stitcher feed this is a
queue rather than a latest-wins mailbox — dropping a frame here both stutters
the video and makes the index lie — and any drops that do happen are counted per
drone and published, never silent.

TIMEBASE: the frames CSV `t_epoch` is authoritative, NOT the MP4 timing.
cv2.VideoWriter takes one constant header rate and has no per-frame timestamp
input, while frames arrive unevenly (~5 Hz of the ~20 Hz fetches carry genuinely
fresh telemetry, and the aircraft cameras run at a mix of 24/25/30 fps). So the
MP4 is written at a nominal 20 fps for roughly-real-time playback, the measured
rate is recorded in session.json, and anything that needs true frame times joins
`frame` -> `t_epoch` through the CSV. Frames are never duplicated or dropped to
force a constant rate: that would destroy the frame<->telemetry correspondence,
which is the point of recording at all.

Flight data for the window comes from FlightLogger's mirror hook: a second
FlightLogger rooted in the clip folder is attached as `logger.mirror` for the
duration, so all four streams are captured with the timestamps the primary
stamped (see flight_logger.py). Rows in the clip are therefore byte-identical to
their counterparts in flight_logs/ and can be joined across the two.

No ds_wrapper import — frames and the decode mode are passed in by the
controller. `python clip_recorder.py` runs a standalone self-check.
"""

import csv
import json
import os
import queue
import shutil
import threading
import time
from datetime import datetime

import cv2
import numpy as np

from flight_logger import FlightLogger


# Wrapper array geometry (see CLAUDE.md): YUV 1920x1080 image in [0:3110400].
RAW_IMAGE_BYTES = 3110400
RAW_ROWS = 1080 * 3 // 2   # 1620 (YUV420 planar / NV12)
RAW_COLS = 1920

DEFAULT_RECORDING_DIR = "recordings"
DEFAULT_MAX_SECONDS = 120.0
MAX_SECONDS_MIN = 1.0
MAX_SECONDS_MAX = 900.0

# Nominal MP4 header rate. Matches the telemetry loop rate, so playback runs at
# roughly wall-clock speed. See the TIMEBASE note above.
DEFAULT_FPS = 20.0
FOURCC = "mp4v"

# Frames buffered per drone before the sink starts dropping. 4 ~= 200 ms at
# 20 Hz, ~12 MB of 1080p YUV.
DEFAULT_QUEUE_DEPTH = 4

# Rough encoded size of one drone's 1080p stream, used only for the free-space
# check at start(). Measured field value goes in flocking.config.psd1.
EST_BYTES_PER_S_PER_DRONE = 2500000

# Index written alongside each MP4. Gimbal angles ride along because a frame
# without them is not enough to place a view; everything else is in the
# mirrored telemetry.csv.
_FRAME_COLS = ['frame', 't_epoch', 'lat', 'lon', 'alt', 'heading',
               'gimbal_pitch', 'gimbal_yaw']

_FLUSH_INTERVAL = 1.0     # frames-CSV flush period (matches FlightLogger)


class _Clip:
    """State for ONE recording. A fresh instance per start(), so a start/stop/
    start cycle shares nothing and a sink call that arrives late (removal is not
    instantaneous) harmlessly sees active == False."""

    def __init__(self, name, path, drone_ids, max_seconds):
        self.name = name
        self.dir = path
        self.active = True
        self.max_seconds = max_seconds
        self.t_start_mono = time.monotonic()
        self.t_start_epoch = time.time()
        self.deadline = self.t_start_mono + max_seconds
        self.frames = {did: 0 for did in drone_ids}
        self.dropped = {did: 0 for did in drone_ids}
        self.results = {}          # did -> per-drone summary dict
        self.queues = {did: None for did in drone_ids}
        self.threads = []
        self.mirror = None
        self.error = None
        self.reason = None
        self.t_end_epoch = None
        self.duration = 0.0

    def elapsed(self):
        return time.monotonic() - self.t_start_mono


class ClipRecorder:
    """GUI-triggered, time-limited video + flight-data recorder.

    Construction touches no disk and starts no threads, so a controller can
    always build one and let the operator decide whether to ever press Record.
    """

    def __init__(self, drones, hw_decode, base_dir=DEFAULT_RECORDING_DIR,
                 max_seconds=DEFAULT_MAX_SECONDS, fps=DEFAULT_FPS,
                 logger=None, meta=None, queue_depth=DEFAULT_QUEUE_DEPTH):
        """
        Args:
            drones: {drone_id (1-based int): DroneController} — each must expose
                    add_frame_sink()/remove_frame_sink().
            hw_decode: ds_wrapper.isHWDecoderEnabled() result — 1 selects the
                    NV12 (hardware) colour conversion, anything else the planar
                    YUV420 one, matching image_stream.py. Both convert straight
                    to BGR because that is what cv2.VideoWriter wants; the live
                    stitcher feed converts to RGB instead because Unity wants
                    that layout. Getting this backwards tints every clip blue.
            base_dir: recordings root; clip folders are created inside it.
            max_seconds: hard cap on one clip; the controller auto-stops there.
            fps: nominal MP4 header rate (see the module TIMEBASE note).
            logger: the live FlightLogger to mirror for the window. None (or
                    --no-log) = video only, with a warning.
            meta: run-config dict seeded into the clip's session.json.
        """
        self._drones = dict(drones)
        self._base_dir = base_dir
        self._max_seconds = float(max_seconds)
        self._fps = float(fps)
        self._logger = logger
        self._meta = dict(meta or {})
        self._queue_depth = int(queue_depth)
        self._cvt = (cv2.COLOR_YUV2BGR_NV12 if hw_decode == 1
                     else cv2.COLOR_YUV420p2BGR)
        # Guards the start/stop transition only — every hot path (the sink, the
        # workers) reads self._clip once and works off that object.
        self._lock = threading.Lock()
        self._clip = None
        self._sinks = {}
        self._finisher = None
        self._last = None          # summary of the most recent finished clip

    # ------------------------------------------------------------------ #
    # Control (called from the control-loop thread)
    # ------------------------------------------------------------------ #

    def start(self):
        """Begin a recording. Returns False (with a reason printed and stored in
        status()["error"]) if one is already running, still finalising, or the
        clip folder cannot be prepared."""
        with self._lock:
            if self._clip is not None:
                print("[clip] already recording", flush=True)
                return False
            if self._finisher is not None and self._finisher.is_alive():
                print("[clip] previous clip still saving", flush=True)
                return False
            try:
                path, name = self._make_clip_dir()
            except OSError as e:
                self._last = {"error": "cannot create clip folder: {}".format(e)}
                print("[clip] ERROR cannot create clip folder: {}".format(e),
                      flush=True)
                return False

            clip = _Clip(name, path, self._drones.keys(), self._max_seconds)
            warning = self._check_free_space(path)
            if warning:
                clip.error = warning
                print("[clip] WARNING {}".format(warning), flush=True)

            # Mirror the flight data for the window. Attached LAST of the
            # setup steps so its window starts as close to the frames as
            # possible; detached first on stop, for the same reason.
            if self._logger is not None:
                try:
                    clip.mirror = FlightLogger(
                        base_dir=path, session_name="",
                        meta=self._clip_meta(clip))
                    self._logger.mirror = clip.mirror
                except OSError as e:
                    print("[clip] WARNING flight-data mirror failed: {}".format(e),
                          flush=True)
                    clip.mirror = None
            else:
                print("[clip] WARNING no flight log attached — video only",
                      flush=True)
                self._write_own_session_json(clip)

            for did in self._drones:
                clip.queues[did] = queue.Queue(maxsize=self._queue_depth)
            for did, drone in self._drones.items():
                t = threading.Thread(target=self._worker, args=(clip, did),
                                     daemon=True, name="ClipRec_{}".format(did))
                clip.threads.append(t)
                t.start()
            self._clip = clip
            # Sinks go on LAST: no frame can arrive before there is a worker to
            # drain it.
            for did, drone in self._drones.items():
                sink = self._make_sink(clip, did)
                self._sinks[did] = sink
                drone.add_frame_sink(sink)

            print("[clip] recording -> {} (max {:.0f} s, {} drones)".format(
                path, self._max_seconds, len(self._drones)), flush=True)
            return True

    def stop(self, reason="operator"):
        """End the current recording. Returns immediately — the encode drain,
        MP4 finalisation and mirror close happen on a ClipFinish thread so this
        never stalls the control loop. Safe to call when nothing is recording."""
        with self._lock:
            clip = self._clip
            if clip is None:
                return False
            # Sinks off FIRST so no new frame can enter a queue nobody will
            # drain, then flip the flag an in-flight sink call checks.
            for did, drone in self._drones.items():
                drone.remove_frame_sink(self._sinks.pop(did, None))
            if self._logger is not None and self._logger.mirror is clip.mirror:
                self._logger.mirror = None
            clip.active = False
            clip.reason = reason
            clip.t_end_epoch = time.time()
            clip.duration = clip.elapsed()
            self._clip = None
            self._finisher = threading.Thread(
                target=self._finish, args=(clip,), daemon=True,
                name="ClipFinish")
            self._finisher.start()
            return True

    def poll(self):
        """True when the active clip has hit its duration limit. Called each
        control tick; the caller does the stop, so every start/stop transition
        stays on one thread."""
        clip = self._clip
        return clip is not None and time.monotonic() >= clip.deadline

    def is_recording(self):
        return self._clip is not None

    def close(self):
        """Stop any recording and wait for finalisation. For the shutdown
        `finally` block — without the join, daemon threads die at interpreter
        exit and the MP4s are left without their moov atom (unplayable)."""
        self.stop("shutdown")
        finisher = self._finisher
        if finisher is not None:
            finisher.join(timeout=20)
            if finisher.is_alive():
                print("[clip] WARNING finalisation did not complete in time; "
                      "the MP4s may be unplayable", flush=True)

    def status(self):
        """Small JSON-serializable snapshot for meta["recording"]. Published
        every tick even when idle, so the GUI can show the saved confirmation.
        Never carries the frame index — that would blow the telemetry
        datagram."""
        clip = self._clip
        finisher = self._finisher
        st = {
            "on": clip is not None,
            "finalizing": bool(finisher is not None and finisher.is_alive()),
            "max_s": self._max_seconds,
            "last": self._last,
        }
        if clip is not None:
            elapsed = clip.elapsed()
            st["clip"] = clip.name
            st["elapsed"] = round(elapsed, 1)
            st["remaining"] = round(max(0.0, clip.max_seconds - elapsed), 1)
            # String keys, like every other per-drone map in meta (link, resp,
            # rotation_check): JSON would stringify them anyway, so keying them
            # here keeps a Python consumer and the GUI reading the same thing.
            st["frames"] = {str(k): v for k, v in clip.frames.items()}
            st["dropped"] = {str(k): v for k, v in clip.dropped.items()}
            st["error"] = clip.error
        else:
            st["error"] = (self._last or {}).get("error")
        return st

    # ------------------------------------------------------------------ #
    # Frame path
    # ------------------------------------------------------------------ #

    def _make_sink(self, clip, drone_id):
        """Build the consumer run on drone_id's telemetry thread.

        Must stay cheap (one ~3 MB memcpy) and must copy immediately: `data`
        aliases the wrapper's shared memory and the next ds_wrapper call
        overwrites it.
        """
        q = clip.queues[drone_id]

        def sink(data, telem):
            try:
                # Belt-and-braces duration cap: even if the control loop wedges
                # and never calls poll(), the clip stops growing here.
                if not clip.active or time.monotonic() >= clip.deadline:
                    return
                item = (
                    np.array(data[:RAW_IMAGE_BYTES], copy=True),
                    time.time(),
                    telem.get("lat"), telem.get("lon"), telem.get("alt"),
                    telem.get("heading"),
                    telem.get("gimbal_pitch"), telem.get("gimbal_yaw"),
                )
                try:
                    q.put_nowait(item)
                except queue.Full:
                    clip.dropped[drone_id] += 1
            except Exception:
                # Recording is non-critical to flight; never break the
                # telemetry loop.
                pass
        return sink

    def _worker(self, clip, drone_id):
        """Encode one drone's frames for the life of `clip`, then close its
        files. Owns its VideoWriter and CSV entirely, so nothing else has to
        reach in to shut them down."""
        q = clip.queues[drone_id]
        mp4_path = os.path.join(clip.dir, "drone{}.mp4".format(drone_id))
        csv_path = os.path.join(clip.dir, "drone{}_frames.csv".format(drone_id))
        writer = None
        writer_ok = False       # tri-state with `writer`: None = not tried yet
        csv_file = None
        csv_writer = None
        n = 0
        t_first = t_last = None
        last_flush = time.time()
        err = None

        try:
            csv_file = open(csv_path, 'w', newline='')
            csv_writer = csv.DictWriter(csv_file, fieldnames=_FRAME_COLS,
                                        extrasaction='ignore')
            csv_writer.writeheader()
            csv_file.flush()

            while True:
                try:
                    item = q.get(timeout=0.25)
                except queue.Empty:
                    # Only ever reached with the queue empty, so once the sinks
                    # are off (active False) this is a complete drain.
                    if not clip.active:
                        break
                    continue

                yuv, t_epoch, lat, lon, alt, heading, gpitch, gyaw = item
                img = cv2.cvtColor(yuv.reshape(RAW_ROWS, RAW_COLS), self._cvt)

                if writer is None:
                    # Created lazily on the FIRST frame: a drone whose telemetry
                    # never parses never fires the sink, and an eagerly-created
                    # writer would leave a 0-byte unplayable MP4 behind.
                    h, w = img.shape[:2]
                    writer = cv2.VideoWriter(
                        mp4_path, cv2.VideoWriter_fourcc(*FOURCC),
                        self._fps, (w, h))
                    writer_ok = writer.isOpened()
                    if not writer_ok:
                        # VideoWriter does NOT raise on a missing codec — it
                        # returns a dead object whose write() silently no-ops.
                        err = ("codec '{}' unavailable; no video for drone {} "
                               "(frame index still written)".format(
                                   FOURCC, drone_id))
                        print("[clip] ERROR {}".format(err), flush=True)
                        clip.error = err
                        writer.release()

                if writer_ok:
                    writer.write(img)

                n += 1
                if t_first is None:
                    t_first = t_epoch
                t_last = t_epoch
                csv_writer.writerow({
                    'frame': n, 't_epoch': t_epoch,
                    'lat': lat, 'lon': lon, 'alt': alt, 'heading': heading,
                    'gimbal_pitch': gpitch, 'gimbal_yaw': gyaw,
                })
                clip.frames[drone_id] = n

                now = time.time()
                if now - last_flush >= _FLUSH_INTERVAL:
                    csv_file.flush()
                    last_flush = now
        except Exception as e:
            err = "drone {} recording failed: {}".format(drone_id, e)
            print("[clip] ERROR {}".format(err), flush=True)
            clip.error = err
        finally:
            if writer_ok:
                # This is what writes the moov atom. Skip it and the MP4 is
                # unplayable no matter how many frames went in.
                writer.release()
            if csv_file is not None:
                try:
                    csv_file.close()
                except (OSError, ValueError):
                    pass
            span = (t_last - t_first) if (n > 1 and t_first is not None) else 0.0
            clip.results[drone_id] = {
                'frames': n,
                'dropped': clip.dropped[drone_id],
                'fps_nominal': self._fps,
                'fps_actual': round((n - 1) / span, 2) if span > 0 else None,
                'duration_s': round(span, 2),
                't_first': t_first,
                't_last': t_last,
                'video': os.path.basename(mp4_path) if writer_ok else None,
                'error': err,
            }

    def _finish(self, clip):
        """Drain + close everything for a stopped clip, off the control loop."""
        for t in clip.threads:
            t.join(timeout=30)

        total = sum(clip.frames.values())
        summary = {
            'clip': clip.name,
            'dir': clip.dir,
            'reason': clip.reason,
            'duration_s': round(clip.duration, 1),
            'frames': total,
            't_end': clip.t_end_epoch,
            'error': clip.error,
        }

        final_meta = {
            'clip_stopped_iso': datetime.now().isoformat(timespec='seconds'),
            'clip_stop_reason': clip.reason,
            'clip_duration_s': summary['duration_s'],
            'drones': {str(k): v for k, v in sorted(clip.results.items())},
        }
        if clip.mirror is not None:
            clip.mirror.update_meta(final_meta)
            clip.mirror.close()
        else:
            self._write_own_session_json(clip, final_meta)

        for did in sorted(clip.results):
            r = clip.results[did]
            if r['frames'] == 0:
                print("[clip] WARNING drone {} recorded NO frames — no video "
                      "(is its telemetry parsing?)".format(did), flush=True)
            elif r['dropped']:
                print("[clip] WARNING drone {} dropped {} frames (encoder fell "
                      "behind)".format(did, r['dropped']), flush=True)
        rates = ", ".join(
            "d{}={}fr@{}fps".format(
                did, clip.results[did]['frames'],
                clip.results[did]['fps_actual'] or '?')
            for did in sorted(clip.results))
        print("[clip] saved {} ({:.1f} s, {}) — {}".format(
            clip.name, summary['duration_s'], rates, clip.dir), flush=True)

        self._last = summary

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _make_clip_dir(self):
        base = "clip_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        name = base
        n = 2
        # Clips are short, so two in one second is plausible in a way two
        # flights never are.
        while os.path.exists(os.path.join(self._base_dir, name)):
            name = "{}_{}".format(base, n)
            n += 1
        path = os.path.join(self._base_dir, name)
        os.makedirs(path)
        return path, name

    def _check_free_space(self, path):
        """Warn (don't refuse) when the disk cannot obviously hold a full-length
        clip. Returns a message or None."""
        try:
            free = shutil.disk_usage(path).free
        except OSError:
            return None
        need = (self._max_seconds * len(self._drones)
                * EST_BYTES_PER_S_PER_DRONE * 1.5)
        if free < need:
            return ("low disk space: {:.1f} GB free, a full {:.0f} s clip for "
                    "{} drones needs ~{:.1f} GB".format(
                        free / 1e9, self._max_seconds, len(self._drones),
                        need / 1e9))
        return None

    def _clip_meta(self, clip):
        meta = dict(self._meta)
        meta.update({
            'clip': clip.name,
            'clip_started_iso': datetime.fromtimestamp(
                clip.t_start_epoch).isoformat(timespec='milliseconds'),
            'clip_started_epoch': clip.t_start_epoch,
            'clip_max_s': clip.max_seconds,
            'fps_nominal': self._fps,
            'frame_size': [RAW_COLS, 1080],
            'flight_data': clip.mirror is not None or self._logger is not None,
            'timebase': ('drone*_frames.csv t_epoch is authoritative; the MP4 '
                         'header rate is nominal (see clip_recorder.py)'),
        })
        return meta

    def _write_own_session_json(self, clip, extra=None):
        """Fallback session.json for the no-flight-log case. When a mirror
        exists it writes this file instead — exactly one writer per path."""
        info = self._clip_meta(clip)
        info['flight_data'] = False
        if extra:
            info.update(extra)
        try:
            with open(os.path.join(clip.dir, 'session.json'), 'w') as f:
                json.dump(info, f, indent=2, default=str)
        except OSError as e:
            print("[clip] session.json write failed: {}".format(e), flush=True)


if __name__ == "__main__":
    # Standalone self-check: drive a fake 2-drone fleet with synthetic 1080p
    # frames through a full start -> auto-stop cycle and verify the artifacts.
    import tempfile

    class _FakeDrone:
        def __init__(self):
            self.sinks = []

        def add_frame_sink(self, sink):
            self.sinks.append(sink)

        def remove_frame_sink(self, sink):
            self.sinks = [s for s in self.sinks if s is not sink]

    def _frame(i):
        """A moving horizontal band, so the MP4 is obviously not a still."""
        buf = np.full(RAW_IMAGE_BYTES, 16, dtype=np.uint8)
        y = (i * 40) % 1000
        buf[y * RAW_COLS:(y + 60) * RAW_COLS] = 235
        buf[1080 * RAW_COLS:] = 128        # neutral chroma
        return buf

    out = tempfile.mkdtemp(prefix="clip_selfcheck_")
    log = FlightLogger(base_dir=os.path.join(out, "flight_logs"),
                       meta={"script": "clip_recorder", "test": True})
    drones = {1: _FakeDrone(), 2: _FakeDrone()}
    rec = ClipRecorder(drones, hw_decode=1,
                       base_dir=os.path.join(out, "recordings"),
                       max_seconds=2.0, logger=log,
                       meta={"script": "clip_recorder", "test": True})

    assert rec.status()["on"] is False
    assert rec.start() is True
    assert rec.start() is False, "double start must be refused"
    assert rec.is_recording()

    i = 0
    while not rec.poll():                  # runs until the 2 s auto-stop
        telem = {"lat": 46.5 + i * 1e-6, "lon": 6.56, "alt": 10.0 + i * 0.01,
                 "heading": (i * 3) % 360, "gimbal_pitch": -2.0,
                 "gimbal_yaw": 0.0}
        for d in drones.values():
            for s in d.sinks:
                s(_frame(i), telem)
        log.log_telemetry(1, telem)
        log.log_drone_command(1, "VS", 0.1, 0.0, 0.0, 10.0, -2.0, 0.0, cmd="VS:…")
        i += 1
        time.sleep(0.05)

    clip_name = rec.status()["clip"]
    elapsed = rec.status()["elapsed"]
    assert 1.9 <= elapsed <= 2.6, elapsed
    rec.stop("max_duration")
    assert rec.status()["on"] is False
    for d in drones.values():
        assert not d.sinks, "sinks must be removed on stop"
    rec.close()
    log.close()

    clip_dir = os.path.join(out, "recordings", clip_name)
    counts = {}
    for did in (1, 2):
        mp4 = os.path.join(clip_dir, "drone{}.mp4".format(did))
        assert os.path.getsize(mp4) > 10000, mp4
        cap = cv2.VideoCapture(mp4)
        assert cap.isOpened(), mp4
        n_video = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        assert (cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) == (1920.0, 1080.0)
        cap.release()
        with open(os.path.join(clip_dir,
                               "drone{}_frames.csv".format(did))) as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == n_video, (len(rows), n_video)
        assert [int(r['frame']) for r in rows] == list(range(1, len(rows) + 1))
        ts = [float(r['t_epoch']) for r in rows]
        assert ts == sorted(ts) and ts[0] > 0
        counts[did] = len(rows)

    with open(os.path.join(clip_dir, "session.json")) as f:
        info = json.load(f)
    assert info['meta']['clip'] == clip_name
    assert info['meta']['clip_stop_reason'] == "max_duration"
    for did, n_rows in counts.items():
        assert info['meta']['drones'][str(did)]['frames'] == n_rows
    for fname in ('telemetry.csv', 'drone_commands.csv',
                  'user_commands.csv', 'swarm_debug.csv'):
        assert os.path.exists(os.path.join(clip_dir, fname)), fname
    with open(os.path.join(clip_dir, 'telemetry.csv')) as f:
        assert len(list(csv.DictReader(f))) == i

    last = rec.status()["last"]
    assert last['reason'] == "max_duration" and last['frames'] > 0
    print("Self-check OK — frames/drone {}, auto-stopped at {:.1f} s".format(
        counts, last['duration_s']))
    print("Inspect (delete when done):", clip_dir)
