"""
LIS_Swarm clip replay — feed a recorded clip to the sim as if the drones were flying
===================================================================================
Reads a `recordings/clip_*` folder written by `clip_recorder.py` and republishes it
into the **`DroneFeedSharedMemory`** mapping in real time, so the Unity DJI scene
(`ImageSharing.cs` -> `BlockSharedMemory` -> `StitcherThreading.py`) runs with **no
change whatsoever** from a live flight. The stitcher cannot tell the difference:
same map, same 48-byte v2 block header, same 800x450 BGR payload, same per-frame
camera pose, same relative capture times, same rate.

That is the point of this tool. Stitcher work needs the same footage over and over
with one variable changed; drones give you a different flight every time.

    cd "AOS server"
    python clip_replay.py --clip recordings\\clip_20260806_143012 --loop

No `ds_wrapper`, no `DroneSwarmServer`, no admin, no drones, no joystick — it only
reads files and writes one shared-memory mapping. Run it in place of
`swarm_flocking.py --image-stream-pose`, with the Unity scene started as usual.

Wire fidelity
-------------
Every byte is written by the SAME `utils.imageSharingUtil.write_memory` the live
publisher uses, and the map name / block capacity / header size are imported from
`image_stream_feed` rather than restated. A wire mismatch between live and replay
is therefore not possible without breaking both at once — which matters, because a
mismatch here is silent: the consumer reads image bytes as a header instead of
failing.

  * `pos`/`quat`/`poseStatus` come from the clip's `drone{N}_frames.csv`, where they
    were already solved by `dji_camera_pose.CameraPoseSolver` against one fleet-wide
    latched origin. They are NOT re-derived — re-deriving would relatch the origin
    from whichever row happened to be read first and put the replay in a different
    frame from the recording.
  * `captureTime` is rebased to `t_epoch - t0`, with `t0` the earliest frame in the
    whole clip. Absolute value is meaningless to the consumer (float32, read only as
    a difference) but the CROSS-DRONE SKEW is preserved exactly as recorded, which is
    what `PlanarStitcher.MAX_CAPTURE_SKEW_S` gates on.
  * Frames are paced off the recorded `t_epoch`, so the per-drone rate, the jitter and
    the drift between aircraft all replay as they happened.

What the sim still needs (this tool cannot supply it)
-----------------------------------------------------
`PyUniSharingFast` must be in the scene and publishing metadata — it is the only
writer of the wire version, the intrinsics and the scene plane, and
`StitcherThreading.planar_inputs_ready()` refuses PLANAR without all three. Settings
that must match the clip are printed at startup; check them against the inspector.

Frames whose `pose_status` is 0 (no GPS fix at capture) are published with
`poseStatus 0`, exactly as they would have been live: PLANAR drops that view and
keeps the rest of the frame.
"""

import argparse
import csv
import mmap
import os
import sys
import threading
import time

import cv2

# Imported, never restated: one definition of the wire for live and replay.
from image_stream_feed import (
    BLOCK_MAP_NAME, BLOCK_HEADER_BYTES, MAX_DRONES,
)
import utils.imageSharingUtil as imageSharingUtil


# Must match ImageSharing.cs ImageWidth/ImageHeight and the live publisher.
OUT_W, OUT_H = 800, 450

# write_memory's post-write sleep is left OFF here: this replayer schedules every
# frame against the recorded t_epoch, and the recorded interval (~50 ms at the
# 20 Hz telemetry rate) already exceeds Unity's 50 ms read interval. Adding pace_s
# on top would stretch the timeline and destroy the thing being replayed. The
# flag handshake inside write_memory still gates each write on the consumer.
PACE_S = 0.0


def load_clip(clip_dir):
    """Return (drones, t0) where drones is {drone_id: [row, ...]} sorted by time
    and t0 is the earliest t_epoch across the whole clip."""
    drones = {}
    for name in sorted(os.listdir(clip_dir)):
        if not (name.startswith("drone") and name.endswith("_frames.csv")):
            continue
        try:
            did = int(name[len("drone"):-len("_frames.csv")])
        except ValueError:
            continue
        mp4 = os.path.join(clip_dir, "drone{}.mp4".format(did))
        if not os.path.exists(mp4):
            print("[replay] drone {}: no video, skipping".format(did))
            continue
        with open(os.path.join(clip_dir, name), newline='') as f:
            rows = list(csv.DictReader(f))
        if not rows:
            print("[replay] drone {}: empty frame index, skipping".format(did))
            continue
        if not 1 <= did <= MAX_DRONES:
            print("[replay] drone {} outside the mapping capacity 1..{}, "
                  "skipping".format(did, MAX_DRONES))
            continue
        drones[did] = rows
    if not drones:
        return {}, 0.0
    t0 = min(float(rows[0]['t_epoch']) for rows in drones.values())
    return drones, t0


def _pose_of(row):
    """((x,y,z), (qx,qy,qz,qw)) from a frame row, or None when it was unposed.

    Returning None makes write_memory publish poseStatus 0 — the same thing the
    live publisher does for a drone without a fix.
    """
    try:
        if not int(row.get('pose_status') or 0):
            return None
        return ((float(row['pos_x']), float(row['pos_y']), float(row['pos_z'])),
                (float(row['quat_x']), float(row['quat_y']),
                 float(row['quat_z']), float(row['quat_w'])))
    except (KeyError, TypeError, ValueError):
        # A clip recorded before the pose columns existed. Publishable, but not
        # to PLANAR; the startup summary says so.
        return None


class _DroneReplay(threading.Thread):
    """Replays one drone's MP4 + frame index into its own block, on its own mmap.

    Own mmap per drone for the same reason the live publisher does it: multiple
    views of one Windows named mapping share pages but keep independent file
    positions, and write_memory drives non-atomic seek()->write() sequences. A
    single shared mmap lets one drone's write resume at another's offset.
    """

    def __init__(self, drone_id, rows, mp4_path, t0, speed, loop, pose_lead_s,
                 stop_evt):
        threading.Thread.__init__(self, name="ClipReplay_{}".format(drone_id),
                                  daemon=True)
        self.drone_id = drone_id
        self.rows = rows
        self.mp4_path = mp4_path
        self.t0 = t0
        self.speed = speed
        self.loop = loop
        self.pose_lead_s = pose_lead_s
        self.stop_evt = stop_evt
        self.image_bytes = OUT_W * OUT_H * 3
        self.block_bytes = BLOCK_HEADER_BYTES + self.image_bytes
        self.published = 0
        self.late = 0
        self.unposed = 0

    def _pose_row_for(self, i):
        """The row whose pose is attached to frame i.

        With `pose_lead_s == 0` that is frame i's own pose — the faithful replay,
        and the tightest pairing the recording could make (frame and telemetry came
        from one ds_wrapper fetch). A non-zero lead re-pairs each frame with the
        pose from `pose_lead_s` seconds later (+) or earlier (-), which is the one
        knob for the dominant error term in the pose-driven mosaic: telemetry is
        only ~5 Hz fresh and the video has its own latency, so a COMMON pose/video
        skew survives the recording and no gate in the stitcher can remove it.
        Sweep it, watch the seams, keep the value.
        """
        if not self.pose_lead_s:
            return self.rows[i]
        want = float(self.rows[i]['t_epoch']) + self.pose_lead_s
        j = min(range(len(self.rows)),
                key=lambda k: abs(float(self.rows[k]['t_epoch']) - want))
        return self.rows[j]

    def run(self):
        mmf = mmap.mmap(-1, MAX_DRONES * self.block_bytes, BLOCK_MAP_NAME)
        offset = (self.drone_id - 1) * self.block_bytes
        try:
            while not self.stop_evt.is_set():
                cap = cv2.VideoCapture(self.mp4_path)
                if not cap.isOpened():
                    print("[replay] drone {}: cannot open {}".format(
                        self.drone_id, self.mp4_path), flush=True)
                    return
                # Wall-clock instant that maps to the clip's t0 for this pass.
                epoch0 = time.monotonic()
                i = 0
                try:
                    while not self.stop_evt.is_set():
                        ok, frame = cap.read()
                        if not ok or i >= len(self.rows):
                            break
                        row = self.rows[i]
                        # Recorded offset from the clip's start, time-scaled.
                        due = ((float(row['t_epoch']) - self.t0) / self.speed)
                        wait = (epoch0 + due) - time.monotonic()
                        if wait > 0:
                            if self.stop_evt.wait(wait):
                                break
                        elif wait < -0.5:
                            self.late += 1

                        img = cv2.resize(frame, (OUT_W, OUT_H))
                        pose = _pose_of(self._pose_row_for(i))
                        if pose is None:
                            self.unposed += 1
                        heading = float(row.get('heading') or 0.0)
                        # capture_time: seconds since the clip's own start, so
                        # cross-drone skew is preserved and float32 keeps its
                        # resolution (see the module docstring).
                        imageSharingUtil.write_memory(
                            mmf, offset, self.image_bytes, img,
                            self.drone_id - 1, heading, pace_s=PACE_S,
                            pose=pose, capture_time=due)
                        self.published += 1
                        i += 1
                finally:
                    cap.release()
                if not self.loop:
                    break
        finally:
            try:
                mmf.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(
        description="Replay a recorded clip into DroneFeedSharedMemory as if the "
                    "drones were flying, so the Unity scene and stitcher run "
                    "unchanged.")
    ap.add_argument("--clip", required=True,
                    help="Clip folder (e.g. recordings\\clip_20260806_143012)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Time scale; 1.0 = real time (default). >1 replays "
                         "faster, but the consumer handshake will throttle it.")
    ap.add_argument("--loop", action="store_true",
                    help="Restart the clip when it ends — what you want while "
                         "tuning the stitcher against fixed footage")
    ap.add_argument("--pose-lead-s", type=float, default=0.0,
                    help="Re-pair each frame with the pose this many seconds "
                         "later (+) or earlier (-). 0 = faithful replay. The one "
                         "knob for pose/video skew, the dominant mosaic error.")
    args = ap.parse_args()

    clip_dir = args.clip
    if not os.path.isabs(clip_dir):
        clip_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                clip_dir)
    if not os.path.isdir(clip_dir):
        sys.exit("clip folder not found: {}".format(clip_dir))
    if args.speed <= 0:
        sys.exit("--speed must be > 0")

    drones, t0 = load_clip(clip_dir)
    if not drones:
        sys.exit("no replayable drones in {} (need drone{{N}}.mp4 + "
                 "drone{{N}}_frames.csv)".format(clip_dir))

    print("Clip replay -> {} ({} blocks x {}x{})".format(
        BLOCK_MAP_NAME, MAX_DRONES, OUT_W, OUT_H))
    print("  {}".format(clip_dir))
    span = max(float(r[-1]['t_epoch']) for r in drones.values()) - t0
    posed = sum(1 for rows in drones.values() for r in rows if _pose_of(r))
    total = sum(len(rows) for rows in drones.values())
    for did in sorted(drones):
        rows = drones[did]
        n_posed = sum(1 for r in rows if _pose_of(r))
        print("  drone {}: {} frames, {} posed, first t+{:.2f}s".format(
            did, len(rows), n_posed, float(rows[0]['t_epoch']) - t0))
    print("  {:.1f} s of footage, {}/{} frames posed{}".format(
        span, posed, total,
        ", speed x{:.2f}".format(args.speed) if args.speed != 1.0 else ""))
    if args.pose_lead_s:
        print("  POSE LEAD {:+.3f} s — poses re-paired, not a faithful replay"
              .format(args.pose_lead_s))
    if posed == 0:
        print("  WARNING no frame carries a camera pose. PLANAR will drop every "
              "view and fall back to the individual feeds; STABSTITCH is fine. "
              "(A clip recorded before the pose columns existed?)")

    # Intrinsics are the sim's to supply — print what the clip was taken with so a
    # mismatch is caught in the inspector instead of showing up as a bent mosaic.
    cam = {}
    try:
        import json
        with open(os.path.join(clip_dir, "session.json")) as f:
            cam = (json.load(f).get('meta') or {}).get('camera') or {}
    except Exception:
        pass
    print("\nSet these in the Unity scene (this tool cannot — PyUniSharingFast "
          "owns the metadata,\nand planar_inputs_ready() refuses PLANAR without "
          "wire version + intrinsics + a plane):")
    print("  PyUniSharingFast: typeOfStitcher=PLANAR, useManualIntrinsics=true,")
    print("                    manualVerticalFovDeg={}, planarBlendMode=Nearest,"
          .format(cam.get('vfov_deg', 46.4)))
    print("                    scenePlaneMode=FormationRelative, "
          "planarStandoffMetres=<distance to the surface>")
    print("  ImageSharing:     stitchSlots >= {}".format(len(drones)))
    print("\nCtrl+C to stop.\n")

    stop_evt = threading.Event()
    workers = [_DroneReplay(did, rows,
                            os.path.join(clip_dir, "drone{}.mp4".format(did)),
                            t0, args.speed, args.loop, args.pose_lead_s, stop_evt)
               for did, rows in sorted(drones.items())]
    for w in workers:
        w.start()
    try:
        while any(w.is_alive() for w in workers):
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopping.")
        stop_evt.set()
    for w in workers:
        w.join(timeout=3)

    print("Published: " + ", ".join(
        "drone {}={}".format(w.drone_id, w.published) for w in workers))
    late = sum(w.late for w in workers)
    if late:
        print("{} frames published >0.5 s behind schedule — the consumer "
              "handshake is throttling the replay (lower --speed, or check that "
              "Unity is reading).".format(late))


if __name__ == "__main__":
    main()
