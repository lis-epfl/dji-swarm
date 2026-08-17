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
    python clip_replay.py --clip grass_nadir_long --loop     # same thing, by label

No `ds_wrapper`, no `DroneSwarmServer`, no admin, no drones, no joystick — it only
reads files and writes one shared-memory mapping. Run it in place of
`swarm_flocking.py --image-stream-pose`, with the Unity scene started as usual.

Clip labels
-----------
`clip_recorder.py` owns the folder name (`clip_<timestamp>`) and it stays that way —
`session.json`'s own `clip` field references it, and the timestamp is what joins a
clip back to `flight_logs/`. The human name is stored *inside* the clip, as
`meta.label` in its `session.json`, so it travels with the data rather than living
in a side index that can drift from the folders:

    python clip_replay.py --list                             # what is on disk
    python clip_replay.py --clip clip_20260806_143012 --set-label grass_nadir_long

`--clip` then accepts a label, a folder name under the recordings root, or a path,
in that order of preference. Labels are matched case-insensitively and `--set-label`
refuses a name already carried by another clip — two clips answering to one `--clip`
would silently replay the wrong footage, which is exactly the kind of mix-up a
stitcher comparison cannot survive.

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
    frame from the recording. The corollary is that a clip recorded before the
    gimbal-yaw fix still carries the bad camera bearing in those columns, and no
    amount of replaying fixes it: run `migrate_clip_poses.py` on the clip once.
    `pose_convention_note` refuses to let one through quietly.
  * `captureTime` is seconds since the REPLAY started — the same "since the producer
    started" clock `image_stream_feed.py` publishes live, so the consumer cannot tell a
    replay from a flight. It is rebased off `t_epoch - t0` (`t0` = the earliest frame in
    the whole clip) and then advances monotonically across loop passes. Absolute value is
    meaningless to the consumer (float32, read only as a difference) but the CROSS-DRONE
    SKEW is preserved exactly as recorded, which is what
    `PlanarStitcher.MAX_CAPTURE_SKEW_S` gates on.
  * Frames are paced off the recorded `t_epoch`, so the per-drone rate, the jitter and
    the drift between aircraft all replay as they happened.
  * Under `--loop` every drone shares ONE pass clock, whose period is the longest
    drone's span (plus `LOOP_GAP_S`). Drones with shorter footage hold their last
    frame until the shared boundary. Without this the cross-drone skew above holds
    only on the first pass and then accumulates without bound, until the stitcher
    drops the slowest drone's view as stale — see `_DroneReplay.run`.

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
import json
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
# The scene-plane measurement lives in its own pure module (no cv2, no shared
# memory) so it can be self-checked without a clip; this file owns the CLI
# because it owns clip addressing and the Unity-settings banner.
import clip_scene_plane as scene_plane
from clip_scene_plane import DEFAULT_SHAPES_FILE
# Read-only view of what the Unity scene is actually set to. Nothing here can
# *set* a Unity value — see that module's docstring for why the mapping is the
# producer's alone — so the help this tool can give is a diff, not an apply.
import unity_stitch_meta as unity_meta
import utils.imageSharingUtil as imageSharingUtil


# Must match ImageSharing.cs ImageWidth/ImageHeight and the live publisher.
OUT_W, OUT_H = 800, 450

# write_memory's post-write sleep is left OFF here: this replayer schedules every
# frame against the recorded t_epoch, and the recorded interval (~50 ms at the
# 20 Hz telemetry rate) already exceeds Unity's 50 ms read interval. Adding pace_s
# on top would stretch the timeline and destroy the thing being replayed. The
# flag handshake inside write_memory still gates each write on the consumer.
PACE_S = 0.0

# Slack added to every loop's pass period, identically for every drone.
#
# It absorbs the per-pass cv2.VideoCapture reopen (~33 ms measured on all three
# aircraft of the MED clips) so that cost delays nobody's first frame.
#
# Kept small on purpose. It is paid as a HOLD on the last frame, and a drone
# holding its last frame is genuinely stale by exactly that long -- so the gap
# lands directly in the cross-drone capture_time spread the stitcher gates on.
# At 0.15 s the spread on the MED clips peaked at 0.19 s against a 0.25 s gate,
# which is too little margin for a constant that buys nothing above the reopen.
LOOP_GAP_S = 0.08

# Mirror of PlanarStitcher.MAX_CAPTURE_SKEW_S in the vr_swarm_simulation repo.
# Not importable across the repo boundary, and only used to warn -- a clip whose
# drones stop at very different times makes the shared pass boundary hold the
# early finishers past what the stitcher will accept, and it drops those views
# near every boundary. That is honest (their footage really has ended) but it
# looks exactly like the accumulating-skew bug this pass clock removes, so say so.
STITCHER_MAX_SKEW_S = 0.25

# Where clips live by default. Matches clip_recorder's --recording-dir default; a
# session started with a different RecordingDir needs --recordings-dir to match.
DEFAULT_RECORDINGS = "recordings"

HERE = os.path.dirname(os.path.abspath(__file__))


def _session_meta(clip_dir):
    """The clip's `session.json` `meta` dict, or {} if absent/unreadable."""
    try:
        with open(os.path.join(clip_dir, "session.json")) as f:
            return json.load(f).get('meta') or {}
    except (IOError, OSError, ValueError, AttributeError):
        return {}


def label_of(clip_dir):
    """The clip's human label, or None. Blank/whitespace counts as unlabelled."""
    lab = _session_meta(clip_dir).get('label')
    if isinstance(lab, str) and lab.strip():
        return lab.strip()
    return None


def pose_convention_note(clip_dir):
    """A warning string when the clip's stored poses predate the gimbal-yaw fix.

    `dji_camera_pose` used to build the camera bearing from telemetry
    `gimbal_yaw`, which on this fleet carries a per-aircraft offset re-rolled at
    every takeoff. That pointed the cameras up to 18 deg apart while they were
    physically parallel — 11.3 m of seam at the MED facade's 34 m standoff, and
    the reason PLANAR would not stitch those clips.

    Replay publishes the stored `quat_*` verbatim, so an unmigrated clip replays
    the bad bearing and looks like a stitcher fault rather than a stale file.
    The marker is the only honest test: a clip whose slip happened to be near
    zero is numerically indistinguishable from a corrected one.

    Returns None when the clip is fine or carries no pose at all.
    """
    meta = _session_meta(clip_dir)
    if meta.get('pose_yaw_source') == 'heading':
        return None
    if not meta.get('pose_origin_latlon'):
        return None  # no solved pose in this clip; nothing to be stale.
    return ("stored poses predate the gimbal-yaw fix (no meta.pose_yaw_source). "
            "PLANAR will not line up. Fix the clip on disk with:\n"
            "    python migrate_clip_poses.py --clip {} --apply"
            .format(label_of(clip_dir) or os.path.basename(clip_dir)))


def iter_clips(root):
    """Every replayable clip folder under `root`, sorted (so chronological, since
    the folder names are timestamps)."""
    if not os.path.isdir(root):
        return []
    out = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d):
            continue
        try:
            entries = os.listdir(d)
        except OSError:
            continue
        if any(e.startswith("drone") and e.endswith("_frames.csv")
               for e in entries):
            out.append(d)
    return out


def resolve_clip(spec, root):
    """Turn a --clip value into a folder: path, then label, then folder name.

    Paths win so existing command lines and scripts keep working untouched.
    """
    cand = spec if os.path.isabs(spec) else os.path.join(HERE, spec)
    if os.path.isdir(cand):
        return cand

    want = spec.strip().lower()
    hits = [d for d in iter_clips(root) if (label_of(d) or '').lower() == want]
    if len(hits) == 1:
        return hits[0]
    if len(hits) > 1:
        # Only reachable for clips labelled before --set-label existed, or by
        # hand-editing session.json. Refuse rather than pick one.
        sys.exit("label {!r} is on {} clips — ambiguous:\n  {}".format(
            spec, len(hits),
            "\n  ".join(os.path.basename(d) for d in hits)))

    cand = os.path.join(root, spec)
    if os.path.isdir(cand):
        return cand

    known = [(label_of(d), os.path.basename(d)) for d in iter_clips(root)]
    sys.exit("no clip {!r} (not a path, label or folder under {}).\n"
             "Known clips:\n  {}".format(
                 spec, root,
                 "\n  ".join("{:<28}{}".format(lab or "(unlabelled)", folder)
                             for lab, folder in known) or "(none)"))


def set_label(clip_dir, label, root):
    """Write `label` into the clip's session.json meta, refusing duplicates."""
    label = label.strip()
    if not label:
        sys.exit("--set-label needs a non-empty name")
    for other in iter_clips(root):
        if os.path.abspath(other) == os.path.abspath(clip_dir):
            continue
        if (label_of(other) or '').lower() == label.lower():
            sys.exit("label {!r} is already on {} - labels must be unique, or "
                     "--clip <label> would replay the wrong footage".format(
                         label, os.path.basename(other)))

    path = os.path.join(clip_dir, "session.json")
    try:
        with open(path) as f:
            doc = json.load(f)
    except (IOError, OSError, ValueError) as e:
        sys.exit("cannot read {}: {}".format(path, e))
    if not isinstance(doc, dict):
        sys.exit("{} is not a JSON object".format(path))
    meta = doc.get('meta')
    if not isinstance(meta, dict):
        meta = {}
    old = label_of(clip_dir)
    meta['label'] = label
    doc['meta'] = meta
    try:
        with open(path, 'w') as f:
            json.dump(doc, f, indent=2)
    except (IOError, OSError) as e:
        sys.exit("cannot write {}: {}".format(path, e))
    print("{}: label {}{}".format(os.path.basename(clip_dir), label,
                                  " (was {})".format(old) if old else ""))


def unity_requirements(clip_dir, cam, n_drones):
    """What the Unity inspector has to say for THIS clip to stitch.

    Everything here is derived from the clip itself rather than hard-coded: the
    intrinsics from its camera block, the standoff from its measured scene plane.
    A clip shot at a different resolution or a different site therefore checks
    against its own numbers.
    """
    want = {
        "stitcher": "PLANAR",
        "fx": scene_plane.wire_focal_px(cam),
        "vfov_deg": cam.get('vfov_deg') or 46.4,
        "plane_mode": unity_meta.PLANE_MODE_FORMATION_RELATIVE,
        "blend_mode": unity_meta.BLEND_NEAREST,
        "wire_version": unity_meta.PLANAR_WIRE_VERSION,
        "drones": n_drones,
    }
    sp = scene_plane.scene_plane_of(clip_dir)
    if sp and sp.get('standoff_m'):
        want["standoff"] = sp['standoff_m']
    return want


class _UnityWatch(threading.Thread):
    """Reports inspector edits made to the running scene while the clip replays.

    Exists for the tuning loop the plane budget forces: with `--loop` running,
    the operator steps `planarStandoffMetres` until the seams close, and without
    this there is no record of which values were tried — the number lives in an
    inspector field in another process. Prints only on change, so a scene left
    alone is silent.

    Read-only and best-effort: a failed read is a skipped tick, never an
    interruption of the replay.
    """

    PERIOD_S = 2.0

    def __init__(self, want, stop_evt):
        threading.Thread.__init__(self, name="UnityWatch", daemon=True)
        self.want = want
        self.stop_evt = stop_evt
        self.last = None
        self.was_open = None

    @staticmethod
    def _render(field, meta):
        if field == "plane_mode":
            return unity_meta.PLANE_MODES.get(meta[field], meta[field])
        if field == "blend_mode":
            return unity_meta.BLEND_MODES.get(meta[field], meta[field])
        if field == "standoff_source":
            # The one watched field that is not an operator edit but a CONSEQUENCE of
            # one: flipping planarStandoffSource to Inspector, or the scene losing our
            # trailer, both show up here. Worth reporting for exactly that reason —
            # it is the moment this replay stops being the thing driving the plane.
            return unity_meta.STANDOFF_SOURCES.get(meta[field], meta[field])
        if field in ("standoff", "sweep_range"):
            return "{:.2f}".format(meta[field])
        return str(meta[field])

    def run(self):
        while not self.stop_evt.wait(self.PERIOD_S):
            try:
                # No liveness wait: this loop polls anyway, so a stalled
                # heartbeat shows up as "nothing changed".
                meta = unity_meta.read(liveness_wait_s=0.0)
            except Exception:
                continue
            is_open = meta is not None and meta.get("plausible")
            if is_open != self.was_open and self.was_open is not None:
                print("[unity] scene {} publishing settings".format(
                    "started" if is_open else "stopped"), flush=True)
            self.was_open = is_open
            if not is_open:
                self.last = None
                continue
            if self.last is not None:
                changed = [f for f in unity_meta.watch_fields
                           if meta.get(f) != self.last.get(f)]
                for f in changed:
                    print("[unity] {}: {} -> {}".format(
                        unity_meta.INSPECTOR_NAMES.get(f, f),
                        self._render(f, self.last), self._render(f, meta)),
                        flush=True)
                if changed:
                    bad, _info = unity_meta.compare(meta, self.want)
                    print("[unity] {}".format(
                        "matches this clip" if not bad else
                        "still wrong: " + ", ".join(
                            "{}={} (needs {})".format(f, c, n)
                            for f, c, n in bad)), flush=True)
            self.last = meta


def check_unity(want, indent="  ", compact=False):
    """Print the live-settings diff. True when the scene is ready for this clip."""
    try:
        meta = unity_meta.read()
    except Exception as e:
        print(indent + "could not read {}: {}".format(unity_meta.MAP_NAME, e))
        return False
    for line in unity_meta.describe(meta, want, indent=indent, compact=compact):
        print(line)
    if meta is None or not meta.get("plausible"):
        return False
    return not unity_meta.compare(meta, want)[0]


def set_scene_plane(clip_dir, args):
    """Measure the clip's standoff from a georeferenced facade and store it.

    Separate from the replay path on purpose: the plane is a property of the
    footage, so it is measured once, written into the clip, and every later
    replay just prints it. Exits on anything that would store a number
    describing different footage than the clip holds.
    """
    given = [bool(args.set_plane_from_line),
             args.set_plane_from_shape is not None,
             args.set_plane_from_facade is not None,
             args.set_plane_standoff is not None,
             bool(args.set_plane_from_altitude)]
    if sum(given) > 1:
        sys.exit("give one plane source: --set-plane-from-line, "
                 "--set-plane-from-facade, --set-plane-from-shape, "
                 "--set-plane-from-altitude or --set-plane-standoff")

    if args.set_plane_standoff is not None:
        try:
            report = scene_plane.manual_report(args.set_plane_standoff, clip_dir)
        except ValueError as e:
            sys.exit(str(e))
    elif args.set_plane_from_altitude:
        # No georeferencing needed and none wanted: the ground plane is already
        # in the clip's own pose frame, so unlike every branch below this one
        # needs no origin, no trace and no shapes.json. --plane-offset carries
        # its one site measurement (how far the ground sits below the pad).
        try:
            report = scene_plane.analyse(
                clip_dir, scene_plane.ground_plane(args.plane_offset),
                _session_meta(clip_dir))
        except ValueError as e:
            sys.exit(str(e))
    else:
        meta = _session_meta(clip_dir)
        origin = meta.get('pose_origin_latlon')
        if not (isinstance(origin, (list, tuple)) and len(origin) == 2):
            sys.exit("this clip has no pose_origin_latlon, so its poses cannot "
                     "be georeferenced — the standoff has to come from "
                     "--set-plane-standoff instead. (Recorded before the pose "
                     "columns existed?)")
        try:
            toward = scene_plane.formation_centroid_xz(clip_dir)
        except ValueError as e:
            sys.exit(str(e))

        if args.set_plane_from_line:
            parts = args.set_plane_from_line.replace(" ", "").split(",")
            if len(parts) != 4:
                sys.exit("--set-plane-from-line takes LAT1,LON1,LAT2,LON2")
            try:
                v = [float(p) for p in parts]
            except ValueError:
                sys.exit("--set-plane-from-line values must be numbers")
            try:
                facade = scene_plane.facade_from_line((v[0], v[1]), (v[2], v[3]),
                                                     origin, toward)
            except ValueError as e:
                sys.exit(str(e))
        elif args.set_plane_from_facade is not None:
            # A wall of a real building footprint, picked on the GUI map. This
            # routes to facade_from_line, so the azimuth is the wall's true
            # bearing — the reason to prefer it over --set-plane-from-shape,
            # whose rectangle can only answer due N/S/E/W.
            shapes = args.shapes
            if not os.path.isabs(shapes):
                shapes = os.path.join(HERE, shapes)
            want = (None if args.set_plane_from_facade == -1
                    else args.set_plane_from_facade)
            rec, err = scene_plane.facade_by_id(shapes, want)
            if rec is None:
                sys.exit("{}. In the GUI, press 'Pick building', click the "
                         "building, then click the wall you filmed (it "
                         "persists to shapes.json with no controller "
                         "running).".format(err))
            try:
                facade = scene_plane.facade_from_stored(rec, origin, toward)
            except ValueError as e:
                sys.exit(str(e))
        else:
            shapes = args.shapes
            if not os.path.isabs(shapes):
                shapes = os.path.join(HERE, shapes)
            want = None if args.set_plane_from_shape == -1 else args.set_plane_from_shape
            rect, err = scene_plane.obstacle_by_id(shapes, want)
            if rect is None:
                sys.exit("{}. Pick the building with the GUI's 'Pick building' "
                         "and click its wall (--set-plane-from-facade, and the "
                         "azimuth is not snapped), draw a box with 'Add "
                         "obstacle', or use --set-plane-from-line.".format(err))
            try:
                facade = scene_plane.facade_from_obstacle(rect, origin, toward)
            except ValueError as e:
                sys.exit(str(e))

        try:
            report = scene_plane.analyse(
                clip_dir, scene_plane.offset_facade(facade, args.plane_offset),
                meta)
        except ValueError as e:
            sys.exit(str(e))

    try:
        stored = scene_plane.store_scene_plane(clip_dir, report)
    except (IOError, OSError, ValueError) as e:
        sys.exit("cannot write the plane into {}: {}".format(
            os.path.join(clip_dir, "session.json"), e))
    print("{}{}".format(os.path.basename(clip_dir),
                        "  [{}]".format(label_of(clip_dir))
                        if label_of(clip_dir) else ""))
    for line in scene_plane.describe(stored):
        print(line)
    print("\nStored in session.json as meta.scene_plane. Set "
          "scenePlaneMode=FormationRelative and\nplanarStandoffMetres="
          "{:.2f} in PyUniSharingFast.".format(stored["standoff_m"]))


# A view starved of frames is the one clip defect that cannot be seen from the
# folder listing and cannot be fixed afterwards: PLANAR needs every view present
# at the same instant, so one aircraft at a fraction of the fetch rate quietly
# costs the mosaic that view for most of the clip. Fractions of the clip's own
# nominal rate, so they hold if the fetch rate ever changes.
FPS_BAD_FRAC = 0.5
FPS_WARN_FRAC = 0.8


def video_health(meta):
    """('OK'|'WARN'|'BAD', note) for a clip's per-drone frame rates.

    Compares each drone's measured rate against the clip's nominal rate rather
    than against the other drones: a clip where every link degraded together is
    still degraded.
    """
    per = meta.get('drones') or {}
    nominal = meta.get('fps_nominal') or 0.0
    duration = meta.get('clip_duration_s') or 0.0
    rates = []
    for did in sorted(per, key=lambda k: str(k)):
        entry = per[did] or {}
        got = entry.get('fps_actual')
        if isinstance(got, (int, float)):
            rates.append((str(did), float(got)))
            continue
        # No rate recorded. NOT the same as "no such drone": the recorder writes
        # an entry per aircraft it opened a video for, and leaves fps_actual null
        # when there were too few frames to measure one. That is the worst case
        # there is — clip_20260814_181306's drone 6 managed a single frame in
        # 120 s — so falling through to "unknown, assume fine" hid a dead view
        # behind an `ok`. Rate it from the frame count instead.
        frames = entry.get('frames')
        if isinstance(frames, (int, float)) and duration > 0:
            rates.append((str(did), float(frames) / duration))
    if not rates or not nominal:
        return 'OK', ''
    worst = min(f for _, f in rates)
    frac = worst / nominal
    if frac >= FPS_WARN_FRAC:
        return 'OK', ''
    level = 'BAD' if frac < FPS_BAD_FRAC else 'WARN'
    low = ", ".join("D{} {:.1f}".format(did, f) for did, f in rates
                    if f / nominal < FPS_WARN_FRAC)
    return level, "{} fps vs {:.0f} nominal".format(low, nominal)


def list_clips(root):
    """Print the clips on disk with their labels — the lookup table for --clip."""
    clips = iter_clips(root)
    print("Clips in {}".format(root))
    if not clips:
        print("  (none)")
        return
    print("  {:<32}{:<32}{:>7}  {:<6}{}".format(
        "LABEL", "FOLDER", "DUR", "VIDEO", "FRAMES POSED"))
    flagged = []
    for d in clips:
        meta = _session_meta(d)
        dur = meta.get('clip_duration_s')
        per = meta.get('drones') or {}
        frames = sum((v or {}).get('frames') or 0 for v in per.values())
        posed = sum((v or {}).get('frames_posed') or 0 for v in per.values())
        level, note = video_health(meta)
        if level != 'OK':
            flagged.append((label_of(d) or os.path.basename(d), level, note))
        print("  {:<32}{:<32}{:>7}  {:<6}{}".format(
            label_of(d) or "(unlabelled)",
            os.path.basename(d),
            "{:.1f}s".format(dur) if isinstance(dur, (int, float)) else "?",
            {'OK': 'ok', 'WARN': 'WARN', 'BAD': '*BAD*'}[level],
            "{}/{} ({} drones)".format(posed, frames, len(per))
            if frames else "?"))
    for label, level, note in flagged:
        print("  {} {}: starved video view - {}. {}".format(
            "!!" if level == 'BAD' else "!", label, note,
            "Cannot support a full-fleet mosaic." if level == 'BAD'
            else "Fewer views than expected at any instant."))


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

    The stored pose is published verbatim, deliberately: re-deriving it here
    would relatch the GPS origin and put the replay in a different frame from
    the recording. A clip recorded before the gimbal-yaw fix therefore carries
    the bad bearing in these columns and has to be corrected on disk first —
    `migrate_clip_poses.py` does that, and `pose_convention_note` is what stops
    one being replayed unnoticed.
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
                 stop_evt, epoch0, pass_period):
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
        # The pass clock is CLIP-WIDE and shared by every drone thread: one wall
        # instant that maps to the clip's t0, and one period every drone's pass
        # boundary lands on. See run() for why it cannot be per-thread.
        self.epoch0 = epoch0
        self.pass_period = pass_period
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
        """Replay this drone's frames, pass after pass, on the SHARED pass clock.

        Every pass k begins at ``epoch0 + k * pass_period`` for every drone, so a
        frame's wall time is a pure function of the clip clock and never of how
        long this thread's own previous passes happened to take.

        That is the whole fix. This loop used to re-latch ``epoch0 =
        time.monotonic()`` at the top of each pass, which made a pass last that
        drone's OWN recorded span: 6.4503 s for d1, 6.4850 s for d2, 6.4655 s for
        d3 on the MED clips. d2 therefore fell 34.6 ms further behind d1 every
        loop, and after ~7 loops (~47 s) the cross-drone spread in ``capture_time``
        exceeded ``PlanarStitcher.MAX_CAPTURE_SKEW_S`` (0.25 s) and the stitcher
        dropped d2's view as stale -- for most of a ~20 minute beat cycle, and
        with nothing in either process reporting why. A fresh replay looked fine
        for the first 45 s, which is what made it intermittent.

        Drones whose footage is shorter than the clip-wide period simply HOLD
        their last frame at the end of each pass: nothing is published, so the
        block keeps its final image and its final capture_time until the shared
        boundary comes round. On the MED clips that hold is 0.15-0.19 s.

        No barrier and no shared state between the threads: each derives the same
        schedule independently from two constants, so a thread stalled inside
        ``write_memory``'s handshake cannot hold up any other drone, and a late
        pass corrects itself rather than displacing every pass after it.
        """
        mmf, _trailer_ok = imageSharingUtil.open_feed_map(BLOCK_MAP_NAME)
        offset = (self.drone_id - 1) * self.block_bytes
        try:
            pass_index = 0
            while not self.stop_evt.is_set():
                # Opened BEFORE the boundary wait, so the ~33 ms reopen is paid
                # out of the hold rather than out of the next pass's first frame.
                cap = cv2.VideoCapture(self.mp4_path)
                if not cap.isOpened():
                    print("[replay] drone {}: cannot open {}".format(
                        self.drone_id, self.mp4_path), flush=True)
                    return
                pass_start = self.epoch0 + pass_index * self.pass_period
                # Hold the last frame until this pass is due. ~0 on pass 0.
                hold = pass_start - time.monotonic()
                if hold > 0 and self.stop_evt.wait(hold):
                    cap.release()
                    break
                i = 0
                try:
                    while not self.stop_evt.is_set():
                        ok, frame = cap.read()
                        if not ok or i >= len(self.rows):
                            break
                        row = self.rows[i]
                        # Recorded offset from the clip's start, time-scaled.
                        due = ((float(row['t_epoch']) - self.t0) / self.speed)
                        wait = (pass_start + due) - time.monotonic()
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
                        # capture_time: seconds since the REPLAY started, not since
                        # this pass started -- the same "since the producer started"
                        # clock image_stream_feed.py publishes live, so the consumer
                        # cannot tell a replay from a flight.
                        #
                        # It must not reset each pass. At a boundary the first drone
                        # to wrap would read ~0 while the others still held ~6.45,
                        # and _drop_stale would drop the drone that had just been
                        # refreshed -- briefly, but every loop, and for exactly the
                        # views that were most up to date. Monotonic removes the
                        # wrap entirely: the spread across drones is then always the
                        # recorded one (9-72 ms here).
                        #
                        # Unbounded growth is fine at float32: the field is only ever
                        # read as a difference, and its resolution stays under 10 ms
                        # out past 23 hours of replay.
                        capture_time = pass_index * self.pass_period + due
                        imageSharingUtil.write_memory(
                            mmf, offset, self.image_bytes, img,
                            self.drone_id - 1, heading, pace_s=PACE_S,
                            pose=pose, capture_time=capture_time)
                        self.published += 1
                        i += 1
                finally:
                    cap.release()
                if not self.loop:
                    break
                pass_index += 1
        finally:
            try:
                mmf.close()
            except Exception:
                pass


class _StandoffReplay(threading.Thread):
    """Publish the clip's scene-plane standoff into the feed trailer, per frame.

    Recomputed from the stored plane and the replayed poses rather than republished
    as the stored scalar, because the stored scalar is a mean over the whole clip:
    on any clip where the formation translates (grass_nadir_long has the operator
    flying the flock throughout) it is wrong at both ends. `meta.scene_plane` already
    records `plane_n`/`plane_d` in the clip's own latched pose frame — the same frame
    the replayed poses are in — so this is a dot product per frame with no
    re-projection and no origin arithmetic.

    Falls back to the stored scalar when the clip's plane came from
    --set-plane-standoff and so describes no plane anyone can re-measure.

    Paced on the same shared pass clock as the drone threads, so --loop and --speed
    need no handling of their own.
    """

    def __init__(self, track, plane, fallback_m, t0, speed, loop, stop_evt,
                 epoch0, pass_period, period_s=0.1):
        threading.Thread.__init__(self, name="ClipStandoff", daemon=True)
        self.track = track
        self.plane = plane
        self.fallback_m = fallback_m
        self.t0 = t0
        self.speed = speed
        self.loop = loop
        self.stop_evt = stop_evt
        self.epoch0 = epoch0
        self.pass_period = pass_period
        self.period_s = period_s
        self.published = 0
        self.min_m = None
        self.max_m = None

    def _standoff_at(self, t_rel):
        """The clip's standoff at clip-time `t_rel`, from the nearest posed instant."""
        if self.plane is None or not self.track:
            return self.fallback_m
        best = min(self.track, key=lambda it: abs((it[0] - self.t0) - t_rel))
        return scene_plane.standoff_of(self.plane,
                                       scene_plane.centroid_of(best[1]))

    def run(self):
        mmf, trailer_ok = imageSharingUtil.open_feed_map(BLOCK_MAP_NAME)
        if not trailer_ok:
            print("[replay] the feed map predates the scene-plane trailer; "
                  "Unity will use its own planarStandoffMetres.", flush=True)
            mmf.close()
            return
        seq = beat = 0
        try:
            while not self.stop_evt.is_set():
                now = time.monotonic()
                elapsed = now - self.epoch0
                # Where in the clip we are, on the same clock the drones use.
                t_rel = (elapsed % self.pass_period) if self.loop else elapsed
                standoff = self._standoff_at(t_rel * self.speed)
                beat += 1
                seq = imageSharingUtil.write_standoff_trailer(
                    mmf, seq, beat, standoff,
                    imageSharingUtil.FEED_TR_STATUS_LOCKED,
                    facade_id=0, view_count=len(self.track[0][1]) if self.track else 0)
                self.published += 1
                self.min_m = standoff if self.min_m is None else min(self.min_m, standoff)
                self.max_m = standoff if self.max_m is None else max(self.max_m, standoff)
                if self.stop_evt.wait(self.period_s):
                    break
        finally:
            # Retire the plane on the way out, rather than leaving the last value
            # sitting in a section that outlives this process. Unity's heartbeat
            # watchdog would catch it within 2 s anyway; this makes it immediate.
            try:
                imageSharingUtil.write_standoff_trailer(
                    mmf, seq, beat + 1, 0.0,
                    imageSharingUtil.FEED_TR_STATUS_NO_FACADE)
            except Exception:
                pass
            try:
                mmf.close()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(
        description="Replay a recorded clip into DroneFeedSharedMemory as if the "
                    "drones were flying, so the Unity scene and stitcher run "
                    "unchanged.")
    ap.add_argument("--clip",
                    help="Clip label (e.g. grass_nadir_long), folder name, or "
                         "path (e.g. recordings\\clip_20260806_143012). "
                         "--list shows what is on disk.")
    ap.add_argument("--recordings-dir", default=DEFAULT_RECORDINGS,
                    help="Root the labels/folder names are looked up in "
                         "(default: %(default)s). Match the session's "
                         "RecordingDir if it was not the default.")
    ap.add_argument("--list", action="store_true",
                    help="List the clips on disk with their labels and exit")
    ap.add_argument("--set-label", metavar="NAME",
                    help="Write NAME into the --clip clip's session.json as its "
                         "label and exit. Must be unique across the recordings "
                         "root. The only write this tool makes outside shared "
                         "memory.")
    plane = ap.add_argument_group(
        "scene plane (PLANAR)",
        "Measure the standoff PLANAR needs and store it in the clip. Retro-"
        "fittable: the surface only has to be georeferenced now, not at capture "
        "time. See clip_scene_plane.py for what the number is worth.")
    plane.add_argument("--set-plane-from-line", metavar="LAT1,LON1,LAT2,LON2",
                       help="Two points along the BASE of the wall (not the "
                            "roofline — satellite imagery displaces a roof from "
                            "its footprint by height x tan(off-nadir)). Any "
                            "bearing. Computes the standoff and exits.")
    plane.add_argument("--set-plane-from-facade", metavar="ID", nargs="?",
                       const=-1, type=int,
                       help="Use a wall saved by the GUI's 'Pick building' tool "
                            "(shapes.json). The building's real footprint edge, "
                            "so the azimuth is the wall's TRUE bearing — this is "
                            "the easy accurate route, equivalent to typing that "
                            "edge into --set-plane-from-line. ID is optional "
                            "when only one facade is on file.")
    plane.add_argument("--set-plane-from-shape", metavar="ID", nargs="?",
                       const=-1, type=int,
                       help="Use an obstacle RECTANGLE drawn on the GUI map "
                            "(shapes.json). The face nearest the formation wins; "
                            "a rectangle has no rotation, so its azimuth is "
                            "snapped to N/S/E/W — prefer --set-plane-from-facade "
                            "or --set-plane-from-line for a wall on a bearing. "
                            "ID is optional when only one obstacle is on file.")
    plane.add_argument("--set-plane-from-altitude", action="store_true",
                       help="NADIR clips: the surface is the ground, so the "
                            "standoff is the flying height, which the clip "
                            "already knows. Needs no trace and no origin, and "
                            "is re-measured per frame, so a survey that climbs "
                            "is still described. --plane-offset says how far "
                            "the ground sits below the launch pad.")
    plane.add_argument("--set-plane-standoff", metavar="METRES", type=float,
                       help="Store a standoff measured by other means (site "
                            "plan, laser). Nothing checks it against the clip.")
    plane.add_argument("--plane-offset", metavar="METRES", type=float,
                       default=0.0,
                       help="The surface is this far BEYOND the traced line "
                            "(negative: nearer). Use it when the trace is a "
                            "drone hover track flown a metre or two off the "
                            "facade — the aircraft's GPS bias is then "
                            "common-mode with the clip's and cancels, which no "
                            "map can do — or when the filmed surface stands "
                            "proud of a cadastral footprint.")
    plane.add_argument("--shapes", default=DEFAULT_SHAPES_FILE,
                       help="shapes.json --set-plane-from-facade / "
                            "--set-plane-from-shape read (default: %(default)s)")
    plane.add_argument("--check-unity", action="store_true",
                       help="Read the running Unity scene's own published "
                            "settings, print the inspector fields that disagree "
                            "with this clip, and exit. Nothing can be set from "
                            "here — PyUniSharingFast rewrites that mapping every "
                            "frame — so this is a diff, not an apply.")
    plane.add_argument("--no-standoff", action="store_true",
                       help="Don't publish the clip's standoff into the feed map's "
                            "scene-plane trailer. Unity then falls back to whatever "
                            "planarStandoffMetres its inspector holds, which is the "
                            "pre-trailer behaviour — use it to compare a hand-typed "
                            "standoff against the clip's measured one.")
    plane.add_argument("--no-unity-watch", action="store_true",
                       help="Don't report inspector edits made while replaying. "
                            "The watch is read-only and prints only on change; "
                            "it exists so hand-stepping the standoff under "
                            "--loop leaves a record of what was tried.")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="Time scale; 1.0 = real time (default). >1 replays "
                         "faster, but the consumer handshake will throttle it.")
    ap.add_argument("--loop", action="store_true",
                    help="Restart the clip when it ends — what you want while "
                         "tuning the stitcher against fixed footage")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Long-form startup banner: per-drone frame counts, the "
                         "full Unity checklist and the whole scene-plane report "
                         "instead of one line each.")
    ap.add_argument("--pose-lead-s", type=float, default=0.0,
                    help="Re-pair each frame with the pose this many seconds "
                         "later (+) or earlier (-). 0 = faithful replay. The one "
                         "knob for pose/video skew, the dominant mosaic error.")
    args = ap.parse_args()

    root = args.recordings_dir
    if not os.path.isabs(root):
        root = os.path.join(HERE, root)

    if args.list:
        list_clips(root)
        return
    if not args.clip:
        sys.exit("--clip is required (or --list to see what is on disk)")

    clip_dir = resolve_clip(args.clip, root)

    if args.set_label:
        set_label(clip_dir, args.set_label, root)
        return
    if (args.set_plane_from_line or args.set_plane_from_shape is not None
            or args.set_plane_from_facade is not None
            or args.set_plane_standoff is not None
            or args.set_plane_from_altitude):
        set_scene_plane(clip_dir, args)
        return
    if args.speed <= 0:
        sys.exit("--speed must be > 0")

    drones, t0 = load_clip(clip_dir)
    if not drones:
        sys.exit("no replayable drones in {} (need drone{{N}}.mp4 + "
                 "drone{{N}}_frames.csv)".format(clip_dir))

    want = unity_requirements(clip_dir, _session_meta(clip_dir).get('camera') or {},
                              len(drones))
    # A stale pose convention is a not-ready condition exactly like a mis-set
    # inspector field: the scene is correct, the clip is not, and the mosaic
    # fails the same way. So it gates --check-unity's exit code too.
    stale_pose = pose_convention_note(clip_dir)
    if args.check_unity:
        print("{}{}\n".format(os.path.basename(clip_dir),
                              "  [{}]".format(label_of(clip_dir))
                              if label_of(clip_dir) else ""))
        if stale_pose:
            print("  [NOT READY] {}\n".format(stale_pose))
        sys.exit(0 if (check_unity(want) and not stale_pose) else 1)

    # Two forms of the same facts: one line per topic by default, the long form
    # under --verbose. The banner is read on every run, so what earns a line is
    # what changes between runs or would change what you conclude.
    label = label_of(clip_dir)
    span = max(float(r[-1]['t_epoch']) for r in drones.values()) - t0
    posed = sum(1 for rows in drones.values() for r in rows if _pose_of(r))
    total = sum(len(rows) for rows in drones.values())
    skew = max(float(rows[0]['t_epoch']) - t0 for rows in drones.values())
    # One pass clock for the whole clip. The period is the LONGEST drone's span, so
    # no drone's footage is ever cut short; the others hold their last frame to the
    # shared boundary. Every drone is scheduled against the same epoch, which is what
    # stops the cross-drone capture_time skew accumulating under --loop -- see
    # _DroneReplay.run for the failure it replaces.
    pass_period = span / args.speed + LOOP_GAP_S
    holds = {did: pass_period - (float(rows[-1]['t_epoch']) - t0) / args.speed
             for did, rows in drones.items()}
    print("Clip replay -> {} ({} blocks x {}x{})".format(
        BLOCK_MAP_NAME, MAX_DRONES, OUT_W, OUT_H))
    if args.verbose:
        print("  {}{}".format(clip_dir,
                              "  [{}]".format(label) if label else "  (unlabelled)"))
        for did in sorted(drones):
            rows = drones[did]
            n_posed = sum(1 for r in rows if _pose_of(r))
            print("  drone {}: {} frames, {} posed, first t+{:.2f}s".format(
                did, len(rows), n_posed, float(rows[0]['t_epoch']) - t0))
        print("  {:.1f} s of footage, {}/{} frames posed".format(
            span, posed, total))
    else:
        print("  {}{}: {:.1f} s, {} frames {} ({}){}".format(
            os.path.basename(clip_dir),
            " [{}]".format(label) if label else " (unlabelled)",
            span, total,
            "all posed" if posed == total else "{} posed".format(posed),
            " ".join("d{} {}".format(did, len(drones[did]))
                     for did in sorted(drones)),
            ", start skew {:.2f} s".format(skew) if skew > 0.005 else ""))
    if args.speed != 1.0 or args.pose_lead_s:
        print("  {}{}{}".format(
            "speed x{:.2f}".format(args.speed) if args.speed != 1.0 else "",
            "  " if args.speed != 1.0 and args.pose_lead_s else "",
            "POSE LEAD {:+.3f} s — poses re-paired, not a faithful replay".format(
                args.pose_lead_s) if args.pose_lead_s else ""))
    if args.loop:
        print("  loop: {:.3f} s pass, all drones on one clock{}".format(
            pass_period,
            ", holding {}".format(
                " ".join("d{} {:.0f}ms".format(d, holds[d] * 1000)
                         for d in sorted(holds)))
            if args.verbose else ""))
        worst = max(holds.values())
        if worst > STITCHER_MAX_SKEW_S:
            late = [d for d in sorted(holds) if holds[d] > STITCHER_MAX_SKEW_S]
            print("  ! this clip's drones stop up to {:.2f} s apart, so {} hold past "
                  "the stitcher's {:.2f} s staleness\n"
                  "    gate near every loop boundary and PLANAR drops {} view{} "
                  "there. Their footage really has\n"
                  "    ended — this is not the accumulating skew the shared pass "
                  "clock removes.".format(
                      worst, ", ".join("d{}".format(d) for d in late),
                      STITCHER_MAX_SKEW_S, "those" if len(late) > 1 else "that",
                      "s" if len(late) > 1 else ""))
    # Said here as well as in --list because this is the moment it changes what
    # you conclude: a thin seam or a missing panel in the mosaic is the clip's
    # fault, not the stitcher's, and nothing downstream can recover the frames.
    level, note = video_health(_session_meta(clip_dir))
    if level != 'OK':
        print("  {} STARVED VIDEO VIEW: {}. {}".format(
            "!!" if level == 'BAD' else "!", note,
            "This clip cannot support a full-fleet mosaic - expect missing "
            "panels, and do not read them as a stitcher fault."
            if level == 'BAD' else
            "Fewer views than expected will be present at any instant."))
    if posed == 0:
        print("  WARNING no frame carries a camera pose. PLANAR will drop every "
              "view and fall back to the individual feeds; STABSTITCH is fine. "
              "(A clip recorded before the pose columns existed?)")
    if stale_pose:
        # Loud, because the failure it prevents is the one that started all this:
        # a mosaic that will not converge with nothing anywhere saying why.
        print("  !! STALE CAMERA POSES: {}".format(stale_pose))

    # Intrinsics are the sim's to supply — print what the clip was taken with so a
    # mismatch is caught in the inspector instead of showing up as a bent mosaic.
    # The standoff is the one PLANAR input the clip can carry itself, so print the
    # measured number when it has one: a placeholder is what gets typed in wrong,
    # and the plane error is quadratic in standoff.
    cam = _session_meta(clip_dir).get('camera') or {}
    stored_plane = scene_plane.scene_plane_of(clip_dir)
    standoff = ("{:.2f}".format(stored_plane['standoff_m'])
                if stored_plane and stored_plane.get('standoff_m') else None)
    # Whether THIS process will be driving the standoff over the feed trailer. When
    # it is, the inspector's planarStandoffMetres is overridden and comparing against
    # it is meaningless — worse than meaningless, because the [unity] watch below
    # would report our own writes as though an operator had typed them. So the field
    # comes out of the checklist entirely and is reported as ours instead.
    drives_standoff = bool(stored_plane) and not args.no_standoff
    if drives_standoff:
        want.pop("standoff", None)

    if args.verbose:
        print("\nSet these in the Unity scene (this tool cannot — PyUniSharingFast "
              "owns the metadata,\nand planar_inputs_ready() refuses PLANAR without "
              "wire version + intrinsics + a plane):")
        print("  PyUniSharingFast: typeOfStitcher=PLANAR, useManualIntrinsics=true,")
        print("                    manualVerticalFovDeg={}, planarBlendMode=Nearest,"
              .format(cam.get('vfov_deg', 46.4)))
        print("                    scenePlaneMode=FormationRelative{}".format(
            "" if drives_standoff else
            ", planarStandoffMetres={}".format(
                standoff or "<distance to the surface>")))
    else:
        print("  needs: PLANAR, manualVerticalFovDeg {}, blend Nearest, "
              "FormationRelative{}".format(
                  cam.get('vfov_deg', 46.4),
                  "" if drives_standoff else ", standoff {}".format(standoff or "?")))
    if drives_standoff:
        print("  standoff: driven by this replay ({} m, recomputed per frame from "
              "the clip's\n            stored plane). Unity uses it only with "
              "planarStandoffSource = Auto;\n            set Inspector there to "
              "override it, or --no-standoff here to not send it.".format(standoff))
    if standoff is None:
        print("  ! no plane measured for this clip: "
              "--set-plane-from-line LAT1,LON1,LAT2,LON2")
    if stored_plane:
        for line in scene_plane.describe(stored_plane, compact=not args.verbose):
            print("  " + line)

    # Then the same list checked against what the scene is actually publishing,
    # which is the only part of this that can be wrong without anyone noticing:
    # every setting above is an inspector field in another process.
    if args.verbose:
        print("\nAgainst the running scene:")
    check_unity(want, compact=not args.verbose)
    print("Ctrl+C to stop.\n")

    stop_evt = threading.Event()
    epoch0 = time.monotonic()
    workers = [_DroneReplay(did, rows,
                            os.path.join(clip_dir, "drone{}.mp4".format(did)),
                            t0, args.speed, args.loop, args.pose_lead_s, stop_evt,
                            epoch0, pass_period)
               for did, rows in sorted(drones.items())]
    standoff_worker = None
    if stored_plane and not args.no_standoff:
        try:
            track = scene_plane.formation_track(
                scene_plane.load_posed_frames(clip_dir))
        except (OSError, ValueError):
            track = []
        standoff_worker = _StandoffReplay(
            track, scene_plane.plane_from_report(stored_plane),
            stored_plane.get("standoff_m") or 0.0, t0, args.speed, args.loop,
            stop_evt, epoch0, pass_period)
    if not args.no_unity_watch:
        workers_watch = _UnityWatch(want, stop_evt)
        workers_watch.start()
    if standoff_worker is not None:
        standoff_worker.start()
    for w in workers:
        w.start()
    try:
        while any(w.is_alive() for w in workers):
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopping.")
    # Set unconditionally, not just on Ctrl+C: it is also what stops the Unity
    # watch when a non-looping clip simply runs out.
    stop_evt.set()
    for w in workers:
        w.join(timeout=3)
    if standoff_worker is not None:
        standoff_worker.join(timeout=3)

    late = sum(w.late for w in workers)
    unposed = sum(w.unposed for w in workers)
    print("Published: {}{}{}".format(
        " ".join("d{} {}".format(w.drone_id, w.published) for w in workers),
        ", {} unposed".format(unposed) if unposed else "",
        ", {} late >0.5 s (consumer handshake throttling: lower --speed, or "
        "check Unity is reading)".format(late) if late else ""))
    if standoff_worker is not None and standoff_worker.published:
        print("Standoff: {} writes, {:.2f}..{:.2f} m{}".format(
            standoff_worker.published, standoff_worker.min_m, standoff_worker.max_m,
            "" if standoff_worker.plane is not None else
            " (stored scalar — this clip's plane has no normal to re-measure)"))


if __name__ == "__main__":
    main()
