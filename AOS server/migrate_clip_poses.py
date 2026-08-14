"""
Re-solve the camera poses stored in recorded clips, after the gimbal-yaw fix.

    cd "AOS server"
    python migrate_clip_poses.py                # dry run over every clip
    python migrate_clip_poses.py --apply
    python migrate_clip_poses.py --clip MED_facade_stationary_1 --apply

Why this exists
---------------
`clip_recorder.py` stores each frame's *solved* Unity-world camera pose in
`drone{N}_frames.csv` (`pos_*`, `quat_* xyzw`, `pose_status`), and `clip_replay.py`
publishes those columns verbatim so a replay reproduces the live wire exactly.
That was the right design, and it means every clip recorded before the fix has
the bad bearing baked into its quaternions.

`dji_camera_pose` used to build the camera rotation from telemetry `gimbal_yaw`.
On this fleet that field is a world bearing carrying a per-aircraft offset which
is **re-rolled at every takeoff** -- 0 on the ground, tens of degrees airborne,
different every flight (see that module's docstring for the measurements).  The
differential part pointed the cameras up to 18 degrees apart while they were
physically parallel, which at the MED facade's 34 m standoff is 11.3 m of seam
and is why PLANAR would not stitch those clips.  The bearing now comes from the
aircraft compass instead.

Nothing has to be re-flown: the recorder also stored the raw `heading`,
`gimbal_pitch` and `gimbal_roll` columns for exactly this case, so the pose can be
rebuilt offline.  This script does that, through the same
`dji_camera_pose.quat_from_gimbal` the recorder itself calls -- so a migrated clip
is byte-identical to what the fixed recorder would write today, rather than
merely close.

What it touches
---------------
* `quat_x/y/z/w` in every `drone*_frames.csv` row that has `pose_status` set.
  `pos_*` is untouched (position was never affected -- it comes from GPS), and so
  are unposed rows, which carry no pose to correct.
* `meta['pose_yaw_source'] = 'heading'` in the clip's `session.json`.  That marker
  is what makes this idempotent and what `clip_replay.py` checks before replaying:
  never infer "already migrated" from the numbers, because a clip whose slip
  happened to be near zero is indistinguishable from a corrected one.

The original CSV is copied to `drone{N}_frames.csv.pre_yawfix` first, and the
rewrite goes through a temp file and `os.replace`, so an interrupted run cannot
leave a half-written frame index behind.

Pure stdlib plus `dji_camera_pose`: no `ds_wrapper`, no DroneSwarmServer, no
OpenCV, no admin, no drones.  Python 3.7.
"""

import argparse
import csv
import json
import os
import shutil
import sys

from dji_camera_pose import quat_from_gimbal

# clip_replay owns clip discovery and --clip resolution; reuse it rather than
# growing a second set of rules that can disagree about what a clip is called.
import clip_replay

# Written into session.json meta, and what clip_replay checks for. Bump only if
# the pose convention changes again -- an old marker must not read as current.
YAW_SOURCE_MARKER = 'heading'

BACKUP_SUFFIX = '.pre_yawfix'

# Rebuilding the quaternion needs all three; a clip predating the pose columns
# has none of them and cannot be migrated (it also cannot drive PLANAR).
REQUIRED_COLS = ('heading', 'gimbal_pitch', 'gimbal_roll',
                 'quat_x', 'quat_y', 'quat_z', 'quat_w', 'pose_status')


def _wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _fnum(row, key):
    """A float from a CSV cell, or None when it is blank/absent/unparseable."""
    val = row.get(key)
    if val is None or val == '':
        return None
    try:
        out = float(val)
    except (TypeError, ValueError):
        return None
    return out if out == out else None  # NaN fails against itself


def frame_files(clip_dir):
    """`drone{N}_frames.csv` paths in the clip, ordered by drone id."""
    out = []
    try:
        names = os.listdir(clip_dir)
    except OSError:
        return out
    for name in sorted(names):
        if not (name.startswith('drone') and name.endswith('_frames.csv')):
            continue
        try:
            did = int(name[len('drone'):-len('_frames.csv')])
        except ValueError:
            continue
        out.append((did, os.path.join(clip_dir, name)))
    return [p for _, p in sorted(out)]


def migrated(clip_dir):
    """True when this clip's stored poses already use the corrected bearing."""
    return clip_replay._session_meta(clip_dir).get(
        'pose_yaw_source') == YAW_SOURCE_MARKER


def plan_file(path):
    """
    Work out the rewrite for one `drone*_frames.csv` without touching it.

    Returns `(rows, fieldnames, stats)`; `stats['slip']` is the median
    `gimbal_yaw - heading` over the posed rows, which is the correction being
    applied and the number to sanity-check against the flight log.
    """
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        rows = list(reader)

    missing = [c for c in REQUIRED_COLS if c not in fieldnames]
    if missing:
        return rows, fieldnames, {'missing': missing}

    slips, changed, unposed, unusable = [], 0, 0, 0
    for row in rows:
        if not int(_fnum(row, 'pose_status') or 0):
            unposed += 1
            continue
        heading = _fnum(row, 'heading')
        pitch = _fnum(row, 'gimbal_pitch')
        roll = _fnum(row, 'gimbal_roll')
        if heading is None or pitch is None or roll is None:
            # Posed, but the raw angles it was solved from are gone. Leaving the
            # stored quaternion is strictly better than inventing one.
            unusable += 1
            continue

        gyaw = _fnum(row, 'gimbal_yaw')
        if gyaw is not None:
            slips.append(_wrap180(gyaw - heading))

        quat = quat_from_gimbal(heading, pitch, roll)
        before = tuple(_fnum(row, k) for k in
                       ('quat_x', 'quat_y', 'quat_z', 'quat_w'))
        (row['quat_x'], row['quat_y'],
         row['quat_z'], row['quat_w']) = ('%.17g' % v for v in quat)
        if any(b is None or abs(b - a) > 1e-12 for b, a in zip(before, quat)):
            changed += 1

    slips.sort()
    stats = {'rows': len(rows), 'changed': changed, 'unposed': unposed,
             'unusable': unusable,
             'slip': slips[len(slips) // 2] if slips else None}
    return rows, fieldnames, stats


def write_rows(path, rows, fieldnames, backup=True):
    """Rewrite the CSV in place, keeping a one-time backup of the original."""
    if backup:
        bak = path + BACKUP_SUFFIX
        if os.path.exists(bak):
            raise IOError(
                "{} already exists - refusing to overwrite the only copy of the "
                "original poses. Delete it if you are sure.".format(bak))
        shutil.copy2(path, bak)

    tmp = path + '.tmp'
    # newline='' is what keeps csv from doubling the line endings on Windows.
    with open(tmp, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    os.replace(tmp, path)


def stamp_session(clip_dir):
    """Record the pose convention in the clip's session.json, non-destructively."""
    path = os.path.join(clip_dir, 'session.json')
    try:
        with open(path) as f:
            doc = json.load(f)
    except (IOError, OSError, ValueError) as e:
        raise IOError("cannot read {}: {}".format(path, e))
    if not isinstance(doc, dict):
        raise IOError("{} is not a JSON object".format(path))
    meta = doc.get('meta')
    if not isinstance(meta, dict):
        meta = {}
    meta['pose_yaw_source'] = YAW_SOURCE_MARKER
    meta['pose_yaw_migrated'] = (
        'quat_* re-solved from heading/gimbal_pitch/gimbal_roll by '
        'migrate_clip_poses.py; originals in drone*_frames.csv' + BACKUP_SUFFIX)
    doc['meta'] = meta
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2, default=str)


def migrate_clip(clip_dir, apply_changes):
    """Migrate one clip. Returns 'done', 'skipped' or 'failed'."""
    name = os.path.basename(clip_dir)
    label = clip_replay.label_of(clip_dir) or '(unlabelled)'
    head = "{:<32} {}".format(label, name)

    if migrated(clip_dir):
        print("{}\n    already migrated - nothing to do".format(head))
        return 'skipped'

    files = frame_files(clip_dir)
    if not files:
        print("{}\n    no drone*_frames.csv - skipped".format(head))
        return 'skipped'

    planned = []
    for path in files:
        try:
            rows, fieldnames, stats = plan_file(path)
        except (IOError, OSError, ValueError) as e:
            print("{}\n    {}: unreadable ({}) - clip skipped".format(
                head, os.path.basename(path), e))
            return 'failed'
        if stats.get('missing'):
            print("{}\n    {} has no {} column - clip predates the pose columns, "
                  "skipped".format(head, os.path.basename(path),
                                   stats['missing'][0]))
            return 'skipped'
        planned.append((path, rows, fieldnames, stats))

    print(head)
    for path, _rows, _fn, st in planned:
        slip = ("{:+6.2f} deg".format(st['slip']) if st['slip'] is not None
                else "     n/a")
        extra = ""
        if st['unposed']:
            extra += ", {} unposed".format(st['unposed'])
        if st['unusable']:
            extra += ", {} posed but missing raw angles (left as-is)".format(
                st['unusable'])
        print("    {:<20} slip {}   {} of {} rows re-solved{}".format(
            os.path.basename(path), slip, st['changed'], st['rows'], extra))

    if not apply_changes:
        return 'skipped'

    try:
        for path, rows, fieldnames, _st in planned:
            write_rows(path, rows, fieldnames)
        stamp_session(clip_dir)
    except (IOError, OSError) as e:
        print("    FAILED: {}".format(e))
        return 'failed'
    print("    written; originals kept as *{}".format(BACKUP_SUFFIX))
    return 'done'


def main():
    ap = argparse.ArgumentParser(
        description="Re-solve recorded clip camera poses after the gimbal-yaw "
                    "fix. Dry run unless --apply is given.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--clip", default=None,
                    help="One clip, by label, folder name or path. Default: "
                         "every clip under --recordings-dir.")
    ap.add_argument("--recordings-dir", default=clip_replay.DEFAULT_RECORDINGS,
                    help="Root the clips live under. Match a non-default "
                         "RecordingDir.")
    ap.add_argument("--apply", action="store_true",
                    help="Actually rewrite the CSVs. Without it this only "
                         "reports what it would change, which is the sane way "
                         "to check the per-drone slips against the flight log "
                         "first.")
    args = ap.parse_args()

    root = args.recordings_dir
    if not os.path.isabs(root):
        root = os.path.join(clip_replay.HERE, root)

    if args.clip:
        clips = [clip_replay.resolve_clip(args.clip, root)]
    else:
        clips = clip_replay.iter_clips(root)
    if not clips:
        sys.exit("no clips under {}".format(root))

    print("{} clip(s) under {}".format(len(clips), root))
    print("mode: {}\n".format(
        "APPLY - rewriting quat_* in place" if args.apply
        else "DRY RUN - nothing will be written (pass --apply)"))

    tally = {'done': 0, 'skipped': 0, 'failed': 0}
    for clip_dir in clips:
        tally[migrate_clip(clip_dir, args.apply)] += 1
        print("")

    print("{} migrated, {} skipped, {} failed".format(
        tally['done'], tally['skipped'], tally['failed']))
    if not args.apply:
        print("Dry run: re-run with --apply once the slips above look right.")
    return 1 if tally['failed'] else 0


if __name__ == "__main__":
    sys.exit(main())
