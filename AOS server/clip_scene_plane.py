"""
Scene plane for a recorded clip — the one number PLANAR needs and nothing in the
field measures.

Why this exists
---------------
`PLANAR` builds every view's homography about a scene plane. In the Unity sim
that plane is a raycast against a collider; a real facade has none, so the DJI
scene runs `ScenePlaneMode.FormationRelative`, where the stitcher takes the
plane's *normal* from the published camera poses and asks the operator for one
scalar — `planarStandoffMetres`, the perpendicular distance from the formation
to the surface.

A clip does not record that distance. What it does record is every camera's
position in a georeferenced frame (`session.json`'s `pose_origin_latlon` plus
the `pos_*` columns of `drone*_frames.csv`), so as soon as the surface itself
has a lat/lon the standoff is a subtraction — computable *after* the flight,
from a facade traced on the GUI map or typed in from a site plan, with no
obstacle configured at capture time. That is what this module does, and it is
the "georeferenced route" the sim's CLAUDE.md lists as not implemented.

The answer is stored inside the clip (`meta.scene_plane` in its `session.json`),
for the same reason the label is: it belongs to that footage, in that clip's
latched pose frame, and a side file would drift from the folders.

What it cannot do
-----------------
* **The trace must be the BASE of the building, not the roofline.** Satellite
  imagery displaces a roof from its footprint by `height x tan(off-nadir)` —
  about 7 m for a 20 m building, which at these standoffs is a bigger error than
  everything else in the pipeline combined.
* **Obstacle RECTANGLES are axis-aligned lat/lon boxes**, so a facade taken from
  one (`facade_from_obstacle`) gets its azimuth snapped to due N/S/E/W — a
  rectangle has no rotation to report. Two routes avoid that, and both end up in
  `facade_from_line`: pass two arbitrary points, or — the easy one — pick the
  building on the GUI map and click the wall you filmed, which saves the two
  endpoints of that footprint edge as a stored facade (`facade_by_id` /
  `facade_from_stored`). Prefer either over the rectangle. (The geofence polygon
  would also carry an arbitrary bearing, but it is a hard flight boundary that
  ejects drones outside it — a facade does not belong in it.)
* **Absolute GPS is the floor.** The standoff is `formation position - traced
  wall`, and the aircraft's own fix is good to a metre or three however well the
  wall is drawn. `analyse` prints what that costs in mosaic pixels at the clip's
  own geometry, because the cost is quadratic in standoff (`f*B*dZ/Z^2`) and at
  short range it is brutal: metres of error are fine at 60 m and hopeless at 10.

So this gets PLANAR *running* on footage that has no plane, with a number that
is honest about its own error bar. It is not a substitute for a photometric or
triangulated depth, and `analyse` says so when the geometry demands one.

Pure module: no `ds_wrapper`, no shared memory, no numpy, no OpenCV — only the
clip's CSVs and JSON. `python clip_scene_plane.py` runs a self-check.
"""

import csv
import json
import math
import os
from datetime import datetime

from olfati_saber import (gps_to_local, load_facades, load_shapes, rect_to_ne,
                          DEFAULT_SHAPES_FILE)

__all__ = ["facade_from_line", "facade_from_obstacle", "facade_from_stored",
           "offset_facade", "analyse", "store_scene_plane", "scene_plane_of",
           "describe", "manual_report", "load_posed_frames", "formation_track",
           "standoff_of", "measure", "plane_from_report", "seam_geometry",
           "centroid_of", "mean_forward_of", "forward_from_quat",
           "pick_facade", "PICK",
           "formation_centroid_xz", "obstacle_by_id", "facade_by_id",
           "wire_focal_px", "DEFAULT_SHAPES_FILE"]


# The wire resolution PLANAR actually solves at (ImageSharing.cs
# ImageWidth/ImageHeight, and clip_replay/image_stream_feed's OUT_W/OUT_H). The
# seam arithmetic below is in *those* pixels, not the clip's 1920x1080 — quoting
# it at capture resolution would overstate every error by 2.4x.
WIRE_W, WIRE_H = 800, 450

# Nominal Mini 3 Pro wide vfov, used only when a clip predates the camera block.
FALLBACK_VFOV_DEG = 46.4

# The sim's "recognisable mosaic with clean seams" line (vr_swarm_simulation
# CLAUDE.md, PLANAR error budget). Everything reported as a tolerance is the
# plane error that spends exactly this much.
SEAM_TARGET_PX = 5.0

# Rows further apart than this are not the same instant. The fetch loop is 20 Hz
# (50 ms), so this allows a couple of missed frames without pairing across a
# manoeuvre. Only affects the reported spread, never the stored standoff.
PAIR_MAX_DT_S = 0.15

# Mirrors PlanarStitcher.FORMATION_PLANARITY_MAX. Used ONLY to report which rule
# the stitcher will pick for the normal; the stored standoff does not depend on
# it, so a drift here misprints a diagnostic rather than mis-scaling a mosaic.
FORMATION_PLANARITY_MAX = 0.2

# Below this the traced wall and the formation-derived normal disagree enough
# that a single perpendicular scalar cannot describe the surface.
TILT_WARN_DEG = 10.0
# Past this tilt the across-formation depth is a tangent running to infinity;
# report it as unbounded rather than as a huge number. 90 deg exactly is
# reachable (a formation flown at one altitude fits a horizontal plane).
TILT_DEPTH_MAX_DEG = 89.0

# A standoff that moves by more than this fraction of itself during the clip is
# not one number. FormationRelative re-derives the plane from the live centroid
# every frame, so a flock translating *parallel* to the wall is fine; this
# catches the flock that flew towards or away from it.
STANDOFF_SPREAD_WARN = 0.10

# What a hand-traced, GPS-referenced plane can plausibly be worth: satellite
# georegistration plus the aircraft's own absolute fix. Used only to tell the
# operator when the clip's geometry needs better than a trace can give.
TRACE_ACCURACY_M = 2.0


# ---------- small vector / matrix helpers (Unity world: x=East, y=Up, z=North) ----------

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    m = _norm(a)
    if m <= 0.0:
        return None
    return (a[0] / m, a[1] / m, a[2] / m)


def _neg(a):
    return (-a[0], -a[1], -a[2])


def _angle_deg(a, b):
    c = _dot(a, b)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _bearing_deg(v):
    """Compass bearing of a Unity-world vector (0 = North, + = clockwise)."""
    return math.degrees(math.atan2(v[0], v[2]))


def _matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _transpose3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _jacobi_sym3(m, sweeps=12):
    """Eigenvalues and eigenvectors of a symmetric 3x3, ascending by eigenvalue.

    Cyclic Jacobi rather than the analytic closed form: the closed form needs
    special cases for repeated eigenvalues, and repeated eigenvalues are exactly
    what a degenerate formation produces (a single row of drones has two equal
    scatter eigenvalues). Returns `(vals, vecs)` with `vecs[k]` the unit vector
    for `vals[k]`.
    """
    a = [row[:] for row in m]
    v = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(sweeps):
        off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2])
        if off < 1e-18:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(a[p][q]) < 1e-18:
                continue
            theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
            t = (1.0 if theta >= 0.0 else -1.0) / (abs(theta) +
                                                   math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            g = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
            g[p][p] = c
            g[q][q] = c
            g[p][q] = s
            g[q][p] = -s
            a = _matmul3(_matmul3(_transpose3(g), a), g)
            v = _matmul3(v, g)
    vals = [a[i][i] for i in range(3)]
    vecs = [(v[0][i], v[1][i], v[2][i]) for i in range(3)]
    order = sorted(range(3), key=lambda i: vals[i])
    return [vals[i] for i in order], [vecs[i] for i in order]


def _forward_from_quat(q):
    """Unity `Transform.forward` for an (x, y, z, w) rotation: q * (0, 0, 1)."""
    x, y, z, w = q
    return (2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y))


def _to_xz(lat, lon, origin):
    """(lat, lon) -> (east, north) metres in the clip's pose frame.

    Goes through the same `gps_to_local` as `dji_camera_pose.CameraPoseSolver`,
    about the same latched origin, so the wall and the cameras land in one frame
    by construction rather than by agreement.
    """
    north, east = gps_to_local(lat, lon, origin[0], origin[1])
    return east, north


# ---------- facade -> plane ----------
# A facade is a plane {x : n.x = d} with `n` oriented TOWARD the cameras, which
# is the convention PlanarStitcher._plane_from_formation uses for its own normal
# (`d = n.centroid - standoff`). With that orientation the standoff is simply
# `n.centroid - d`, and a negative value means the formation is behind the wall.

def facade_from_line(p1, p2, origin, toward_xz):
    """Vertical plane through two (lat, lon) points, normal facing `toward_xz`.

    Two points and "vertical" is the whole model: a facade's azimuth is what
    matters to the mosaic, its height is irrelevant (the plane is infinite), and
    a wall's tilt from vertical is far below what the standoff error already
    costs. `toward_xz` is an (east, north) point on the side the cameras are on
    — the formation centroid.
    """
    x1, z1 = _to_xz(p1[0], p1[1], origin)
    x2, z2 = _to_xz(p2[0], p2[1], origin)
    u = _unit((x2 - x1, 0.0, z2 - z1))
    if u is None:
        raise ValueError("the two facade points are the same place")
    # Horizontal perpendicular. Sign chosen below, so either branch is fine here.
    n = (u[2], 0.0, -u[0])
    base = (x1, 0.0, z1)
    toward = (toward_xz[0], 0.0, toward_xz[1])
    if _dot(n, _sub(toward, base)) < 0.0:
        n = _neg(n)
    return {"n": n, "d": _dot(n, base), "kind": "line",
            "latlon": [[p1[0], p1[1]], [p2[0], p2[1]]],
            "azimuth_deg": _bearing_deg(u) % 180.0,
            "extent_m": _norm((x2 - x1, 0.0, z2 - z1)),
            "span": ((x1, z1), (x2, z2))}


def facade_from_obstacle(rect, origin, toward_xz):
    """The face of an obstacle rectangle that the cameras are looking at.

    Picks the nearest face the formation is actually outside of, so a rectangle
    drawn over the whole building yields the wall being filmed rather than an
    arbitrary edge. The result's normal is axis-aligned by construction — see
    the module docstring on why that costs a few degrees of azimuth.
    """
    n_min, n_max, e_min, e_max = rect_to_ne(rect, origin[0], origin[1])
    cx, cz = toward_xz
    faces = [
        ((0.0, 0.0, 1.0), n_max, "north", (e_min, e_max), cx),   # wall faces N
        ((0.0, 0.0, -1.0), -n_min, "south", (e_min, e_max), cx),
        ((1.0, 0.0, 0.0), e_max, "east", (n_min, n_max), cz),
        ((-1.0, 0.0, 0.0), -e_min, "west", (n_min, n_max), cz),
    ]
    c = (cx, 0.0, cz)
    outside = [(f, _dot(f[0], c) - f[1]) for f in faces if _dot(f[0], c) - f[1] > 0.0]
    if not outside:
        raise ValueError("the formation centroid is inside obstacle {} — the "
                         "rectangle covers the drones, not the wall"
                         .format(rect.get("id")))
    (n, d, name, span, along), _ = min(outside, key=lambda it: it[1])
    within = span[0] <= along <= span[1]
    return {"n": n, "d": d, "kind": "obstacle",
            "obstacle_id": rect.get("id"),
            "face": name, "face_within_extent": within,
            "extent_m": span[1] - span[0],
            "azimuth_deg": _bearing_deg((n[2], 0.0, -n[0])) % 180.0,
            "rect_ne": [n_min, n_max, e_min, e_max]}


def ground_plane(offset_m=0.0):
    """The horizontal plane a NADIR clip films: y = -offset_m, normal up.

    A ground mosaic has a scene plane like any other, and this one needs no trace
    at all — which is the point. The pose frame's y IS height above the latched
    origin (dji_camera_pose builds it from the aircraft's own takeoff-relative
    altitude), so the standoff is simply the flying height, and `analyse`
    re-measures it per frame like any other plane. That last part is what a
    hand-typed `--set-plane-standoff` cannot do: a survey that climbs 3 m during
    the clip is described by a scalar only at one instant.

    The normal points up, toward the cameras, matching both `standoff_of`'s
    convention and what `formation_normal` returns for drones at one altitude —
    so the tilt reads ~0 instead of the 90 deg that a vertical facade earns from
    this same geometry.

    `offset_m` is how far the imaged ground lies BELOW the launch pad (the pad is
    y = 0). Sloping site, or taking off from a roof: measure the drop and pass it.
    """
    return {"n": (0.0, 1.0, 0.0), "d": -float(offset_m), "kind": "ground",
            "ground_below_pad_m": round(float(offset_m), 3)}


def offset_facade(facade, offset_m):
    """Move the plane `offset_m` metres further from the cameras (negative: nearer).

    The traced line is often not the surface itself. Two cases it exists for, both
    of which beat trying to trace the wall directly on satellite imagery:

    * the trace is a **drone hover track** flown a metre or two off the facade —
      the strongest reference available, because the aircraft's absolute GPS bias
      is common-mode with the clip's and cancels out of the subtraction, leaving
      differential error instead of absolute;
    * the trace is a **cadastral footprint** and the filmed surface stands proud
      of it (ledges, balconies, cladding).

    `n` points toward the cameras and `standoff = n.c - d`, so pushing the plane
    away is a subtraction from `d`.
    """
    if not offset_m:
        return facade
    out = dict(facade)
    out["d"] = facade["d"] - float(offset_m)
    out["offset_m"] = round(float(offset_m), 3)
    return out


def facade_from_stored(rec, origin, toward_xz):
    """The plane for a facade saved by the GUI's "Pick building" tool.

    A stored facade is one EDGE of a real building footprint, so this is
    facade_from_line with the two endpoints already chosen — and that is the
    whole point: the azimuth is the wall's true bearing, where
    facade_from_obstacle can only ever answer 0 or 90 because the rectangle it
    reads has no rotation to report. No new geometry lives here.
    """
    out = facade_from_line(rec["p1"], rec["p2"], origin, toward_xz)
    out["kind"] = "facade"
    out["facade_id"] = rec.get("id")
    for key in ("label", "source", "obstacle_id"):
        if rec.get(key) is not None:
            out[key] = rec[key]
    return out


def facade_by_id(shapes_path, want_id=None):
    """One stored facade from a shapes.json, or None. `want_id=None` and a
    single facade on file is unambiguous; anything else must be named."""
    facades = load_facades(shapes_path)
    if not facades:
        return None, ("no saved facades in {} — pick a building on the GUI map "
                      "and click the wall you filmed".format(shapes_path))
    if want_id is None:
        if len(facades) == 1:
            return facades[0], None
        return None, ("{} facades in {} ({}) — say which one"
                      .format(len(facades), shapes_path,
                              ", ".join("{}={}".format(f["id"],
                                                       f.get("label", "?"))
                                        for f in facades)))
    for fa in facades:
        if fa["id"] == want_id:
            return fa, None
    return None, "no facade with id {} in {}".format(want_id, shapes_path)


def obstacle_by_id(shapes_path, want_id=None):
    """One obstacle dict from a shapes.json, or None. `want_id=None` and a
    single obstacle on file is unambiguous; anything else must be named."""
    obstacles, _fence = load_shapes(shapes_path)
    if not obstacles:
        return None, "no obstacles in {}".format(shapes_path)
    if want_id is None:
        if len(obstacles) == 1:
            return obstacles[0], None
        return None, ("{} obstacles in {} (ids {}) — say which one"
                      .format(len(obstacles), shapes_path,
                              ", ".join(str(o["id"]) for o in obstacles)))
    for ob in obstacles:
        if ob["id"] == want_id:
            return ob, None
    return None, "no obstacle with id {} in {}".format(want_id, shapes_path)


# ---------- the clip's own geometry ----------

def load_posed_frames(clip_dir):
    """`{drone_id: [{t, pos, fwd}, ...]}` for the rows that carry a pose.

    Unposed rows are skipped rather than interpolated: a row without a fix has
    no position, and PLANAR drops that view for the same reason.
    """
    out = {}
    for name in sorted(os.listdir(clip_dir)):
        if not (name.startswith("drone") and name.endswith("_frames.csv")):
            continue
        try:
            did = int(name[len("drone"):-len("_frames.csv")])
        except ValueError:
            continue
        rows = []
        with open(os.path.join(clip_dir, name), newline='') as f:
            for r in csv.DictReader(f):
                try:
                    if not int(r.get('pose_status') or 0):
                        continue
                    rows.append({
                        "t": float(r['t_epoch']),
                        "pos": (float(r['pos_x']), float(r['pos_y']),
                                float(r['pos_z'])),
                        "fwd": _forward_from_quat(
                            (float(r['quat_x']), float(r['quat_y']),
                             float(r['quat_z']), float(r['quat_w']))),
                    })
                except (KeyError, TypeError, ValueError):
                    continue
        if rows:
            out[did] = rows
    return out


def formation_track(per_drone):
    """`[(t, [(drone_id, pos, fwd), ...]), ...]` — the fleet at each instant.

    The drone with the most posed frames provides the clock and every other
    drone contributes its nearest row within `PAIR_MAX_DT_S`. This is a
    reporting stand-in for the freshness pairing the stitcher does on the wire
    (`MAX_CAPTURE_SKEW_S`); it only has to be good enough to measure how much
    the formation moved relative to the wall.
    """
    if not per_drone:
        return []
    clock = max(per_drone, key=lambda k: len(per_drone[k]))
    others = [k for k in sorted(per_drone) if k != clock]
    idx = dict((k, 0) for k in others)
    track = []
    for row in per_drone[clock]:
        t = row["t"]
        views = [(clock, row["pos"], row["fwd"])]
        for k in others:
            rows = per_drone[k]
            i = idx[k]
            while i + 1 < len(rows) and abs(rows[i + 1]["t"] - t) <= abs(rows[i]["t"] - t):
                i += 1
            idx[k] = i
            if abs(rows[i]["t"] - t) <= PAIR_MAX_DT_S:
                views.append((k, rows[i]["pos"], rows[i]["fwd"]))
        track.append((t, views))
    return track


def _centroid(views):
    n = float(len(views))
    return (sum(v[1][0] for v in views) / n,
            sum(v[1][1] for v in views) / n,
            sum(v[1][2] for v in views) / n)


def _mean_forward(views):
    n = float(len(views))
    return _unit((sum(v[2][0] for v in views) / n,
                  sum(v[2][1] for v in views) / n,
                  sum(v[2][2] for v in views) / n))


def _max_baseline(views):
    b = 0.0
    for i in range(len(views)):
        for j in range(i + 1, len(views)):
            b = max(b, _norm(_sub(views[i][1], views[j][1])))
    return b


def formation_centroid_xz(clip_dir):
    """The clip's mean formation centroid as (east, north) metres.

    Which side of the traced wall the cameras are on is what fixes the plane's
    normal, so every facade constructor needs this one point. Averaged over the
    whole clip because it is only used to pick a sign.
    """
    track = formation_track(load_posed_frames(clip_dir))
    if not track:
        raise ValueError(
            "no posed frames in {} — nothing to place the wall relative to"
            .format(os.path.basename(clip_dir)))
    cs = [_centroid(views) for _t, views in track]
    return (sum(c[0] for c in cs) / len(cs), sum(c[2] for c in cs) / len(cs))


def formation_normal(views):
    """`(n, rule)` — the normal PLANAR will derive, oriented toward the cameras.

    Mirrors `PlanarStitcher._plane_from_formation`: a plane fitted to the camera
    *positions* where they span one, the mean camera *forward* where they do not
    (a single row of drones fits every normal perpendicular to the row equally
    well). Reported, never stored — see FORMATION_PLANARITY_MAX.
    """
    f_mean = _mean_forward(views)
    if f_mean is None:
        return None, "none"
    n, rule = None, "forward"
    if len(views) >= 3:
        c = _centroid(views)
        m = [[0.0] * 3 for _ in range(3)]
        for v in views:
            p = _sub(v[1], c)
            for i in range(3):
                for j in range(3):
                    m[i][j] += p[i] * p[j]
        vals, vecs = _jacobi_sym3(m)
        # numpy's descending singular values: s[1] = sqrt(mid), s[2] = sqrt(min).
        s_mid = math.sqrt(max(0.0, vals[1]))
        s_min = math.sqrt(max(0.0, vals[0]))
        if s_mid > 1e-6 and (s_min / s_mid) < FORMATION_PLANARITY_MAX:
            n, rule = vecs[0], "positions"
    if n is None:
        n = _neg(f_mean)
    if _dot(n, f_mean) > 0.0:
        n = _neg(n)
    return _unit(n), rule


# ---------- the measurement, shared by every caller ----------
#
# Live flight, clip replay, the recorder and the offline checker all need the same
# arithmetic. It lives here, once, because the whole point of the exercise is that a
# number measured in one of them is comparable with a number measured in another --
# and two copies of "the standoff" that drift apart would quietly destroy that.

def forward_from_quat(q):
    """Unity `Transform.forward` for an (x, y, z, w) rotation."""
    return _forward_from_quat(q)


def centroid_of(views):
    """Formation centroid as an (east, up, north) triple. `views` is
    `[(drone_id, pos, forward), ...]` -- the shape load_posed_frames builds."""
    return _centroid(views)


def mean_forward_of(views):
    """Unit mean camera forward, or None when the views cancel out."""
    return _mean_forward(views)


def standoff_of(plane, c):
    """The standoff: perpendicular distance from point `c` to `plane`, signed.

    THE measurement this whole path exists to produce, and the reason it is a named
    function rather than an expression repeated at each call site. `plane["n"]` is
    oriented toward the cameras, which is the convention
    `PlanarStitcher._plane_from_formation` uses (`d = n.centroid - standoff`), so a
    positive result is a formation in front of the wall.

    Do NOT mistake this for the ray-plane hit distance `pick_facade` scores on: they
    differ by 1/cos(look_off), which at 30 deg is 15% -- 4.5 m at a 30 m standoff,
    four times the entire plane-error budget.
    """
    return _dot(plane["n"], c) - plane["d"]


def measure(facade, views):
    """One instant's geometry against a facade, or None if there are no views.

    The live and replay counterpart of `analyse`, which does the same thing over a
    whole clip and adds the warnings. Both take the standoff from `standoff_of`, so
    a live reading and a post-hoc one describe the same quantity by construction.
    """
    if not views:
        return None
    c = centroid_of(views)
    f = mean_forward_of(views)
    n_form, rule = formation_normal(views)
    return {
        "standoff_m": standoff_of(facade, c),
        "look_off_deg": (_angle_deg(f, _neg(facade["n"])) if f is not None else None),
        "tilt_deg": (_angle_deg(n_form, facade["n"]) if n_form else None),
        "plane_rule": rule,
        "view_count": len(views),
        "centroid": c,
        "forward": f,
    }


def plane_from_report(report):
    """`{"n", "d"}` from a stored `meta.scene_plane`, or None.

    `analyse` already records `plane_n`/`plane_d` in the clip's own latched pose
    frame, which is exactly the frame the replayed poses are in -- so a replay can
    re-measure per frame with no re-projection and no origin arithmetic at all.
    `manual_report` leaves both None, which is the correct signal to fall back to the
    stored scalar: a hand-typed standoff describes no plane anyone can re-measure.
    """
    if not report:
        return None
    n, d = report.get("plane_n"), report.get("plane_d")
    if not n or d is None or len(n) != 3:
        return None
    return {"n": (float(n[0]), float(n[1]), float(n[2])), "d": float(d)}


def seam_geometry(track, meta, standoff_m):
    """`(baseline, focal_px, px_per_m, tolerance_m)`. Public for the live read-out,
    which wants the seam cost without having a clip to hand."""
    return _seam_geometry(track, meta, standoff_m)


# ---------- which wall is the formation filming? ----------

# Tuning for pick_facade. A module-level dict so the live loop and the offline
# checker share it by import rather than by two copies that drift.
PICK = {
    # A wall nearer than this is not something a 46-degree camera is imaging; one
    # further is a different building that happens to line up.
    "min_standoff_m": 5.0,
    "max_range_m": 150.0,
    # Past this the cameras are not pointed at it in any useful sense, and it is
    # also what rejects a wall traced on the FAR side of the building -- see below.
    "max_look_off_deg": 60.0,
    # A trace shorter than this is a stray map click, not a facade.
    "min_extent_m": 5.0,
    # How far past a wall's ends the sight line may land and still count. Generous
    # on purpose: an 800x450 view at 30 m is ~26 m wide, so a formation a few metres
    # beyond a corner is still filming that wall.
    "lateral_margin_frac": 0.35,
    # A challenger must beat the incumbent by this much, continuously, for this
    # long. Metre- and second-valued so the settings mean what they say, following
    # PyUniSharingFast.SelectPlanarCentreCamera's metre-valued hysteresis.
    "hysteresis_m": 3.0,
    "dwell_s": 2.5,
    # How long an incumbent that has stopped passing its gates is held before the
    # pick is dropped, so one bad telemetry tick does not yank the plane.
    "grace_s": 2.0,
    # Low-pass on the VALUE. The pick itself is never filtered -- see below.
    "standoff_tau_s": 0.5,
}


def _facade_candidate(rec, origin, views, c, f, cfg):
    """Score one stored facade against the formation. Returns a dict, always --
    with `reject` set when it fails a gate, so the checker can say why."""
    out = {"facade_id": rec.get("id"), "label": rec.get("label"),
           "reject": None, "t_m": None, "standoff_m": None,
           "look_off_deg": None, "lateral_frac": None}
    try:
        fac = facade_from_stored(rec, origin, (c[0], c[2]))
    except (ValueError, KeyError, TypeError) as e:
        out["reject"] = "unusable trace ({})".format(e)
        return out
    out["facade"] = fac

    if fac["extent_m"] < cfg["min_extent_m"]:
        out["reject"] = "wall is only {:.1f} m long".format(fac["extent_m"])
        return out

    standoff = standoff_of(fac, c)
    out["standoff_m"] = standoff

    # THE BEHIND-THE-WALL TEST IS THE LOOK-OFF ANGLE, NOT A NEGATIVE STANDOFF.
    # facade_from_line orients the normal toward `toward_xz`, which is the formation
    # centroid -- so a wall traced on the far side of the building comes back with a
    # flipped normal and a perfectly POSITIVE standoff. What actually flips is where
    # the cameras are looking, which is why this gate is the angle. `analyse` raises
    # on the same test past 90 degrees.
    cos_look = None
    if f is not None:
        look_off = _angle_deg(f, _neg(fac["n"]))
        out["look_off_deg"] = look_off
        cos_look = math.cos(math.radians(look_off))
        if look_off > cfg["max_look_off_deg"]:
            out["reject"] = "cameras {:.0f} deg off it".format(look_off)
            return out
    if cos_look is None or cos_look <= 1e-6:
        out["reject"] = "no usable camera forward"
        return out

    if standoff < cfg["min_standoff_m"]:
        out["reject"] = "standoff {:.1f} m < {:.0f} m".format(
            standoff, cfg["min_standoff_m"])
        return out
    if standoff > cfg["max_range_m"]:
        out["reject"] = "standoff {:.0f} m > {:.0f} m".format(
            standoff, cfg["max_range_m"])
        return out

    # Where the formation's sight line actually meets the plane, and whether that
    # lands within the traced wall's extent. This is what makes "nearest hit" mean
    # the right thing for a finite wall -- an infinite plane is always hit.
    t = standoff / cos_look
    hit = (c[0] + f[0] * t, c[1] + f[1] * t, c[2] + f[2] * t)
    (x1, z1), (x2, z2) = fac["span"]
    u = _unit((x2 - x1, 0.0, z2 - z1))
    length = fac["extent_m"]
    s = _dot(_sub(hit, (x1, 0.0, z1)), u)
    margin = cfg["lateral_margin_frac"] * length
    out["t_m"] = t
    out["lateral_frac"] = (s / length) if length > 0 else 0.0
    if s < -margin or s > length + margin:
        out["reject"] = "sight line lands {:.0f} m off the end".format(
            -s if s < 0 else s - length)
        return out
    return out


def pick_facade(facades, origin, views, state=None, now=0.0, cfg=None):
    """Which traced wall is this formation filming? -> `(result, state)`.

    PURE. `now` is an argument and never read from the clock, and every scrap of
    carry-over lives in `state`, which the caller hands back next tick. That is what
    lets the offline checker replay a clip's own timeline and reproduce, exactly, the
    switching the live loop would have done on that footage -- which is the only way
    to validate a rule whose whole job is to not flap.

    `views` is `[(drone_id, pos, forward), ...]` in the LATCHED pose frame (the one
    `CameraPoseSolver.origin` defines and the blocks carry), and `origin` is that
    frame's (lat, lon).

    Score: the NEAREST positive ray-plane hit along the mean camera forward. This is
    the analytic form of what the sim does with a raycast in UpdateScenePlane, so the
    live rule and the sim rule are one rule; and unlike "smallest look-off" or
    "nearest standoff" it gets two parallel walls of one building right, because the
    near one occludes the far one along the actual line of sight.
    """
    cfg = dict(PICK, **(cfg or {}))
    state = dict(state or {})
    prev_id = state.get("facade_id")

    if not facades:
        return ({"facade_id": -1, "state": "none", "candidates": [],
                 "reason": "no facade has been traced"}, {})
    if origin is None:
        return ({"facade_id": -1, "state": "none", "candidates": [],
                 "reason": "no pose origin yet (no GPS fix)"}, state)
    c = centroid_of(views) if views else None
    f = mean_forward_of(views) if views else None
    if c is None or f is None:
        return ({"facade_id": -1, "state": "none", "candidates": [],
                 "reason": "no posed drones"}, state)

    cands = [_facade_candidate(r, origin, views, c, f, cfg) for r in facades]
    ok = [x for x in cands if x["reject"] is None]

    if not ok:
        # Hold the incumbent through a short outage rather than dropping the plane on
        # one bad tick. Same shape as the stitcher's 2 s REASON_PLANE_INVALID grace.
        since = state.get("lost_since")
        if prev_id is not None and since is not None and now - since <= cfg["grace_s"]:
            held = dict(state.get("last_result") or {})
            held["state"] = "grace"
            held["candidates"] = cands
            return held, state
        if prev_id is not None and since is None:
            state["lost_since"] = now
            held = dict(state.get("last_result") or {})
            held["state"] = "grace"
            held["candidates"] = cands
            return held, state
        return ({"facade_id": -1, "state": "none", "candidates": cands,
                 "reason": "; ".join(
                     "{}: {}".format(x["facade_id"], x["reject"]) for x in cands)},
                {})
    state.pop("lost_since", None)

    best = min(ok, key=lambda x: (x["t_m"], x["look_off_deg"] or 0.0,
                                  x["facade_id"]))
    incumbent = next((x for x in ok if x["facade_id"] == prev_id), None)

    chosen, dwelling = best, False
    if incumbent is not None and best["facade_id"] != incumbent["facade_id"]:
        if best["t_m"] < incumbent["t_m"] - cfg["hysteresis_m"]:
            # A real challenger, but it has to hold the claim. A formation yawing
            # across a corner otherwise chatters between two walls, and every switch
            # steps the published plane out from under the stitcher's plane sweep.
            if state.get("challenger_id") != best["facade_id"]:
                state["challenger_id"] = best["facade_id"]
                state["challenger_since"] = now
            if now - state.get("challenger_since", now) < cfg["dwell_s"]:
                chosen, dwelling = incumbent, True
            else:
                state.pop("challenger_id", None)
                state.pop("challenger_since", None)
        else:
            state.pop("challenger_id", None)
            state.pop("challenger_since", None)
            chosen = incumbent
    elif incumbent is not None:
        state.pop("challenger_id", None)
        state.pop("challenger_since", None)

    raw = chosen["standoff_m"]
    prev_val = state.get("standoff_m")
    dt = max(0.0, now - state.get("t", now))
    if (prev_val is None or chosen["facade_id"] != prev_id
            or cfg["standoff_tau_s"] <= 0.0):
        # Snap on a switch, never filter across one: easing from one wall's standoff
        # to another's walks the plane through half a second of distances that
        # describe neither. Same reasoning as the sweep's ACQUIRE snapping.
        value = raw
    else:
        alpha = 1.0 - math.exp(-dt / cfg["standoff_tau_s"]) if dt > 0 else 0.0
        value = prev_val + alpha * (raw - prev_val)

    lo = min(state.get("lo", raw), raw)
    hi = max(state.get("hi", raw), raw)
    if chosen["facade_id"] != prev_id:
        lo = hi = raw

    result = {
        "facade_id": chosen["facade_id"],
        "label": chosen.get("label"),
        "standoff_m": value,
        "raw_standoff_m": raw,
        "look_off_deg": chosen.get("look_off_deg"),
        "tilt_deg": None,
        "lateral_frac": chosen.get("lateral_frac"),
        "t_m": chosen.get("t_m"),
        "spread_m": hi - lo,
        "view_count": len(views),
        "state": "dwelling" if dwelling else "locked",
        "reason": "",
        "candidates": cands,
    }
    n_form, _rule = formation_normal(views)
    if n_form and chosen.get("facade"):
        result["tilt_deg"] = _angle_deg(n_form, chosen["facade"]["n"])

    state.update({"facade_id": chosen["facade_id"], "standoff_m": value,
                  "t": now, "lo": lo, "hi": hi, "last_result": result})
    return result, state


def wire_focal_px(camera):
    """A clip's focal length in WIRE pixels.

    Public because it is also what the Unity scene must be publishing: Unity
    derives `fx` from `manualVerticalFovDeg` at the block resolution, so this is
    the number a live-settings check compares against.
    """
    return _focal_px(camera)


def _focal_px(camera):
    """Focal length in WIRE pixels — what the seam arithmetic is denominated in."""
    try:
        fx = float(camera["fx"])
        width = float(camera["width"])
        if fx > 0.0 and width > 0.0:
            return fx * WIRE_W / width
    except (KeyError, TypeError, ValueError):
        pass
    vfov = FALLBACK_VFOV_DEG
    try:
        vfov = float(camera["vfov_deg"]) or FALLBACK_VFOV_DEG
    except (KeyError, TypeError, ValueError):
        pass
    return (WIRE_H / 2.0) / math.tan(math.radians(vfov) / 2.0)


def _seam_geometry(track, meta, standoff_m):
    """`(baseline, focal_px, px_per_m, tolerance_m)` for this clip and standoff.

    The sim's PLANAR error budget: a plane-distance error `dZ` displaces a view
    by `f*B*dZ/Z^2` pixels — quadratic in standoff, linear in baseline. Worst
    pair, not mean: the seam an operator sees is the one between the two most
    separated views.
    """
    baseline = max((_max_baseline(views) for _t, views in track), default=0.0)
    focal = _focal_px((meta or {}).get('camera') or {})
    px_per_m = (focal * baseline / (standoff_m ** 2)) if standoff_m > 0 else 0.0
    tol_m = (SEAM_TARGET_PX / px_per_m) if px_per_m > 0 else float('inf')
    return baseline, focal, px_per_m, tol_m


def analyse(clip_dir, facade, meta=None):
    """Measure `facade` against the clip's poses. Returns a report dict.

    Raises ValueError when the trace cannot describe this clip at all — the
    formation behind the wall, or pointing away from it. Those are traces of the
    wrong building or the wrong side of the right one, and a standoff computed
    from them would be a plausible-looking number for footage it does not
    describe.
    """
    if meta is None:
        meta = _session_meta(clip_dir)
    per_drone = load_posed_frames(clip_dir)
    track = formation_track(per_drone)
    if not track:
        raise ValueError("no posed frames in {} — nothing to measure against "
                         "(was the clip recorded without --image-stream-pose, "
                         "or with no GPS fix?)".format(os.path.basename(clip_dir)))

    n_f, d_f = facade["n"], facade["d"]
    standoffs, forwards, view_counts = [], [], []
    for _t, views in track:
        c = _centroid(views)
        standoffs.append(standoff_of(facade, c))
        f = _mean_forward(views)
        if f is not None:
            forwards.append(f)
        view_counts.append(len(views))

    mean_standoff = sum(standoffs) / len(standoffs)
    if mean_standoff <= 0.0:
        raise ValueError(
            "the formation is {:.1f} m BEHIND the traced wall — the trace is on "
            "the wrong side of the drones (or is the wrong wall)"
            .format(-mean_standoff))

    # Are the cameras actually looking at it? A facade traced correctly but a
    # clip flown facing elsewhere is a silent scale error, not a visible one.
    f_all = _unit((sum(f[0] for f in forwards), sum(f[1] for f in forwards),
                   sum(f[2] for f in forwards))) if forwards else None
    look_off_deg = None
    if f_all is not None:
        look_off_deg = _angle_deg(f_all, _neg(n_f))
        if look_off_deg > 90.0:
            raise ValueError(
                "the cameras point {:.0f} deg away from the traced wall — they "
                "were not filming it".format(look_off_deg))

    # The rule and the normal the stitcher itself will use, from the fullest frame.
    best = max(track, key=lambda it: len(it[1]))[1]
    n_form, rule = formation_normal(best)
    tilt_deg = _angle_deg(n_form, n_f) if n_form else None

    baseline, focal, px_per_m, tol_m = _seam_geometry(track, meta, mean_standoff)
    # A normal tilted off the wall cannot be fixed by any scalar: across the
    # formation's own span the plane departs from the wall by this much. The
    # tangent runs away at 90 deg, which is a REACHABLE case — drones all at one
    # altitude fit a horizontal plane, perpendicular to any wall normal — so
    # past TILT_DEPTH_MAX_DEG the depth is reported as unbounded (None) instead
    # of as a number with sixteen digits in it.
    if tilt_deg and tilt_deg < TILT_DEPTH_MAX_DEG:
        tilt_depth_m = (baseline / 2.0) * math.tan(math.radians(tilt_deg))
    elif tilt_deg:
        tilt_depth_m = None
    else:
        tilt_depth_m = 0.0

    spread = max(standoffs) - min(standoffs)
    warnings = []
    # Every warning is written claim first, advice second: _first_sentence keeps
    # the claim for the compact printout, so the fact survives the truncation.
    if facade["kind"] == "obstacle":
        warnings.append(
            "this plane's azimuth is due {} ({:.0f} deg), not the wall's true "
            "bearing. Obstacle faces are axis-aligned; trace two points along "
            "the wall for the accurate version.".format(
                facade["face"], _bearing_deg(n_f) % 360.0))
        if not facade.get("face_within_extent", True):
            warnings.append(
                "the formation is off the end of that face. It is looking past "
                "a corner of the rectangle, so the wall it sees may not be this "
                "one.")
    if facade["kind"] == "facade" and str(facade.get("source", "")).startswith("osm:"):
        warnings.append(
            "this wall was traced from OpenStreetMap ({}), which is "
            "crowd-sourced rather than surveyed. The bearing is real (unlike an "
            "obstacle face) but the position is not guaranteed; compare it "
            "against the GUI's Cadastral basemap, and note the tolerance below."
            .format(facade.get("source")))
    if look_off_deg is not None and look_off_deg > 30.0:
        warnings.append(
            "the views are {:.0f} deg oblique to the wall. The mosaic's scale "
            "varies across the canvas accordingly.".format(look_off_deg))
    if tilt_deg is not None and tilt_deg > TILT_WARN_DEG:
        warnings.append(
            "the pose-derived normal ({} rule) is {:.1f} deg off the traced "
            "wall, i.e. {} across the formation's {:.1f} m span. "
            "No standoff can correct a tilt; check the trace's bearing and the "
            "drones' differential GPS."
            .format(rule, tilt_deg,
                    "{:.2f} m of depth".format(tilt_depth_m)
                    if tilt_depth_m is not None else "unbounded depth",
                    baseline))
        if tilt_depth_m is None and rule == "positions":
            warnings.append(
                "that normal is perpendicular to the wall, which is what a "
                "formation flown at ONE altitude gives — its cameras fit a "
                "horizontal plane, so their positions say nothing about a "
                "vertical wall. Stagger the drones in height (vertical-plane "
                "mode does this) or PLANAR has no formation normal to use.")
    if spread > STANDOFF_SPREAD_WARN * mean_standoff:
        warnings.append(
            "the standoff moved {:.2f} m during the clip ({:.0f}% of it), so one "
            "scalar cannot describe this footage. Split the clip, or replay the "
            "stationary part.".format(spread, 100.0 * spread / mean_standoff))
    # A ground plane has no trace, so the trace-accuracy note below would be
    # wrong in kind rather than merely pessimistic. What this standoff is worth
    # instead is whatever the aircraft's own height above its pad is worth, plus
    # the site's slope — both site facts, so the honest thing is to name them and
    # quote the clip's own tolerance rather than assert a number for either.
    if facade["kind"] == "ground":
        warnings.append(
            "this standoff is the aircraft's altitude above its launch pad, so "
            "it carries no map-trace error — what is left is altitude drift and "
            "any drop between the pad and the ground being imaged. {:.0f} px of "
            "seam needs it within {:.2f} m; if the site is not level, measure "
            "the drop and pass it as --plane-offset."
            .format(SEAM_TARGET_PX, tol_m))
    # Graded, not a threshold: what matters is how many pixels the trace's own
    # error is worth at THIS standoff, and the same 2 m is a rounding error at
    # 60 m and a wrecked mosaic at 12. Quoting the px directly also keeps the
    # note honest when the geometry is good.
    trace_px = px_per_m * TRACE_ACCURACY_M
    if facade["kind"] != "ground" and trace_px >= SEAM_TARGET_PX:
        note = ("a trace is worth ~{:.0f} m of standoff once DJI's absolute GPS "
                "is included, so ~{:.0f} px of seam here. {:.0f} px would need "
                "{:.2f} m."
                .format(TRACE_ACCURACY_M, trace_px, SEAM_TARGET_PX, tol_m))
        if trace_px > 4.0 * SEAM_TARGET_PX:
            note += (" Expect visible seams. Plane error is quadratic in "
                     "standoff and linear in baseline, so standing further back "
                     "or tightening the formation buys more than a better "
                     "trace; failing that, step the standoff by hand under "
                     "--loop until the seams close.")
        else:
            note += (" Tuning the standoff by hand under --loop is what closes "
                     "the last of it.")
        warnings.append(note)

    return {
        "mode": "FormationRelative",
        "standoff_m": round(mean_standoff, 3),
        "standoff_min_m": round(min(standoffs), 3),
        "standoff_max_m": round(max(standoffs), 3),
        "samples": len(standoffs),
        "views_max": max(view_counts) if view_counts else 0,
        "baseline_m": round(baseline, 3),
        "plane_rule": rule,
        "tilt_deg": None if tilt_deg is None else round(tilt_deg, 2),
        # None = unbounded (the normal is perpendicular to the wall).
        "tilt_depth_m": None if tilt_depth_m is None else round(tilt_depth_m, 3),
        "look_off_deg": None if look_off_deg is None else round(look_off_deg, 2),
        "normal_bearing_deg": round(_bearing_deg(n_f) % 360.0, 2),
        "focal_px_wire": round(focal, 1),
        "px_per_m": round(px_per_m, 1),
        "tolerance_m": round(tol_m, 3),
        "facade": dict((k, v) for k, v in facade.items()
                       if k not in ("n", "d", "span")),
        "plane_n": [round(v, 6) for v in n_f],
        "plane_d": round(d_f, 4),
        "origin_latlon": list((meta or {}).get('pose_origin_latlon') or []),
        "warnings": warnings,
    }


def _first_sentence(text):
    """The claim without the advice — every warning below is written claim-first
    so a compact printout can keep the fact and drop the paragraph."""
    head = text.split(". ")[0].strip()
    return head if head.endswith(".") else head + "."


def describe(report, compact=False):
    """The report as printable lines — shared by the CLI and the replay banner.

    Leads with the standoff and its spread rather than restating the setting
    name: the banner has already printed the value in the Unity checklist, and
    what is worth reading twice is how well the one scalar fits the clip.

    `compact` is the replay banner's form: same numbers on two lines, warnings
    reduced to their first sentence. Nothing is lost — the full text is in the
    clip's `session.json` and behind the replayer's `--verbose`.
    """
    r = report
    if compact:
        head = "plane: {:.2f} m".format(r["standoff_m"])
        if r.get("samples"):
            head += " [{:.2f}-{:.2f}] brg {:.0f}".format(
                r["standoff_min_m"], r["standoff_max_m"], r["normal_bearing_deg"])
        if r.get("tilt_deg") is not None:
            head += ", poses {:.1f} off".format(r["tilt_deg"])
        if r.get("px_per_m"):
            head += ", {:.0f} px/m ({:.0f} px needs {:.2f} m)".format(
                r["px_per_m"], SEAM_TARGET_PX, r["tolerance_m"])
        lines = [head, "  from " + _facade_str(r["facade"])]
        for w in r.get("warnings") or []:
            lines.append("  ! " + _first_sentence(w))
        return lines
    lines = ["Scene plane: {:.2f} m standoff from {}".format(
        r["standoff_m"], _facade_str(r["facade"]))]
    if r.get("samples"):
        lines.append("  {:.2f} .. {:.2f} m over {} frames of up to {} views; "
                     "wall normal bearing {:.0f} deg".format(
                         r["standoff_min_m"], r["standoff_max_m"], r["samples"],
                         r["views_max"], r["normal_bearing_deg"]))
    if r.get("tilt_deg") is not None:
        lines.append("  pose-derived normal ({} rule) sits {:.1f} deg off that "
                     "wall".format(r["plane_rule"], r["tilt_deg"]))
    if r.get("px_per_m"):
        lines.append(
            "  seam cost {:.0f} px per metre of plane error (f={:.0f} px at "
            "{}x{}, baseline {:.1f} m): {:.0f} px needs it within {:.2f} m"
            .format(r["px_per_m"], r["focal_px_wire"], WIRE_W, WIRE_H,
                    r["baseline_m"], SEAM_TARGET_PX, r["tolerance_m"]))
    for w in r.get("warnings") or []:
        lines.append("  ! " + w)
    return lines


def _facade_str(facade):
    off = facade.get("offset_m")
    tail = "" if not off else " {:+.2f} m offset".format(off)
    if facade.get("kind") == "obstacle":
        return "obstacle {} {} face{}".format(facade.get("obstacle_id"),
                                             facade.get("face"), tail)
    if facade.get("kind") == "manual":
        return "standoff given by hand"
    if facade.get("kind") == "ground":
        drop = facade.get("ground_below_pad_m")
        return "the ground, from flying altitude{}".format(
            "" if not drop else " ({:+.2f} m below the pad)".format(drop))
    ll = facade.get("latlon") or []
    if len(ll) == 2:
        return "traced line {:.6f},{:.6f} -> {:.6f},{:.6f}{}".format(
            ll[0][0], ll[0][1], ll[1][0], ll[1][1], tail)
    return facade.get("kind", "?") + tail


# ---------- session.json ----------

def _session_meta(clip_dir):
    try:
        with open(os.path.join(clip_dir, "session.json")) as f:
            return json.load(f).get('meta') or {}
    except (IOError, OSError, ValueError, AttributeError):
        return {}


def scene_plane_of(clip_dir):
    """The clip's stored scene plane, or None."""
    sp = _session_meta(clip_dir).get('scene_plane')
    return sp if isinstance(sp, dict) else None


def store_scene_plane(clip_dir, report):
    """Write `report` into the clip's `session.json` as `meta.scene_plane`.

    Overwrites any previous one: the standoff is a measurement of this clip, and
    keeping a history of guesses in the clip that PLANAR reads from is a way to
    replay the wrong one.
    """
    path = os.path.join(clip_dir, "session.json")
    with open(path) as f:
        doc = json.load(f)
    if not isinstance(doc, dict):
        raise ValueError("{} is not a JSON object".format(path))
    meta = doc.get('meta')
    if not isinstance(meta, dict):
        meta = {}
    stored = dict(report)
    stored["computed_iso"] = datetime.now().isoformat(timespec='seconds')
    meta['scene_plane'] = stored
    doc['meta'] = meta
    with open(path, 'w') as f:
        json.dump(doc, f, indent=2)
    return stored


def manual_report(standoff_m, clip_dir=None, meta=None, note=None):
    """A report for a standoff measured by other means (laser, site plan).

    Carries no trace diagnostics — there is no trace — but still reports the
    clip's seam sensitivity when given the clip: how much a metre of plane error
    costs is a property of the formation and the standoff, not of where the
    number came from, and it is the thing that says whether the measurement was
    good enough.
    """
    if not (standoff_m > 0.0):
        raise ValueError("standoff must be > 0 m")
    standoff_m = float(standoff_m)
    baseline = focal = px_per_m = 0.0
    tol_m = 0.0
    if clip_dir is not None:
        if meta is None:
            meta = _session_meta(clip_dir)
        track = formation_track(load_posed_frames(clip_dir))
        if track:
            baseline, focal, px_per_m, tol_m = _seam_geometry(
                track, meta, standoff_m)
    return {
        "mode": "FormationRelative",
        "standoff_m": round(standoff_m, 3),
        "standoff_min_m": round(standoff_m, 3),
        "standoff_max_m": round(standoff_m, 3),
        "samples": 0,
        "views_max": 0,
        "baseline_m": round(baseline, 3),
        "plane_rule": "n/a",
        "tilt_deg": None,
        "tilt_depth_m": 0.0,
        "look_off_deg": None,
        "normal_bearing_deg": 0.0,
        "focal_px_wire": round(focal, 1),
        "px_per_m": round(px_per_m, 1),
        "tolerance_m": round(tol_m, 3),
        "facade": {"kind": "manual", "note": note or ""},
        "plane_n": None,
        "plane_d": None,
        "origin_latlon": [],
        "warnings": ["standoff given by hand, not measured against the clip's "
                     "poses — nothing here checks it against the footage"],
    }


# ---------- self-check ----------

def _selftest():
    """Synthetic checks: no clip, no Unity, no drones.

    Builds a wall at a known distance from known camera positions and asserts
    the standoff comes back, then the refusals that matter (wrong side, facing
    away) and the obstacle face choice.
    """
    ok = [True]

    def check(name, cond, detail=""):
        print("  {:<44} {}{}".format(name, "PASS" if cond else "FAIL",
                                     "" if cond else "  <- " + str(detail)))
        if not cond:
            ok[0] = False

    origin = (46.5189342, 6.5669576)
    # A wall running due east-west, 12 m north of the origin, and three cameras
    # in a vertical L 2 m north of the origin looking north at it.
    dlat = lambda m: m / 111320.0
    dlon = lambda m: m / (111320.0 * math.cos(math.radians(origin[0])))
    wall = ((origin[0] + dlat(12.0), origin[1] - dlon(30.0)),
            (origin[0] + dlat(12.0), origin[1] + dlon(30.0)))
    cams = [(-5.0, 16.0, 2.0), (5.0, 16.0, 2.0), (0.0, 6.0, 2.0)]
    centroid_xz = (sum(c[0] for c in cams) / 3.0, sum(c[2] for c in cams) / 3.0)

    f = facade_from_line(wall[0], wall[1], origin, centroid_xz)
    standoff = _dot(f["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - f["d"]
    check("line facade standoff = 10 m", abs(standoff - 10.0) < 0.01, standoff)
    check("line facade normal faces the cameras (bearing 180)",
          abs((_bearing_deg(f["n"]) % 360.0) - 180.0) < 0.5,
          _bearing_deg(f["n"]) % 360.0)
    check("line facade azimuth is due east-west",
          abs(f["azimuth_deg"] - 90.0) < 0.5, f["azimuth_deg"])

    # A hover track traced 2 m off the wall: the surface is 2 m further out.
    f_off = offset_facade(f, 2.0)
    so_off = _dot(f_off["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - f_off["d"]
    check("offset pushes the plane away from the cameras",
          abs(so_off - 12.0) < 0.01, so_off)
    check("zero offset is the identity", offset_facade(f, 0.0)["d"] == f["d"])

    # Same wall traced in the opposite direction must give the same plane: the
    # normal is chosen by which side the cameras are on, not by point order.
    f_rev = facade_from_line(wall[1], wall[0], origin, centroid_xz)
    check("trace direction does not change the plane",
          abs(f_rev["d"] - f["d"]) < 1e-6 and
          _angle_deg(f_rev["n"], f["n"]) < 1e-6)

    # The formation normal: an L of three cameras spans a vertical plane facing
    # south, so the positions rule must win and agree with the wall.
    views = [(i + 1, p, (0.0, 0.0, 1.0)) for i, p in enumerate(cams)]
    n_form, rule = formation_normal(views)
    check("formation normal uses the positions rule", rule == "positions", rule)
    check("formation normal matches the wall", _angle_deg(n_form, f["n"]) < 0.5,
          _angle_deg(n_form, f["n"]))

    # A single row of cameras fits no plane; the forward rule must take over.
    row = [(1, (-5.0, 16.0, 2.0), (0.0, 0.0, 1.0)),
           (2, (0.0, 16.0, 2.0), (0.0, 0.0, 1.0)),
           (3, (5.0, 16.0, 2.0), (0.0, 0.0, 1.0))]
    _n_row, rule_row = formation_normal(row)
    check("collinear formation falls back to forward", rule_row == "forward",
          rule_row)

    # Obstacle: a rectangle covering the building north of the cameras. The face
    # they see is its south face, 10 m away.
    rect = {"id": 7,
            "lat_min": origin[0] + dlat(12.0), "lat_max": origin[0] + dlat(40.0),
            "lon_min": origin[1] - dlon(30.0), "lon_max": origin[1] + dlon(30.0)}
    fo = facade_from_obstacle(rect, origin, centroid_xz)
    so = _dot(fo["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - fo["d"]
    check("obstacle picks the south face", fo["face"] == "south", fo["face"])
    check("obstacle standoff = 10 m", abs(so - 10.0) < 0.02, so)
    check("obstacle face covers the formation", fo["face_within_extent"])

    # Off the end of the rectangle: still the south face, but flagged.
    fo_off = facade_from_obstacle(rect, origin, (200.0, centroid_xz[1]))
    check("off-the-end formation is flagged",
          not fo_off["face_within_extent"])

    # A wall on a REAL bearing — the whole reason the stored-facade route
    # exists. Build one at 108 deg, exactly 10 m from the cameras, then read it
    # back both ways: the footprint edge reports the truth, and the obstacle
    # rectangle enclosing that same edge cannot.
    az = 108.0
    u_e, u_n = (math.sin(math.radians(az)), math.cos(math.radians(az)))
    # Outward normal (wall -> cameras) is the wall bearing + 90 deg, negated.
    nb = math.radians(az + 90.0)
    base_e = centroid_xz[0] - 10.0 * math.sin(nb)
    base_n = centroid_xz[1] - 10.0 * math.cos(nb)
    ends = [(base_e - 20.0 * u_e, base_n - 20.0 * u_n),
            (base_e + 20.0 * u_e, base_n + 20.0 * u_n)]
    stored = {"id": 3, "label": "ME D south",
              "source": "osm:way/407504253#edge3", "obstacle_id": 2,
              "p1": [origin[0] + dlat(ends[0][1]), origin[1] + dlon(ends[0][0])],
              "p2": [origin[0] + dlat(ends[1][1]), origin[1] + dlon(ends[1][0])]}

    fs = facade_from_stored(stored, origin, centroid_xz)
    so_s = _dot(fs["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - fs["d"]
    check("stored facade standoff = 10 m", abs(so_s - 10.0) < 0.05, so_s)
    check("stored facade azimuth is the WALL'S TRUE bearing (108 deg)",
          abs(fs["azimuth_deg"] - az) < 0.5, fs["azimuth_deg"])
    check("stored facade keeps its identity",
          fs["kind"] == "facade" and fs["facade_id"] == 3
          and fs["label"] == "ME D south" and fs["obstacle_id"] == 2, fs)

    # The same wall as an axis-aligned obstacle: this is what the operator got
    # before, and it is wrong twice over — snapped azimuth AND a face that is
    # the bounding box's edge rather than the wall.
    lats = [stored["p1"][0], stored["p2"][0]]
    lons = [stored["p1"][1], stored["p2"][1]]
    bbox = {"id": 8, "lat_min": min(lats), "lat_max": max(lats) + dlat(20.0),
            "lon_min": min(lons), "lon_max": max(lons)}
    fb2 = facade_from_obstacle(bbox, origin, centroid_xz)
    check("obstacle route snaps that same wall to a compass axis",
          abs(fb2["azimuth_deg"] % 90.0) < 0.5
          and abs(fb2["azimuth_deg"] - az) > 15.0, fb2["azimuth_deg"])
    so_b = _dot(fb2["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - fb2["d"]
    check("obstacle route also mis-reads the standoff", abs(so_b - 10.0) > 1.0,
          so_b)

    # offset_facade must work on a stored facade exactly as on a traced line —
    # the cadastral-footprint / hover-track correction is the same subtraction.
    fs_off = offset_facade(fs, 1.5)
    so_fo = _dot(fs_off["n"], (centroid_xz[0], 0.0, centroid_xz[1])) - fs_off["d"]
    check("stored facade honours --plane-offset", abs(so_fo - 11.5) < 0.05,
          so_fo)

    # Refusals. A wall traced south of the cameras is the wrong side.
    behind = ((origin[0] - dlat(5.0), origin[1] - dlon(30.0)),
              (origin[0] - dlat(5.0), origin[1] + dlon(30.0)))
    fb = facade_from_line(behind[0], behind[1], origin, centroid_xz)
    check("wall behind the cameras gives a normal pointing north",
          abs((_bearing_deg(fb["n"]) % 360.0)) < 0.5,
          _bearing_deg(fb["n"]) % 360.0)

    # Seam arithmetic: the sim's own worked example, 3 m of plane error at 15 m
    # standoff with an 8 m baseline and f = 525 px, is quoted as ~56 px.
    px = 525.0 * 8.0 / (15.0 ** 2) * 3.0
    check("seam formula reproduces the sim's 56 px example", abs(px - 56.0) < 2.0,
          px)

    # Jacobi against a plane with a known normal and unequal spread.
    m = [[4.0, 0.0, 0.0], [0.0, 9.0, 0.0], [0.0, 0.0, 0.0]]
    vals, vecs = _jacobi_sym3(m)
    check("jacobi orders eigenvalues ascending", vals[0] < vals[1] < vals[2],
          vals)
    check("jacobi smallest eigenvector is the plane normal",
          abs(abs(vecs[0][2]) - 1.0) < 1e-9, vecs[0])

    # ---------- pick_facade ----------
    # Two walls of one building: a near one due east-west 12 m north, and a far one
    # 40 m north. Cameras face north, so the near one must win on the sight line.
    print()
    north = lambda m_: [[origin[0] + dlat(m_), origin[1] - dlon(30.0)],
                        [origin[0] + dlat(m_), origin[1] + dlon(30.0)]]
    near_rec = {"id": 1, "label": "near", "p1": north(12.0)[0], "p2": north(12.0)[1]}
    far_rec = {"id": 2, "label": "far", "p1": north(40.0)[0], "p2": north(40.0)[1]}
    look_n = (0.0, 0.0, 1.0)
    views_n = [(i, c, look_n) for i, c in enumerate(cams)]

    res, st = pick_facade([near_rec, far_rec], origin, views_n, None, 0.0)
    check("pick takes the nearer of two parallel walls", res["facade_id"] == 1,
          res.get("facade_id"))
    check("pick reports that wall's standoff, not its hit distance",
          abs(res["standoff_m"] - 10.0) < 0.05, res.get("standoff_m"))
    check("pick reports locked", res["state"] == "locked", res.get("state"))

    # Facing SOUTH at the same two walls: both are now behind the cameras. The
    # standoff is still positive (facade_from_line orients toward the formation),
    # so only the look-off gate can catch this -- the point of that gate.
    views_s = [(i, c, (0.0, 0.0, -1.0)) for i, c in enumerate(cams)]
    res_s, _ = pick_facade([near_rec, far_rec], origin, views_s, None, 0.0)
    check("cameras facing away pick nothing", res_s["facade_id"] == -1,
          res_s.get("facade_id"))
    check("...and the standoff was positive, so look-off is what caught it",
          all(x["standoff_m"] > 0 for x in res_s["candidates"]),
          [x.get("standoff_m") for x in res_s["candidates"]])

    # A wall 200 m away is out of range; a 2 m scrap of trace is not a facade.
    res_r, _ = pick_facade([{"id": 3, "label": "distant",
                             "p1": north(200.0)[0], "p2": north(200.0)[1]}],
                           origin, views_n, None, 0.0)
    check("out-of-range wall is rejected", res_r["facade_id"] == -1,
          res_r["candidates"][0].get("reject"))

    # Lateral extent: a wall whose traced span is far to the east of the sight line.
    east_rec = {"id": 4, "label": "off to one side",
                "p1": [origin[0] + dlat(12.0), origin[1] + dlon(200.0)],
                "p2": [origin[0] + dlat(12.0), origin[1] + dlon(260.0)]}
    res_l, _ = pick_facade([east_rec], origin, views_n, None, 0.0)
    check("sight line off the end of the wall is rejected",
          res_l["facade_id"] == -1, res_l["candidates"][0].get("reject"))

    # Hysteresis + dwell. Put the far wall 2.0 m nearer than the incumbent -- inside
    # the 3 m hysteresis -- and it must NOT take over however long it holds.
    close_rec = {"id": 2, "label": "barely nearer",
                 "p1": north(10.0)[0], "p2": north(10.0)[1]}
    st2 = {"facade_id": 1}
    res_h, st2 = pick_facade([near_rec, close_rec], origin, views_n, st2, 0.0)
    check("a challenger inside the hysteresis never switches",
          res_h["facade_id"] == 1, res_h.get("facade_id"))

    # Now clearly nearer: a wall 8 m north is a 6 m standoff, 4 m better than the
    # incumbent's 10 and so past the 3 m hysteresis. It must still serve the dwell
    # before switching -- this is what stops a formation yawing across a corner from
    # chattering, and every switch steps the plane out from under the stitcher's
    # sweep. (Not 4 m north: that is a 2 m standoff, which the min_standoff gate
    # rejects outright, and the dwell would never come into it.)
    nearer_rec = {"id": 2, "label": "clearly nearer",
                  "p1": north(8.0)[0], "p2": north(8.0)[1]}
    st3 = {"facade_id": 1}
    seen = []
    for k in range(9):
        t = k * 0.5
        r, st3 = pick_facade([near_rec, nearer_rec], origin, views_n, st3, t)
        seen.append((t, r["facade_id"], r["state"]))
    switched_at = next((t for t, fid, _s in seen if fid == 2), None)
    check("a real challenger waits out the dwell",
          switched_at is not None and switched_at >= PICK["dwell_s"] - 1e-9,
          seen)
    check("...and says it is dwelling while it waits",
          any(s == "dwelling" for _t, _f, s in seen), seen)

    # A switch must SNAP the value, never ease across it: filtering from one wall's
    # standoff to another's walks the plane through distances describing neither.
    # State set up mid-dwell so this call is the switching one.
    st_sw = {"facade_id": 1, "standoff_m": 10.0, "t": 0.0,
             "challenger_id": 2, "challenger_since": 0.0}
    after, _ = pick_facade([near_rec, nearer_rec], origin, views_n, st_sw, 10.0)
    check("the switch this sets up actually happened", after["facade_id"] == 2,
          after.get("facade_id"))
    check("the standoff snaps on a switch, not eases",
          abs(after["standoff_m"] - after["raw_standoff_m"]) < 1e-6,
          (after["standoff_m"], after["raw_standoff_m"]))

    # Grace: the incumbent survives a tick with no usable candidate.
    st4 = {"facade_id": 1}
    pick_facade([near_rec], origin, views_n, st4, 0.0)
    r_lost, st4b = pick_facade([near_rec], origin, views_s, st4, 0.5)
    check("a one-tick outage holds the incumbent", r_lost["state"] == "grace",
          r_lost.get("state"))

    # Purity: the same inputs and the same clock give the same answer, so the
    # offline checker's replay of a clip is the live loop's behaviour.
    a, _ = pick_facade([near_rec, far_rec], origin, views_n, {"facade_id": 1}, 3.0)
    b, _ = pick_facade([near_rec, far_rec], origin, views_n, {"facade_id": 1}, 3.0)
    check("pick_facade is pure (same in, same out)",
          a["facade_id"] == b["facade_id"]
          and abs(a["standoff_m"] - b["standoff_m"]) < 1e-12)

    print("\n{}".format("SELF-CHECK PASSED" if ok[0] else "SELF-CHECK FAILED"))
    return 0 if ok[0] else 1


def explain_pick(clip_dir, shapes_path, verbose=False, cfg=None):
    """Replay `pick_facade` over a recorded clip and print what it chose, when.

    The offline half of "which wall did it decide it was filming?". Because
    `pick_facade` is pure and takes its clock as an argument, walking a clip's own
    `t_epoch` timeline here reproduces exactly the switching the live loop would have
    done on that footage -- so a rule whose entire job is to not flap can be checked
    against real flights instead of against a hope.

    Returns 0 when it settled on one facade for the whole clip.
    """
    facades = load_facades(shapes_path)
    if not facades:
        print("no facades in {} — pick a building on the GUI map and click the "
              "wall you filmed".format(shapes_path))
        return 1
    meta = _session_meta(clip_dir)
    origin = meta.get('pose_origin_latlon')
    if not (isinstance(origin, (list, tuple)) and len(origin) == 2):
        print("this clip has no pose_origin_latlon, so its poses cannot be "
              "georeferenced and no facade can be measured against them")
        return 1
    track = formation_track(load_posed_frames(clip_dir))
    if not track:
        print("no posed frames in {}".format(os.path.basename(clip_dir)))
        return 1

    t0 = track[0][0]
    state, rows, switches = None, [], []
    for t, views in track:
        res, state = pick_facade(facades, tuple(origin), views, state, t - t0, cfg)
        rows.append((t - t0, res))
        if not rows[:-1] or rows[-2][1].get("facade_id") != res.get("facade_id"):
            switches.append((t - t0, res))

    print("{} facades on file, {} posed instants over {:.1f} s"
          .format(len(facades), len(track), track[-1][0] - t0))
    held = {}
    for i, (t, res) in enumerate(rows):
        dt = (rows[i + 1][0] - t) if i + 1 < len(rows) else 0.0
        held[res.get("facade_id")] = held.get(res.get("facade_id"), 0.0) + dt
    total = sum(held.values()) or 1.0
    for fid, secs in sorted(held.items(), key=lambda kv: -kv[1]):
        name = next((f.get("label") for f in facades if f.get("id") == fid), None)
        print("  facade {:<4} {:>5.1f}% of the clip  {}".format(
            "-" if fid == -1 else fid, 100.0 * secs / total,
            "(nothing picked)" if fid == -1 else (name or "")))
    print("  {} change{} of pick".format(
        len(switches) - 1, "" if len(switches) == 2 else "s"))

    for t, res in switches:
        so = res.get("standoff_m")
        print("  t+{:6.2f}  -> facade {}{}{}".format(
            t, res.get("facade_id"),
            "  {:.2f} m".format(so) if so else "",
            "  look-off {:.1f} deg".format(res["look_off_deg"])
            if res.get("look_off_deg") is not None else ""))
        if verbose:
            for cand in res.get("candidates") or []:
                print("      {:<3} {:<22} {}".format(
                    cand.get("facade_id"),
                    (cand.get("label") or "")[:22],
                    cand["reject"] or "OK  t={:.1f} m  standoff={:.2f} m".format(
                        cand.get("t_m") or 0.0, cand.get("standoff_m") or 0.0)))

    steady = len(held) == 1 and -1 not in held
    print("\n{}".format(
        "Settled on one facade for the whole clip."
        if steady else
        "NOT steady — the pick changed or lapsed. Re-run with -v for the "
        "per-candidate reasons at each change."))
    return 0 if steady else 1


if __name__ == "__main__":
    import argparse
    import sys

    ap = argparse.ArgumentParser(
        description="Scene-plane maths for a recorded clip. With no arguments, "
                    "runs the synthetic self-check (no clip, no Unity, no drones).")
    ap.add_argument("--clip", help="clip folder to run the facade auto-pick over")
    ap.add_argument("--pick", action="store_true",
                    help="replay pick_facade over --clip's poses and report which "
                         "wall it chose, when it changed, and why each candidate "
                         "was rejected. The offline half of verifying the rule.")
    ap.add_argument("--shapes", default=DEFAULT_SHAPES_FILE,
                    help="shapes.json to read facades from (default: %(default)s)")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="per-candidate reject reasons at every change of pick")
    args = ap.parse_args()

    if args.pick:
        if not args.clip:
            sys.exit("--pick needs --clip")
        clip = args.clip
        if not os.path.isdir(clip):
            here = os.path.dirname(os.path.abspath(__file__))
            for cand in (os.path.join(here, clip),
                         os.path.join(here, "recordings", clip)):
                if os.path.isdir(cand):
                    clip = cand
                    break
            else:
                sys.exit("no clip folder {!r}".format(args.clip))
        shapes = args.shapes
        if not os.path.isabs(shapes):
            shapes = os.path.join(os.path.dirname(os.path.abspath(__file__)), shapes)
        sys.exit(explain_pick(clip, shapes, args.verbose))

    print("clip_scene_plane self-check\n")
    sys.exit(_selftest())
