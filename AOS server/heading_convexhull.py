"""
Convex-hull heading control (GLOBAL_CONVEXHULL) for the flocking swarm
======================================================================
Port of the Unity VR sim's GLOBAL_CONVEXHULL attitude algorithm
(`AttitudeAlgorithm.cs::getYawRateFromGlobalConvexHull` + `ConvexHull.cs`
in vr_swarm_simulation) to the DJI swarm.

Each control tick the swarm's 2D positions (local north/east metres) are
reduced to their convex hull. A drone that is a hull vertex is a *boundary*
drone: its target heading is the interior-angle bisector at its vertex,
flipped to face outward, away from the swarm (or inward with
``point_inwards=True``). Interior drones get no target — the caller should
hold their current heading (yaw rate 0).

Kept from the Unity source (same defaults):
  - Boundary-flag debounce (BoundaryHysteresisTime, 0.5 s): a hull-membership
    reading must persist before the published flag flips, so the
    boundary-vs-interior decision doesn't chatter as a drone jitters across
    the hull edge.
  - Target-heading low-pass (TargetHeadingFilterTime, 0.3 s): circular,
    frame-rate-independent smoothing so drones don't chase every hull
    deformation during manoeuvres.
  - Momentary-dropout hold: while the debounced flag still says "boundary",
    a drone that briefly drops off the hull keeps servoing to its last
    hull-derived heading instead of free-drifting.
  - Unit-edge bisector with hull-centroid fallback for nearly-collinear
    vertices (ConvexHull.ComputeBisector).

Deliberate differences from the Unity source:
  - Compass degrees ([-180, 180], 0 = north, + = clockwise/east) instead of
    radians, matching the PC-side telemetry and heading_hold_rate.
  - Emits target HEADINGS, not yaw rates. The caller converts each target
    through joystick_controller.heading_hold_rate so the DJI VS
    angular-velocity yaw channel keeps its single tuning knob (KP_YAW);
    Unity's YawCorrectionFactor plays that role there.
  - Hull membership is tracked by index instead of Unity's exact-float
    Vector2 `Contains` (which only works there because the very same values
    were inserted into the hull).
  - A drone missing from `positions` (lost GPS fix / dropped from the swarm
    snapshot) has its state reset, like AttitudeAlgorithm.Reset().

Pure Python, no ds_wrapper import — testable on any Python.
"""

import math


def _wrap180(deg):
    """Wrap any heading/angle into [-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0


def _normalize(vx, vy, eps=1e-9):
    """Unit vector, or None when the input is (near) zero length."""
    mag = math.hypot(vx, vy)
    if mag < eps:
        return None
    return vx / mag, vy / mag


def convex_hull_indices(points):
    """Andrew's monotone chain over a list of (x, y) tuples.

    Returns the indices of the hull vertices, ordered as a walk around the
    polygon. Collinear interior points and duplicates are excluded (strict
    turns only). 1 or 2 points are trivially their own hull.
    """
    n = len(points)
    if n <= 2:
        return list(range(n))

    order = sorted(range(n), key=lambda i: points[i])

    def cross(o, a, b):
        return ((points[a][0] - points[o][0]) * (points[b][1] - points[o][1])
                - (points[a][1] - points[o][1]) * (points[b][0] - points[o][0]))

    lower = []
    for i in order:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= 0:
            lower.pop()
        lower.append(i)
    upper = []
    for i in reversed(order):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= 0:
            upper.pop()
        upper.append(i)
    hull = lower[:-1] + upper[:-1]
    # All-coincident degenerate set collapses to a single vertex repeated;
    # dedupe defensively so bisector index arithmetic stays sane.
    return hull if len(hull) >= 1 else [order[0]]


def inward_bisector(hull_pts, k):
    """Interior-angle bisector (unit vector, pointing into the hull) at hull
    vertex ``k``. Port of ConvexHull.ComputeBisector with pointInwards=false
    semantics — i.e. this returns the INWARD direction; callers negate it to
    face outward.

    Nearly-collinear vertex (unit edges cancel): falls back to aiming at the
    hull centroid, which is always inward for a convex polygon. Returns None
    only when even that is degenerate (all points coincident).
    """
    count = len(hull_pts)
    px, py = hull_pts[k]
    nx, ny = hull_pts[(k + 1) % count]
    vx, vy = hull_pts[(k - 1) % count]

    e1 = _normalize(nx - px, ny - py)
    e2 = _normalize(vx - px, vy - py)
    if e1 is None or e2 is None:
        bis = None
    else:
        bis = _normalize(e1[0] + e2[0], e1[1] + e2[1])

    if bis is None:
        cx = sum(p[0] for p in hull_pts) / count
        cy = sum(p[1] for p in hull_pts) / count
        bis = _normalize(cx - px, cy - py)
    return bis


class ConvexHullHeading:
    """Per-swarm stateful GLOBAL_CONVEXHULL heading controller.

    Call update(positions, dt) once per control tick; it returns a target
    heading (deg) for every boundary drone and None for interior drones.
    """

    def __init__(self, point_inwards=False,
                 boundary_hysteresis_s=0.5, target_filter_s=0.3):
        self.point_inwards = point_inwards
        self.boundary_hysteresis_s = boundary_hysteresis_s
        self.target_filter_s = target_filter_s
        # drone_id -> {boundary, timer, target, has_target}
        self._state = {}

    def boundary_ids(self):
        """Drone ids currently flagged as boundary (debounced)."""
        return sorted(did for did, st in self._state.items() if st["boundary"])

    def reset(self):
        """Drop all per-drone state (AttitudeAlgorithm.Reset for the swarm).
        Call when hull heading control is (re-)activated so debounce timers
        and held targets don't carry over from a previous activation."""
        self._state.clear()

    def set_point_inwards(self, value):
        """Flip the facing direction at runtime. Mirrors Unity's
        OnSwarmParamsChanged: a parameter change invalidates the held hull
        headings (the goal rotates 180°), so drop them and let the next
        update() rebuild from scratch."""
        value = bool(value)
        if value == self.point_inwards:
            return
        self.point_inwards = value
        for st in self._state.values():
            st["has_target"] = False

    def update(self, positions, dt):
        """Advance one tick.

        Args:
            positions: {drone_id: (north_m, east_m)} — only drones with a
                       usable fix; missing drones have their state reset.
            dt:        seconds since the previous call.

        Returns:
            {drone_id: target_heading_deg or None} for every id in positions.
            None means interior drone → hold current heading (yaw rate 0).
        """
        # Reset state of drones that dropped out of the snapshot (Unity Reset()).
        for did in list(self._state):
            if did not in positions:
                del self._state[did]

        ids = sorted(positions)
        pts = [positions[did] for did in ids]
        hull = convex_hull_indices(pts) if pts else []
        hull_set = set(hull)
        hull_pts = [pts[k] for k in hull]

        out = {}
        for idx, did in enumerate(ids):
            st = self._state.setdefault(did, {
                "boundary": False, "timer": 0.0,
                "target": 0.0, "has_target": False,
            })
            on_hull = idx in hull_set

            # Raw hull-derived heading, when computable. A hull needs >= 2
            # vertices for edges to exist (a lone drone has no facing).
            raw = None
            if on_hull and len(hull) >= 2:
                bis = inward_bisector(hull_pts, hull.index(idx))
                if bis is not None:
                    dx, dy = bis if self.point_inwards else (-bis[0], -bis[1])
                    # (north, east) direction -> compass heading, matching
                    # telemetry: 0 = north, + = clockwise (east).
                    raw = math.degrees(math.atan2(dy, dx))

            self._update_boundary(st, on_hull, dt)

            # Mirrors AttitudeAlgorithm.getYawRateFromHull: recompute while on
            # the hull, hold while the debounced flag still says boundary,
            # release once debounced interior.
            if raw is not None:
                if not st["has_target"] or self.target_filter_s <= 0.0:
                    st["target"] = raw
                    st["has_target"] = True
                else:
                    alpha = 1.0 - math.exp(-dt / self.target_filter_s)
                    st["target"] = _wrap180(
                        st["target"] + alpha * _wrap180(raw - st["target"]))
            elif not on_hull and not st["boundary"]:
                st["has_target"] = False
            # else: on hull with degenerate geometry, or momentarily off the
            # hull but still boundary per the debounced flag — hold the target.

            out[did] = st["target"] if st["has_target"] else None
        return out

    def _update_boundary(self, st, on_hull, dt):
        """Symmetric debounce of the boundary flag (UpdateBoundaryEstimate):
        a reading disagreeing with the published flag must persist for
        boundary_hysteresis_s before the flip commits."""
        if on_hull == st["boundary"]:
            st["timer"] = 0.0
            return
        st["timer"] += dt
        if st["timer"] >= self.boundary_hysteresis_s:
            st["boundary"] = on_hull
            st["timer"] = 0.0
