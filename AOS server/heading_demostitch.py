"""
Demo Stitch heading control for the flocking swarm
==================================================
A third attitude mode between MANUAL and GLOBAL_CONVEXHULL, built to promote
image stitching between laterally adjacent camera views: the drone in the
lateral middle of the formation points exactly at the operator's stick-steered
global yaw, and each neighbour outward fans away by a fixed per-rank offset
(default 30 deg), so every adjacent pair of cameras keeps partial overlap.

Each control tick the swarm's 2D positions (local north/east metres) are
projected onto the axis PERPENDICULAR to the global yaw and ordered
left-to-right as seen along the view direction. Rank r in 0..N-1 with centre
c = (N-1)/2 gets target = wrap180(global_yaw + (r - c) * offset). Odd N puts
one drone dead-centre (offset 0); even N gives symmetric half-step offsets
(+-offset/2, +-3*offset/2, ...). N == 1 degenerates to manual. As the operator
rotates the global yaw the lateral ordering changes, so a DIFFERENT drone can
become the centre — role re-election is continuous.

Two guards keep the re-election from thrashing the noses:
  - Sticky ranking (rank_hysteresis_m, 1.5 m): adjacent drones only swap ranks
    when one's lateral coordinate clearly crosses the other's by more than the
    margin. Without it, two drones abreast within GPS jitter (~1 m) would
    rank-flip at a few Hz and any amount of filtering just turns that square
    wave into a sustained wag.
  - Offset low-pass (offset_filter_s, 0.5 s): the per-drone offset component
    delta = (r - c) * offset is filtered, NOT the full target heading —
    filtering the full target would lag the stick feed-forward by up to
    ff_rate * filter_s (~20 deg at the 40 deg/s clamp) during a rotation.
    Stick yaw therefore applies instantly, exactly like manual mode; only
    role handovers and offset changes slew smoothly.

Conventions match heading_convexhull.py: compass degrees ([-180, 180],
0 = north, + = clockwise/east), positions in local (north_m, east_m). Emits
target HEADINGS; the caller converts them through
joystick_controller.heading_hold_rate (with the shared stick feed-forward,
since the whole fan rotates rigidly with the global yaw).

Pure Python, no ds_wrapper import — testable on any Python
(python heading_demostitch.py runs a self-check).
"""

import math

from heading_convexhull import _wrap180

# Per-rank fan offset (deg). DEFAULT seeds meta["stitch_offset"] via
# --stitch-offset; MIN/MAX bound both the CLI value and the GUI's live edits
# (swarm_gui.py validates against these same constants).
DEFAULT_OFFSET_DEG = 30.0
OFFSET_MIN_DEG = 5.0
OFFSET_MAX_DEG = 90.0

# Cap the total fan span (N-1)*offset so the outermost wings can never alias
# past each other around the compass.
_MAX_SPAN_DEG = 330.0


class DemoStitchHeading:
    """Per-swarm stateful Demo Stitch heading controller.

    Call update(positions, global_yaw_deg, offset_deg, dt) once per control
    tick; it returns a target heading (deg) for EVERY drone in positions.
    """

    def __init__(self, offset_filter_s=0.5, rank_hysteresis_m=1.5):
        self.offset_filter_s = offset_filter_s
        self.rank_hysteresis_m = rank_hysteresis_m
        self._order = []      # drone ids, sticky left->right lateral ordering
        self._delta = {}      # drone_id -> filtered offset component (deg)
        # Drone at the exact centre rank (odd N only; None for even N) — for
        # the GUI's CENTRE pill and the 1 Hz status line.
        self.centre_id = None

    def reset(self):
        """Drop all per-drone state. Call when the mode is (re-)activated so
        a stale ordering/filter state doesn't carry over."""
        self._order = []
        self._delta = {}
        self.centre_id = None

    def update(self, positions, global_yaw_deg, offset_deg, dt):
        """Advance one tick.

        Args:
            positions:      {drone_id: (north_m, east_m)} — only drones with a
                            usable fix; missing drones are dropped from the
                            ordering (they must not occupy a rank).
            global_yaw_deg: the operator's stick-integrated global yaw
                            (compass deg) — the centre drone's target.
            offset_deg:     per-rank fan offset (deg); read live each tick so
                            GUI changes apply immediately.
            dt:             seconds since the previous call.

        Returns:
            {drone_id: target_heading_deg} for every id in positions.
        """
        # Lateral coordinate: metres to the RIGHT of the view axis. View
        # direction in (N, E) is (cos psi, sin psi); rightward is
        # (-sin psi, cos psi). Only differences matter, so no centroid
        # subtraction is needed.
        psi = math.radians(global_yaw_deg)
        sin_p, cos_p = math.sin(psi), math.cos(psi)
        lat = {did: -n * sin_p + e * cos_p
               for did, (n, e) in positions.items()}

        # Membership: drop ids that left the snapshot (GPS dropout / geofence
        # removal), insert newcomers where their lateral coordinate dictates.
        self._order = [did for did in self._order if did in lat]
        for did in sorted(positions):
            if did not in self._order:
                k = 0
                while k < len(self._order) and lat[self._order[k]] <= lat[did]:
                    k += 1
                self._order.insert(k, did)
                self._delta.pop(did, None)  # no stale filter state on rejoin
        for did in list(self._delta):
            if did not in lat:
                del self._delta[did]

        # Sticky re-rank: bubble passes swapping adjacent pairs only when the
        # left one is clearly (> margin) to the right of its neighbour. Near
        # ties — including fast sweeps of the lateral axis while the operator
        # rotates the yaw — keep the previous stable order.
        changed = True
        while changed:
            changed = False
            for k in range(len(self._order) - 1):
                a, b = self._order[k], self._order[k + 1]
                if lat[a] > lat[b] + self.rank_hysteresis_m:
                    self._order[k], self._order[k + 1] = b, a
                    changed = True

        n = len(self._order)
        centre = (n - 1) / 2.0
        offset_eff = offset_deg if n < 2 else min(offset_deg,
                                                  _MAX_SPAN_DEG / (n - 1))

        out = {}
        for rank, did in enumerate(self._order):
            delta_raw = (rank - centre) * offset_eff
            if did not in self._delta or self.offset_filter_s <= 0.0:
                self._delta[did] = delta_raw
            else:
                alpha = 1.0 - math.exp(-dt / self.offset_filter_s)
                self._delta[did] += alpha * (delta_raw - self._delta[did])
            out[did] = _wrap180(global_yaw_deg + self._delta[did])

        self.centre_id = self._order[(n - 1) // 2] if n % 2 == 1 else None
        return out


if __name__ == "__main__":
    # Self-check (no hardware, any Python): fan values, centre election,
    # rank hysteresis under jitter, filter convergence, yaw re-election.
    def settle(ctrl, positions, yaw, off, ticks=200, dt=0.05):
        for _ in range(ticks):
            out = ctrl.update(positions, yaw, off, dt)
        return out

    # 3 drones in an east-west line, viewing north: fan is [-30, 0, +30]
    # west->east, centre = the middle drone.
    pos = {1: (0.0, -20.0), 2: (0.0, 0.0), 3: (0.0, 20.0)}
    ctrl = DemoStitchHeading()
    out = settle(ctrl, pos, 0.0, 30.0)
    assert ctrl.centre_id == 2, ctrl.centre_id
    for did, want in ((1, -30.0), (2, 0.0), (3, 30.0)):
        assert abs(out[did] - want) < 0.5, (did, out[did], want)

    # Rotate the view to south: left/right swap, so the fan reverses sign
    # (targets are now 180 +- 30 with drone 3 on the left).
    out = settle(ctrl, pos, 180.0, 30.0)
    assert ctrl.centre_id == 2
    assert abs(_wrap180(out[1] - (-150.0))) < 0.5, out[1]  # 180 + 30 wrapped
    assert abs(_wrap180(out[3] - 150.0)) < 0.5, out[3]     # 180 - 30

    # Jitter two near-abreast drones by +-0.5 m laterally: within the 1.5 m
    # hysteresis margin the ordering must never flip.
    ctrl.reset()
    import random
    random.seed(7)
    base = {1: (0.0, -10.0), 2: (0.0, -0.2), 3: (0.0, 0.2)}
    ctrl.update(base, 0.0, 30.0, 0.05)
    order0 = list(ctrl._order)
    for _ in range(500):
        jit = {did: (n, e + random.uniform(-0.5, 0.5))
               for did, (n, e) in base.items()}
        ctrl.update(jit, 0.0, 30.0, 0.05)
        assert ctrl._order == order0, ctrl._order

    # A clear crossing (beyond the margin) DOES re-rank and re-elects the
    # centre; the filtered targets converge to the new fan.
    crossed = {1: (0.0, -10.0), 2: (0.0, 5.0), 3: (0.0, 0.2)}
    out = settle(ctrl, crossed, 0.0, 30.0)
    assert ctrl._order == [1, 3, 2], ctrl._order
    assert ctrl.centre_id == 3
    assert abs(out[3]) < 0.5 and abs(out[2] - 30.0) < 0.5

    # Live offset change 30 -> 10 converges exponentially onto the new fan.
    out = settle(ctrl, crossed, 0.0, 10.0)
    assert abs(out[1] + 10.0) < 0.5 and abs(out[2] - 10.0) < 0.5

    # Even N: symmetric half-step offsets, no exact centre.
    ctrl.reset()
    pos4 = {i: (0.0, 10.0 * i) for i in range(1, 5)}
    out = settle(ctrl, pos4, 0.0, 30.0)
    assert ctrl.centre_id is None
    for did, want in ((1, -45.0), (2, -15.0), (3, 15.0), (4, 45.0)):
        assert abs(out[did] - want) < 0.5, (did, out[did], want)

    # Span cap: 13 drones at 30 deg would span 360 — capped to 330/(N-1).
    ctrl.reset()
    pos13 = {i: (0.0, 5.0 * i) for i in range(1, 14)}
    out = settle(ctrl, pos13, 0.0, 30.0)
    span = max(_wrap180(t) for t in out.values()) - min(_wrap180(t)
                                                        for t in out.values())
    assert span <= 330.0 + 0.5, span

    # N == 1 degenerates to manual; a dropout mid-flight re-centres the rest.
    ctrl.reset()
    out = settle(ctrl, {7: (3.0, 4.0)}, 42.0, 30.0)
    assert ctrl.centre_id == 7 and abs(out[7] - 42.0) < 1e-6
    ctrl.reset()
    settle(ctrl, pos, 0.0, 30.0)
    out = settle(ctrl, {1: pos[1], 2: pos[2]}, 0.0, 30.0)  # drone 3 dropped
    assert ctrl.centre_id is None
    assert abs(out[1] + 15.0) < 0.5 and abs(out[2] - 15.0) < 0.5

    print("heading_demostitch self-check OK")
