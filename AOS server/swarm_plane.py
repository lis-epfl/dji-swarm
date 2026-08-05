"""
Vertical-plane swarming for the flocking swarm
==============================================
Port of the Unity sim's vertical-plane swarm mode (vr_swarm_simulation
Assets/Scripts/swarm/SwarmPlaneController.cs, commit 2fccac01) to the real
drones: a binary toggle that swaps the plane the Olfati-Saber law is
constrained to, from horizontal to a VERTICAL WALL, so the swarm re-forms as a
billboard of drones instead of a ring. Nothing is teleported — the drones fly
into the wall under the same cohesion law that holds the horizontal formation
together.

How the 2D law is reused UNCHANGED
----------------------------------
The Olfati-Saber cohesion potential is isotropic, so it does not care which
orthonormal basis its 2D coordinates are expressed in. In vertical mode we
project every drone into the wall's own axes and call the existing, untouched
`OlfatiSaber.GetSwarmAcceleration` on those coordinates. With psi = the plane
azimuth (compass deg) and NEU coordinates x = (north, east, alt):

    n  = ( cos psi,  sin psi, 0)     wall normal (horizontal)
    e1 = (-sin psi,  cos psi, 0)     along-wall  (horizontal)
    e2 = (       0,        0, 1)     up

    a1 = x . e1 = -north*sin psi + east*cos psi      in-plane, along-wall
    a2 = alt                                         in-plane, vertical
    a0 = x . n  =  north*cos psi + east*sin psi      out-of-plane offset

    (f1, f2) = OlfatiSaber.GetSwarmAcceleration((a1, a2), (w1, w2), nbrs, d_ref)

    v_north = -f1*sin psi + plane_term*cos psi
    v_east  =  f1*cos psi + plane_term*sin psi
    v_up    =  f2

`plane_term = clamp(gain * (a0_target - a0_self), +-MAX_PLANE_MPS)` is the
restoring pull onto the plane (the C# `planeCorrection`), and a0_target is
`n . centroid` — the C# no-anchor fallback. Centroid-pinning is deliberate: the
sim pins the plane to a pilot-flown anchor drone whose swarm force is zeroed,
and there is no per-drone pilot here. A centroid target is also zero-sum, so
the wall cannot drift sideways on its own.

The vertical channel is an ALTITUDE SETPOINT, not a velocity
-----------------------------------------------------------
DJI VS runs `VerticalControlMode.POSITION`: the `VS:` throttle field is an
absolute altitude in metres, and there is no vertical-velocity channel without
changing the Android app. So `v_up` is integrated into a per-drone altitude
setpoint, which is exactly the handover the sim does (VelocityControl's
`verticalSwarmAuthority` setpoint integrator) and is dimensionally exact here
because this repo already treats the O-S output as an m/s velocity correction:

    alt_cmd_i += clamp(v_up_i, +-MAX_VERT_MPS) * speed_scale * dt
    alt_cmd_i  = clamp(alt_cmd_i, max(min_alt, alt_ref - leash),
                                  min(max_alt, alt_ref + leash))

`alt_ref` is the swarm's centroid altitude captured on entry and then moved by
the climb stick, so the operator keeps authority over the whole wall while no
single drone can wander more than `leash` from it.

This is the first per-drone vertical control in the repo. With plane mode OFF
none of it runs: the caller keeps its single shared `target_alt` and its plain
(north, east) call, so the horizontal path is bit-identical to before.

Deviations from the sim, and why
--------------------------------
  - ENTRY STAGGER. At the instant of the flip every drone is at the same
    altitude, so in the wall's own axes the swarm is a horizontal LINE — a
    saddle configuration this fleet is already known to get stuck in. The sim
    gets away with it (fast, noisy drones); real ones may never unfold. So
    `enter()` seeds alternating +-0.5*d_ref vertical offsets by along-wall rank
    to break the symmetry.
  - EXIT RAMP. Handing straight back to a shared `target_alt` would step every
    drone's setpoint by metres at once and let the flight controller fly it at
    whatever rate it likes. `exit_ramp()` converges the per-drone setpoints onto
    their mean at MAX_VERT_MPS first; the caller hands back when it reports done.
  - AZIMUTH FROM THE STICK, not from a drone's compass. The sim low-passes an
    anchor aircraft's heading. Telemetry heading here is only ~4-5 Hz and this
    fleet has a logged incident (2026-07-05) of aircraft executing velocity
    commands rotated 90-180 deg from suspected FC compass error — that sensor
    must not be in the formation-geometry loop. psi = the caller's
    stick-integrated `target_yaw`, which is PC-owned and authoritative. With
    heading mode `manual` every drone then holds psi, which reproduces the sim's
    yaw lock (all noses along the wall normal) for free.
  - PLANE GAIN IS CLAMPED AND IN PHYSICAL UNITS. The C# `c_plane` is not divided
    by ScaleFactor while cohesion is, so its default 1.0 turns a 10 m offset into
    10 m/s of demand — instant saturation against MAX_CMD_MPS = 6. Here the gain
    is m/s per metre of offset and the term is clamped to MAX_PLANE_MPS, which is
    also what keeps obstacle/geofence avoidance (unprojected, horizontal, and
    therefore partly opposed to the restoring pull) the stronger authority.

Known hazards this module does NOT solve (caller's job)
-------------------------------------------------------
  - Two drones separated ONLY along the normal project to the same in-plane
    point: the cohesion force vanishes (zero-length relative vector) while the
    plane term pulls both onto the plane, i.e. toward each other. The caller's
    minimum-separation failsafe must be 3D, and it must be 3D anyway or a
    forming wall trips it instantly (drones 8 m apart vertically are ~0 m apart
    horizontally).
  - Rotor downwash. A 90 deg wall puts drones directly above one another; the sim
    models none of it.
  - Telemetry `alt` is takeoff-relative per aircraft, so the true vertical gap
    between two drones is (alt_i - alt_j) + (ground_i - ground_j). Launch from
    one flat pad. `alt_spread()` measures the observable part of this so the
    caller can gate entry on it.

Pure Python, no ds_wrapper import — testable on any Python
(python swarm_plane.py runs a self-check).
"""

import math

# Restoring pull onto the plane: m/s of horizontal command per metre of
# out-of-plane offset. DEFAULT seeds meta["plane_gain"] via --plane-gain;
# MIN/MAX bound both the CLI value and the GUI's live edits (swarm_gui.py
# validates against these same constants).
DEFAULT_PLANE_GAIN = 0.25
PLANE_GAIN_MIN = 0.02
PLANE_GAIN_MAX = 1.0

# Hard cap on the restoring term (m/s). Well under MAX_CMD_MPS (6.0) and under
# the obstacle/geofence term's authority, so avoidance always wins the argument
# when a repulsion pushes a drone out of the plane.
MAX_PLANE_MPS = 1.5

# Cap on the commanded climb/descent rate of one drone's altitude setpoint
# (m/s). The flight controller closes the position loop at its own rate; this
# bounds how fast we can ask it to.
MAX_VERT_MPS = 1.0

# How far a drone's altitude setpoint may sit from the wall's reference altitude
# (m). Bounds the wall's vertical extent and stops a runaway climb/descent.
# The sim uses 30 m; smaller here because our altitude band is 1-30 m total.
DEFAULT_PLANE_LEASH_M = 12.0

# Entry gate: all drones have been commanded the SAME shared altitude while
# horizontal, so the spread of their REPORTED altitudes is a direct measure of
# takeoff-frame bias plus tracking error. Above this (m), a wall built in
# altitude space would be skewed and 3D separation wrong by the same amount, so
# the caller refuses to enter plane mode.
ALT_SPREAD_GATE_M = 3.0

# Downwash advisory (m): a pair closer than this HORIZONTALLY with a nonzero
# altitude difference has the upper drone's prop wash over the lower one. The
# caller warns, it never stops the swarm.
DOWNWASH_RADIUS_M = 2.0

# Entry stagger as a fraction of the physical spacing target.
_STAGGER_FRAC = 0.5

# Altitude finite-difference low-pass (s) for the in-plane vertical velocity.
# Telemetry vz is ~1-2 Hz and documented as near-useless, so the vertical
# component of the velocity-consensus term is derived from reported altitude.
_ALT_RATE_FILTER_S = 0.5


def plane_basis(azimuth_deg):
    """Return (n, e1) for the vertical plane facing `azimuth_deg`.

    Each is a (north, east) unit 2-vector; the third basis vector e2 is world
    up = (0, 0, 1) and is implicit everywhere below. `n` is the wall normal
    (the compass direction it faces), `e1` the along-wall horizontal axis.
    """
    psi = math.radians(azimuth_deg)
    sin_p, cos_p = math.sin(psi), math.cos(psi)
    return (cos_p, sin_p), (-sin_p, cos_p)


def alt_spread(alts):
    """max - min over `alts` (m); 0.0 for fewer than two samples."""
    vals = [a for a in alts if a is not None]
    if len(vals) < 2:
        return 0.0
    return max(vals) - min(vals)


class SwarmPlane:
    """Per-swarm stateful vertical-plane controller.

    Lifecycle mirrors the heading modes: construct once, `enter()` on the
    rising edge of (plane mode AND swarming), `update()` once per control tick
    while active, `exit_ramp()` on the falling edge, then keep calling
    `update()` until `ramping` clears before handing the vertical channel back
    to the caller's shared altitude target.
    """

    def __init__(self, gain=DEFAULT_PLANE_GAIN, leash_m=DEFAULT_PLANE_LEASH_M,
                 min_alt=1.0, max_alt=30.0):
        self.gain = gain
        self.leash_m = leash_m
        self.min_alt = min_alt
        self.max_alt = max_alt

        self.active = False
        self.ramping = False        # exit ramp in progress (vertical still ours)
        self.azimuth_deg = 0.0      # psi, published for the GUI
        self.alt_ref = None         # wall reference altitude (leash centre)
        self.alt_cmd = {}           # drone_id -> commanded altitude (m)
        self.offsets = {}           # drone_id -> out-of-plane offset a0 (m)
        self._alt_rate = {}         # drone_id -> filtered d(alt)/dt (m/s)
        self._last_alt = {}         # drone_id -> previous reported alt (m)

    # ---- lifecycle ----

    def reset(self):
        """Drop all state. Call when the mode is (re-)armed so a stale wall
        reference or filter state cannot carry over."""
        self.active = False
        self.ramping = False
        self.alt_ref = None
        self.alt_cmd = {}
        self.offsets = {}
        self._alt_rate = {}
        self._last_alt = {}

    def enter(self, positions_ne, alts, azimuth_deg, d_ref_m):
        """Build the wall. Seeds the reference altitude and per-drone setpoints.

        Args:
            positions_ne: {drone_id: (north_m, east_m)} — drones with a fix.
            alts:         {drone_id: alt_m} reported altitudes.
            azimuth_deg:  initial plane azimuth (compass deg) = target_yaw.
            d_ref_m:      PHYSICAL spacing target (m), for the entry stagger.

        Returns the seeded {drone_id: alt_cmd_m}. The caller should gate on
        `alt_spread(alts) <= ALT_SPREAD_GATE_M` before calling this.
        """
        self.reset()
        self.active = True
        self.azimuth_deg = azimuth_deg
        _, e1 = plane_basis(azimuth_deg)

        have = [did for did in positions_ne if alts.get(did) is not None]
        if not have:
            # Nothing to anchor on; enter inert and let update() seed lazily.
            return {}
        self.alt_ref = sum(alts[did] for did in have) / len(have)

        # Entry stagger: rank along the wall and alternate the vertical offset.
        # A wall seeded flat is a horizontal line in its own axes — a saddle the
        # cohesion law has no gradient to leave. Alternating +-0.5*d_ref gives
        # it one.
        stagger = _STAGGER_FRAC * d_ref_m
        ranked = sorted(have, key=lambda d: (positions_ne[d][0] * e1[0]
                                             + positions_ne[d][1] * e1[1]))
        for rank, did in enumerate(ranked):
            offset = stagger * (0.5 if rank % 2 == 0 else -0.5)
            self.alt_cmd[did] = self._bound_alt(self.alt_ref + offset)
            self._last_alt[did] = alts[did]
            self._alt_rate[did] = 0.0
        return dict(self.alt_cmd)

    def exit_ramp(self):
        """Leave plane mode. The per-drone setpoints converge onto their mean at
        MAX_VERT_MPS over the following ticks; `ramping` stays True until they
        agree, and the caller must keep using our altitudes until then."""
        self.active = False
        self.ramping = bool(self.alt_cmd)
        if self.ramping:
            self.alt_ref = (sum(self.alt_cmd.values()) / len(self.alt_cmd))

    # ---- per-tick ----

    def update(self, olfati, positions_ne, alts, vels_ne, azimuth_deg,
               d_ref, stick_climb_mps, dt, speed_scale=1.0, exclude=()):
        """Advance one tick.

        Args:
            olfati:          the shared OlfatiSaber instance (used unmodified;
                             we only hand it in-plane 2D coordinates).
            positions_ne:    {drone_id: (north_m, east_m)}.
            alts:            {drone_id: alt_m} reported altitudes.
            vels_ne:         {drone_id: (vn, ve)} m/s world frame.
            azimuth_deg:     plane azimuth (compass deg) = the caller's
                             stick-integrated target_yaw. Read live, so the yaw
                             stick re-aims the wall.
            d_ref:           spacing target in SCALED units (as the 2D path).
            stick_climb_mps: operator climb command (m/s), moves the whole wall.
            dt:              seconds since the previous call.
            speed_scale:     --slow scale, applied to the vertical channel (the
                             caller applies it to the horizontal command).
            exclude:         drone ids that must not act as neighbours
                             (geofence-breached hoverers).

        Returns {drone_id: (v_north, v_east, alt_cmd_m, v_up)} for every drone
        in positions_ne with a reported altitude. v_north/v_east are the swarm
        correction only — the caller still adds the stick velocity, the obstacle
        term, and its own clamp/scale, exactly as on the horizontal path.
        """
        if self.ramping and not self.active:
            return self._ramp_tick(dt, speed_scale)

        self.azimuth_deg = azimuth_deg
        n_hat, e1 = plane_basis(azimuth_deg)
        excl = set(exclude)

        have = [did for did in sorted(positions_ne)
                if alts.get(did) is not None and did not in excl]
        if not have:
            return {}

        # In-plane coordinates and the out-of-plane offset.
        coords = {}
        for did in have:
            nn, ee = positions_ne[did]
            a1 = nn * e1[0] + ee * e1[1]
            a0 = nn * n_hat[0] + ee * n_hat[1]
            coords[did] = (a1, alts[did], a0)
        self.offsets = {did: coords[did][2] for did in have}

        # Wall reference altitude: seeded on entry, then moved by the stick. The
        # leash is measured against it, so the operator raises/lowers the whole
        # wall rather than fighting the leash.
        if self.alt_ref is None:
            self.alt_ref = sum(alts[did] for did in have) / len(have)
        self.alt_ref = self._bound_alt(
            self.alt_ref + stick_climb_mps * speed_scale * dt)

        # Plane offset target: the swarm's centroid projected onto the normal.
        # Zero-sum, so the wall cannot drift along its own normal.
        a0_target = sum(coords[did][2] for did in have) / len(have)

        # In-plane velocities. Along-wall from telemetry; vertical from a
        # filtered altitude finite difference (telemetry vz is ~1-2 Hz).
        planar_vel = {}
        for did in have:
            vn, ve = vels_ne.get(did, (0.0, 0.0))
            w1 = vn * e1[0] + ve * e1[1]
            planar_vel[did] = (w1, self._alt_rate_of(did, alts[did], dt))

        out = {}
        for did in have:
            a1, a2, a0 = coords[did]
            neighbours = [((coords[j][0], coords[j][1]), planar_vel[j])
                          for j in have if j != did]
            f1, f2 = olfati.GetSwarmAcceleration(
                (a1, a2), planar_vel[did], neighbours, d_ref=d_ref)

            # Restoring pull onto the plane, along the (horizontal) normal.
            plane_term = _clamp(self.gain * (a0_target - a0),
                                -MAX_PLANE_MPS, MAX_PLANE_MPS)

            # Map the in-plane along-wall force and the normal-direction
            # restoring term back into world north/east.
            v_n = f1 * e1[0] + plane_term * n_hat[0]
            v_e = f1 * e1[1] + plane_term * n_hat[1]

            v_up = _clamp(f2, -MAX_VERT_MPS, MAX_VERT_MPS)
            alt = self.alt_cmd.get(did)
            if alt is None:
                alt = alts[did]     # joined late (GPS reacquired): start where it is
            alt += (v_up + stick_climb_mps) * speed_scale * dt
            alt = self._bound_alt(alt, leashed=True)
            self.alt_cmd[did] = alt

            out[did] = (v_n, v_e, alt, v_up)

        # Drop state for drones that left the snapshot, so a rejoin starts clean.
        for did in list(self.alt_cmd):
            if did not in have:
                self.alt_cmd.pop(did, None)
                self._alt_rate.pop(did, None)
                self._last_alt.pop(did, None)
        return out

    def _ramp_tick(self, dt, speed_scale):
        """Exit ramp: converge the per-drone setpoints onto their mean at
        MAX_VERT_MPS. Horizontal correction is zero — the plane is gone, the
        caller's normal 2D path is already producing the horizontal command."""
        step = MAX_VERT_MPS * max(speed_scale, 1e-3) * dt
        target = self.alt_ref if self.alt_ref is not None else 0.0
        done = True
        out = {}
        for did, alt in list(self.alt_cmd.items()):
            err = target - alt
            if abs(err) <= step:
                alt = target
            else:
                alt += step if err > 0 else -step
                done = False
            alt = self._bound_alt(alt)
            self.alt_cmd[did] = alt
            out[did] = (0.0, 0.0, alt, 0.0)
        if done:
            self.ramping = False
        return out

    # ---- helpers ----

    def _bound_alt(self, alt, leashed=False):
        lo, hi = self.min_alt, self.max_alt
        if leashed and self.alt_ref is not None:
            lo = max(lo, self.alt_ref - self.leash_m)
            hi = min(hi, self.alt_ref + self.leash_m)
        if lo > hi:          # leash pushed outside the band: band wins
            lo = hi = min(max(self.alt_ref or lo, self.min_alt), self.max_alt)
        return _clamp(alt, lo, hi)

    def _alt_rate_of(self, did, alt, dt):
        prev = self._last_alt.get(did)
        self._last_alt[did] = alt
        if prev is None or dt <= 0.0:
            return self._alt_rate.setdefault(did, 0.0)
        raw = (alt - prev) / dt
        cur = self._alt_rate.get(did, 0.0)
        alpha = 1.0 - math.exp(-dt / _ALT_RATE_FILTER_S)
        cur += alpha * (raw - cur)
        self._alt_rate[did] = cur
        return cur

    def status(self):
        """Snapshot for meta/the GUI (plain JSON-able types)."""
        return {
            "on": self.active,
            "ramping": self.ramping,
            "az": round(self.azimuth_deg, 1),
            "gain": round(self.gain, 3),
            "leash": round(self.leash_m, 1),
            "alt_ref": None if self.alt_ref is None else round(self.alt_ref, 2),
            "offsets": {str(d): round(v, 2) for d, v in self.offsets.items()},
            "alt_cmd": {str(d): round(v, 2) for d, v in self.alt_cmd.items()},
        }


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def separation_3d(pos_a, alt_a, pos_b, alt_b):
    """True 3D separation (m) between two drones.

    math.hypot takes exactly two args on Python 3.7 (the wrapper pins 3.7), so
    this is spelled out rather than hypot(dn, de, dalt).
    """
    dn = pos_a[0] - pos_b[0]
    de = pos_a[1] - pos_b[1]
    du = (alt_a or 0.0) - (alt_b or 0.0)
    return math.sqrt(dn * dn + de * de + du * du)


if __name__ == "__main__":
    # Self-check (no hardware, any Python): basis, projection round-trip, the
    # restoring term, wall unfolding from a staggered start, leash/band clamps,
    # the exit ramp, and 3D separation.
    from olfati_saber import OlfatiSaber

    # --- basis: orthonormal at several azimuths, and matches the doc formulas
    for az in (0.0, 37.0, 90.0, 180.0, -125.0):
        n_hat, e1 = plane_basis(az)
        assert abs(n_hat[0] ** 2 + n_hat[1] ** 2 - 1.0) < 1e-12
        assert abs(e1[0] ** 2 + e1[1] ** 2 - 1.0) < 1e-12
        assert abs(n_hat[0] * e1[0] + n_hat[1] * e1[1]) < 1e-12
    n_hat, e1 = plane_basis(0.0)          # facing north
    assert abs(n_hat[0] - 1.0) < 1e-12 and abs(n_hat[1]) < 1e-12
    assert abs(e1[0]) < 1e-12 and abs(e1[1] - 1.0) < 1e-12
    n_hat, e1 = plane_basis(90.0)         # facing east
    assert abs(n_hat[0]) < 1e-12 and abs(n_hat[1] - 1.0) < 1e-12
    assert abs(e1[0] + 1.0) < 1e-12 and abs(e1[1]) < 1e-12

    olfati = OlfatiSaber(scale=10.0)
    D_REF_M = 8.0
    d_ref = D_REF_M / olfati.scale
    DT = 0.05

    # --- entry: stagger breaks the flat (saddle) start
    pos = {1: (0.0, -8.0), 2: (0.0, 0.0), 3: (0.0, 8.0)}
    alts = {1: 10.0, 2: 10.0, 3: 10.0}
    plane = SwarmPlane(min_alt=1.0, max_alt=30.0)
    seeded = plane.enter(pos, alts, 0.0, D_REF_M)   # wall faces north
    assert len(seeded) == 3
    assert abs(plane.alt_ref - 10.0) < 1e-9
    assert alt_spread(seeded.values()) > 1.0, seeded   # not flat any more
    assert alt_spread(alts.values()) < 1e-9            # gate would have passed

    # --- a drone off the plane is pulled back ALONG THE NORMAL only.
    # Wall faces north, so the normal is north: an offset drone gets a north
    # correction and (being alone on that axis) no along-wall one.
    solo = SwarmPlane()
    solo.enter({1: (0.0, 0.0), 2: (0.0, 0.0)}, {1: 10.0, 2: 10.0}, 0.0, D_REF_M)
    out = solo.update(olfati, {1: (6.0, 0.0), 2: (-6.0, 0.0)},
                      {1: 10.0, 2: 10.0}, {1: (0.0, 0.0), 2: (0.0, 0.0)},
                      0.0, d_ref, 0.0, DT)
    assert out[1][0] < -0.1, out[1]     # north of the plane -> pushed south
    assert out[2][0] > 0.1, out[2]      # south of it -> pushed north
    assert abs(out[1][1]) < 1e-9 and abs(out[2][1]) < 1e-9   # no east term
    assert abs(out[1][0] + out[2][0]) < 1e-9                 # zero-sum
    # Same geometry with the wall facing east puts the whole correction on east.
    out = solo.update(olfati, {1: (0.0, 6.0), 2: (0.0, -6.0)},
                      {1: 10.0, 2: 10.0}, {1: (0.0, 0.0), 2: (0.0, 0.0)},
                      90.0, d_ref, 0.0, DT)
    assert abs(out[1][0]) < 1e-9 and out[1][1] < -0.1, out[1]
    # Restoring term is clamped.
    out = solo.update(olfati, {1: (500.0, 0.0), 2: (-500.0, 0.0)},
                      {1: 10.0, 2: 10.0}, {1: (0.0, 0.0), 2: (0.0, 0.0)},
                      0.0, d_ref, 0.0, DT)
    assert abs(out[1][0]) <= MAX_PLANE_MPS + 1e-9, out[1]

    # --- the wall unfolds: settle the 3-drone swarm and check it ends up
    # spread VERTICALLY (in-plane) and flat against the normal.
    plane = SwarmPlane(min_alt=1.0, max_alt=30.0)
    plane.enter(pos, alts, 0.0, D_REF_M)
    cur_pos = dict(pos)
    cur_alt = dict(plane.alt_cmd)
    for _ in range(4000):
        res = plane.update(olfati, cur_pos, cur_alt,
                           {d: (0.0, 0.0) for d in cur_pos},
                           0.0, d_ref, 0.0, DT)
        # Kinematic drones: horizontal follows the command, altitude tracks the
        # setpoint immediately (the real FC lags; this is a math check).
        for did, (v_n, v_e, alt_c, _v_up) in res.items():
            nn, ee = cur_pos[did]
            cur_pos[did] = (nn + v_n * DT, ee + v_e * DT)
            cur_alt[did] = alt_c
    assert alt_spread(cur_alt.values()) > 4.0, cur_alt      # a real wall
    assert max(abs(v) for v in plane.offsets.values()) < 0.5, plane.offsets
    # Spacing settled near the target (in-plane, so 3D too — offsets are ~0).
    ids = sorted(cur_pos)
    dists = [separation_3d(cur_pos[a], cur_alt[a], cur_pos[b], cur_alt[b])
             for i, a in enumerate(ids) for b in ids[i + 1:]]
    assert min(dists) > 3.0, dists
    # Every drone stayed inside the band and the leash.
    for did, a in cur_alt.items():
        assert 1.0 <= a <= 30.0, (did, a)
        assert abs(a - plane.alt_ref) <= DEFAULT_PLANE_LEASH_M + 1e-6, (did, a)

    # --- stick climb moves the whole wall, leash rides along
    before = plane.alt_ref
    for _ in range(40):     # 2 s at +1 m/s
        plane.update(olfati, cur_pos, cur_alt,
                     {d: (0.0, 0.0) for d in cur_pos}, 0.0, d_ref, 1.0, DT)
        cur_alt = dict(plane.alt_cmd)
    assert plane.alt_ref > before + 1.5, (before, plane.alt_ref)

    # --- ceiling/floor win over the leash
    tight = SwarmPlane(min_alt=1.0, max_alt=6.0)
    tight.enter({1: (0.0, 0.0)}, {1: 5.0}, 0.0, D_REF_M)
    for _ in range(200):
        tight.update(olfati, {1: (0.0, 0.0)}, {1: 5.0}, {1: (0.0, 0.0)},
                     0.0, d_ref, 5.0, DT)   # slam the climb stick
    assert tight.alt_cmd[1] <= 6.0 + 1e-9, tight.alt_cmd

    # --- exit ramp: rate-limited convergence onto the mean, then done
    plane.exit_ramp()
    assert plane.ramping and not plane.active
    target = plane.alt_ref
    spread0 = alt_spread(plane.alt_cmd.values())
    assert spread0 > 1.0, spread0
    prev = dict(plane.alt_cmd)
    ticks = 0
    while plane.ramping and ticks < 2000:
        res = plane.update(olfati, cur_pos, cur_alt,
                           {d: (0.0, 0.0) for d in cur_pos},
                           0.0, d_ref, 0.0, DT)
        for did, (v_n, v_e, alt_c, _v) in res.items():
            assert v_n == 0.0 and v_e == 0.0          # plane is gone
            assert abs(alt_c - prev[did]) <= MAX_VERT_MPS * DT + 1e-9
            prev[did] = alt_c
        ticks += 1
    assert not plane.ramping and ticks > 1, ticks
    assert alt_spread(plane.alt_cmd.values()) < 1e-6
    assert abs(list(plane.alt_cmd.values())[0] - target) < 1e-6

    # --- late joiner starts from its own altitude, not someone else's setpoint
    plane = SwarmPlane()
    plane.enter({1: (0.0, 0.0)}, {1: 10.0}, 0.0, D_REF_M)
    res = plane.update(olfati, {1: (0.0, 0.0), 9: (0.0, 8.0)},
                       {1: 10.0, 9: 17.0}, {1: (0.0, 0.0), 9: (0.0, 0.0)},
                       0.0, d_ref, 0.0, DT)
    assert abs(res[9][2] - 17.0) < 0.2, res[9]
    # A drone that leaves the snapshot loses its state.
    plane.update(olfati, {1: (0.0, 0.0)}, {1: 10.0}, {1: (0.0, 0.0)},
                 0.0, d_ref, 0.0, DT)
    assert 9 not in plane.alt_cmd

    # --- excluded (geofenced) drones are neither commanded nor neighbours
    plane = SwarmPlane()
    plane.enter(pos, alts, 0.0, D_REF_M)
    res = plane.update(olfati, pos, alts, {d: (0.0, 0.0) for d in pos},
                       0.0, d_ref, 0.0, DT, exclude=(3,))
    assert set(res) == {1, 2}, set(res)

    # --- 3D separation: stacked drones are NOT zero apart
    assert abs(separation_3d((0.0, 0.0), 10.0, (0.0, 0.0), 18.0) - 8.0) < 1e-9
    assert abs(separation_3d((3.0, 4.0), 5.0, (0.0, 0.0), 5.0) - 5.0) < 1e-9

    # --- alt_spread / gate helper
    assert abs(alt_spread([4.0, 9.5, 6.0]) - 5.5) < 1e-9
    assert alt_spread([7.0]) == 0.0 and alt_spread([]) == 0.0
    assert alt_spread([3.0, None, 4.5]) == 1.5

    print("swarm_plane self-check OK")
