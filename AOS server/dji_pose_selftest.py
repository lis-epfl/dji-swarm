"""
Self-test for `dji_camera_pose`.  No hardware, no shared memory, no Unity.

    cd "AOS server" && python dji_pose_selftest.py

Why an independent derivation
-----------------------------
Coordinate-frame errors are the dominant failure mode of a pose-initialised
stitcher, and they are silent: a mosaic built from a mirrored or 90-degree-rotated
pose still renders, it just never lines up.  So the module under test is *not*
checked against itself.

`dji_camera_pose.quat_from_gimbal` composes Unity Euler angles in Unity's Z-X-Y
order.  This file instead builds the camera's three world axes directly from
compass/gimbal geometry -- "yaw 0 looks North, yaw 90 looks East, pitch -90 looks
down" -- and asserts the two agree.  The constructions share no code, so a sign
error would have to appear identically in both to pass, which reasoning about the
axes independently makes very unlikely.

The one thing this cannot pin is the sign of DJI's *roll*: there is no independent
statement of that convention available here, only self-consistency.  A stabilised
gimbal holds roll within about a degree of zero, so it is also the term that
matters least -- but verify it on hardware before trusting a rolled camera.

Section 8 is a different kind of check from the rest.  Sections 1-7 pin sign and
frame conventions, which are properties of the maths.  Section 8 pins where the
bearing is *sourced from*, which is a property of the hardware: `gimbal_yaw` slips
by a per-aircraft amount at every takeoff, so a pose built from it points the
fleet's cameras up to 18 degrees apart when they are physically parallel.  That
cost 11.3 m of seam on the 2026-08-11 MED clips and is invisible to sections 1-7,
because every one of those angles is self-consistent -- just wrong.  It uses the
real measured slips as its fixture.
"""

import math
import sys

from dji_camera_pose import (CameraPoseSolver, camera_bearing, quat_from_gimbal,
                             POSE_VALID, YAW_SOURCES)
from olfati_saber import EARTH_M_PER_DEG

# Median `gimbal_yaw - heading` per drone on clip MED_facade_stationary_1
# (clip_20260811_162553). The cameras were physically parallel; these are the
# slips section 8 asserts never reach the pose.
MED_SLIP_DEG = (2.8, -15.4, -6.6)

_failures = []


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                           "  --  " + detail if detail else ""))
    if not ok:
        _failures.append(name)
    return ok


# ----------------------------------------------------------------------------------
# Independent construction: camera axes straight from compass/gimbal geometry
# ----------------------------------------------------------------------------------

def axes_from_gimbal(yaw_deg, pitch_deg, roll_deg=0.0):
    """
    `(right, up, forward)` in Unity world (+X East, +Y Up, +Z North).

    Derived from what the angles physically mean, not from any Euler order:
      * forward's horizontal bearing is the compass yaw (0 = North, clockwise),
        and it tilts by the gimbal pitch (negative = down).
      * right is horizontal, 90 degrees clockwise from forward's ground track --
        the right of North is East.
      * up completes the Unity basis, where right x up = forward.
    Roll then spins right/up about forward.
    """
    y = math.radians(yaw_deg)
    p = math.radians(pitch_deg)
    cp = math.cos(p)

    forward = (math.sin(y) * cp, math.sin(p), math.cos(y) * cp)
    right = (math.cos(y), 0.0, -math.sin(y))
    up = _cross(forward, right)

    if roll_deg:
        # Positive DJI roll drops the right-hand side, i.e. right rotates toward
        # -up about the forward axis. Self-consistency only; see the module note.
        r = math.radians(roll_deg)
        c, s = math.cos(r), math.sin(r)
        right, up = (_add(_scale(right, c), _scale(up, -s)),
                     _add(_scale(right, s), _scale(up, c)))
    return right, up, forward


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def quat_to_columns(q):
    """(x,y,z,w) -> the rotation's columns, i.e. the camera's (right, up, forward)."""
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    right = (1.0 - (yy + zz), xy + wz, xz - wy)
    up = (xy - wz, 1.0 - (xx + zz), yz + wx)
    forward = (xz + wy, yz - wx, 1.0 - (xx + yy))
    return right, up, forward


def _maxdiff(a, b):
    return max(abs(p - q) for p, q in zip(a, b))


# ----------------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------------

def test_cardinals():
    """The four cardinal bearings, level gimbal, checked against plain geography."""
    print("\n1. Cardinal bearings (level gimbal)")
    expect = {
        0.0:   ("North", (0.0, 0.0, 1.0)),
        90.0:  ("East",  (1.0, 0.0, 0.0)),
        180.0: ("South", (0.0, 0.0, -1.0)),
        270.0: ("West",  (-1.0, 0.0, 0.0)),
    }
    for yaw in sorted(expect):
        name, want = expect[yaw]
        _, _, fwd = quat_to_columns(quat_from_gimbal(yaw, 0.0, 0.0))
        check("yaw %5.1f looks %s" % (yaw, name), _maxdiff(fwd, want) < 1e-9,
              "forward = (%.3f, %.3f, %.3f)" % fwd)


def test_pitch():
    """Gimbal pitch is negative-down, and Unity's X Euler is positive-down."""
    print("\n2. Gimbal pitch sign")
    _, _, fwd = quat_to_columns(quat_from_gimbal(0.0, -90.0, 0.0))
    check("pitch -90 looks straight down", _maxdiff(fwd, (0.0, -1.0, 0.0)) < 1e-9,
          "forward = (%.3f, %.3f, %.3f)" % fwd)

    _, _, fwd = quat_to_columns(quat_from_gimbal(0.0, -30.0, 0.0))
    check("pitch -30 tilts down, not up", fwd[1] < -0.4,
          "forward y = %.3f (negative = downward)" % fwd[1])

    # Nadir is the configuration the sim already mosaics, so it is worth pinning
    # that a -90 gimbal at any bearing still points at the ground.
    ok = True
    for yaw in (0.0, 37.0, 123.0, 271.0):
        _, _, f = quat_to_columns(quat_from_gimbal(yaw, -90.0, 0.0))
        ok = ok and _maxdiff(f, (0.0, -1.0, 0.0)) < 1e-9
    check("a -90 gimbal points down at every bearing", ok)


def test_against_independent_axes():
    """The decisive one: Euler composition vs geometry, sharing no code."""
    print("\n3. Euler composition vs independent axis construction")
    cases = [
        (0.0, 0.0, 0.0), (37.0, 0.0, 0.0), (180.0, -15.0, 0.0),
        (271.0, -45.0, 0.0), (95.0, -90.0, 0.0), (12.0, 8.0, 0.0),
        (150.0, -22.0, 5.0), (300.0, -60.0, -7.5),
    ]
    worst = 0.0
    for yaw, pitch, roll in cases:
        got = quat_to_columns(quat_from_gimbal(yaw, pitch, roll))
        want = axes_from_gimbal(yaw, pitch, roll)
        d = max(_maxdiff(g, w) for g, w in zip(got, want))
        worst = max(worst, d)
        if d >= 1e-9:
            check("yaw %.1f pitch %.1f roll %.1f" % (yaw, pitch, roll), False,
                  "axis mismatch %.3e" % d)
    check("all %d attitudes match the geometric construction" % len(cases),
          worst < 1e-9, "worst axis error %.2e" % worst)


def test_orthonormal():
    """A rotation must stay a rotation: orthonormal, right-handed in Unity's sense."""
    print("\n4. Rotation well-formedness")
    worst_dot = 0.0
    worst_len = 0.0
    worst_hand = 0.0
    for yaw in range(0, 360, 45):
        for pitch in (-90.0, -45.0, 0.0, 20.0):
            r, u, f = quat_to_columns(quat_from_gimbal(float(yaw), pitch, 0.0))
            worst_dot = max(worst_dot, abs(_dot(r, u)), abs(_dot(r, f)), abs(_dot(u, f)))
            for v in (r, u, f):
                worst_len = max(worst_len, abs(math.sqrt(_dot(v, v)) - 1.0))
            # Unity's basis satisfies right x up = forward.
            worst_hand = max(worst_hand, _maxdiff(_cross(r, u), f))
    check("axes stay orthogonal", worst_dot < 1e-9, "worst dot %.2e" % worst_dot)
    check("axes stay unit length", worst_len < 1e-9, "worst error %.2e" % worst_len)
    check("basis stays left-handed (right x up = forward)", worst_hand < 1e-9,
          "worst error %.2e" % worst_hand)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def test_position():
    """Position: latched origin, metric baselines, correct axis assignment."""
    print("\n5. Position and the latched origin")
    lat0, lon0 = 46.5191, 6.5668          # EPFL, the usual flying site
    solver = CameraPoseSolver()

    pos0, _, st0 = solver.pose_for({'lat': lat0, 'lon': lon0, 'alt': 20.0})
    check("first fix latches the origin", solver.origin == (lat0, lon0),
          "origin = %s" % (solver.origin,))
    check("first fix sits at the origin", _maxdiff(pos0, (0.0, 20.0, 0.0)) < 1e-9,
          "pos = (%.3f, %.3f, %.3f)" % pos0)
    check("a valid fix is marked POSE_VALID", st0 == POSE_VALID, "status = %d" % st0)

    # 100 m north and 100 m east, converted through the same equirectangular
    # constant the flocking code uses.
    dlat = 100.0 / EARTH_M_PER_DEG
    dlon = 100.0 / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))

    pos_n, _, _ = solver.pose_for({'lat': lat0 + dlat, 'lon': lon0, 'alt': 20.0})
    check("100 m north lands on +Z", _maxdiff(pos_n, (0.0, 20.0, 100.0)) < 1e-6,
          "pos = (%.3f, %.3f, %.3f)" % pos_n)

    pos_e, _, _ = solver.pose_for({'lat': lat0, 'lon': lon0 + dlon, 'alt': 20.0})
    check("100 m east lands on +X", _maxdiff(pos_e, (100.0, 20.0, 0.0)) < 1e-6,
          "pos = (%.3f, %.3f, %.3f)" % pos_e)

    pos_up, _, _ = solver.pose_for({'lat': lat0, 'lon': lon0, 'alt': 35.0})
    check("altitude lands on +Y", abs(pos_up[1] - 35.0) < 1e-9,
          "pos y = %.3f" % pos_up[1])

    # The origin must NOT follow the swarm. swarm_flocking recomputes lat_ref per
    # tick from the current fixes; doing that here would put every frame's poses in
    # a different frame while the scene plane sits in a fixed one.
    check("the origin does not drift with later fixes",
          solver.origin == (lat0, lon0), "origin = %s" % (solver.origin,))

    # Two drones, one solver: the baseline between them is what the stitcher's
    # geometry actually consumes, so it is the thing worth asserting in metres.
    a, _, _ = solver.pose_for({'lat': lat0, 'lon': lon0, 'alt': 20.0})
    b, _, _ = solver.pose_for({'lat': lat0 + dlat * 0.08, 'lon': lon0, 'alt': 20.0})
    baseline = math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))
    check("an 8 m baseline comes out 8 m", abs(baseline - 8.0) < 1e-4,
          "measured %.5f m" % baseline)


def test_no_fix():
    """No GPS lock must cost that view, not the frame."""
    print("\n6. Missing / sentinel GPS")
    solver = CameraPoseSolver()
    for label, telem in (
            ("(0, 0) sentinel", {'lat': 0.0, 'lon': 0.0, 'alt': 10.0}),
            ("missing fields", {'alt': 10.0}),
            ("NaN latitude", {'lat': float('nan'), 'lon': 6.5, 'alt': 10.0}),
            ("empty telemetry", {})):
        pos, quat, status = solver.pose_for(telem)
        check("%s yields an invalid pose, no exception" % label,
              status == 0 and pos == (0.0, 0.0, 0.0) and quat == (0.0, 0.0, 0.0, 1.0))
    check("a rejected fix does not latch the origin", solver.origin is None,
          "origin = %s" % (solver.origin,))


def test_facade_scenario():
    """End to end: a wall of drones facing a facade, the intended configuration."""
    print("\n7. Facade wall scenario")
    lat0, lon0 = 46.5191, 6.5668
    solver = CameraPoseSolver(origin=(lat0, lon0))
    dlon = 1.0 / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))

    # Five drones spread 8 m apart along an east-west line, all facing north at a
    # facade, gimbal level. Their `gimbal_yaw` carries the per-aircraft takeoff
    # slip the real fleet reports, so this is the configuration as telemetry
    # actually delivers it, not an idealised one.
    poses = []
    for i in range(5):
        east = (i - 2) * 8.0
        slip = MED_SLIP_DEG[i % len(MED_SLIP_DEG)]
        pos, quat, status = solver.pose_for({
            'lat': lat0, 'lon': lon0 + dlon * east, 'alt': 15.0,
            'heading': 0.0, 'gimbal_yaw': slip,
            'gimbal_pitch': 0.0, 'gimbal_roll': 0.0})
        poses.append((pos, quat, status))

    check("all five are posed", all(p[2] == POSE_VALID for p in poses))
    spread = [p[0][0] for p in poses]
    check("the wall spans 32 m east-west",
          abs((max(spread) - min(spread)) - 32.0) < 1e-3,
          "span %.4f m" % (max(spread) - min(spread)))
    check("the wall has no depth variation",
          max(abs(p[0][2]) for p in poses) < 1e-6,
          "worst |z| = %.2e m" % max(abs(p[0][2]) for p in poses))

    fwds = [quat_to_columns(p[1])[2] for p in poses]
    check("every camera faces north at the facade, despite the gimbal_yaw slip",
          all(_maxdiff(f, (0.0, 0.0, 1.0)) < 1e-9 for f in fwds))


def test_yaw_source():
    """The bearing comes off the compass, and a gimbal_yaw slip cannot reach it."""
    print("\n8. Bearing source (the gimbal-yaw slip)")

    check("the default source is the compass",
          CameraPoseSolver().yaw_source == 'heading',
          "yaw_source = %r" % CameraPoseSolver().yaw_source)

    base = {'heading': 137.0, 'gimbal_pitch': -30.0, 'gimbal_roll': 0.0}

    # The whole point: the same aircraft, the same physical camera bearing, with
    # every slip the 2026-08-11 fleet showed. The pose must not move.
    clean = quat_from_gimbal(137.0, -30.0, 0.0)
    worst = 0.0
    for slip in MED_SLIP_DEG + (0.0, 55.5, -22.1, 157.6):
        telem = dict(base, gimbal_yaw=137.0 + slip)
        worst = max(worst, _maxdiff(quat_from_gimbal(
            camera_bearing(telem), -30.0, 0.0), clean))
    check("a gimbal_yaw slip of up to 158 deg does not move the pose",
          worst < 1e-12, "worst quaternion error %.2e" % worst)

    # ...and the old reading is still reachable, or the slip could not be
    # re-measured after a fleet change.
    biased = camera_bearing(dict(base, gimbal_yaw=137.0 - 15.4), 'gimbal')
    check("yaw_source='gimbal' still reports the raw field",
          abs(biased - 121.6) < 1e-9, "bearing = %.4f deg" % biased)

    # A partial telem dict must not silently point the camera North: 0.0 is a
    # legal bearing, so a missing field has to fall through to the other one.
    check("a missing heading falls back to gimbal_yaw",
          abs(camera_bearing({'gimbal_yaw': 41.0}) - 41.0) < 1e-9)
    check("a missing gimbal_yaw falls back to heading",
          abs(camera_bearing({'heading': 41.0}, 'gimbal') - 41.0) < 1e-9)
    check("NaN is skipped like a missing field",
          abs(camera_bearing({'heading': float('nan'),
                              'gimbal_yaw': 41.0}) - 41.0) < 1e-9)

    bad = True
    try:
        CameraPoseSolver(yaw_source='compass')
        bad = False
    except ValueError:
        pass
    check("an unknown yaw_source is refused, not silently accepted", bad,
          "valid sources: %s" % (YAW_SOURCES,))

    # What the slip was worth, so the number in the docstring stays honest. A
    # rotation error dtheta puts two views Z*tan(dtheta) apart on the plane.
    spread = max(MED_SLIP_DEG) - min(MED_SLIP_DEG)
    seam = 34.255 * math.tan(math.radians(spread))
    check("the MED fleet's 18.2 deg spread was worth 11.3 m of seam",
          abs(spread - 18.2) < 0.05 and abs(seam - 11.3) < 0.1,
          "spread %.1f deg -> %.2f m at a 34.255 m standoff (%.0f px at f=525)"
          % (spread, seam, seam * 525.0 / 34.255))


def main():
    print("=" * 74)
    print("dji_camera_pose self-test")
    print("=" * 74)
    test_cardinals()
    test_pitch()
    test_against_independent_axes()
    test_orthonormal()
    test_position()
    test_no_fix()
    test_facade_scenario()
    test_yaw_source()

    print("\n" + "=" * 74)
    if _failures:
        print("FAILED (%d): %s" % (len(_failures), ", ".join(_failures)))
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
