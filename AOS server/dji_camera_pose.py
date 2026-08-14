"""
DJI telemetry -> Unity-world camera pose, for the VR sim's PLANAR stitcher.

Why this exists
---------------
The sim's `PLANAR` stitcher builds each view's homography in closed form from
`G = K R [e1 | e2 | (O - C)]`.  It therefore needs a per-frame camera *pose* --
position `C` and rotation `R`.  The real-drone path publishes only a compass
heading (the v1 12-byte block header), which is one scalar: no position at all,
and one of three rotation degrees of freedom.  That is why `PLANAR` cannot run
on real drones today, and this module is the missing piece.

Everything needed is already in the 17-field telemetry string: GPS position,
compass heading and the full gimbal attitude.  Nothing new has to be fetched
from the aircraft.

Frame convention -- read this before changing anything
------------------------------------------------------
The output is **Unity world, left-handed**, with

    +X = East      +Y = Up      +Z = North

Position is metres from a **latched** origin: the first valid fix seen, held for
the life of the object.  This is deliberately *not* how `swarm_flocking.py`
does it -- there `lat_ref/lon_ref` is the per-tick mean of the current fixes,
which drifts with the swarm.  That is correct for flocking, where only relative
distances matter, and wrong here: a drifting origin puts every frame's poses in
a different frame while the scene plane sits in a fixed one, so the mosaic would
crawl.  `response_monitor.py` latches its reference for the same reason.

The absolute origin is arbitrary and never has to be agreed with Unity.  The
plane is specified *relative to the formation* (`ScenePlaneMode.FormationRelative`
plus a standoff), so a common translation cancels out of the whole solve.

Rotation is the **gimbal** attitude for pitch and roll -- the camera is on the
gimbal, and on a stabilised gimbal the aircraft attitude is the part that has
already been removed.  The *bearing*, though, comes from the aircraft compass and
NOT from `gimbal_yaw`; see below, this is the whole reason `yaw_source` exists.

Sign conventions, and how much each is trusted
----------------------------------------------
* bearing -- compass bearing, 0 = North, increasing clockwise.  Unity's Y Euler is
  also clockwise-from-+Z looking down, and +Z is North here, so this maps across
  unchanged.  **Pinned by the self-test.**  Taken from telemetry `heading`, not
  `gimbal_yaw` -- see "The gimbal-yaw slip" below.
* `gimbal_pitch` -- 0 = horizontal, negative = pointing down.  Unity's X Euler is
  positive nose-*down*, so the sign flips.  **Pinned by the self-test.**  This
  field is faithful: across the 2026-08-11 clips a commanded -90.0 reads -90.0
  and a commanded +1.0 reads +0.9.
* `gimbal_roll` -- flips sign, by symmetry with pitch.  A stabilised gimbal holds
  this within a degree of zero, so it is the least consequential of the three and
  the only one the self-test can check for *self-consistency* only: there is no
  independent statement of DJI's roll sign here to check it against.  Verify on
  hardware before relying on a rolled camera.

The gimbal-yaw slip -- why the bearing is `heading`
---------------------------------------------------
Telemetry `gimbal_yaw` (the app's `GimbalKey.KeyGimbalAttitude` yaw) *is* a world
bearing -- regress it on `heading` across the 2026-08-11 clips' 187 deg of
rotation and the slope is 1.0000 with a 0.22 deg residual.  But it carries a
per-aircraft offset that is **re-rolled at every takeoff**:

* on the ground, pre-takeoff, `gimbal_yaw - heading` is 0 on every airframe on
  every date on file (|median| < 0.5 deg);
* airborne it is not, and it differs every flight -- across 30 airborne flights in
  `flight_logs/` drone 1's median spans -22 deg to +158 deg, drone 2 -15 deg to
  +50 deg, drone 3 -14 deg to +48 deg;
* it is a **random walk that steps during fast yaw and holds between steps**.
  Bucketing every fresh telemetry sample in `flight_logs/` by the yaw rate at the
  time, the offset moves by a median 1.30 deg (p95 18 deg) per sample above
  25 deg/s, against 0.10 deg (p95 2.0 deg) below it.  In
  `flight_20260811_161828` drone 2 sat at -9 deg, stepped to -15.5 deg in 0.2 s
  while the aircraft yawed 6 deg -- at a constant 14.7 m, nothing to do with
  takeoff -- and then held -15.5 +/- 0.1 deg for nine minutes and 620 deg of
  cumulative yaw travel.

Why it slips: the gimbal's yaw has **no absolute reference**.  Pitch and roll do
-- the accelerometer sees "down", so gravity continuously corrects them, which is
exactly why those two fields are faithful.  There is no equivalent for yaw (no
magnetometer in the gimbal), so an error made during a fast slew has nothing to
pull it back and simply persists.  It is the gimbal that moved, not the compass:
a magnetometer-referenced heading cannot hold a constant 15.5 deg error through
620 deg of rotation without a heading-dependent signature, and the slope of
1.0000 says there is none.

It differs *per aircraft* because the **maneuver history** differs, not the
hardware: each drone in a flock takes a different set of yaw slews, so each
accumulates a different sum of slips.  That is also why it is 0 before an
airframe has flown and different on every power cycle.

**No stored constant can fix this**, and not merely a per-fleet one -- even a
per-*flight* constant is unsafe, because the offset can step mid-flight, as
drone 2 did by 6 deg between two clips of one hover.  Nor is anything lost by
ignoring the field:
`SwarmActivity.sendGimbalCommand` issues an `ABSOLUTE_ANGLE` yaw of 0 at 20 Hz on
every frame and the gimbal never goes there, because the Mini 3 Pro's pan axis is
not user-controllable.  There is no genuine pan for `gimbal_yaw` to carry.

It matters because the error is *differential* and *rotational*.  Two cameras
whose bearings disagree by `dtheta` land `Z*tan(dtheta)` apart on the scene plane,
which no per-view translation can absorb, so the PLANAR pose refiner is
structurally unable to help.  At the MED facade's 34.255 m standoff the fleet's
18.2 deg spread was 11.3 m of seam -- larger than every other term in the error
budget combined, and 15.3 px of mosaic per metre (`f/Z = 525/34.255`).

`yaw_source='gimbal'` restores the old reading for anyone re-deriving the above.

The composition order below mirrors `Quaternion.Euler`, which applies Z, then X,
then Y.  `dji_pose_selftest.py` cross-checks the result against camera axes built
straight from compass/gimbal geometry -- a construction that shares no code with
this one, so a sign error cannot cancel out of both.

Pure module: no `ds_wrapper`, no shared memory, no I/O.  Python 3.7.
"""

import math

from olfati_saber import gps_to_local

__all__ = ["CameraPoseSolver", "quat_from_gimbal", "camera_bearing",
           "POSE_VALID", "YAW_SOURCES"]


# Bit 0 of the block header's poseStatus. Mirrors POSE_VALID in
# PyUniSharingFast.cs / StitcherThreading.py / PlanarStitcher.py.
POSE_VALID = 1 << 0

# A drone with no GPS lock reports exactly (0, 0). swarm_flocking.py screens for
# the same sentinel before letting a drone into the flock.
_GPS_SENTINEL_EPS = 1e-9

# Where the camera bearing comes from. 'heading' is correct on this fleet and is
# the default; 'gimbal' is the pre-fix reading, kept only so the gimbal-yaw slip
# documented above can be re-measured. See `camera_bearing`.
YAW_SOURCES = ('heading', 'gimbal')


def _quat_mul(a, b):
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _quat_axis(axis, deg):
    """Right-handed rotation of `deg` about a unit basis axis, as (x, y, z, w)."""
    h = math.radians(deg) * 0.5
    s, c = math.sin(h), math.cos(h)
    return (s if axis == 0 else 0.0,
            s if axis == 1 else 0.0,
            s if axis == 2 else 0.0,
            c)


def quat_from_gimbal(gimbal_yaw_deg, gimbal_pitch_deg, gimbal_roll_deg):
    """
    Gimbal attitude -> Unity `Transform.rotation` as (x, y, z, w).

    Composed in Unity's Euler order (Z, then X, then Y) so the result is exactly
    what `Quaternion.Euler(-pitch, yaw, -roll)` would produce in the editor.
    """
    q_yaw = _quat_axis(1, gimbal_yaw_deg)
    q_pitch = _quat_axis(0, -gimbal_pitch_deg)
    q_roll = _quat_axis(2, -gimbal_roll_deg)
    return _quat_mul(_quat_mul(q_yaw, q_pitch), q_roll)


def camera_bearing(telem, yaw_source='heading'):
    """
    The camera's compass bearing in degrees, from a telemetry dict.

    `yaw_source='heading'` (the default, and the only correct one on this fleet)
    reads the aircraft compass; `'gimbal'` reads `gimbal_yaw`, which slips by a
    per-aircraft amount at every takeoff -- see the module docstring.

    Falls back to the other field when the preferred one is missing, so a caller
    passing a partial telem dict gets the best bearing available rather than a
    silent 0 (which would point the camera North).
    """
    if yaw_source not in YAW_SOURCES:
        raise ValueError("yaw_source must be one of {}, got {!r}"
                         .format(YAW_SOURCES, yaw_source))
    order = ('heading', 'gimbal_yaw') if yaw_source == 'heading' \
        else ('gimbal_yaw', 'heading')
    for key in order:
        val = telem.get(key) if telem else None
        if val is None:
            continue
        val = float(val)
        # NaN fails every comparison, including against itself.
        if val == val:
            return val
    return 0.0


class CameraPoseSolver:
    """
    Turns telemetry samples into Unity-world camera poses about a latched origin.

    One instance per fleet, not per drone: every drone must share one origin or
    their positions are not in a common frame, which is the entire point.
    """

    def __init__(self, origin=None, yaw_source='heading'):
        """
        `origin` -- optional (lat, lon) to pin the frame explicitly.  Left None,
        the first valid fix passed to `pose_for` becomes the origin.  Pin it when
        comparing runs, or the same flight replays into a different frame each
        time (harmless for the mosaic, confusing for logs).

        `yaw_source` -- 'heading' (default, correct) or 'gimbal'.  Only change it
        to re-measure the gimbal-yaw slip; 'gimbal' produces the biased poses that
        broke PLANAR on the 2026-08-11 clips.
        """
        if yaw_source not in YAW_SOURCES:
            raise ValueError("yaw_source must be one of {}, got {!r}"
                             .format(YAW_SOURCES, yaw_source))
        self._origin = tuple(origin) if origin is not None else None
        self._yaw_source = yaw_source

    @property
    def origin(self):
        """The latched (lat, lon), or None if no valid fix has been seen yet."""
        return self._origin

    @property
    def yaw_source(self):
        """Which telemetry field the camera bearing is taken from."""
        return self._yaw_source

    @staticmethod
    def has_fix(telem):
        """True when this sample carries a usable GPS position."""
        if not telem:
            return False
        lat = telem.get('lat')
        lon = telem.get('lon')
        if lat is None or lon is None:
            return False
        if abs(lat) < _GPS_SENTINEL_EPS and abs(lon) < _GPS_SENTINEL_EPS:
            return False
        # NaN fails every comparison, including against itself.
        return lat == lat and lon == lon

    def pose_for(self, telem):
        """
        Telemetry dict -> `(pos_xyz, quat_xyzw, status)` in Unity world.

        `status` is the block header's poseStatus bitfield: `POSE_VALID` when the
        pose is usable, 0 when it is not.  A drone without a fix yields
        `((0,0,0), (0,0,0,1), 0)` rather than raising -- the consumer drops an
        unposed view and keeps the rest of the frame, which is the behaviour that
        makes one bad drone cost its own view instead of the whole panorama.
        """
        if not self.has_fix(telem):
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0

        if self._origin is None:
            self._origin = (telem['lat'], telem['lon'])

        north, east = gps_to_local(telem['lat'], telem['lon'],
                                   self._origin[0], self._origin[1])

        # alt is metres above the takeoff point. The datum is arbitrary and cancels:
        # the plane is derived relative to the formation, so a constant offset shared
        # by every drone moves the plane with them.
        alt = float(telem.get('alt', 0.0) or 0.0)

        # Pitch and roll come off the gimbal: the camera rides it, and on a
        # stabilised gimbal the aircraft attitude is the part already removed.
        # The bearing does NOT -- `gimbal_yaw` slips at every takeoff, so it is
        # the compass that carries it. See "The gimbal-yaw slip" up top.
        pos = (east, alt, north)
        quat = quat_from_gimbal(camera_bearing(telem, self._yaw_source),
                                float(telem.get('gimbal_pitch', 0.0) or 0.0),
                                float(telem.get('gimbal_roll', 0.0) or 0.0))
        return pos, quat, POSE_VALID
