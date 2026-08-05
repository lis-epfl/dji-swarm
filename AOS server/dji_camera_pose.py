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

Rotation is the **gimbal** attitude, not the aircraft attitude: the camera is on
the gimbal, and on a stabilised gimbal the two differ by exactly the stabilisation
the geometry must not double-count.

Sign conventions, and how much each is trusted
----------------------------------------------
* `gimbal_yaw`  -- compass bearing, 0 = North, increasing clockwise.  Unity's Y
  Euler is also clockwise-from-+Z looking down, and +Z is North here, so this
  maps across unchanged.  **Pinned by the self-test.**
* `gimbal_pitch` -- 0 = horizontal, negative = pointing down.  Unity's X Euler is
  positive nose-*down*, so the sign flips.  **Pinned by the self-test.**
* `gimbal_roll` -- flips sign, by symmetry with pitch.  A stabilised gimbal holds
  this within a degree of zero, so it is the least consequential of the three and
  the only one the self-test can check for *self-consistency* only: there is no
  independent statement of DJI's roll sign here to check it against.  Verify on
  hardware before relying on a rolled camera.

The composition order below mirrors `Quaternion.Euler`, which applies Z, then X,
then Y.  `dji_pose_selftest.py` cross-checks the result against camera axes built
straight from compass/gimbal geometry -- a construction that shares no code with
this one, so a sign error cannot cancel out of both.

Pure module: no `ds_wrapper`, no shared memory, no I/O.  Python 3.7.
"""

import math

from olfati_saber import gps_to_local

__all__ = ["CameraPoseSolver", "quat_from_gimbal", "POSE_VALID"]


# Bit 0 of the block header's poseStatus. Mirrors POSE_VALID in
# PyUniSharingFast.cs / StitcherThreading.py / PlanarStitcher.py.
POSE_VALID = 1 << 0

# A drone with no GPS lock reports exactly (0, 0). swarm_flocking.py screens for
# the same sentinel before letting a drone into the flock.
_GPS_SENTINEL_EPS = 1e-9


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


class CameraPoseSolver:
    """
    Turns telemetry samples into Unity-world camera poses about a latched origin.

    One instance per fleet, not per drone: every drone must share one origin or
    their positions are not in a common frame, which is the entire point.
    """

    def __init__(self, origin=None):
        """
        `origin` -- optional (lat, lon) to pin the frame explicitly.  Left None,
        the first valid fix passed to `pose_for` becomes the origin.  Pin it when
        comparing runs, or the same flight replays into a different frame each
        time (harmless for the mosaic, confusing for logs).
        """
        self._origin = tuple(origin) if origin is not None else None

    @property
    def origin(self):
        """The latched (lat, lon), or None if no valid fix has been seen yet."""
        return self._origin

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

        # Gimbal, not aircraft: the camera rides the gimbal, and on a stabilised one
        # the aircraft attitude is the part that has already been removed.
        pos = (east, alt, north)
        quat = quat_from_gimbal(float(telem.get('gimbal_yaw', 0.0) or 0.0),
                                float(telem.get('gimbal_pitch', 0.0) or 0.0),
                                float(telem.get('gimbal_roll', 0.0) or 0.0))
        return pos, quat, POSE_VALID
