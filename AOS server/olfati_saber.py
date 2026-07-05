"""
Olfati-Saber flocking math + virtual obstacles / geofence geometry
==================================================================
Port of the Unity sim's `OlfatiSaber.cs` (vr_swarm_simulation) for the real
DJI swarm, extracted from swarm_flocking.py so the growing algorithm lives in
one importable place:

  - `OlfatiSaber`        — cohesion + velocity-consensus correction
                           (GetSwarmAcceleration, moved verbatim from
                           swarm_flocking.py).
  - `ObstacleAvoidance`  — the C# `GetObstacleForce` β-agent term, 2D, for
                           axis-aligned rectangular virtual obstacles and one
                           geofence polygon repelling inward from its edges.
  - Geometry helpers     — lat/lon → local-metres conversion for shapes,
                           closest-point-on-rect/polygon, point-in-polygon.

Frames & units (same conventions as swarm_flocking.py):
  - Shapes are stored/transported in lat/lon degrees (GUI-native, persistence-
    stable) and converted per control tick to local (north, east) metres with
    `gps_to_local` against the SAME per-tick reference point as the drone
    positions, so drones and shapes always share one frame.
  - All force math runs in world N/E; outputs are m/s velocity corrections
    added to the joystick's desired velocity (DJI VS GROUND/VELOCITY mode —
    never rotate world→body here).
  - Like the cohesion math, the β-agent kernel runs distances in *scaled*
    units (physical metres / scale, scale = 10). ObstacleAvoidance takes its
    d_obs/r0_obs parameters in PHYSICAL metres and divides by `scale`
    internally: Unity's raw d_obs = 5.0 is 5 *scaled* units = 50 m physical,
    which next to a real d_ref of 5-10 m physical would shove drones around
    from half a football pitch away.

Shape validation + shapes.json persistence also live here (bottom of the
file) so the controller (swarm_flocking.command_listener) and the GUI server
(swarm_gui.py, which must be able to save shapes with NO controller running)
share one implementation and can never drift apart.

Pure Python (stdlib only), no ds_wrapper import — runs and is testable on any
Python. Must stay Python 3.7-compatible (the controller runs cp37).
"""

import json
import math
import os


# Equirectangular-projection constant for converting (lat, lon) deltas to
# local meters. Valid for swarm scales (tens of meters).
EARTH_M_PER_DEG = 111320.0


# ---------- frame conversion / small vector helpers ----------

def gps_to_local(lat, lon, lat_ref, lon_ref):
    """Convert (lat, lon) in degrees to local (north, east) meters about
    a reference point. Equirectangular approximation — fine for small swarms."""
    cos_lat = math.cos(math.radians(lat_ref))
    north = (lat - lat_ref) * EARTH_M_PER_DEG
    east  = (lon - lon_ref) * EARTH_M_PER_DEG * cos_lat
    return north, east


def clamp_mag2(vx, vy, max_mag):
    """Clamp the magnitude of a 2D vector to max_mag."""
    mag = math.hypot(vx, vy)
    if mag > max_mag and mag > 0:
        s = max_mag / mag
        return vx * s, vy * s
    return vx, vy


# ---------- shape geometry (local N/E metres) ----------

def rect_to_ne(rect, lat_ref, lon_ref):
    """Convert an obstacle dict {'lat_min','lat_max','lon_min','lon_max'} to
    (n_min, n_max, e_min, e_max) local metres. gps_to_local is monotonic in
    lat and lon, so a lat/lon-axis-aligned rect stays N/E-axis-aligned."""
    n_min, e_min = gps_to_local(rect["lat_min"], rect["lon_min"], lat_ref, lon_ref)
    n_max, e_max = gps_to_local(rect["lat_max"], rect["lon_max"], lat_ref, lon_ref)
    if n_min > n_max:
        n_min, n_max = n_max, n_min
    if e_min > e_max:
        e_min, e_max = e_max, e_min
    return n_min, n_max, e_min, e_max


def polygon_to_ne(vertices, lat_ref, lon_ref):
    """Convert [[lat, lon], ...] to [(n, e), ...] local metres."""
    return [gps_to_local(v[0], v[1], lat_ref, lon_ref) for v in vertices]


def closest_point_on_rect(n, e, rect_ne):
    """Closest point on the BOUNDARY of an axis-aligned rect to (n, e).

    Returns (cn, ce, inside). Outside the rect this is the standard
    coordinate clamp. Inside, it is the projection onto the nearest of the
    four edges — never the query point itself, so the direction vector
    (closest - pos) is always well-defined."""
    n_min, n_max, e_min, e_max = rect_ne
    inside = (n_min <= n <= n_max) and (e_min <= e <= e_max)
    if not inside:
        cn = min(max(n, n_min), n_max)
        ce = min(max(e, e_min), e_max)
        return cn, ce, False
    # Inside: distance to each of the four edges; project onto the nearest.
    d_n_min = n - n_min
    d_n_max = n_max - n
    d_e_min = e - e_min
    d_e_max = e_max - e
    m = min(d_n_min, d_n_max, d_e_min, d_e_max)
    if m == d_n_min:
        return n_min, e, True
    if m == d_n_max:
        return n_max, e, True
    if m == d_e_min:
        return n, e_min, True
    return n, e_max, True


def point_in_polygon(n, e, poly_ne):
    """Ray-casting (even-odd) point-in-polygon test in local N/E metres.

    A point exactly on an edge or vertex counts as INSIDE — a geofence breach
    must not fire from float jitter while a drone sits on the line."""
    num = len(poly_ne)
    if num < 3:
        return False
    inside = False
    j = num - 1
    for i in range(num):
        ni, ei = poly_ne[i]
        nj, ej = poly_ne[j]
        # On-edge check: collinear and within the segment's bounding box.
        cross = (nj - ni) * (e - ei) - (ej - ei) * (n - ni)
        if (abs(cross) < 1e-9
                and min(ni, nj) - 1e-9 <= n <= max(ni, nj) + 1e-9
                and min(ei, ej) - 1e-9 <= e <= max(ei, ej) + 1e-9):
            return True
        if (ei > e) != (ej > e):
            n_cross = (nj - ni) * (e - ei) / (ej - ei) + ni
            if n < n_cross:
                inside = not inside
        j = i
    return inside


def closest_point_on_segment(n, e, a, b):
    """Closest point to (n, e) on segment a→b (each an (n, e) tuple)."""
    an, ae = a
    bn, be = b
    dn = bn - an
    de = be - ae
    seg_len2 = dn * dn + de * de
    if seg_len2 <= 0.0:
        return an, ae
    t = ((n - an) * dn + (e - ae) * de) / seg_len2
    t = max(0.0, min(1.0, t))
    return an + t * dn, ae + t * de


def closest_point_on_polygon(n, e, poly_ne):
    """Closest point on the polygon BOUNDARY to (n, e).

    Returns (cn, ce, dist_m). Assumes len(poly_ne) >= 2."""
    best = None
    best_d = float("inf")
    num = len(poly_ne)
    for i in range(num):
        cn, ce = closest_point_on_segment(n, e, poly_ne[i], poly_ne[(i + 1) % num])
        d = math.hypot(cn - n, ce - e)
        if d < best_d:
            best_d = d
            best = (cn, ce)
    return best[0], best[1], best_d


# ---------- cohesion + velocity consensus (mirrors OlfatiSaber.cs) ----------

class OlfatiSaber:
    """2D port of the Unity OlfatiSaber component (vr_swarm_simulation
    Assets/Scripts/swarm/OlfatiSaber.cs). Method names and formulas mirror the
    C# one-to-one — when editing, diff against the C# file, NOT the paper.
    Stateless math — one instance can serve the whole swarm; per-drone state
    comes through args.

    Not ported from the C# on purpose:
      - Is3D=false altitude correction (c_altitude_2d pull toward the mean
        neighbour altitude): the real drones fly a shared ABSOLUTE altitude
        setpoint (DJI VS VerticalControlMode.POSITION), which already does
        this job; adding the Unity term would fight that channel.
      - gamma, lambda_obs, MaxMigrationDistance: declared in the C# but not
        used by GetSwarmAcceleration.
      - GetObstacleForce lives in ObstacleAvoidance below (the Unity version
        queries the physics engine; here the shapes are explicit geometry).
    """

    def __init__(self, r0_coh=150.0, delta=0.1, a=0.9, b=1.5, c=0.0,
                 c_vm=0.0, scale=10.0):
        self.r0_coh = r0_coh
        self.delta  = delta
        self.a = a
        self.b = b
        self.c = c
        self.c_vm = c_vm
        self.scale = scale

    # C# GetCohesionIntensity(r, ref_d)
    def GetCohesionIntensity(self, r, ref_d):
        diff = r - ref_d
        return (((self.a + self.b) / 2.0)
                * (math.sqrt(1 + (diff + self.c) ** 2) - math.sqrt(1 + self.c ** 2))
                + ((self.a - self.b) * diff) / 2.0)

    # C# GetCohesionIntensityDerivative(r, ref_d)
    def GetCohesionIntensityDerivative(self, r, ref_d):
        diff = r - ref_d
        return (((self.a + self.b) / 2.0)
                * (diff + self.c) / math.sqrt(1 + (diff + self.c) ** 2)
                + (self.a - self.b) / 2.0)

    # C# GetNeighbourWeight(r, r0)
    def GetNeighbourWeight(self, r, r0):
        r_ratio = r / r0
        if r_ratio < self.delta:
            return 1.0
        if r_ratio < 1.0:
            arg = math.pi * (r_ratio - self.delta) / (1 - self.delta)
            return (0.5 * (1.0 + math.cos(arg))) ** 2
        return 0.0

    # C# GetNeighbourWeightDerivative(r, r0)
    def GetNeighbourWeightDerivative(self, r, r0):
        r_ratio = r / r0
        if r_ratio < self.delta:
            return 0.0
        if r_ratio < 1.0:
            arg = math.pi * (r_ratio - self.delta) / (1 - self.delta)
            return 0.5 * (-math.pi) / (1 - self.delta) * (1 + math.cos(arg)) * math.sin(arg)
        return 0.0

    # C# GetCohesionForce(r, ref_d, r0):
    #   1/r0 * GetNeighbourWeightDerivative * GetCohesionIntensity
    #   + GetNeighbourWeight * GetCohesionIntensityDerivative
    def GetCohesionForce(self, r, ref_d, r0=None):
        if r0 is None:
            r0 = self.r0_coh
        neighbour_weight_derivative = self.GetNeighbourWeightDerivative(r, r0)
        cohesion_intensity = self.GetCohesionIntensity(r, ref_d)
        neighbour_weight = self.GetNeighbourWeight(r, r0)
        cohesion_intensity_derivative = self.GetCohesionIntensityDerivative(r, ref_d)
        return (1.0 / r0 * neighbour_weight_derivative * cohesion_intensity
                + neighbour_weight * cohesion_intensity_derivative)

    def GetSwarmAcceleration(self, self_pos_ne, self_vel_ne, neighbours, d_ref):
        """Return the world-frame swarm correction for one drone, to be ADDED
        to the joystick's desired velocity before sending to DJI VS.

        Mirrors `OlfatiSaber.cs::GetSwarmAcceleration` (2D, minus the parts
        listed in the class docstring):
            return velocityConsensus + cohesion
        where velocityConsensus sums c_vm*(v_neighbour - v_self) over neighbours
        (pulling each drone toward its neighbours' velocities), and cohesion
        sums GetCohesionForce along each relative-position unit vector.

        The desired joystick velocity is NOT mixed in here — it goes straight
        to DJI VS via set_velocity, with this correction added on top.

        Args:
            self_pos_ne:  (n, e) meters
            self_vel_ne:  (vn, ve) m/s (world frame)
            neighbours:   iterable of ((n, e), (vn, ve)) tuples for other drones
            d_ref:        desired inter-drone spacing in scaled units
        """
        sn, se = self_pos_ne
        vn, ve = self_vel_ne

        consensus_n = 0.0
        consensus_e = 0.0
        coh_n = 0.0
        coh_e = 0.0

        for n_pos, n_vel in neighbours:
            # Velocity consensus: pull toward each neighbour's velocity
            consensus_n += self.c_vm * (n_vel[0] - vn)
            consensus_e += self.c_vm * (n_vel[1] - ve)

            # Cohesion: spacing potential along the relative-position unit
            # vector. distance = relativePosition.magnitude / ScaleFactor
            rel_n = n_pos[0] - sn
            rel_e = n_pos[1] - se
            rel_mag = math.hypot(rel_n, rel_e)
            if rel_mag < 1e-6:
                # Unity's Vector3.normalized is the zero vector here too
                continue
            distance = rel_mag / self.scale
            force = self.GetCohesionForce(distance, d_ref)
            ux = rel_n / rel_mag
            uy = rel_e / rel_mag
            coh_n += force * ux
            coh_e += force * uy

        return consensus_n + coh_n, consensus_e + coh_e

    # Back-compat alias (pre-C#-naming callers)
    compute = GetSwarmAcceleration


# ---------- β-agent obstacle / geofence repulsion ----------

class ObstacleAvoidance:
    """2D port of Unity `OlfatiSaber.cs::GetObstacleForce` for axis-aligned
    rectangular virtual obstacles and one geofence polygon (whose edges repel
    inward with the same β-agent kernel). Stateless — one instance serves the
    whole swarm.

    UNITS: `d_obs_m` (repulsion cutoff) and `r0_obs_m` (detection radius) are
    PHYSICAL metres — what the operator thinks in — divided by `scale`
    internally so the kernel runs in the same scaled regime as the cohesion
    math. With the defaults (d_obs_m=5, c_obs=4.3) the push is ~6.2 m/s at
    contact, decaying smoothly to exactly 0 at 5 m — commensurate with
    MAX_CMD_MPS = 6.0 and the ~1 m/s cohesion corrections.

    The C# ObsVel virtual-agent velocity term uses the SAME `c_vm` gain as the
    cohesion velocity consensus (OlfatiSaber.cs line `c_obs * ObsCoh +
    c_vm * ObsVel`); pass the swarm's c_vm here to match. The real flights run
    c_vm = 0, so it is inert unless deliberately enabled.
    """

    def __init__(self, d_obs_m=5.0, r0_obs_m=6.0, c_obs=4.3,
                 c_vm=0.0, delta=0.1, scale=10.0):
        self.scale    = scale
        self.d_obs    = d_obs_m / scale     # kernel cutoff, scaled units
        self.r0_obs   = r0_obs_m / scale    # detection radius, scaled units
        self.c_obs    = c_obs
        self.c_vm     = c_vm
        self.delta    = delta

    # C# Sigma1(z) = z / sqrt(1 + z²)
    @staticmethod
    def Sigma1(z):
        return z / math.sqrt(1.0 + z * z)

    # C# GetNeighbourWeight(r, r0) — same bump as OlfatiSaber's (kept local so
    # this class stays self-contained and independently testable).
    def GetNeighbourWeight(self, r, r0):
        r_ratio = r / r0
        if r_ratio < self.delta:
            return 1.0
        if r_ratio < 1.0:
            arg = math.pi * (r_ratio - self.delta) / (1 - self.delta)
            return (0.5 * (1.0 + math.cos(arg))) ** 2
        return 0.0

    def GetObstacleRepulsion(self, r):
        """C# GetObstacleRepulsion(r):
        GetNeighbourWeight(r, d_obs)·(Sigma1(r − d_obs) − 1). Always <= 0;
        exactly 0 for r >= d_obs (the bump cuts). `r` in scaled units."""
        return (self.GetNeighbourWeight(r, self.d_obs)
                * (self.Sigma1(r - self.d_obs) - 1.0))

    def beta_force(self, pos_ne, vel_ne, closest_ne, inside=False):
        """One β-agent contribution from a closest obstacle point.

        Args:
            pos_ne:     drone (n, e) metres
            vel_ne:     drone (vn, ve) m/s world frame
            closest_ne: (n, e) of the closest point on the obstacle boundary
            inside:     True when the drone is INSIDE a rect — repulsion is
                        then maximal and flipped to push OUT toward that
                        nearest-edge point instead of away from it.

        Returns (fn, fe) world N/E m/s: c_obs·ObsCoh + c_vm·ObsVel
        (the C# GetObstacleForce return line).
        """
        dn = closest_ne[0] - pos_ne[0]
        de = closest_ne[1] - pos_ne[1]
        dist_m = math.hypot(dn, de)
        if inside:
            r = 0.0
        else:
            r = dist_m / self.scale
            if r >= self.r0_obs:
                return 0.0, 0.0
        if dist_m > 1e-9:
            un = dn / dist_m
            ue = de / dist_m
        else:
            return 0.0, 0.0  # degenerate (on the boundary): no defined direction
        rep = self.GetObstacleRepulsion(r)   # <= 0
        if inside:
            rep = -rep                   # push TOWARD the nearest edge (= out)
        coh_n = rep * un
        coh_e = rep * ue

        # Unity's virtual β-agent velocity term (velocities scaled like the
        # positions so s/s_der match the C# scaled frame; inert at gain 0).
        vel_n = 0.0
        vel_e = 0.0
        if self.c_vm != 0.0:
            vn_s = vel_ne[0] / self.scale
            ve_s = vel_ne[1] / self.scale
            s = 1.0 / (r + 1.0)
            s_der = (vn_s * un + ve_s * ue) / ((1.0 + r) ** 2)
            # vel_obs = s*v - (s_der/s)*u ; ObsVel = vel_obs - v  (scaled), then
            # back to physical m/s.
            vel_n = ((s - 1.0) * vn_s - (s_der / s) * un) * self.scale
            vel_e = ((s - 1.0) * ve_s - (s_der / s) * ue) * self.scale

        return (self.c_obs * coh_n + self.c_vm * vel_n,
                self.c_obs * coh_e + self.c_vm * vel_e)

    def rect_force(self, pos_ne, vel_ne, rect_ne):
        """Repulsion from one axis-aligned rect (n_min, n_max, e_min, e_max)."""
        cn, ce, inside = closest_point_on_rect(pos_ne[0], pos_ne[1], rect_ne)
        return self.beta_force(pos_ne, vel_ne, (cn, ce), inside=inside)

    def fence_force(self, pos_ne, vel_ne, poly_ne):
        """Soft inward repulsion from the geofence edges, ONLY while inside
        the polygon. The closest boundary point acts as the β-agent: u points
        toward the boundary and repulsion() <= 0 flips the force inward — the
        same formula as an obstacle seen from outside. Outside the polygon
        returns (0, 0): the hard breach handling is the caller's job, not a
        force."""
        if len(poly_ne) < 3:
            return 0.0, 0.0
        if not point_in_polygon(pos_ne[0], pos_ne[1], poly_ne):
            return 0.0, 0.0
        cn, ce, _ = closest_point_on_polygon(pos_ne[0], pos_ne[1], poly_ne)
        return self.beta_force(pos_ne, vel_ne, (cn, ce), inside=False)

    def GetObstacleForce(self, pos_ne, vel_ne, rects_ne, fence_ne=None):
        """Total obstacle + geofence force for one drone (C# GetObstacleForce;
        the Unity version finds obstacles via Physics.OverlapSphere — here the
        rects/fence come in as explicit geometry).

        Args:
            pos_ne:   (n, e) metres
            vel_ne:   (vn, ve) m/s world frame
            rects_ne: iterable of (n_min, n_max, e_min, e_max) rects
            fence_ne: [(n, e), ...] geofence polygon or None

        Returns (fn, fe) world N/E m/s to be added to the drone's command
        (before the caller's magnitude clamp)."""
        total_n = 0.0
        total_e = 0.0
        for rect in rects_ne:
            fn, fe = self.rect_force(pos_ne, vel_ne, rect)
            total_n += fn
            total_e += fe
        if fence_ne:
            fn, fe = self.fence_force(pos_ne, vel_ne, fence_ne)
            total_n += fn
            total_e += fe
        return total_n, total_e

    # Back-compat alias (pre-C#-naming callers)
    compute = GetObstacleForce


# ---------- shape validation + shapes.json persistence ----------
# Shared by swarm_flocking.command_listener (controller) and swarm_gui.py
# (which persists shapes even when no controller is running). File format:
#   {"version": 1,
#    "obstacles": [{"id":1,"lat_min":..,"lat_max":..,"lon_min":..,"lon_max":..}],
#    "geofence": [[lat, lon], ...] | null}

DEFAULT_SHAPES_FILE = "shapes.json"
MAX_OBSTACLES = 32
MAX_FENCE_VERTICES = 100
# Reject rectangles thinner than this on either axis (a stray click-drag must
# not create an invisible sliver that still repels drones).
MIN_OBSTACLE_SPAN_M = 0.5


def finite_latlon(lat, lon):
    return (math.isfinite(lat) and math.isfinite(lon)
            and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0)


def normalize_obstacle(lat1, lon1, lat2, lon2):
    """Validate two obstacle corners and return the normalized rect dict
    (WITHOUT an id — the caller assigns it), or None when rejected
    (non-finite, out of range, or thinner than MIN_OBSTACLE_SPAN_M)."""
    try:
        lat1 = float(lat1); lon1 = float(lon1)
        lat2 = float(lat2); lon2 = float(lon2)
    except (TypeError, ValueError):
        return None
    if not (finite_latlon(lat1, lon1) and finite_latlon(lat2, lon2)):
        return None
    lat_min, lat_max = min(lat1, lat2), max(lat1, lat2)
    lon_min, lon_max = min(lon1, lon2), max(lon1, lon2)
    span_n = (lat_max - lat_min) * EARTH_M_PER_DEG
    span_e = (lon_max - lon_min) * EARTH_M_PER_DEG * math.cos(math.radians(lat_min))
    if span_n < MIN_OBSTACLE_SPAN_M or span_e < MIN_OBSTACLE_SPAN_M:
        return None
    return {"lat_min": lat_min, "lat_max": lat_max,
            "lon_min": lon_min, "lon_max": lon_max}


def validate_fence(vertices):
    """Validate a geofence vertex list and return it as [[lat, lon], ...]
    floats, or None when rejected (not a list, <3 or >MAX_FENCE_VERTICES
    vertices, non-finite or out-of-range values)."""
    if not isinstance(vertices, list) or not (3 <= len(vertices) <= MAX_FENCE_VERTICES):
        return None
    try:
        fence = [[float(v[0]), float(v[1])] for v in vertices]
    except (TypeError, ValueError, IndexError):
        return None
    if not all(finite_latlon(la, lo) for la, lo in fence):
        return None
    return fence


def load_shapes(path):
    """Load (obstacles, geofence) from `path`. Missing or corrupt file is not
    fatal — returns ([], None) with a warning so startup never breaks on it."""
    if not os.path.isfile(path):
        return [], None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        obstacles = []
        for ob in data.get("obstacles") or []:
            obstacles.append({
                "id": int(ob["id"]),
                "lat_min": float(ob["lat_min"]), "lat_max": float(ob["lat_max"]),
                "lon_min": float(ob["lon_min"]), "lon_max": float(ob["lon_max"]),
            })
        fence = data.get("geofence")
        if fence is not None:
            fence = [[float(v[0]), float(v[1])] for v in fence]
            if len(fence) < 3:
                fence = None
        return obstacles, fence
    except (OSError, ValueError, KeyError, TypeError) as e:
        print("WARNING: could not load shapes file %s: %s — starting with no "
              "obstacles/geofence" % (path, e))
        return [], None


def save_shapes(path, obstacles, geofence):
    """Atomically persist the current shapes (tmp file + os.replace). Each
    process calls this only from one thread (the controller's command_listener
    / the GUI's locked shapes store); when the controller and GUI both save
    the same edit the contents are identical, so last-writer-wins is safe."""
    tmp = path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"version": 1, "obstacles": obstacles,
                       "geofence": geofence}, f, indent=2)
        os.replace(tmp, path)
    except OSError as e:
        print("WARNING: could not save shapes file %s: %s" % (path, e))
