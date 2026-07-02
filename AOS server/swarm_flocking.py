"""
LIS_Swarm Olfati-Saber Flocking Controller
==========================================
Port of the Unity `OlfatiSaber.cs::GetSwarmAcceleration` swarm flocking
algorithm to drive a fleet of DJI Mini 3 Pros from a single joystick.

Architecture (mirrors the Unity sim's SwarmAlgorithm + VelocityControl pair):
  - Joystick desired velocity (world N/E) → straight into the VS pitch/roll
    fields. The Android app sets DJI VS to GROUND/VELOCITY mode
    (`FlightCoordinateSystem.GROUND` in SwarmActivity.java), so pitch = north
    m/s and roll = east m/s; the drone does its own world→body rotation
    internally using its heading. We must NOT rotate world→body in Python
    or the rotation gets applied twice (this was a real bug — symptom: at
    headings other than 0, the drone moved in scrambled directions).
  - Olfati-Saber → only produces a *correction* per drone: velocity consensus
    with neighbours (c_vm * Σ (v_j - v_i)) + cohesion (σ-norm spacing potential).
  - We add the correction to v_des, clamp magnitude, send as (pitch, roll).

Joystick → swarm mapping:
    linear.x   → desired north velocity   (m/s, world frame)
    linear.y   → desired east  velocity   (m/s, world frame)
    linear.z   → climb (integrated into shared target altitude)
    angular.z  → yaw rate (feed-forward; a shared target heading is held per
                 drone via heading_hold_rate → smooth yaw RATE to the DJI VS).
                 Ignored with --heading convexhull: there the per-drone target
                 heading comes from the swarm's convex hull instead (boundary
                 drones face outward, interior drones hold heading — port of
                 the Unity sim's GLOBAL_CONVEXHULL, see heading_convexhull.py)
    angular.x  → d_ref, linear map [0.6, 1.4] → scaled [0.3, 0.8]
                 (≈ physical [3, 8] m at ScaleFactor = 10)
    switch s1  → toggle ENABLE_VS / DISABLE_VS for all drones (rising edge)
    switch s2  → LAND all drones (rising edge)
    PC key 'q' → zero velocities, hold current position, disable VS, exit

Obstacle avoidance from the C# original is intentionally omitted.

Note on ScaleFactor: the OlfatiSaber math uses the Unity-sim tuning verbatim
(ScaleFactor = 10.0). To keep the cohesion potential in the same regime, the
`d_ref` we feed it is in *scaled* units (Unity's convention) — physical
spacing ≈ d_ref * ScaleFactor. The joystick angular.x is mapped to scaled
d_ref ∈ [0.3, 0.8], which corresponds to a physical d_ref ∈ [3, 8] m at
ScaleFactor = 10.

Usage:
    python swarm_flocking.py --drones 3
    python swarm_flocking.py --drones 2 --c-vm 0.5     # gentler velocity matching
    python swarm_flocking.py --drones 3 --slow         # slow test mode (30% speed)
    python swarm_flocking.py --drones 3 --slow 0.5     # slow test mode at 50% speed
    python swarm_flocking.py --drones 3 --dry-run      # print VS commands, do not transmit
    python swarm_flocking.py --drones 3 --heading convexhull   # hull-facing headings
"""

import argparse
import json
import math
import msvcrt    # Windows console: non-blocking keyboard read for the 'q' stop
import socket
import sys
import threading
import time

import ds_wrapper as w

# Force line-buffered stdout so we actually see startup prints in the PowerShell
# launcher (block-buffered stdout has made debug sessions painful before).
try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:
    pass  # Python <3.7

from udp_joystick_receiver import JoystickReceiver
from flight_logger import FlightLogger
from swarm_telemetry_feed import (
    TelemetryFeedPublisher,
    DEFAULT_GUI_HOST,
    DEFAULT_GUI_PORT,
)
from heading_convexhull import ConvexHullHeading
from joystick_controller import (
    DroneController,
    SwarmController,
    _deadzone,
    _normalize_heading_180,
    heading_hold_rate,
    MAX_PITCH_MPS,
    MAX_ROLL_MPS,
    YAW_RATE_DEG_S,
    VERT_RATE_MPS,
    MIN_ALT_M,
    MAX_ALT_M,
    INITIAL_ALT_M,
    SLOW_DEFAULT_SCALE,
)


# Cap on the world-frame velocity command (v_des + swarm_correction) before
# rotating into body frame. Joystick alone produces up to ~4.24 m/s at the
# diagonal; this leaves ~1.8 m/s of headroom for the swarm correction before
# clamping. DroneController.set_velocity also clamps body pitch/roll to ±15
# m/s independently.
MAX_CMD_MPS = 6.0

# Gimbal pitch (tilt) target shared between the GUI slider and the swarm.
# Bounded to the DJI Mini 3 Pro's controllable tilt range: -90° (straight down)
# to +60° (up). The GUI (swarm_gui.py) forwards slider changes as
# {"action":"gimbal","value":deg} to command_listener, which stores the clamped
# target in swarm_meta; run() applies it to every drone whenever it changes (the
# 20 Hz send loop then relays it to the flight controller — gimbal only actually
# moves while VS is enabled).
GIMBAL_PITCH_MIN = -90.0
GIMBAL_PITCH_MAX = 60.0
DEFAULT_GIMBAL_PITCH = -10.0

# Equirectangular-projection constants for converting (lat, lon) deltas to
# local meters. Valid for swarm scales (tens of meters).
EARTH_M_PER_DEG = 111320.0

# Drones with fewer satellites than this are excluded from the swarm snapshot.
MIN_SAT_COUNT = 6

# Floor for the shared altitude target seeded when swarming starts. The seed is
# the average altitude of the drones we actually have telemetry from; if that
# average is below this (e.g. drones still on the ground) we climb to this instead.
START_ALT_FLOOR_M = 3.0


# ---------- helpers ----------

def gps_to_local(lat, lon, lat_ref, lon_ref):
    """Convert (lat, lon) in degrees to local (north, east) meters about
    a reference point. Equirectangular approximation — fine for small swarms."""
    cos_lat = math.cos(math.radians(lat_ref))
    north = (lat - lat_ref) * EARTH_M_PER_DEG
    east  = (lon - lon_ref) * EARTH_M_PER_DEG * cos_lat
    return north, east


def body_to_world(v_forward, v_right, heading_deg):
    """Rotate body-frame (forward, right) velocity into world (north, east).

    Used only for telemetry interpretation (--vel-frame body), since DJI's
    KeyAircraftVelocity is documented as NED ground-frame and the GROUND/
    VELOCITY VS mode means commands going OUT don't need any rotation either."""
    theta = math.radians(heading_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    v_n = v_forward * cos_t - v_right * sin_t
    v_e = v_forward * sin_t + v_right * cos_t
    return v_n, v_e


def d_ref_from_ax(ax, scale=10.0):
    """Map angular.x ∈ [0.6, 1.4] → scaled d_ref ∈ [0.3, 0.8].

    The OlfatiSaber math operates in scaled units (ScaleFactor=10 by default),
    so the returned d_ref is divided by `scale` to match. Physical spacing
    sits roughly at `d_ref * scale + 2.58 m` (the cohesion well's equilibrium
    is offset slightly from d_ref by the (a-b)/2 term in ψ')."""
    ax = max(0.6, min(1.4, ax))
    physical = 3.0 + (ax - 0.6) * 6.25     # 3 .. 8 m physical
    return physical / scale


def clamp_mag2(vx, vy, max_mag):
    """Clamp the magnitude of a 2D vector to max_mag."""
    mag = math.hypot(vx, vy)
    if mag > max_mag and mag > 0:
        s = max_mag / mag
        return vx * s, vy * s
    return vx, vy


# ---------- Olfati-Saber math (mirrors OlfatiSaber.cs) ----------

class OlfatiSaber:
    """2D port of the Unity OlfatiSaber component. Stateless math — one
    instance can serve the whole swarm; per-drone state comes through args."""

    def __init__(self, r0_coh=150.0, delta=0.1, a=0.9, b=1.5, c=0.0,
                 c_vm=0.0, scale=10.0):
        self.r0_coh = r0_coh
        self.delta  = delta
        self.a = a
        self.b = b
        self.c = c
        self.c_vm = c_vm
        self.scale = scale

    # cohesion intensity ψ(r, d_ref)
    def psi(self, r, d_ref):
        diff = r - d_ref
        return (((self.a + self.b) / 2.0)
                * (math.sqrt(1 + (diff + self.c) ** 2) - math.sqrt(1 + self.c ** 2))
                + ((self.a - self.b) * diff) / 2.0)

    # ψ'(r, d_ref)
    def psi_prime(self, r, d_ref):
        diff = r - d_ref
        return (((self.a + self.b) / 2.0)
                * (diff + self.c) / math.sqrt(1 + (diff + self.c) ** 2)
                + (self.a - self.b) / 2.0)

    # neighbour weight w(r, r0)
    def w_fn(self, r, r0):
        rr = r / r0
        if rr < self.delta:
            return 1.0
        if rr < 1.0:
            arg = math.pi * (rr - self.delta) / (1 - self.delta)
            return (0.5 * (1.0 + math.cos(arg))) ** 2
        return 0.0

    # w'(r, r0)
    def w_prime(self, r, r0):
        rr = r / r0
        if rr < self.delta:
            return 0.0
        if rr < 1.0:
            arg = math.pi * (rr - self.delta) / (1 - self.delta)
            return 0.5 * (-math.pi) / (1 - self.delta) * (1 + math.cos(arg)) * math.sin(arg)
        return 0.0

    # Cohesion force scalar. Mirrors the C# verbatim, including the
    # `1/r0_coh` term using the field (not the passed r0). Harmless here
    # because we only call it with r0 == self.r0_coh anyway.
    def cohesion_force(self, r, d_ref):
        wp   = self.w_prime(r, self.r0_coh)
        ps   = self.psi(r, d_ref)
        ww   = self.w_fn(r, self.r0_coh)
        psp  = self.psi_prime(r, d_ref)
        return (1.0 / self.r0_coh) * wp * ps + ww * psp

    def compute(self, self_pos_ne, self_vel_ne, neighbours, d_ref):
        """Return the world-frame swarm correction for one drone, to be ADDED
        to the joystick's desired velocity before sending to DJI VS.

        Mirrors the new `OlfatiSaber.cs::GetSwarmAcceleration`:
            return velocityConsensus + cohesion
        where velocityConsensus sums c_vm*(v_neighbour - v_self) over neighbours
        (pulling each drone toward its neighbours' velocities), and cohesion is
        the σ-norm spacing potential.

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

            # Cohesion: spacing potential along the relative-position unit vector
            rel_n = n_pos[0] - sn
            rel_e = n_pos[1] - se
            rel_mag = math.hypot(rel_n, rel_e)
            if rel_mag < 1e-6:
                continue
            r_scaled = rel_mag / self.scale
            if r_scaled >= self.r0_coh:
                continue  # outside cohesion well; contribution is zero anyway
            force = self.cohesion_force(r_scaled, d_ref)
            ux = rel_n / rel_mag
            uy = rel_e / rel_mag
            coh_n += force * ux
            coh_e += force * uy

        return consensus_n + coh_n, consensus_e + coh_e


# Reverse command channel: the browser GUI (swarm_gui.py) forwards Start/Stop
# button presses here as JSON UDP datagrams. Distinct from the joystick (:5055)
# and telemetry (:5099) ports.
DEFAULT_CMD_HOST = "127.0.0.1"
DEFAULT_CMD_PORT = 5098


def command_listener(swarming, meta, host, port):
    """Receive GUI command datagrams (Start/Stop, gimbal slider, heading
    mode + point-inwards toggles) over UDP.

    This thread NEVER touches ds_wrapper — it only mutates the shared `swarming`
    Event and the shared `meta` dict. The control loop (run) detects the edge /
    reads meta and performs the actual VS arm/disarm and gimbal set, so every
    hardware poke stays on the control-loop thread.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"  UDP command listener on {host}:{port} (GUI Start/Stop + gimbal)")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            msg = json.loads(data.decode("utf-8"))
            action = (msg.get("action") or "").lower()
            if action == "start":
                swarming.set()
                print("[gui] START swarming")
            elif action == "stop":
                swarming.clear()
                print("[gui] STOP swarming")
            elif action == "toggle":
                (swarming.clear if swarming.is_set() else swarming.set)()
                print(f"[gui] TOGGLE swarming -> {'ON' if swarming.is_set() else 'off'}")
            elif action == "gimbal":
                # GUI slider: update the shared gimbal pitch target (deg). run()
                # applies it to every drone on its next tick.
                try:
                    v = float(msg.get("value"))
                except (TypeError, ValueError):
                    continue
                v = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX, v))
                if meta is not None:
                    meta["gimbal_pitch"] = v
                print(f"[gui] gimbal pitch -> {v:+.1f}°")
            elif action == "heading":
                # GUI heading-mode selector. run() reads meta["heading_mode"]
                # each tick and handles the manual↔convexhull transition.
                v = (str(msg.get("value") or "")).lower()
                if v in ("manual", "convexhull") and meta is not None:
                    meta["heading_mode"] = v
                    print(f"[gui] heading mode -> {v}")
            elif action == "point_inwards":
                # GUI toggle: boundary drones face the centroid instead of
                # outward. run() forwards it to the hull controller.
                if meta is not None:
                    meta["point_inwards"] = bool(msg.get("value"))
                    print(f"[gui] point inwards -> {meta['point_inwards']}")
        except Exception:
            continue  # ignore malformed packets, keep listening


# ---------- main loop ----------

def run(swarm, receiver, olfati, swarming, dry_run=False, vel_frame="ned",
        logger=None, speed_scale=1.0, meta=None, heading_ctrl=None):
    print("\n--- Olfati-Saber Swarm Mode ---")
    print(f"  Drones: {sorted(swarm.drones.keys())}")
    print(f"  c_vm={olfati.c_vm}  r0_coh={olfati.r0_coh}  scale={olfati.scale}")
    mode0 = (meta or {}).get("heading_mode", "manual")
    print(f"  Heading: {mode0} — switchable live from the GUI. "
          f"manual = stick yaw steers a shared target heading; "
          f"convexhull = GLOBAL_CONVEXHULL (boundary drones face outward, "
          f"interior drones hold heading, stick yaw ignored)")
    print(f"  Telemetry velocity frame: {vel_frame}  "
          f"(switch via --vel-frame if consensus oscillates)")
    if speed_scale != 1.0:
        print(f"  SLOW TEST MODE: commanded velocities/rates scaled to {speed_scale:.0%}")
    print(f"  s1 / GUI button: toggle swarming (arm VS + flock)    s2: LAND-all")
    print(f"  q  (PC keyboard): stop, hold position, disable VS, exit")
    print(f"  Ctrl+C: same, abrupt\n")

    target_yaw = 0.0
    target_alt = INITIAL_ALT_M
    vs_on = False
    last_heading_mode = mode0           # detect GUI mode switches (below)
    last_swarming = swarming.is_set()   # starts cleared = held (do nothing)
    last_gimbal = None                  # last gimbal pitch applied to the drones
    last_s1 = 0
    last_s2 = 0
    last_t = time.time()
    last_print = 0.0
    no_fix_warned = set()

    while True:
        now = time.time()
        dt = now - last_t
        last_t = now

        # 'q' on the PC keyboard: hold position then disable VS and exit
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b'q', b'Q'):
                print("[q] STOP: zeroing velocities, holding position, disabling VS")
                for did, drone in swarm.drones.items():
                    t = drone.telemetry
                    if t:
                        hold_alt = max(t.get('alt', target_alt), MIN_ALT_M)
                    else:
                        hold_alt = target_alt
                    # Yaw rate 0 = hold current heading; altitude holds at hold_alt.
                    drone.set_velocity(0.0, 0.0, 0.0, hold_alt)
                # Let the 20 Hz send threads emit a few zero-velocity commands
                # so the drones are visibly braking before VS goes off.
                time.sleep(0.4)
                swarm.disable_vs_all()
                vs_on = False
                swarming.clear()
                print("[q] VS disabled — drones holding position autonomously")
                return

        js = receiver.get_state()

        # s1 rising edge: toggle the shared swarming gate (identical to the GUI
        # Start/Stop buttons). Needs the joystick, so only when one is present.
        if js is not None:
            if js.s1 == 1 and last_s1 == 0:
                (swarming.clear if swarming.is_set() else swarming.set)()
                print(f"[s1] {'START' if swarming.is_set() else 'STOP'} swarming")
            last_s1 = js.s1

            # s2 rising edge: LAND all drones
            if js.s2 == 1 and last_s2 == 0:
                for d in swarm.drones.values():
                    d.land()
                print("[s2] LAND_ALL")
            last_s2 = js.s2

        # Swarming edge → arm/disarm VS. Done here (not in the listener thread or
        # the s1 branch) so every ds_wrapper poke stays on this control-loop
        # thread, and so the GUI can Start/Stop even with no joystick connected.
        sw = swarming.is_set()
        if sw and not last_swarming:
            # Rising: arm VS. Seed heading from a drone with a fix, and the shared
            # altitude target from the AVERAGE altitude of all drones we actually
            # have telemetry from (e.g. with --drones 3 but only 2 connected, just
            # those 2), floored at START_ALT_FLOOR_M.
            fixes = [d.telemetry for d in swarm.drones.values() if d.telemetry]
            alts = [t['alt'] for t in fixes if t.get('alt') is not None]
            if fixes:
                target_yaw = _normalize_heading_180(fixes[0].get('heading', 0.0))
            if alts:
                target_alt = max(sum(alts) / len(alts), START_ALT_FLOOR_M)
            else:
                target_alt = max(INITIAL_ALT_M, START_ALT_FLOOR_M)
            swarm.enable_vs_all()
            vs_on = True
            print(f"[swarm] START: VS armed  heading={target_yaw:+.1f}°  "
                  f"alt={target_alt:.1f} m")
        elif (not sw) and last_swarming:
            # Falling: zero velocities, brake briefly, then disarm VS so each
            # drone holds position via its own autonomous GPS hover (same as 'q').
            print("[swarm] STOP: zeroing velocities, holding position, disabling VS")
            for did, drone in swarm.drones.items():
                t = drone.telemetry
                hold_alt = max(t.get('alt', target_alt), MIN_ALT_M) if t else target_alt
                drone.set_velocity(0.0, 0.0, 0.0, hold_alt)
            time.sleep(0.4)
            swarm.disable_vs_all()
            vs_on = False
            print("[swarm] VS disabled — drones holding position autonomously")
        last_swarming = sw

        if meta is not None:
            meta["swarming"] = sw

        # GUI gimbal slider: apply the shared pitch target to every drone
        # whenever it changes. set_gimbal only updates the send-loop's cached
        # value (no ds_wrapper poke here); the 20 Hz send loop relays it and the
        # gimbal actually moves once VS is enabled.
        if meta is not None:
            gp = meta.get("gimbal_pitch")
            if gp is not None and gp != last_gimbal:
                for d in swarm.drones.values():
                    d.set_gimbal(gp, 0.0)
                last_gimbal = gp

        # Heading mode + point-inwards are runtime-switchable from the GUI
        # (command_listener writes meta["heading_mode"]/meta["point_inwards"];
        # --heading/--point-inwards just seed them). Applied here, outside the
        # swarming gate, so the operator can preselect the mode while held.
        hull_mode = False
        if meta is not None and heading_ctrl is not None:
            hull_mode = meta.get("heading_mode") == "convexhull"
            mode = "convexhull" if hull_mode else "manual"
            if mode != last_heading_mode:
                if hull_mode:
                    # Fresh activation: stale debounce timers / held targets
                    # from a previous stint must not leak in.
                    heading_ctrl.reset()
                else:
                    # Back to manual: re-seed the shared target from a live
                    # heading so drones don't all snap to a stale target_yaw.
                    for d in swarm.drones.values():
                        t = d.telemetry
                        if t and t.get('heading') is not None:
                            target_yaw = _normalize_heading_180(t['heading'])
                            break
                    meta["hull_boundary"] = []   # nothing is hull-steered now
                print(f"[heading] mode -> {mode}")
                last_heading_mode = mode
            pin = bool(meta.get("point_inwards"))
            if pin != heading_ctrl.point_inwards:
                heading_ctrl.set_point_inwards(pin)
                print(f"[heading] point inwards -> {pin}")

        # Held → do nothing: VS is off and the drones hover autonomously until
        # Start is pressed again.
        if not sw:
            time.sleep(0.02)
            continue

        # Armed but stale joystick → hold last commands, don't integrate.
        if js is None:
            time.sleep(0.05)
            continue

        # Stick channels
        lin_x = _deadzone(js.linear_x)
        lin_y = _deadzone(js.linear_y)
        lin_z = _deadzone(js.linear_z)
        ang_z = _deadzone(js.angular_z)

        # Shared integrated targets (--slow scales the yaw and climb rates too).
        # ff_yaw_rate is the shared stick feed-forward; each drone then gets a
        # yaw RATE = ff + heading-hold P term on its own heading (below), so all
        # drones smoothly servo their nose to the shared target_yaw.
        # In convex-hull heading mode the hull owns every drone's heading
        # (mirrors the Unity AttitudeAlgorithm suppressing the input yaw rate),
        # so the stick doesn't integrate the shared target.
        ff_yaw_rate = ang_z * YAW_RATE_DEG_S * speed_scale
        if not hull_mode:
            target_yaw = _normalize_heading_180(target_yaw + ff_yaw_rate * dt)
        target_alt = max(MIN_ALT_M, min(MAX_ALT_M,
                                        target_alt + lin_z * VERT_RATE_MPS * speed_scale * dt))
        d_ref = d_ref_from_ax(js.angular_x, scale=olfati.scale)
        # Publish the physical target spacing (metres) for the GUI. d_ref is in
        # scaled units, so physical spacing = d_ref * scale (matches the "~Xm"
        # the per-second status line prints).
        if meta is not None:
            meta["d_ref_m"] = round(d_ref * olfati.scale, 2)

        # World-frame group desired velocity, shared across the swarm: every
        # drone tries to move in the same compass direction regardless of its
        # own heading. Per drone the world v_des gets rotated into body pitch/
        # roll using that drone's individual heading (world_to_body below).
        v_n_des = lin_x * MAX_PITCH_MPS    # north (m/s)
        v_e_des = lin_y * MAX_ROLL_MPS     # east  (m/s)

        # Snapshot the swarm state in local (N, E) meters
        fixes = []
        for did, drone in swarm.drones.items():
            t = drone.telemetry
            if not t:
                continue
            if t.get('sat_count', 0) < MIN_SAT_COUNT:
                if did not in no_fix_warned:
                    print(f"[drone {did}] dropped from swarm: sats={t.get('sat_count', 0)}")
                    no_fix_warned.add(did)
                continue
            if t['lat'] == 0 and t['lon'] == 0:
                if did not in no_fix_warned:
                    print(f"[drone {did}] dropped from swarm: GPS sentinel (0,0)")
                    no_fix_warned.add(did)
                continue
            if did in no_fix_warned:
                no_fix_warned.discard(did)
                print(f"[drone {did}] GPS fix acquired, rejoining swarm")
            fixes.append((did, t))

        if not fixes:
            time.sleep(0.02)
            continue

        lat_ref = sum(t['lat'] for _, t in fixes) / len(fixes)
        lon_ref = sum(t['lon'] for _, t in fixes) / len(fixes)
        snap = {}
        for did, t in fixes:
            pos_ne = gps_to_local(t['lat'], t['lon'], lat_ref, lon_ref)
            if vel_frame == "body":
                # vx = body forward, vy = body right. Rotate into world NED.
                hdg_rad = math.radians(t['heading'])
                cos_h = math.cos(hdg_rad)
                sin_h = math.sin(hdg_rad)
                v_n_world = t['vx'] * cos_h - t['vy'] * sin_h
                v_e_world = t['vx'] * sin_h + t['vy'] * cos_h
                vel_ne = (v_n_world, v_e_world)
            else:
                vel_ne = (t['vx'], t['vy'])
            snap[did] = (pos_ne, vel_ne, t['heading'])

        # Convex-hull heading control: derive a per-drone target heading from
        # the swarm's hull (boundary drones face outward along their vertex
        # bisector; interior drones get None → hold current heading).
        hull_targets = None
        if hull_mode:
            hull_targets = heading_ctrl.update(
                {did: pos for did, (pos, _, _) in snap.items()}, dt)
            if meta is not None:
                meta["hull_boundary"] = heading_ctrl.boundary_ids()

        # Per-drone flocking command. The joystick's desired velocity goes
        # straight through (DJI VS already runs a velocity tracker); the swarm
        # algorithm only contributes a correction (neighbour-velocity consensus
        # + cohesion) that gets added on top.
        for did, ctrl in swarm.drones.items():
            if did not in snap:
                continue  # no fix → DroneController holds last set_velocity
            try:
                self_pos, self_vel, hdg = snap[did]
                if hull_targets is not None:
                    # Hull mode: servo boundary drones onto their hull-derived
                    # heading (no stick feed-forward); interior drones hold
                    # their current heading (rate 0).
                    drone_target = hull_targets.get(did)
                    if drone_target is None:
                        cmd_yaw_rate = 0.0
                        drone_target = hdg  # for the dry-run printout
                    else:
                        cmd_yaw_rate = heading_hold_rate(drone_target, hdg)
                else:
                    # Smooth yaw RATE for this drone: shared stick feed-forward
                    # plus a P term holding its own heading on the shared
                    # target_yaw.
                    drone_target = target_yaw
                    cmd_yaw_rate = heading_hold_rate(target_yaw, hdg, ff_yaw_rate)
                neighbours = [(snap[j][0], snap[j][1]) for j in snap if j != did]
                # Swarm correction (consensus + cohesion) in world frame
                v_n_corr, v_e_corr = olfati.compute(
                    self_pos, self_vel, neighbours, d_ref=d_ref,
                )
                v_n_total = v_n_des + v_n_corr
                v_e_total = v_e_des + v_e_corr
                v_n_total, v_e_total = clamp_mag2(v_n_total, v_e_total, MAX_CMD_MPS)
                # --slow: scale the combined command (joystick desired + swarm
                # correction) uniformly so the whole motion just runs slower —
                # the relative behaviour being tuned keeps its shape.
                v_n_total *= speed_scale
                v_e_total *= speed_scale
                if logger:
                    logger.log_swarm_debug(
                        did, v_n_des, v_e_des, v_n_corr, v_e_corr,
                        v_n_total, v_e_total, d_ref, len(neighbours))
                # DJI VS is in GROUND/VELOCITY mode (SwarmActivity sets
                # FlightCoordinateSystem.GROUND), so pitch = north m/s and
                # roll = east m/s. We send world-frame velocities directly —
                # the drone does its own world→body rotation internally.
                if dry_run:
                    print(f"  [dry] drone {did}  "
                          f"v_des=({v_n_des:+.2f}N,{v_e_des:+.2f}E)  "
                          f"corr=({v_n_corr:+.2f},{v_e_corr:+.2f})  "
                          f"cmd=({v_n_total:+.2f}N,{v_e_total:+.2f}E)  "
                          f"hdg={hdg:+6.1f}°→{drone_target:+.1f}° "
                          f"yawrate={cmd_yaw_rate:+5.1f}°/s  "
                          f"alt={target_alt:.1f}m  "
                          f"d_ref={d_ref:.3f} (~{d_ref*olfati.scale:.1f}m)")
                else:
                    ctrl.set_velocity(v_n_total, v_e_total, cmd_yaw_rate, target_alt)
            except Exception as e:
                print(f"[drone {did}] flocking error: {e}")

        if now - last_print > 1.0:
            if hull_mode:
                yaw_desc = ("hull-boundary=" +
                            (",".join(map(str, heading_ctrl.boundary_ids())) or "none"))
            else:
                yaw_desc = f"yaw={target_yaw:+6.1f}°"
            print(f"  v_des=({v_n_des:+5.2f}N,{v_e_des:+5.2f}E) world  "
                  f"{yaw_desc}  alt={target_alt:5.1f}m  "
                  f"d_ref={d_ref:.3f} (~{d_ref*olfati.scale:.1f}m)  "
                  f"fixes={len(snap)}/{len(swarm.drones)}  "
                  f"SWARM={'ON' if sw else 'off'}  VS={'ON' if vs_on else 'off'}")
            # Per-drone local position + the per-drone world-frame command
            # (which equals v_des once the swarm correction is added).
            for did in sorted(snap.keys()):
                (n_m, e_m), self_vel, hdg = snap[did]
                drone_alt = swarm.drones[did].telemetry.get('alt', 0.0)
                v_n_self, v_e_self = self_vel
                print(f"    drone {did}  pos=({n_m:+6.2f}N,{e_m:+6.2f}E)  "
                      f"alt={drone_alt:5.1f}m  hdg={hdg:+6.1f}°  "
                      f"vel=({v_n_self:+5.2f}N,{v_e_self:+5.2f}E)")
            # Pairwise distances (physical meters)
            ids = sorted(snap.keys())
            if len(ids) >= 2:
                pairs = []
                for i, a in enumerate(ids):
                    for b in ids[i+1:]:
                        (na, ea), _, _ = snap[a]
                        (nb, eb), _, _ = snap[b]
                        d = math.hypot(na - nb, ea - eb)
                        pairs.append(f"{a}-{b}={d:5.2f}m")
                print(f"    distances: {'  '.join(pairs)}")
            last_print = now

        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm Olfati-Saber flocking controller")
    ap.add_argument("--drones", type=int, default=1,
                    help="Number of drones (creates IDs 1..N)")
    ap.add_argument("--port", type=int, default=5055,
                    help="UDP port for joystick (default 5055)")
    ap.add_argument("--c-vm", type=float, default=0.0,
                    help="Velocity-matching gain (default 0.0)")
    ap.add_argument("--r0", type=float, default=150.0,
                    help="Cohesion neighbour radius r0_coh (default 150.0)")
    ap.add_argument("--scale", type=float, default=10.0,
                    help="Distance scale factor (default 10.0, matches Unity sim)")
    ap.add_argument("--gimbal-pitch", type=float, default=DEFAULT_GIMBAL_PITCH,
                    metavar="DEG",
                    help=f"Initial gimbal pitch (tilt) in degrees for all drones "
                         f"and the GUI slider's starting position; used on Start. "
                         f"Range [{GIMBAL_PITCH_MIN:.0f}, {GIMBAL_PITCH_MAX:.0f}] "
                         f"(DJI Mini 3 Pro). Default {DEFAULT_GIMBAL_PITCH:.0f}.")
    ap.add_argument("--heading", choices=["manual", "convexhull"], default="manual",
                    help="Initial heading-control mode (live-switchable from the "
                         "GUI afterwards): 'manual' (default) integrates a shared "
                         "target heading from the stick's angular.z; 'convexhull' "
                         "ports the Unity sim's GLOBAL_CONVEXHULL attitude "
                         "algorithm — drones on the swarm's convex hull face "
                         "outward along their vertex bisector, interior drones "
                         "hold heading, stick yaw is ignored.")
    ap.add_argument("--point-inwards", action="store_true",
                    help="Seed the point-inwards toggle (live-switchable from "
                         "the GUI): in convexhull mode, boundary drones face "
                         "the swarm centroid instead of outward.")
    ap.add_argument("--vel-frame", choices=["ned", "body"], default="ned",
                    help="Frame of telemetry vx/vy. Default 'ned' assumes DJI "
                         "reports ground-frame velocity; switch to 'body' if "
                         "the consensus term causes oscillation (which would "
                         "indicate vx/vy are actually body-frame forward/right).")
    ap.add_argument("--slow", nargs="?", const=SLOW_DEFAULT_SCALE, type=float, default=1.0,
                    metavar="SCALE",
                    help="Test mode: scale every commanded velocity (joystick "
                         "desired + swarm correction) and the yaw/climb rates for "
                         f"slow, controlled tuning. Bare --slow uses {SLOW_DEFAULT_SCALE}; "
                         "pass a value (e.g. --slow 0.5) to override. Default 1.0 "
                         "(full speed).")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print VS commands but do not send to drones")
    ap.add_argument("--no-gui", action="store_true",
                    help="Do not push telemetry to the browser GUI (swarm_gui.py)")
    ap.add_argument("--gui-host", default=DEFAULT_GUI_HOST,
                    help=f"GUI telemetry UDP host (default {DEFAULT_GUI_HOST})")
    ap.add_argument("--gui-port", type=int, default=DEFAULT_GUI_PORT,
                    help=f"GUI telemetry UDP port (default {DEFAULT_GUI_PORT})")
    ap.add_argument("--cmd-host", default=DEFAULT_CMD_HOST,
                    help=f"UDP bind host for GUI Start/Stop commands "
                         f"(default {DEFAULT_CMD_HOST})")
    ap.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT,
                    help=f"UDP port for GUI Start/Stop commands "
                         f"(default {DEFAULT_CMD_PORT})")
    ap.add_argument("--no-log", action="store_true",
                    help="Disable background flight-data logging to flight_logs/")
    ap.add_argument("--log-dir", default="flight_logs",
                    help="Directory for flight-log session folders (default flight_logs)")
    args = ap.parse_args()

    if args.drones < 1:
        ap.error("--drones must be >= 1")
    if args.slow <= 0:
        ap.error("--slow SCALE must be > 0")
    if not (GIMBAL_PITCH_MIN <= args.gimbal_pitch <= GIMBAL_PITCH_MAX):
        ap.error(f"--gimbal-pitch must be in "
                 f"[{GIMBAL_PITCH_MIN:.0f}, {GIMBAL_PITCH_MAX:.0f}] (DJI Mini 3 Pro)")

    print("LIS_Swarm Flocking Controller (Olfati-Saber)")
    print(f"  ds_wrapper HW Decoder: "
          f"{'enabled' if w.isHWDecoderEnabled() == 1 else 'disabled (SW)'}")

    swarm = SwarmController()
    for did in range(1, args.drones + 1):
        swarm.add_drone(did)
        print(f"  Added drone {did}")

    logger = None
    if not args.no_log:
        logger = FlightLogger(base_dir=args.log_dir, meta={
            "script": "swarm_flocking",
            "drones": args.drones, "port": args.port,
            "c_vm": args.c_vm, "r0": args.r0, "scale": args.scale,
            "vel_frame": args.vel_frame, "dry_run": args.dry_run,
            "slow": args.slow, "gimbal_pitch": args.gimbal_pitch,
        })
        swarm.attach_logger(logger)
        print(f"  Flight logging -> {logger.session_dir} (disable with --no-log)")

    # Synchronous probe: call sendWayPointData once per drone from the main
    # thread BEFORE starting any background threads. If a slot is missing in
    # DroneSwarmServer.exe the C extension may block here without releasing
    # the GIL, which would otherwise starve the main thread silently.
    for did, drone in sorted(swarm.drones.items()):
        print(f"  Probing drone {did} (sendWayPointData)... ", end="", flush=True)
        try:
            drone.send_vs()
            print("OK", flush=True)
        except Exception as e:
            print(f"FAIL: {e}", flush=True)

    # Start background threads one drone at a time, with a brief sleep so each
    # thread can do its first iteration and surface any error before we move on.
    print(f"  Starting send + telemetry threads...")
    for did, drone in sorted(swarm.drones.items()):
        print(f"    starting drone {did}... ", end="", flush=True)
        drone.start(send_rate_hz=20, telemetry_rate_hz=10)
        time.sleep(0.3)
        print("done", flush=True)
    print(f"  Started: 20 Hz commands, 10 Hz telemetry")

    # angular.x is repurposed for d_ref in swarm mode, so the gimbal would
    # otherwise stay at the DroneController default (-90°). Park it at the
    # launch value (--gimbal-pitch) as soon as the send threads are running; the
    # GUI slider seeds to the same value and can retarget it live afterwards
    # (see command_listener / run).
    for d in swarm.drones.values():
        d.set_gimbal(args.gimbal_pitch, 0.0)
    print(f"  Gimbal pitch set to {args.gimbal_pitch:+.0f}° (yaw 0°) for all drones")

    olfati = OlfatiSaber(r0_coh=args.r0, c_vm=args.c_vm, scale=args.scale)

    # Always instantiated: the GUI can switch heading modes at runtime, so the
    # hull controller must exist even when starting in manual mode. run() only
    # consults it while meta["heading_mode"] == "convexhull".
    heading_ctrl = ConvexHullHeading(point_inwards=args.point_inwards)

    receiver = JoystickReceiver(port=args.port, logger=logger)
    receiver.start()
    print(f"  UDP joystick listener on :{args.port}")

    # Shared with run(): the live physical target spacing (d_ref in metres) and
    # whether swarming is currently active, both published to the GUI by the feed
    # below. swarming starts cleared so the drones do nothing until Start.
    #
    # "olfati" carries the STATIC Olfati-Saber parameters actually used to
    # compute each drone's command (velocity consensus + cohesion) so the GUI
    # can display the live tuning. d_ref_m is the dynamic spacing target
    # (joystick angular.x) and is refreshed each tick in run().
    swarm_meta = {
        "d_ref_m": None,
        "swarming": False,
        # Heading-control settings, seeded from the CLI and then owned by the
        # GUI (command_listener overwrites them; run() applies changes each
        # tick). hull_boundary is the live list of boundary drone ids in
        # convexhull mode so the GUI can mark which drones the hull steers.
        "heading_mode": args.heading,
        "point_inwards": args.point_inwards,
        "hull_boundary": [],
        # Live gimbal pitch target (deg): seeded from --gimbal-pitch, then driven
        # by the GUI slider via command_listener. Published so the slider can
        # seed its starting position.
        "gimbal_pitch": args.gimbal_pitch,
        "olfati": {
            "c_vm": olfati.c_vm,
            "r0_coh": olfati.r0_coh,
            "scale": olfati.scale,
            "a": olfati.a,
            "b": olfati.b,
            "c": olfati.c,
            "delta": olfati.delta,
        },
    }

    swarming = threading.Event()   # cleared = held (do nothing); set = flocking
    threading.Thread(target=command_listener,
                     args=(swarming, swarm_meta, args.cmd_host, args.cmd_port),
                     daemon=True, name="GuiCommandListener").start()

    gui_feed = None
    if not args.no_gui:
        gui_feed = TelemetryFeedPublisher(
            source=swarm.get_all_telemetry,
            stats_source=lambda: {did: {"send_hz": d.send_hz(), "recv_hz": d.recv_hz()}
                                  for did, d in swarm.drones.items()},
            meta_source=lambda: swarm_meta,
            host=args.gui_host, port=args.gui_port)
        gui_feed.start()
        print(f"  GUI telemetry feed -> {args.gui_host}:{args.gui_port} "
              f"(open swarm_gui.py; disable with --no-gui)")

    try:
        run(swarm, receiver, olfati, swarming,
            dry_run=args.dry_run, vel_frame=args.vel_frame, logger=logger,
            speed_scale=args.slow, meta=swarm_meta, heading_ctrl=heading_ctrl)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if gui_feed is not None:
            gui_feed.stop()
        receiver.stop()
        swarm.stop_all()
        if logger is not None:
            logger.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
