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
    linear.z   → climb (integrated into the shared target altitude; in
                 vertical-plane mode it raises/lowers the whole wall instead —
                 see swarm_plane.py)
    angular.z  → yaw rate (feed-forward; a shared target heading is held per
                 drone via heading_hold_rate → smooth yaw RATE to the DJI VS).
                 With --heading convexhull the per-drone heading comes from the
                 swarm's convex hull instead (boundary drones face outward,
                 interior drones hold heading — port of the Unity sim's
                 GLOBAL_CONVEXHULL, see heading_convexhull.py), so the stick no
                 longer steers the drones; there it rotates the operator's
                 command reference frame (cmd_frame_yaw) — the compass heading
                 that linear.x/linear.y point along — decoupling "push forward"
                 from where the drones are actually facing.
    angular.x  → d_ref, linear map [0.6, 1.4] → scaled [0.5, 1.0]
                 (≈ physical [5, 10] m at ScaleFactor = 10)
    switches   → intentionally unused; swarming Start/Stop is GUI-only (the
                 browser Start/Stop buttons, via command_listener). Leaving the
                 s1/s2 switches unwired means a switch left 'on' at connect can
                 never auto-start the swarm.
    PC key 'q' → zero velocities, hold current position, disable VS, exit

Vertical-plane ("wall") swarming is ported in swarm_plane.py: a GUI toggle that
swaps the plane the cohesion law is constrained to, from horizontal to a
vertical wall facing the stick-steered heading. It is the only path in this
script that commands a PER-DRONE altitude — everywhere else the swarm shares
one absolute altitude target.

Obstacle avoidance from the C# original is ported in olfati_saber.py
(ObstacleAvoidance): 2D rectangular virtual obstacles plus one geofence
polygon, both drawn on the browser GUI's map and persisted to shapes.json.
The fence edges repel inward with the same β-agent kernel, and a drone that
ends up OUTSIDE the fence is braked, gets DISABLE_VS, and is removed from
the flock (not a neighbour) until swarming is stopped and started again.

Note on ScaleFactor: the OlfatiSaber math uses the Unity-sim tuning verbatim
(ScaleFactor = 10.0). To keep the cohesion potential in the same regime, the
`d_ref` we feed it is in *scaled* units (Unity's convention) — physical
spacing ≈ d_ref * ScaleFactor. The joystick angular.x is mapped to scaled
d_ref ∈ [0.5, 1.0], which corresponds to a physical d_ref ∈ [5, 10] m at
ScaleFactor = 10.

Usage:
    python swarm_flocking.py --drones 3
    python swarm_flocking.py --drones 2 --c-vm 0.5     # gentler velocity matching
    python swarm_flocking.py --drones 3 --slow         # slow test mode (30% speed)
    python swarm_flocking.py --drones 3 --slow 0.5     # slow test mode at 50% speed
    python swarm_flocking.py --drones 3 --dry-run      # print VS commands, do not transmit
    python swarm_flocking.py --drones 3 --heading convexhull   # hull-facing headings
    python swarm_flocking.py --drones 3 --plane-mode --max-alt 40  # vertical wall
"""

import argparse
import json
import math
import msvcrt    # Windows console: non-blocking keyboard read for the 'q' stop
import os
import re
import socket
import subprocess
import sys
import threading
import time

import ds_wrapper as w

# Force line-buffered stdout so we actually see startup prints in the PowerShell
# launcher (block-buffered stdout has made debug sessions painful before).
try:
    # errors='replace': the console codepage is often cp1252, which cannot
    # encode characters like U+2192. Without this, ONE such character in a
    # status line raises UnicodeEncodeError mid-print — inside the per-drone
    # try/except that turns into "flocking error" on every drone every tick,
    # with no other symptom. Degrade to '?' instead of losing the output.
    sys.stdout.reconfigure(line_buffering=True, errors='replace')
except (AttributeError, ValueError):
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
from heading_convexhull import ConvexHullHeading, _wrap180
from heading_demostitch import (
    DemoStitchHeading,
    DEFAULT_OFFSET_DEG,
    OFFSET_MIN_DEG,
    OFFSET_MAX_DEG,
)
from swarm_plane import (
    SwarmPlane,
    alt_spread,
    separation_3d,
    DEFAULT_PLANE_GAIN,
    PLANE_GAIN_MIN,
    PLANE_GAIN_MAX,
    DEFAULT_PLANE_LEASH_M,
    ALT_SPREAD_GATE_M,
    DOWNWASH_RADIUS_M,
)
from olfati_saber import (
    OlfatiSaber,
    ObstacleAvoidance,
    gps_to_local,
    clamp_mag2,
    rect_to_ne,
    polygon_to_ne,
    point_in_polygon,
    DEFAULT_SHAPES_FILE,
    MAX_FACADES,
    MAX_OBSTACLES,
    normalize_obstacle,
    normalize_polygon_obstacle,
    validate_facade,
    validate_fence,
    load_facades,
    load_shapes,
    save_shapes,
)
from image_stream_feed import ImageStreamPublisher
from clip_recorder import (
    ClipRecorder, DEFAULT_RECORDING_DIR, DEFAULT_MAX_SECONDS,
    MAX_SECONDS_MIN, MAX_SECONDS_MAX,
)
from dji_camera_pose import CameraPoseSolver
# Pure module (stdlib only, no ds_wrapper / numpy / cv2), so the flight loop can
# import it. The facade auto-pick and the standoff maths live there because the
# offline checker, the replayer and the recorder all need the SAME rule.
import clip_scene_plane as scene_plane
from mqtt_command_sender import MqttCommandSender
from response_monitor import ResponseMonitor
from joystick_controller import (
    DroneController,
    SwarmController,
    _deadzone,
    heading_hold_rate,
    integrate_target_heading,
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

# Drones with fewer satellites than this are excluded from the swarm snapshot.
MIN_SAT_COUNT = 6

# Floor for the shared altitude target seeded when swarming starts. The seed is
# the average altitude of the drones we actually have telemetry from; if that
# average is below this (e.g. drones still on the ground) we climb to this instead.
START_ALT_FLOOR_M = 3.0

# Minimum-separation failsafe: if any pair of drones with a fix gets closer
# than this (physical metres), swarming auto-STOPs (zero velocities, brake,
# disable VS — the same path as the GUI Stop button). Added after the
# 2026-07-04 flight where a command/telemetry channel permutation drove pair
# 2-3 to 2.9 m with nothing reacting. <= 0 disables the check.
DEFAULT_MIN_SEPARATION_M = 3.0

# Identity probe: how long to wait for one slot's marker to come back through
# an RC broker. The server's fallback send path takes ~220 ms per command
# (connect -> publish -> disconnect); 2 s absorbs a slow broker comfortably.
IDCHECK_TIMEOUT_S = 2.0

# Pre-flight link scan: how long to wait for every RC's LINKDIAG answer. Longer
# than the identity probe because the app answers only after five separate
# KeyManager.getValue() round-trips to the aircraft (band, channel mode,
# bandwidth, frequency point, interference sweep), any of which the firmware
# may leave hanging until it times out on its side.
LINK_SCAN_TIMEOUT_S = 5.0

# How long to let an AIRLINK: one-shot settle before the link scan reads the
# radio back. SwarmActivity.applyAirlinkSettings fires an ASYNC setValue and
# returns, so without this wait the scan races the set and reports the OLD
# value — which reads exactly like a rejection. A bandwidth change in
# particular renegotiates the link, so it is not instant. Only paid when an
# AIRLINK was actually sent.
AIRLINK_SETTLE_S = 3.0

# SDK enum names the app reports in a LINKSCAN, keyed by the token we sent, so
# the scan can say "you asked for 10 MHz and the aircraft is on 40".
_AIRLINK_EXPECTED = {
    "band": {"2G4": "BAND_2_DOT_4G", "5G8": "BAND_5_DOT_8G",
             "DUAL": "BAND_DUAL"},
    "bw": {"40": "BANDWIDTH_40MHZ", "20": "BANDWIDTH_20MHZ",
           "10": "BANDWIDTH_10MHZ", "5": "BANDWIDTH_5MHZ"},
}

# ---- Rotation check (open-loop actuation probe; GUI "Rotation check") ----
# One drone at a time: enable VS, fly a short pulse NORTH then EAST at the
# drone's current altitude, measure the GPS displacement of each pulse, and
# fit rotation+gain between commanded and flown direction. Added after the
# 2026-07-05 flight where several aircraft executed velocity commands rotated
# 90-180° (suspected FC yaw/compass error) and the defect was only visible in
# offline analysis. Deliberately NOT scaled by --slow: the measurement needs
# ~2 m of displacement to stand clear of GPS noise.
PROBE_SPEED_MPS = 0.6         # pulse speed (m/s)
PROBE_PULSE_S = 3.0           # pulse duration -> ~1.8 m per axis
PROBE_ENABLE_S = 1.5          # wait after ENABLE_VS before pulsing
PROBE_SETTLE_S = 2.0          # brake/settle after each pulse
PROBE_LAG_S = 0.6             # actuation+GPS lag: sample positions this late
PROBE_OK_DEG = 25.0           # |rot| <= this -> OK
PROBE_ROTATED_DEG = 60.0      # |rot| > this -> ROTATED (in between: SKEWED)
PROBE_MIN_GAIN = 0.3          # gain below this -> DEAD (didn't follow at all)
# Worst-case travel of one drone over the whole check is
# sqrt(2)*PROBE_SPEED*PROBE_PULSE_S; two drones probed in successive turns can
# close twice that, so refuse to start unless every pair has this much room
# beyond the min-separation failsafe distance.
PROBE_CLEARANCE_M = 2.0 * 1.4142 * PROBE_SPEED_MPS * PROBE_PULSE_S


# ---------- helpers ----------
# (gps_to_local / clamp_mag2 / the OlfatiSaber class moved to olfati_saber.py)

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


def swarm_mean_heading(swarm):
    """Circular mean (deg, [-180, 180]) of the connected drones' headings, or
    None when no drone has telemetry yet. Used as the reference the shared
    target heading is seeded from and lead-clamped against
    (integrate_target_heading) — with one shared target for several drones,
    the mean nose direction is the natural 'where the swarm points now'."""
    sum_sin = 0.0
    sum_cos = 0.0
    count = 0
    for d in swarm.drones.values():
        t = d.telemetry
        if t and t.get('heading') is not None:
            h = math.radians(t['heading'])
            sum_sin += math.sin(h)
            sum_cos += math.cos(h)
            count += 1
    if count == 0:
        return None
    return math.degrees(math.atan2(sum_sin, sum_cos))


def d_ref_from_ax(ax, scale=10.0):
    """Map angular.x ∈ [0.6, 1.4] → scaled d_ref ∈ [0.4, 1.2].

    The OlfatiSaber math operates in scaled units (ScaleFactor=10 by default),
    so the returned d_ref is divided by `scale` to match. Physical spacing
    sits roughly at `d_ref * scale + 2.58 m` (the cohesion well's equilibrium
    is offset slightly from d_ref by the (a-b)/2 term in ψ')."""
    ax = max(0.6, min(1.4, ax))
    physical = 4.0 + (ax - 0.6) * 10.0     # 4 .. 12 m physical
    return physical / scale


# ---------- command<->telemetry identity check ----------

def _probe_slot_receivers(slots, sender, timeout=IDCHECK_TIMEOUT_S):
    """Send one inert IDCHECK marker through each DroneSwarmServer slot and
    record which of `sender`'s per-IP connections it arrives on.

    The server publishes ``sendWayPointData(marker, slot)`` to slot's RC broker
    on topic MQTTWayPoints; `sender`'s connections subscribe to that topic for
    the duration. Markers are inert on the app side (SwarmActivity ignores
    unknown command strings).

    Slots are probed concurrently, one worker thread each: every
    ``sendWayPointData`` still blocks ~220 ms in the server (its send path
    reconnects per command) and an unresolved slot still waits the full
    `timeout`, but the round-trips overlap, so the whole probe costs about one
    slot's worst case instead of N of them (~25 s -> ~2.5 s at 10 drones on
    the swarming-Start edge). Safe to overlap: each slot has its own shared-
    memory status byte, the wrapper releases the GIL while it spins, the
    server handles each send on a detached thread, and probe_wait_for filters
    the shared capture buffer by its own marker under a lock.

    Returns ({slot: receiver_id}, problems): receiver_id is the sender key
    (drone id / provisional index) whose broker got the marker; `problems` is
    a list of human-readable strings (empty = every slot resolved uniquely).
    """
    nonce = "{}_{}".format(os.getpid(), int(time.time()))
    received = {}
    problems = []
    slot_problems = {}

    def probe_one(slot):
        marker = "IDCHECK:{}:{}".format(slot, nonce)
        try:
            w.sendWayPointData(marker, slot)
        except Exception as e:
            slot_problems[slot] = ("server slot {}: sendWayPointData failed "
                                   "({})".format(slot, e))
            return
        hits = sender.probe_wait_for(marker, timeout=timeout)
        if len(hits) == 1:
            received[slot] = hits[0]
        elif not hits:
            slot_problems[slot] = ("server slot {}: marker never arrived on "
                                   "any known broker (slot not connected in "
                                   "the server, or its RC is not among the "
                                   "command IPs)".format(slot))
        else:
            slot_problems[slot] = ("server slot {}: marker arrived on "
                                   "multiple command connections {} — "
                                   "duplicate IP?".format(slot, hits))

    offline = sender.probe_start()
    try:
        if offline:
            problems.append("no MQTT connection to broker(s) {} ({}) — "
                            "invisible to the probe".format(
                                offline,
                                ", ".join(sender.ip_of(d) or "?" for d in offline)))
        workers = [threading.Thread(target=probe_one, args=(slot,),
                                    name="IDProbe_{}".format(slot), daemon=True)
                   for slot in slots]
        for t in workers:
            t.start()
        for t in workers:
            t.join()
        # Per-slot verdicts land in dicts keyed by slot (one writer each, so
        # no lock needed); report problems in slot order for stable logs.
        for slot in slots:
            if slot in slot_problems:
                problems.append(slot_problems[slot])
    finally:
        sender.probe_stop()

    # Each receiver may answer for at most one slot (bijection).
    seen = list(received.values())
    for rid in sorted(set(seen)):
        if seen.count(rid) > 1:
            problems.append("RC {} answered for {} server slots"
                            .format(sender.ip_of(rid), seen.count(rid)))
    return received, problems


def discover_rc_ips(rtsp_port=8554):
    """Enumerate connected RC IPs by inspecting DroneSwarmServer's established
    RTSP control connections (TCP to <rc-ip>:8554 — rtsp_transport=udp only
    moves the media; the control socket stays open per connected slot).

    Uses Get-NetTCPConnection so the output is structured and locale-proof
    (netstat's state column is localized). Returns a sorted list of unique
    IPv4 addresses; empty when the server isn't running or no slot is
    connected.
    """
    # try/catch keeps the exit code 0 when there are simply no matches —
    # PowerShell 5.1 reports a failed (empty) Get-NetTCPConnection as exit 1
    # even under -ErrorAction SilentlyContinue.
    ps = ("$c = @(try {{ Get-NetTCPConnection -RemotePort {} "
          "-State Established -ErrorAction Stop }} catch {{}}); "
          "$c | Where-Object {{ "
          "(Get-Process -Id $_.OwningProcess -ErrorAction SilentlyContinue)"
          ".ProcessName -eq 'DroneSwarmServer' }} | "
          "Select-Object -ExpandProperty RemoteAddress").format(rtsp_port)
    try:
        out = subprocess.check_output(
            ["powershell", "-NoProfile", "-Command", ps],
            universal_newlines=True, stderr=subprocess.DEVNULL, timeout=15)
    except Exception as e:
        print("[discover] Get-NetTCPConnection failed: {}".format(e))
        return []
    ips = []
    for line in out.splitlines():
        ip = line.strip()
        # IPv4 only; the RCs live on the drone LAN.
        if ip and ip.count(".") == 3 and ip not in ips:
            ips.append(ip)
    return sorted(ips)


def auto_bind_command_channels(swarm, candidate_ips, timeout=IDCHECK_TIMEOUT_S):
    """Auto-discovery mode: bind each DroneSwarmServer slot to its RC broker.

    Given the set of RC IPs the server is streaming from (discover_rc_ips),
    connect a provisional MqttCommandSender to all of them and use the marker
    probe to learn which IP each server slot reaches. Drone ID = server slot,
    so commands and telemetry share ONE identity source (the server) and a
    mismatch is impossible by construction — the same property the original
    AOS broker had.

    Returns a connected MqttCommandSender keyed by drone id (= slot), or None
    when the binding could not be resolved (caller falls back to the legacy
    server command path).
    """
    ids = sorted(swarm.drones)
    if len(candidate_ips) < len(ids):
        print("[autobind] only {} RC(s) discovered for --drones {} — connect "
              "the missing slot(s) in DroneSwarmServer or lower the drone "
              "count".format(len(candidate_ips), len(ids)))

    provisional = MqttCommandSender(
        {i + 1: ip for i, ip in enumerate(candidate_ips)})
    try:
        deadline = time.monotonic() + 3.0
        while (time.monotonic() < deadline and
               not all(provisional.connected(i + 1)
                       for i in range(len(candidate_ips)))):
            time.sleep(0.1)
        slot_map, problems = _probe_slot_receivers(ids, provisional, timeout)
    finally:
        provisional.stop()   # ip_of() stays valid after stop

    if problems or len(slot_map) != len(ids):
        print("[autobind] FAILED — could not bind every server slot to an RC "
              "broker:")
        for p in problems:
            print("[autobind]   - " + p)
        return None

    final = MqttCommandSender(
        {slot: provisional.ip_of(rid) for slot, rid in slot_map.items()})
    deadline = time.monotonic() + 3.0
    while (time.monotonic() < deadline and
           not all(final.connected(d) for d in ids)):
        time.sleep(0.1)
    for slot in ids:
        state = ("connected" if final.connected(slot)
                 else "NOT connected yet (auto-reconnect active)")
        print("[autobind] drone {} (= server slot {}) -> RC {} [{}]"
              .format(slot, slot, final.ip_of(slot), state))
    return final


def run_identity_check(swarm, cmd_sender, logger=None, timeout=IDCHECK_TIMEOUT_S):
    """Verify — and auto-fix — the command↔telemetry channel mapping.

    Two independent things call a drone "N": commands go to the N-th IP in
    DroneIPs (the canonical identity: RC/switch-port), while telemetry comes
    from DroneSwarmServer slot N (whatever IP the operator/scanner put in that
    slot). On 2026-07-04 those disagreed by a 3-cycle and the heading-hold and
    flocking loops closed across the wrong aircraft.

    The server's slot→IP table isn't queryable, but its fallback send path IS
    an oracle for it: ``sendWayPointData(marker, slot)`` publishes the marker
    to slot's RC broker on topic MQTTWayPoints. Our persistent per-IP command
    connections subscribe to that topic and watch which IP each slot's marker
    lands on. The markers are inert on the app side (SwarmActivity ignores
    unknown command strings — just a verbose log line on the RC), so this is
    safe to run mid-session, drones hovering or flying.

    On a resolvable mismatch the fix is applied by pointing each
    DroneController's ``telemetry_slot`` at the slot whose RC its commands go
    to; telemetry, logs, the GUI map and the flocking loop all follow.

    Returns True when the mapping is verified (after any remap), False when it
    could not be resolved (offline RC, slot not connected in the server, an IP
    outside DroneIPs, or two slots on one RC).
    """
    ids = sorted(swarm.drones)
    slot_to_cmd, problems = _probe_slot_receivers(ids, cmd_sender, timeout)

    if problems or len(slot_to_cmd) != len(ids):
        print("[idcheck] FAILED — command<->telemetry identity could not be "
              "verified:")
        for p in problems:
            print("[idcheck]   - " + p)
        if logger:
            logger.log_drone_command(0, "EVENT",
                                     cmd="IDCHECK_FAIL:" + "; ".join(problems))
        return False

    # slot_to_cmd[s] = drone id whose RC the server reaches from slot s, i.e.
    # slot s's telemetry belongs to that drone.
    remapped = []
    for slot, did in sorted(slot_to_cmd.items()):
        ctrl = swarm.drones[did]
        if ctrl.telemetry_slot != slot:
            ctrl.telemetry_slot = slot
            remapped.append((did, slot))

    if remapped:
        print("[idcheck] MISMATCH DETECTED and corrected — server slot order "
              "differs from DroneIPs:")
        for did, slot in remapped:
            print("[idcheck]   drone {} (RC {}) telemetry <- server slot {}"
                  .format(did, cmd_sender.ip_of(did), slot))
        if logger:
            for did, slot in remapped:
                logger.log_drone_command(
                    did, "EVENT", cmd="IDCHECK_REMAP:telemetry_slot={}".format(slot))
    else:
        print("[idcheck] OK — server slots match DroneIPs order "
              f"({len(slot_to_cmd)} drones verified)")
        if logger:
            logger.log_drone_command(0, "EVENT", cmd="IDCHECK_OK")
    return True


class RotationProbe:
    """Open-loop actuation check — the GUI's "Rotation check" button.

    One drone at a time (drones assumed sufficiently spaced): ENABLE_VS, hold,
    pulse NORTH for PROBE_PULSE_S at PROBE_SPEED_MPS, brake, pulse EAST,
    brake, DISABLE_VS, next drone. Each pulse's GPS displacement is measured
    (positions sampled PROBE_LAG_S after pulse start/end to absorb actuation
    lag) and the two pulses give a least-squares rotation+gain between the
    commanded and flown directions:

        OK       |rot| <= PROBE_OK_DEG and gain healthy
        SKEWED   PROBE_OK_DEG < |rot| <= PROBE_ROTATED_DEG
        ROTATED  |rot| > PROBE_ROTATED_DEG
        SWAPPED  N command flew east and E command flew north — the DJI GROUND
                 pitch/roll (N/E) axes are transposed (the 2026-07-05 failure
                 mode; fixed app-side, this catches a regression)
        DEAD     gain < PROBE_MIN_GAIN        (didn't follow at all)
        VS FAIL  vs_enabled never went true after ENABLE_VS
        NO GPS   no usable fix; drone skipped entirely

    Run this BEFORE Start swarming — any verdict other than OK means the
    cohesion loop would close with the wrong sign/direction on that drone.

    This is a state machine ticked at ~50 Hz from run()'s held branch, so
    every hardware poke stays on the control-loop thread (same rule as the
    rest of the controller; command_listener only files the request in meta).
    It never runs while swarming is armed; pressing Start aborts it.

    Rotation sign: positive = response rotated clockwise (toward east) from
    the command, compass sense — comparable to ResponseMonitor's live fit.
    """

    def __init__(self, swarm, meta=None, logger=None,
                 min_separation=DEFAULT_MIN_SEPARATION_M):
        self.swarm = swarm
        self.meta = meta
        self.logger = logger
        self.min_separation = max(min_separation, 0.0)
        self.active = False
        self.results = {}          # did -> {"rot","gain","verdict"}
        self._queue = []           # drone ids still to probe
        self._did = None           # drone under test
        self._phase = None
        self._phase_end = 0.0
        self._hold_alt = None
        self._marks = []           # [(t_due, key), ...] position captures
        self._disp = {}            # key -> (lat, lon)

    # ---- lifecycle ----

    def start(self, now):
        """Pre-check and begin. Publishes 'refused' to meta and returns False
        when it cannot run safely; True when the state machine is live."""
        fixes = {}
        for did, ctrl in sorted(self.swarm.drones.items()):
            t = ctrl.telemetry
            if (t and t.get('sat_count', 0) >= MIN_SAT_COUNT
                    and not (t['lat'] == 0 and t['lon'] == 0)):
                fixes[did] = t
            else:
                self.results[did] = {"rot": None, "gain": None,
                                     "verdict": "NO GPS"}
        if not fixes:
            return self._refuse("no drone has a GPS fix")
        # Every pair needs room for the worst-case probe travel on top of the
        # min-separation failsafe distance.
        need = self.min_separation + PROBE_CLEARANCE_M
        ids = sorted(fixes)
        lat0 = fixes[ids[0]]['lat']
        lon0 = fixes[ids[0]]['lon']
        pos = {d: gps_to_local(fixes[d]['lat'], fixes[d]['lon'], lat0, lon0)
               for d in ids}
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                dist = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
                if dist < need:
                    return self._refuse(
                        "drones %d-%d only %.1f m apart (need >= %.1f m); "
                        "spread the swarm out first" % (a, b, dist, need))
        self._queue = ids
        self.active = True
        print("[rotcheck] START — %.1f m/s pulses N then E, %d drone(s): %s"
              % (PROBE_SPEED_MPS, len(ids), ids))
        if self.logger:
            self.logger.log_drone_command(0, "EVENT", cmd="ROTCHECK_START")
        self._next_drone(now)
        return True

    def abort(self, now, reason):
        """Zero + DISABLE_VS the drone under test and stop. Partial results
        stay visible in the GUI."""
        if not self.active:
            return
        if self._did is not None:
            ctrl = self.swarm.drones[self._did]
            ctrl.set_velocity(0.0, 0.0, 0.0, self._hold_alt or MIN_ALT_M)
            ctrl.disable_vs()
            self.results[self._did] = {"rot": None, "gain": None,
                                       "verdict": "ABORTED"}
        self.active = False
        self._did = None
        print("[rotcheck] ABORTED: %s" % reason)
        if self.logger:
            self.logger.log_drone_command(
                0, "EVENT", cmd="ROTCHECK_ABORT:%s" % reason)
        self._publish("done", now, msg="aborted: %s" % reason)

    # ---- state machine ----

    def tick(self, now):
        if not self.active:
            return
        # Capture due position marks (fresh telemetry is ~5 Hz; PROBE_LAG_S
        # of margin makes the +-0.2 s sample jitter irrelevant at ~1.8 m of
        # displacement).
        for due, key in list(self._marks):
            if now >= due:
                t = self.swarm.drones[self._did].telemetry
                if t and not (t['lat'] == 0 and t['lon'] == 0):
                    self._disp[key] = (t['lat'], t['lon'])
                self._marks.remove((due, key))
        if now < self._phase_end:
            self._publish("running", now)
            return
        ctrl = self.swarm.drones[self._did]
        if self._phase == "enable":
            t = ctrl.telemetry
            if not t or not t.get('vs_enabled'):
                # The d3 failure mode from 2026-07-05: ENABLE_VS sent, app
                # never armed. Don't pulse a drone that isn't listening.
                self._finish_drone(now, None, None, "VS FAIL")
                return
            self._begin_pulse(now, "pulse_n", PROBE_SPEED_MPS, 0.0)
        elif self._phase == "pulse_n":
            ctrl.set_velocity(0.0, 0.0, 0.0, self._hold_alt)
            self._phase = "settle_n"
            self._phase_end = now + PROBE_SETTLE_S
        elif self._phase == "settle_n":
            self._begin_pulse(now, "pulse_e", 0.0, PROBE_SPEED_MPS)
        elif self._phase == "pulse_e":
            ctrl.set_velocity(0.0, 0.0, 0.0, self._hold_alt)
            self._phase = "settle_e"
            self._phase_end = now + PROBE_SETTLE_S
        elif self._phase == "settle_e":
            rot, gain, verdict = self._evaluate()
            self._finish_drone(now, rot, gain, verdict)

    def _begin_pulse(self, now, phase, v_n, v_e):
        ctrl = self.swarm.drones[self._did]
        ctrl.set_velocity(v_n, v_e, 0.0, self._hold_alt)
        self._phase = phase
        self._phase_end = now + PROBE_PULSE_S
        self._marks.append((now + PROBE_LAG_S, phase + "_start"))
        self._marks.append((now + PROBE_PULSE_S + PROBE_LAG_S, phase + "_end"))
        print("[rotcheck] drone %d %s (%.1f m/s N=%.1f E=%.1f, %.0f s)"
              % (self._did, phase, PROBE_SPEED_MPS, v_n, v_e, PROBE_PULSE_S))

    def _next_drone(self, now):
        if not self._queue:
            self.active = False
            self._did = None
            summary = "  ".join(
                "%d:%s" % (d, r["verdict"]) for d, r in sorted(self.results.items()))
            print("[rotcheck] DONE — %s" % summary)
            if self.logger:
                self.logger.log_drone_command(0, "EVENT", cmd="ROTCHECK_DONE")
            self._publish("done", now)
            return
        self._did = self._queue.pop(0)
        ctrl = self.swarm.drones[self._did]
        t = ctrl.telemetry
        self._hold_alt = max(t.get('alt', MIN_ALT_M) if t else MIN_ALT_M,
                             MIN_ALT_M)
        self._marks = []
        self._disp = {}
        ctrl.set_velocity(0.0, 0.0, 0.0, self._hold_alt)
        ctrl.enable_vs()
        self._phase = "enable"
        self._phase_end = now + PROBE_ENABLE_S
        print("[rotcheck] drone %d: ENABLE_VS, hold alt %.1f m"
              % (self._did, self._hold_alt))
        self._publish("running", now)

    def _finish_drone(self, now, rot, gain, verdict):
        ctrl = self.swarm.drones[self._did]
        ctrl.set_velocity(0.0, 0.0, 0.0, self._hold_alt)
        ctrl.disable_vs()
        self.results[self._did] = {
            "rot": None if rot is None else round(rot, 1),
            "gain": None if gain is None else round(gain, 2),
            "verdict": verdict,
        }
        print("[rotcheck] drone %d: %s%s"
              % (self._did, verdict,
                 "" if rot is None else
                 "  rot=%+.1f deg  gain=%.2f" % (rot, gain)))
        if self.logger:
            self.logger.log_drone_command(
                self._did, "EVENT",
                cmd="ROTCHECK:rot=%s:gain=%s:%s"
                    % ("" if rot is None else "%+.1f" % rot,
                       "" if gain is None else "%.2f" % gain, verdict))
        self._did = None
        self._next_drone(now)

    def _evaluate(self):
        """Least-squares rotation+gain from the two pulse displacements
        (complex n + 1j*e; same convention as ResponseMonitor), plus an
        explicit North<->East axis-swap check.

        A pure axis swap (the 2026-07-05 failure: DJI's GROUND pitch axis drives
        EAST and roll drives NORTH, so a north command flies east and vice-versa)
        makes the two pulses' contributions cancel in the rotation fit and
        deflates the gain to ~0 — it reads as DEAD with a garbage angle even
        though each axis actually followed at ~unity gain. So test for the swap
        directly (N command landing on the E axis and E command on the N axis,
        both with healthy gain) and report SWAPPED before the DEAD fallthrough.
        """
        L = PROBE_SPEED_MPS * PROBE_PULSE_S
        # Per-pulse world displacement, keyed by commanded axis.
        meas = {}
        for phase in ("pulse_n", "pulse_e"):
            p0 = self._disp.get(phase + "_start")
            p1 = self._disp.get(phase + "_end")
            if p0 is None or p1 is None:
                continue
            meas[phase] = gps_to_local(p1[0], p1[1], p0[0], p0[1])
        if not meas:
            return None, None, "NO GPS"

        num_re = num_im = den = 0.0
        for phase, (cn, ce) in (("pulse_n", (L, 0.0)), ("pulse_e", (0.0, L))):
            if phase not in meas:
                continue
            dn, de = meas[phase]
            num_re += dn * cn + de * ce
            num_im += de * cn - dn * ce
            den += cn * cn + ce * ce
        rot = math.degrees(math.atan2(num_im, num_re))
        gain = math.hypot(num_re, num_im) / den

        # Axis-swap check (needs both pulses): the N command must land mostly on
        # the E axis and the E command mostly on the N axis, each at a healthy
        # gain. A true ~90 deg rotation instead sends E south (dn_e < 0), so it
        # fails the sign test here and falls through to ROTATED below.
        if "pulse_n" in meas and "pulse_e" in meas:
            dn_n, de_n = meas["pulse_n"]
            dn_e, de_e = meas["pulse_e"]
            g_swap_n = de_n / L      # north command -> east response
            g_swap_e = dn_e / L      # east command  -> north response
            if (g_swap_n >= PROBE_MIN_GAIN and g_swap_e >= PROBE_MIN_GAIN
                    and abs(de_n) > abs(dn_n) and abs(dn_e) > abs(de_e)):
                return 90.0, 0.5 * (g_swap_n + g_swap_e), "SWAPPED"

        if gain < PROBE_MIN_GAIN:
            return rot, gain, "DEAD"
        if abs(rot) <= PROBE_OK_DEG:
            return rot, gain, "OK"
        if abs(rot) <= PROBE_ROTATED_DEG:
            return rot, gain, "SKEWED"
        return rot, gain, "ROTATED"

    # ---- reporting ----

    def _refuse(self, msg):
        print("[rotcheck] REFUSED: %s" % msg)
        if self.logger:
            self.logger.log_drone_command(0, "EVENT",
                                          cmd="ROTCHECK_REFUSED:%s" % msg)
        self._publish("refused", time.time(), msg=msg)
        return False

    def _publish(self, state, now, msg=None):
        if self.meta is None:
            return
        # Replace the whole dict (concurrency rule: GIL-atomic assignment).
        self.meta["rotation_check"] = {
            "state": state,
            "t": now,
            "drone": self._did,
            "phase": self._phase if self.active else None,
            "results": {str(d): dict(r) for d, r in self.results.items()},
            "msg": msg,
        }


# Reverse command channel: the browser GUI (swarm_gui.py) forwards Start/Stop
# button presses here as JSON UDP datagrams. Distinct from the joystick (:5055)
# and telemetry (:5099) ports.
DEFAULT_CMD_HOST = "127.0.0.1"
DEFAULT_CMD_PORT = 5098


def command_listener(swarming, meta, host, port, shapes_path=None):
    """Receive GUI command datagrams (Start/Stop, gimbal slider, heading
    mode + point-inwards + stitch-offset controls, vertical-plane toggle +
    gain, obstacle/geofence edits) over UDP.

    This thread NEVER touches ds_wrapper — it only mutates the shared `swarming`
    Event and the shared `meta` dict. The control loop (run) detects the edge /
    reads meta and performs the actual VS arm/disarm and gimbal set, so every
    hardware poke stays on the control-loop thread.

    Shape edits also persist to `shapes_path` (atomic write) so they survive a
    controller restart. CONCURRENCY RULE for meta["obstacles"]/meta["geofence"]/
    meta["facades"]: REPLACE, never mutate — always assign a brand-new list (or
    None) in a single statement. Dict item assignment is GIL-atomic, so run()
    sees either the old or the new complete object, never a half-edited one. An
    in-place .append() here would race the control loop's per-tick read.
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
                # each tick and handles the mode transition.
                v = (str(msg.get("value") or "")).lower()
                if v in ("manual", "convexhull", "demostitch") and meta is not None:
                    meta["heading_mode"] = v
                    print(f"[gui] heading mode -> {v}")
            elif action == "point_inwards":
                # GUI toggle: boundary drones face the centroid instead of
                # outward. run() forwards it to the hull controller.
                if meta is not None:
                    meta["point_inwards"] = bool(msg.get("value"))
                    print(f"[gui] point inwards -> {meta['point_inwards']}")
            elif action == "stitch_offset":
                # GUI number input: per-rank fan offset for demostitch mode
                # (deg). run() reads meta["stitch_offset"] live each tick.
                try:
                    v = float(msg.get("value"))
                except (TypeError, ValueError):
                    continue
                v = max(OFFSET_MIN_DEG, min(OFFSET_MAX_DEG, v))
                if meta is not None:
                    meta["stitch_offset"] = v
                    print(f"[gui] stitch offset -> {v:.0f}°")
            elif action == "plane":
                # GUI toggle: vertical-plane ("wall") swarming. Only a request
                # flag — run() owns the transition (it needs the position/
                # altitude snapshot to seed the wall, and it gates entry on the
                # reported-altitude spread), and clears this back to False if it
                # refuses.
                if meta is not None:
                    meta["plane_mode"] = bool(msg.get("value"))
                    print(f"[gui] vertical plane -> "
                          f"{'ON' if meta['plane_mode'] else 'off'}")
            elif action == "plane_gain":
                # GUI number input: restoring pull onto the plane (m/s per
                # metre of out-of-plane offset). run() reads it live each tick.
                try:
                    v = float(msg.get("value"))
                except (TypeError, ValueError):
                    continue
                v = max(PLANE_GAIN_MIN, min(PLANE_GAIN_MAX, v))
                if meta is not None:
                    meta["plane_gain"] = v
                    print(f"[gui] plane gain -> {v:.2f}")
            elif action == "rotation_check":
                # GUI button: open-loop actuation probe (RotationProbe). Only
                # a request marker — run() starts/ticks the probe on the
                # control-loop thread, and only while swarming is held.
                if meta is not None:
                    meta["rotation_check_req"] = time.time()
                    print("[gui] rotation check requested")
            elif action in ("record_start", "record_stop"):
                # GUI Record button: clip recording (video + the flight data
                # for the window). Only a request marker — run() owns the
                # transition, because start() creates folders, spawns encoder
                # threads and installs frame sinks, and because keeping
                # start/stop/auto-stop on ONE thread makes them unable to
                # interleave. Replace, never mutate.
                if meta is not None:
                    meta["record_req"] = {"cmd": action[len("record_"):],
                                          "t": time.time()}
                    print(f"[gui] clip recording {action[len('record_'):]} "
                          f"requested")
            elif action == "add_obstacle":
                # GUI map drag: a lat/lon-axis-aligned rectangle given by two
                # opposite corners. Validation/normalization is shared with
                # swarm_gui.py (olfati_saber.normalize_obstacle).
                if meta is None:
                    continue
                new_ob = normalize_obstacle(msg.get("lat1"), msg.get("lon1"),
                                            msg.get("lat2"), msg.get("lon2"))
                if new_ob is None:
                    print("[gui] obstacle rejected: invalid or too small")
                    continue
                current = meta.get("obstacles") or []
                if len(current) >= MAX_OBSTACLES:
                    print(f"[gui] obstacle rejected: limit of "
                          f"{MAX_OBSTACLES} reached")
                    continue
                new_ob["id"] = max([ob["id"] for ob in current] or [0]) + 1
                meta["obstacles"] = current + [new_ob]   # replace, not mutate
                if shapes_path:
                    save_shapes(shapes_path, meta["obstacles"], meta.get("geofence"))
                print(f"[gui] obstacle {new_ob['id']} added "
                      f"({len(meta['obstacles'])} total)")
            elif action == "add_building":
                # GUI "Pick building": a real footprint resolved from the map
                # click (building_footprint.py). Stored as a polygon obstacle,
                # which ALSO carries its bounding box — see the compatibility
                # contract in olfati_saber.py. The flock avoids the outline
                # rather than a box around it; the box is only what an older
                # reader of shapes.json falls back to.
                if meta is None:
                    continue
                new_ob = normalize_polygon_obstacle(
                    msg.get("vertices"), msg.get("label"), msg.get("source"))
                if new_ob is None:
                    print("[gui] building rejected: invalid outline")
                    continue
                current = meta.get("obstacles") or []
                if len(current) >= MAX_OBSTACLES:
                    print(f"[gui] building rejected: limit of "
                          f"{MAX_OBSTACLES} reached")
                    continue
                new_ob["id"] = max([ob["id"] for ob in current] or [0]) + 1
                meta["obstacles"] = current + [new_ob]   # replace, not mutate
                if shapes_path:
                    save_shapes(shapes_path, meta["obstacles"],
                                meta.get("geofence"), meta.get("facades"))
                print(f"[gui] building {new_ob['id']} added: "
                      f"{new_ob.get('label', 'building')} "
                      f"({len(new_ob['vertices'])} walls, "
                      f"{len(meta['obstacles'])} obstacles total)")
            elif action == "add_facade":
                # One wall of a footprint, kept as a PLANAR inspection plane.
                #
                # NO LONGER INERT. run() reads meta["facades"] every tick and
                # auto-picks the wall the formation is filming, whose standoff is
                # published to the Unity stitcher. So adding one MID-FLIGHT can step
                # the published scene plane — bounded to one deliberate transition by
                # pick_facade's dwell, and announced on the console and the GUI chip,
                # but real. It still repels nothing: a facade is a measurement
                # surface, not a keep-out shape.
                if meta is None:
                    continue
                pair = validate_facade(msg.get("p1"), msg.get("p2"))
                if pair is None:
                    print("[gui] facade rejected: not a wall")
                    continue
                current = meta.get("facades") or []
                if len(current) >= MAX_FACADES:
                    print(f"[gui] facade rejected: limit of "
                          f"{MAX_FACADES} reached")
                    continue
                rec = {"id": max([f["id"] for f in current] or [0]) + 1,
                       "p1": pair[0], "p2": pair[1]}
                try:
                    if msg.get("obstacle_id") is not None:
                        rec["obstacle_id"] = int(msg["obstacle_id"])
                except (TypeError, ValueError):
                    pass
                for key in ("label", "source"):
                    if msg.get(key):
                        rec[key] = str(msg[key])[:120]
                meta["facades"] = current + [rec]        # replace, not mutate
                if shapes_path:
                    save_shapes(shapes_path, meta.get("obstacles") or [],
                                meta.get("geofence"), meta["facades"])
                print(f"[gui] facade {rec['id']} saved: "
                      f"{rec.get('label', 'wall')} "
                      f"(--set-plane-from-facade {rec['id']})")
            elif action == "delete_facade":
                if meta is None:
                    continue
                try:
                    fa_id = int(msg.get("id"))
                except (TypeError, ValueError):
                    continue
                current = meta.get("facades") or []
                meta["facades"] = [f for f in current if f["id"] != fa_id]
                if shapes_path:
                    save_shapes(shapes_path, meta.get("obstacles") or [],
                                meta.get("geofence"), meta["facades"])
                print(f"[gui] facade {fa_id} deleted "
                      f"({len(meta['facades'])} remain)")
            elif action == "clear_facades":
                if meta is None:
                    continue
                meta["facades"] = []
                if shapes_path:
                    save_shapes(shapes_path, meta.get("obstacles") or [],
                                meta.get("geofence"), [])
                print("[gui] all facades cleared")
            elif action == "delete_obstacle":
                if meta is None:
                    continue
                try:
                    ob_id = int(msg.get("id"))
                except (TypeError, ValueError):
                    continue
                current = meta.get("obstacles") or []
                meta["obstacles"] = [ob for ob in current if ob["id"] != ob_id]
                if shapes_path:
                    save_shapes(shapes_path, meta["obstacles"], meta.get("geofence"))
                print(f"[gui] obstacle {ob_id} deleted "
                      f"({len(meta['obstacles'])} remain)")
            elif action == "clear_obstacles":
                if meta is None:
                    continue
                meta["obstacles"] = []
                if shapes_path:
                    save_shapes(shapes_path, [], meta.get("geofence"))
                print("[gui] all obstacles cleared")
            elif action == "set_geofence":
                # GUI polygon: ordered [lat, lon] vertices. Replaces any
                # existing fence. run() picks it up on its next tick — a drone
                # already outside is braked and removed immediately.
                if meta is None:
                    continue
                fence = validate_fence(msg.get("vertices"))
                if fence is None:
                    continue
                meta["geofence"] = fence                 # replace, not mutate
                if shapes_path:
                    save_shapes(shapes_path, meta.get("obstacles") or [], fence)
                print(f"[gui] geofence set ({len(fence)} vertices)")
            elif action == "clear_geofence":
                if meta is None:
                    continue
                meta["geofence"] = None
                if shapes_path:
                    save_shapes(shapes_path, meta.get("obstacles") or [], None)
                print("[gui] geofence cleared")
        except Exception:
            continue  # ignore malformed packets, keep listening


# ---------- PLANAR scene plane ----------

def _scene_plane_meta(pick):
    """The GUI's view of the scene plane. JSON-safe, and without `candidates` --
    that list is for the offline checker, not for a 5 Hz telemetry push."""
    if not pick:
        return None
    out = {k: pick.get(k) for k in
           ("facade_id", "label", "standoff_m", "look_off_deg", "tilt_deg",
            "spread_m", "view_count", "state")}
    out["reason"] = pick.get("reason") or ""
    return out


def _announce_scene_plane(pick, said):
    """Print one line when the scene plane changes. Returns the new `said` key.

    Prints on a CHANGE only, never per tick -- but note what counts as a change
    includes falling back to nothing. With no operator override flag, a flight with
    no facade traced silently uses whatever Unity's inspector holds and otherwise
    looks exactly like success, so "no wall" has to be as loud as a wall.
    """
    fid = pick.get("facade_id", -1) if pick else -1
    state = pick.get("state") if pick else "none"
    key = (fid, state)
    if key == said:
        return said
    if fid < 0:
        print("[plane] no facade picked ({}). PLANAR will use Unity's own "
              "planarStandoffMetres — trace the wall on the GUI map to drive it "
              "from here.".format(pick.get("reason") or "no reason given"))
    else:
        print("[plane] facade {}{}: {:.2f} m standoff, {:.0f} deg off the wall, "
              "{} views{}".format(
                  fid, " ({})".format(pick["label"]) if pick.get("label") else "",
                  pick.get("standoff_m") or 0.0, pick.get("look_off_deg") or 0.0,
                  pick.get("view_count") or 0,
                  " [settling]" if state == "dwelling" else ""))
    return key


# ---------- main loop ----------

def run(swarm, receiver, olfati, swarming, dry_run=False, vel_frame="ned",
        logger=None, speed_scale=1.0, meta=None, heading_ctrl=None,
        stitch_ctrl=None, cmd_sender=None, identity_check=True,
        min_separation=DEFAULT_MIN_SEPARATION_M, avoid=None,
        plane_ctrl=None, max_alt=MAX_ALT_M, recorder=None,
        pose_solver=None, img_stream=None):
    print("\n--- Olfati-Saber Swarm Mode ---")
    print(f"  Drones: {sorted(swarm.drones.keys())}")
    print(f"  c_vm={olfati.c_vm}  r0_coh={olfati.r0_coh}  scale={olfati.scale}")
    if min_separation > 0:
        print(f"  Min-separation failsafe: auto-STOP below {min_separation:.1f} m")
    else:
        print(f"  Min-separation failsafe: DISABLED")
    mode0 = (meta or {}).get("heading_mode", "manual")
    print(f"  Heading: {mode0} — switchable live from the GUI. "
          f"manual = stick yaw steers a shared target heading; "
          f"convexhull = GLOBAL_CONVEXHULL (boundary drones face outward, "
          f"interior drones hold heading; stick yaw rotates the command "
          f"reference frame instead of steering the drones); "
          f"demostitch = the laterally-middle drone points at the stick yaw, "
          f"neighbours fan out by meta['stitch_offset']° per rank (camera "
          f"overlap for stitching)")
    if plane_ctrl is not None:
        print(f"  Vertical plane: toggle from the GUI — the swarm re-forms as a "
              f"wall facing the stick-steered heading, on PER-DRONE altitude "
              f"setpoints (leash ±{plane_ctrl.leash_m:.0f} m, gain "
              f"{plane_ctrl.gain:.2f} m/s per m). Entry is refused if the "
              f"reported altitudes disagree by more than "
              f"{ALT_SPREAD_GATE_M:.1f} m — launch from one flat pad.")
    print(f"  Altitude band: {MIN_ALT_M:.0f}–{max_alt:.0f} m")
    print(f"  Telemetry velocity frame: {vel_frame}  "
          f"(switch via --vel-frame if consensus oscillates)")
    if speed_scale != 1.0:
        print(f"  SLOW TEST MODE: commanded velocities/rates scaled to {speed_scale:.0%}")
    print(f"  GUI Start/Stop buttons: toggle swarming (arm VS + flock)")
    print(f"  q  (PC keyboard): stop, hold position, disable VS, exit")
    print(f"  Ctrl+C: same, abrupt\n")

    target_yaw = 0.0
    # Convex-hull mode: the yaw stick does not steer the drones (the hull owns
    # their headings), so it instead rotates the operator's command reference
    # frame. cmd_frame_yaw is the compass heading that stick-forward (linear.x)
    # points along; stick-right (linear.y) is 90° clockwise of it. 0 = world
    # frame (forward=north). Seeded from the swarm mean heading on entering
    # hull mode, then integrated from the stick like a free (unclamped) yaw.
    cmd_frame_yaw = 0.0
    target_alt = INITIAL_ALT_M
    vs_on = False
    last_heading_mode = mode0           # detect GUI mode switches (below)
    last_plane_req = False              # detect the GUI vertical-plane toggle
    last_downwash = ()                  # advisory pairs, to print each change once
    last_swarming = swarming.is_set()   # starts cleared = held (do nothing)
    last_gimbal = None                  # last gimbal pitch applied to the drones
    last_t = time.time()
    last_print = 0.0
    no_fix_warned = set()
    # Geofence breach bookkeeping. `removed` = drone ids braked + VS-disabled
    # after crossing outside the fence; they get no commands and are not
    # neighbours until swarming is stopped and started again (cleared on the
    # rising edge below). `pending_disable` maps a breached drone id to the
    # deadline after which its DISABLE_VS goes out — a non-blocking 0.4 s
    # brake window (a time.sleep here would freeze every OTHER drone's
    # control for ~8 ticks; one zero-velocity command suffices because the
    # app re-sends the last VS command at its own 20 Hz).
    removed = set()
    pending_disable = {}

    # Live command->response rotation fit per drone (logged to swarm_debug +
    # published as meta["resp"] for the GUI). Fed only with what was actually
    # sent, so it stays silent in --dry-run and while held.
    monitor = ResponseMonitor()
    # Open-loop actuation probe (GUI "Rotation check"); created on request,
    # ticked from the held branch below.
    probe = None
    last_probe_req = (meta or {}).get("rotation_check_req")
    # GUI clip recorder (video + windowed flight data); the marker's timestamp
    # is what dedupes it, so a repeated UDP datagram is a no-op.
    last_record_req = ((meta or {}).get("record_req") or {}).get("t")
    # Facade auto-pick carry-over: the incumbent, its dwelling challenger and the
    # low-passed standoff. Held here rather than inside pick_facade so that function
    # stays pure and the offline checker can reproduce this loop's decisions exactly.
    plane_state = None
    plane_said = None

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

        # Swarming edge → arm/disarm VS. Done here (not in the listener thread)
        # so every ds_wrapper poke stays on this control-loop thread. Stop (and
        # 'q') work with no joystick connected; Start is gated on a live one
        # below. The joystick s1/s2 switches are intentionally NOT wired to
        # swarming/LAND — Start/Stop is GUI-only, so a switch left 'on' at
        # connect can't auto-arm the swarm.
        sw = swarming.is_set()
        if sw and not last_swarming:
            # Joystick gate: refuse to arm without a live joystick feed. js is
            # None when nothing fresh arrived on :5055 inside the receiver's
            # staleness window (readController.py not running, or the physical
            # controller unplugged) — armed drones would flock with zero
            # operator input and no way to steer them clear of trouble.
            # --dry-run skips the gate (desk testing, nothing flies).
            if js is None and not dry_run:
                print("[swarm] REFUSING TO ARM: no joystick input — start "
                      "readController.py / reconnect the controller, then "
                      "press Start again")
                if logger:
                    logger.log_drone_command(0, "EVENT",
                                             cmd="START_REFUSED_NO_JOYSTICK")
                swarming.clear()
                time.sleep(0.02)
                continue
            # A running rotation check must not overlap the swarm arming
            # (Start wins; the probe's partial results stay in the GUI).
            if probe is not None and probe.active:
                probe.abort(now, "swarming started")
            # Rising: re-verify the command<->telemetry identity before arming.
            # The probe is inert (marker strings the app ignores) and takes
            # ~0.5 s per drone, so it runs on every Start — an RC re-plugged or
            # a server slot reconnected mid-session gets caught here.
            if identity_check and cmd_sender is not None and not dry_run:
                print("[swarm] verifying command<->telemetry identity...")
                if not run_identity_check(swarm, cmd_sender, logger):
                    print("[swarm] REFUSING TO ARM: fix the DroneIPs order / "
                          "server slots (or relaunch with --no-identity-check "
                          "to override) and press Start again")
                    swarming.clear()
                    time.sleep(0.02)
                    continue
            # Arm VS. Seed heading from a drone with a fix, and the shared
            # altitude target from the AVERAGE altitude of all drones we actually
            # have telemetry from (e.g. with --drones 3 but only 2 connected, just
            # those 2), floored at START_ALT_FLOOR_M.
            fixes = [d.telemetry for d in swarm.drones.values() if d.telemetry]
            alts = [t['alt'] for t in fixes if t.get('alt') is not None]
            mh = swarm_mean_heading(swarm)
            if mh is not None:
                target_yaw = mh
            if alts:
                target_alt = max(sum(alts) / len(alts), START_ALT_FLOOR_M)
            else:
                target_alt = max(INITIAL_ALT_M, START_ALT_FLOOR_M)
            target_alt = min(target_alt, max_alt)
            # A wall from a previous stint must not carry over: Start re-enters
            # plane mode from scratch (below) if the GUI toggle is still on.
            if plane_ctrl is not None:
                plane_ctrl.reset()
            # Re-admit any geofence-breached drones: Stop→Start is the
            # explicit operator action that clears the removed set, and
            # enable_vs_all() below re-arms them along with everyone else.
            if removed:
                print(f"[fence] breach list cleared "
                      f"(re-admitting drones {sorted(removed)})")
                removed.clear()
            pending_disable.clear()
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
            # Abandon the wall: each drone was just commanded its OWN current
            # altitude and now GPS-hovers there, so there is nothing left to
            # ramp. The GUI toggle keeps its state; the next Start rebuilds.
            if plane_ctrl is not None and (plane_ctrl.active or plane_ctrl.ramping):
                print("[plane] wall abandoned on Stop — drones hold their "
                      "current altitudes")
                plane_ctrl.reset()
            if meta is not None:
                meta["resp"] = {}   # live rotation fits are meaningless held
        last_swarming = sw

        # RF link quality pushed by the app, for EVERY drone. Built here rather
        # than in the command loop below for two reasons: that loop is behind
        # `if not sw: continue`, and pre-flight (held) is exactly when the
        # operator wants to see the radio, and it also skips drones with no GPS
        # fix and geofence-removed ones — precisely the drones whose link is
        # worth looking at. None per drone on the server command path (no
        # app->PC diagnostic channel) or once its feed goes stale.
        link_map = {
            str(did): (cmd_sender.link_of(did) if cmd_sender is not None
                       else None)
            for did in swarm.drones
        }

        if meta is not None:
            meta["link"] = link_map
            meta["swarming"] = sw
            # Joystick feed health for the GUI: False greys out the Start
            # button and shows a NO JOYSTICK chip (the arm gate above is the
            # authoritative check — this is just the operator-visible mirror).
            meta["joystick"] = js is not None
            # Live list of geofence-breached (VS-disabled) drone ids for the
            # GUI's FENCED OUT badges; refreshed every tick, also while held,
            # so a Stop→Start visibly clears it.
            meta["removed"] = sorted(removed)
            # Wall state every tick, INCLUDING while held — the plane block
            # below sits behind the swarming gate, so publishing only from
            # there would leave the GUI showing a WALL chip after a Stop
            # abandoned it. The block re-publishes with fresher data mid-tick.
            if plane_ctrl is not None:
                meta["plane"] = plane_ctrl.status()
            if not sw:
                meta["downwash"] = []   # stacking is only meaningful under command
            # A rotation-check click while armed is refused NOW — consuming it
            # here stops it from firing as a surprise right after Stop.
            req = meta.get("rotation_check_req")
            if sw and req is not None and req != last_probe_req:
                last_probe_req = req
                print("[rotcheck] refused: stop swarming first")
                meta["rotation_check"] = {
                    "state": "refused", "t": now, "drone": None,
                    "phase": None, "results": {},
                    "msg": "stop swarming first"}
            # Publish the live target spacing to the GUI even while held (before
            # the swarming gate below), so the operator can see what d_ref the
            # joystick angular.x currently maps to *before* pressing Start. When
            # swarming this is recomputed identically in the control block.
            if js is not None:
                meta["d_ref_m"] = round(
                    d_ref_from_ax(js.angular_x, scale=olfati.scale) * olfati.scale, 2)

        # GUI Record button + the clip duration cap. Deliberately OUTSIDE the
        # swarming gate below and allowed in --dry-run: recording is read-only,
        # it must work while held (the operator lines the shot up before Start),
        # and a Stop — including a min-separation or geofence auto-STOP — must
        # NOT cut the clip. A clip spanning an incident, with the DISABLE_VS
        # visible in its own drone_commands.csv, is exactly the artifact worth
        # having.
        if recorder is not None:
            if meta is not None:
                req = meta.get("record_req") or {}
                if req.get("t") is not None and req["t"] != last_record_req:
                    last_record_req = req["t"]
                    if req.get("cmd") == "start":
                        if recorder.start() and logger:
                            logger.log_drone_command(
                                0, "EVENT",
                                cmd=f"CLIP_START:{recorder.status()['clip']}")
                    elif recorder.stop("operator") and logger:
                        logger.log_drone_command(0, "EVENT", cmd="CLIP_STOP")
            if recorder.poll():
                print("[clip] duration limit reached — saving")
                if recorder.stop("max_duration") and logger:
                    logger.log_drone_command(0, "EVENT", cmd="CLIP_STOP_MAX")
            if meta is not None:
                meta["recording"] = recorder.status()

        # GUI gimbal slider: apply the shared pitch target to every drone
        # whenever it changes. set_gimbal only updates the send-loop's cached
        # value (no ds_wrapper poke here); the 20 Hz send loop relays it and the
        # gimbal actually moves once VS is enabled.
        if meta is not None:
            gp = meta.get("gimbal_pitch")
            if gp is not None and gp != last_gimbal:
                for d in swarm.drones.values():
                    d.set_gimbal(gp, 0.0)
                if logger and last_gimbal is not None:
                    # Skip the startup application (== --gimbal-pitch, already
                    # in session.json); log only live GUI retargets.
                    logger.log_drone_command(
                        0, "EVENT", cmd=f"GIMBAL_PITCH:{gp:+.1f}")
                last_gimbal = gp

        # Heading mode + point-inwards + stitch offset are runtime-switchable
        # from the GUI (command_listener writes meta["heading_mode"]/
        # meta["point_inwards"]/meta["stitch_offset"]; the CLI flags just seed
        # them). Applied here, outside the swarming gate, so the operator can
        # preselect the mode while held.
        # Vertical-plane request (GUI toggle). Read here, before the swarming
        # gate, so the heading override below is visible while held; the wall
        # itself is only built while swarming (further down, where the position/
        # altitude snapshot exists).
        plane_req = bool(meta.get("plane_mode")) if meta is not None else False
        if plane_ctrl is not None and meta is not None:
            try:
                plane_ctrl.gain = max(PLANE_GAIN_MIN, min(
                    PLANE_GAIN_MAX, float(meta.get("plane_gain",
                                                   plane_ctrl.gain))))
            except (TypeError, ValueError):
                pass
            if plane_req != last_plane_req:
                print(f"[plane] vertical plane -> "
                      f"{'ON' if plane_req else 'off'}")
                if logger:
                    logger.log_drone_command(
                        0, "EVENT", cmd=f"PLANE_MODE:{plane_req}")
                last_plane_req = plane_req

        hull_mode = False
        stitch_mode = False
        if meta is not None and heading_ctrl is not None:
            mode = meta.get("heading_mode")
            if mode not in ("manual", "convexhull", "demostitch"):
                mode = "manual"
            # Vertical plane forces MANUAL heading. Both other modes degenerate
            # on a wall: the convex hull is computed from horizontal positions,
            # which collapse to a line, and demostitch ranks drones laterally,
            # which stacked drones cannot be. Manual is also exactly what the
            # sim does in plane mode — every nose on the plane azimuth (=
            # target_yaw), so the whole wall of cameras faces out of it.
            if plane_req and mode != "manual":
                print(f"[plane] heading mode {mode} not available on a wall "
                      f"-> manual")
                mode = "manual"
                meta["heading_mode"] = mode   # so the GUI shows what is in force
            hull_mode = mode == "convexhull"
            stitch_mode = mode == "demostitch" and stitch_ctrl is not None
            if mode != last_heading_mode:
                mh = swarm_mean_heading(swarm)
                if hull_mode:
                    # Fresh activation: stale debounce timers / held targets
                    # from a previous stint must not leak in.
                    heading_ctrl.reset()
                    # Seed the command reference frame from where the swarm
                    # currently points, so stick-forward starts out matching
                    # the swarm's mean heading before the operator rotates it.
                    cmd_frame_yaw = mh if mh is not None else 0.0
                else:
                    # Manual or demostitch: re-seed the shared target from the
                    # live mean heading so drones don't snap to a stale
                    # target_yaw. For demostitch that means the fan builds out
                    # around wherever the swarm already points — the initial
                    # fan-out is the only motion.
                    if stitch_mode:
                        stitch_ctrl.reset()
                    if mh is not None:
                        target_yaw = mh
                if not hull_mode:
                    meta["hull_boundary"] = []   # nothing is hull-steered now
                if not stitch_mode:
                    meta["stitch_centre"] = None
                print(f"[heading] mode -> {mode}")
                if logger:
                    logger.log_drone_command(0, "EVENT", cmd=f"HEADING_MODE:{mode}")
                last_heading_mode = mode
            pin = bool(meta.get("point_inwards"))
            if pin != heading_ctrl.point_inwards:
                heading_ctrl.set_point_inwards(pin)
                print(f"[heading] point inwards -> {pin}")
                if logger:
                    logger.log_drone_command(0, "EVENT", cmd=f"POINT_INWARDS:{pin}")

        # Held → VS is off and the drones hover autonomously until Start is
        # pressed again. This is also the only place the rotation check may
        # run (it arms/pulses one drone at a time on this thread).
        if not sw:
            if meta is not None:
                req = meta.get("rotation_check_req")
                if req is not None and req != last_probe_req:
                    last_probe_req = req
                    if probe is not None and probe.active:
                        print("[rotcheck] already running — request ignored")
                    elif dry_run:
                        print("[rotcheck] not available in --dry-run")
                        meta["rotation_check"] = {
                            "state": "refused", "t": now, "drone": None,
                            "phase": None, "results": {},
                            "msg": "not available in --dry-run"}
                    else:
                        probe = RotationProbe(swarm, meta, logger,
                                              min_separation=min_separation)
                        probe.start(now)
            if probe is not None and probe.active:
                probe.tick(now)
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
        # so the stick doesn't steer the drones. Instead it rotates the
        # operator's command reference frame (cmd_frame_yaw): a free integrator
        # with no lead clamp, since there is no measured heading to servo it
        # against — it's a pure operator-chosen frame. In manual mode the
        # integration steers the shared target heading, lead-clamped against the
        # swarm's mean heading (anti-windup — see integrate_target_heading) so
        # releasing the stick leaves at most MAX_TARGET_LEAD_DEG of catch-up turn.
        # Demostitch shares the manual path exactly: the stick steers the same
        # target_yaw (= the fan's centre direction); each drone's per-rank
        # offset rides on top of it (below).
        ff_yaw_rate = ang_z * YAW_RATE_DEG_S * speed_scale
        if not hull_mode:
            target_yaw = integrate_target_heading(
                target_yaw, ff_yaw_rate, dt, swarm_mean_heading(swarm))
        else:
            cmd_frame_yaw = _wrap180(cmd_frame_yaw + ff_yaw_rate * dt)
        # Shared altitude target (horizontal mode). In vertical-plane mode the
        # per-drone setpoints below take over and the climb stick moves the
        # whole wall instead; this keeps integrating so the hand-back on exit
        # lands somewhere sensible.
        target_alt = max(MIN_ALT_M, min(max_alt,
                                        target_alt + lin_z * VERT_RATE_MPS * speed_scale * dt))
        # d_ref (scaled units); physical spacing = d_ref * scale. Already
        # published to the GUI as meta["d_ref_m"] every loop above (pre-gate).
        d_ref = d_ref_from_ax(js.angular_x, scale=olfati.scale)

        # World-frame group desired velocity, shared across the swarm: every
        # drone tries to move in the same compass direction regardless of its
        # own heading. The DJI VS (GROUND mode) does the world→body rotation
        # per drone internally, so we always hand it world N/E.
        if hull_mode:
            # Convex-hull mode: the stick commands motion in the operator's
            # rotatable command frame (steered by the yaw stick above), so
            # rotate stick forward/right by cmd_frame_yaw into world N/E.
            v_n_des, v_e_des = body_to_world(
                lin_x * MAX_PITCH_MPS, lin_y * MAX_ROLL_MPS, cmd_frame_yaw)
        else:
            # Manual AND demostitch: raw world frame (forward = north).
            # Demostitch deliberately shares this — rotating translation by
            # target_yaw ("fly along the view direction") is a possible future
            # variant, not done.
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
            # ResponseMonitor keeps its own FIXED reference internally — the
            # per-tick centroid ref below drifts with the swarm and would
            # alias into the GPS-derived velocities.
            monitor.note_fix(did, now, t['lat'], t['lon'])
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
            # Altitude rides along in the snapshot: the 3D separation failsafe
            # and vertical-plane mode both need it. Takeoff-relative per
            # aircraft (SwarmActivity KeyAircraftLocation3D) — the same
            # reference the VS throttle setpoint is in.
            snap[did] = (pos_ne, vel_ne, t['heading'], t.get('alt'))

        # Virtual obstacles / geofence: convert the GUI-drawn lat/lon shapes
        # into local N/E metres with the SAME per-tick reference point as the
        # drone positions above, so shapes and drones always share one frame
        # no matter how the ref drifts with the swarm. Cheap (a few
        # gps_to_local calls). The meta lists are replaced whole by
        # command_listener (never mutated), so reading them here is safe.
        obstacles = (meta or {}).get("obstacles") or []
        fence = (meta or {}).get("geofence")
        # Partition by kind. A polygon obstacle (a picked building footprint)
        # ALSO carries a bounding box for old readers, so it satisfies both
        # branches — feeding it to both would count its repulsion TWICE.
        rects_ne = [rect_to_ne(ob, lat_ref, lon_ref) for ob in obstacles
                    if ob.get("kind") != "poly"]
        polys_ne = [polygon_to_ne(ob["vertices"], lat_ref, lon_ref)
                    for ob in obstacles
                    if ob.get("kind") == "poly" and ob.get("vertices")]
        fence_ne = (polygon_to_ne(fence, lat_ref, lon_ref)
                    if fence and len(fence) >= 3 else None)

        # PLANAR scene plane: which traced wall is the formation filming, and how far
        # in front of it is it? Published into the feed map's trailer for the Unity
        # stitcher, replacing a number that used to be typed into an inspector field
        # by hand -- and a wrong one there is the single largest error in the mosaic.
        #
        # Deliberately NOT in the per-tick lat_ref/lon_ref frame the obstacles above
        # use. That reference drifts with the swarm, which is right for flocking and
        # wrong here: the stitcher builds its plane from the poses the BLOCKS carry,
        # which are in CameraPoseSolver's latched frame, and a standoff derived in a
        # different frame would be describing a wall in one frame and cameras in
        # another. Using the solver's own pose_for() also means the centroid here is
        # byte-for-byte the centroid those poses describe -- and it is what latches
        # the origin at all on a run with no --image-stream-pose.
        if img_stream is not None and pose_solver is not None:
            pose_views = []
            for did, t in fixes:
                p, q, st = pose_solver.pose_for(t)
                if st:
                    pose_views.append((did, p, scene_plane.forward_from_quat(q)))
            plane_pick, plane_state = scene_plane.pick_facade(
                (meta or {}).get("facades") or [], pose_solver.origin,
                pose_views, plane_state, now)
            if plane_pick.get("facade_id", -1) >= 0:
                img_stream.set_standoff(plane_pick, now)
            else:
                img_stream.set_standoff(None, now)
            if meta is not None:
                meta["scene_plane"] = _scene_plane_meta(plane_pick)
            plane_said = _announce_scene_plane(plane_pick, plane_said)

        # Minimum-separation failsafe: any pair too close -> auto-STOP swarming
        # (the falling edge above then zeroes velocities, brakes, and disables
        # VS, exactly like the GUI Stop button; the drones GPS-hover apart).
        # TRUE 3D distance, not horizontal: vertical-plane mode stacks drones
        # deliberately, and a horizontal-only check would read a forming wall as
        # 0 m separated and stop the swarm the moment it started working. The
        # vertical component is only as good as the per-aircraft takeoff-relative
        # altitude frames (launch from one flat pad).
        if min_separation > 0 and len(snap) >= 2:
            ids_ = sorted(snap.keys())
            tripped = None
            for i, a in enumerate(ids_):
                for b in ids_[i+1:]:
                    (pa, _, _, alt_a) = snap[a]
                    (pb, _, _, alt_b) = snap[b]
                    d = separation_3d(pa, alt_a, pb, alt_b)
                    if d < min_separation:
                        tripped = (a, b, d)
                        break
                if tripped:
                    break
            if tripped:
                a, b, d = tripped
                print(f"[FAILSAFE] drones {a}-{b} at {d:.2f} m "
                      f"(< {min_separation:.1f} m) — STOPPING swarm")
                if logger:
                    logger.log_drone_command(
                        0, "EVENT", cmd=f"MINSEP_STOP:{a}-{b}:{d:.2f}m")
                swarming.clear()
                time.sleep(0.02)
                continue

        # Downwash advisory. A pair this close HORIZONTALLY with an altitude
        # difference has the upper drone's prop wash over the lower one — the
        # hazard a vertical wall creates by design and the Unity sim models not
        # at all (its drones are independent rigid bodies). Advisory ONLY: it
        # warns, logs and shows a GUI chip, and never stops the swarm; the 3D
        # min-separation check above is the hard failsafe.
        downwash = []
        if len(snap) >= 2:
            ids_ = sorted(snap.keys())
            for i, a in enumerate(ids_):
                for b in ids_[i+1:]:
                    (pa, _, _, alt_a) = snap[a]
                    (pb, _, _, alt_b) = snap[b]
                    if alt_a is None or alt_b is None:
                        continue
                    if abs(alt_a - alt_b) < 0.5:
                        continue    # level flight: no wash geometry
                    if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < DOWNWASH_RADIUS_M:
                        hi, lo = (a, b) if alt_a > alt_b else (b, a)
                        downwash.append([hi, lo])
        if meta is not None:
            meta["downwash"] = downwash      # replace whole (concurrency rule)
        dw_key = tuple(tuple(p) for p in downwash)
        if dw_key != last_downwash:
            for hi, lo in downwash:
                if [hi, lo] not in [list(p) for p in last_downwash]:
                    print(f"[DOWNWASH] drone {hi} is directly above drone {lo} "
                          f"(< {DOWNWASH_RADIUS_M:.1f} m horizontally) — prop "
                          f"wash on the lower aircraft")
                    if logger:
                        logger.log_drone_command(
                            0, "EVENT", cmd=f"DOWNWASH:{hi}over{lo}")
            last_downwash = dw_key

        # Geofence hard cutoff: a drone whose GPS fix lands OUTSIDE the fence
        # polygon is braked (one zero-velocity command; the app re-sends it at
        # 20 Hz), then gets DISABLE_VS after a 0.4 s non-blocking brake window
        # and is removed from the flock — no commands, not a neighbour — until
        # swarming is stopped and started again. The soft inward repulsion
        # (avoid.fence_force) is the real protection; this is the failsafe.
        # Note breached drones stay in `snap`, so the min-separation failsafe
        # above still covers the airspace they hover in.
        if fence_ne is not None:
            for did in snap:
                if did in removed:
                    continue
                (n_pos, e_pos), _, _, _ = snap[did]
                if point_in_polygon(n_pos, e_pos, fence_ne):
                    continue
                removed.add(did)
                t = swarm.drones[did].telemetry
                hold_alt = (max(t.get('alt', target_alt), MIN_ALT_M)
                            if t else target_alt)
                if not dry_run:
                    swarm.drones[did].set_velocity(0.0, 0.0, 0.0, hold_alt)
                pending_disable[did] = now + 0.4
                print(f"[FENCE] drone {did} OUTSIDE geofence — braking, "
                      f"DISABLE_VS in 0.4 s; removed from flock until "
                      f"swarming restart")
                if logger:
                    logger.log_drone_command(did, "EVENT", cmd="FENCE_BREACH")
        for did in list(pending_disable):
            if now >= pending_disable[did]:
                if not dry_run:
                    swarm.drones[did].disable_vs()
                del pending_disable[did]
                print(f"[FENCE] drone {did} VS disabled — GPS-hovering; "
                      f"recover on the RC. Stop/Start re-admits it.")
                if logger:
                    logger.log_drone_command(did, "EVENT",
                                             cmd="FENCE_DISABLE_VS")

        # Convex-hull heading control: derive a per-drone target heading from
        # the swarm's hull (boundary drones face outward along their vertex
        # bisector; interior drones get None → hold current heading).
        # Geofence-breached drones must not steer the flock's facing.
        hull_targets = None
        if hull_mode:
            hull_targets = heading_ctrl.update(
                {did: pos for did, (pos, _, _, _) in snap.items()
                 if did not in removed}, dt)
            if meta is not None:
                meta["hull_boundary"] = heading_ctrl.boundary_ids()

        # Demo-stitch heading control: the laterally-middle drone points at
        # the shared target_yaw, neighbours fan out by stitch_offset per rank
        # (adjacent camera views keep partial overlap for the stitcher).
        # Geofence-breached drones must not occupy a rank in the fan.
        stitch_targets = None
        stitch_off = None
        if stitch_mode:
            try:
                stitch_off = float((meta or {}).get("stitch_offset",
                                                    DEFAULT_OFFSET_DEG))
            except (TypeError, ValueError):
                stitch_off = DEFAULT_OFFSET_DEG
            stitch_targets = stitch_ctrl.update(
                {did: pos for did, (pos, _, _, _) in snap.items()
                 if did not in removed},
                target_yaw, stitch_off, dt)
            if meta is not None:
                meta["stitch_centre"] = stitch_ctrl.centre_id

        # ---- Vertical-plane ("wall") swarming ----
        # The plane is a swap of the constraint the SAME cohesion law runs
        # under: in-plane forces come from the untouched 2D
        # OlfatiSaber.GetSwarmAcceleration, evaluated in the wall's own axes
        # (swarm_plane.py). Its vertical component becomes a PER-DRONE altitude
        # setpoint, because DJI VS gives us absolute-altitude control and no
        # vertical velocity channel.
        #
        # plane_corr / plane_alt are the two things it hands to the per-drone
        # loop below: a horizontal correction (replacing the 2D one) and an
        # altitude (replacing the shared target_alt). During the exit ramp only
        # the altitude is ours — the horizontal command comes from the normal 2D
        # path again, since the plane is already gone.
        plane_corr = None
        plane_alt = None
        plane_vup = None
        if plane_ctrl is not None:
            fix_pos = {did: pos for did, (pos, _, _, _) in snap.items()
                       if did not in removed}
            fix_alt = {did: a for did, (_, _, _, a) in snap.items()
                       if did not in removed and a is not None}
            fix_vel = {did: vel for did, (_, vel, _, _) in snap.items()
                       if did not in removed}
            if plane_req and not plane_ctrl.active and not plane_ctrl.ramping:
                # Entry gate. Every drone has been holding the SAME shared
                # target_alt, so the spread of their REPORTED altitudes measures
                # takeoff-frame bias plus tracking error directly. A wall built
                # on skewed frames has wrong vertical gaps — and the 3D
                # separation failsafe would be wrong by the same amount.
                spread = alt_spread(fix_alt.values())
                if len(fix_alt) < 2:
                    print("[plane] REFUSING: need at least 2 drones with a GPS "
                          "fix and an altitude")
                    meta["plane_mode"] = False
                elif spread > ALT_SPREAD_GATE_M:
                    print(f"[plane] REFUSING: reported altitudes disagree by "
                          f"{spread:.1f} m (> {ALT_SPREAD_GATE_M:.1f} m). "
                          f"Altitude is takeoff-relative per aircraft — launch "
                          f"from one flat pad, or let the swarm settle on the "
                          f"shared target first.")
                    if logger:
                        logger.log_drone_command(
                            0, "EVENT",
                            cmd=f"PLANE_REFUSED_ALT_SPREAD:{spread:.2f}m")
                    meta["plane_mode"] = False
                else:
                    seeded = plane_ctrl.enter(fix_pos, fix_alt, target_yaw,
                                              d_ref * olfati.scale)
                    # Headroom advisory. A wall of N drones at d_ref spacing
                    # needs roughly (N-1)*d_ref of vertical room around its
                    # reference altitude; too low and the bottom of the wall
                    # just piles up against MIN_ALT_M, so the formation is
                    # squashed and the low drones sit near the ground with a
                    # downward demand they can't follow. Warn rather than
                    # refuse — the operator may want a deliberately short wall.
                    need = 0.5 * (len(fix_alt) - 1) * d_ref * olfati.scale
                    if plane_ctrl.alt_ref - need < MIN_ALT_M:
                        print(f"[plane] WARNING: alt_ref {plane_ctrl.alt_ref:.1f} m "
                              f"leaves the wall short of ground clearance — "
                              f"{len(fix_alt)} drones at "
                              f"{d_ref * olfati.scale:.1f} m spacing want "
                              f"±{need:.1f} m, so the bottom row will clamp at "
                              f"the {MIN_ALT_M:.0f} m floor. Climb to "
                              f"~{need + MIN_ALT_M:.0f} m first.")
                        if logger:
                            logger.log_drone_command(
                                0, "EVENT",
                                cmd=f"PLANE_LOW_HEADROOM:{plane_ctrl.alt_ref:.1f}m")
                    seed_desc = "  ".join(f"{d}:{a:.1f}m"
                                          for d, a in sorted(seeded.items()))
                    print(f"[plane] wall ON: azimuth {target_yaw:+.1f}°, "
                          f"alt_ref {plane_ctrl.alt_ref:.1f} m, alt spread "
                          f"{spread:.2f} m, staggered seed  {seed_desc}")
                    if logger:
                        logger.log_drone_command(
                            0, "EVENT",
                            cmd=f"PLANE_ENTER:az={target_yaw:.1f}:"
                                f"alt_ref={plane_ctrl.alt_ref:.2f}:"
                                f"spread={spread:.2f}")
            elif not plane_req and plane_ctrl.active:
                # Leaving: converge the per-drone setpoints onto their mean at a
                # bounded rate before handing the vertical channel back, so no
                # drone gets a multi-metre step to fly at the FC's own pace.
                plane_ctrl.exit_ramp()
                print(f"[plane] wall OFF: ramping altitudes to "
                      f"{plane_ctrl.alt_ref:.1f} m")
                if logger:
                    logger.log_drone_command(
                        0, "EVENT",
                        cmd=f"PLANE_EXIT:alt_ref={plane_ctrl.alt_ref:.2f}")

            if plane_ctrl.active or plane_ctrl.ramping:
                was_ramping = plane_ctrl.ramping
                plane_out = plane_ctrl.update(
                    olfati, fix_pos, fix_alt, fix_vel, target_yaw, d_ref,
                    lin_z * VERT_RATE_MPS, dt, speed_scale=speed_scale,
                    exclude=removed)
                plane_alt = {did: v[2] for did, v in plane_out.items()}
                plane_vup = {did: v[3] for did, v in plane_out.items()}
                if plane_ctrl.active:
                    plane_corr = {did: (v[0], v[1])
                                  for did, v in plane_out.items()}
                if was_ramping and not plane_ctrl.ramping:
                    # Ramp finished: hand the vertical channel back with the
                    # shared target where the drones actually are.
                    target_alt = max(MIN_ALT_M,
                                     min(max_alt, plane_ctrl.alt_ref))
                    plane_alt = None
                    print(f"[plane] altitudes converged — shared target "
                          f"{target_alt:.1f} m")
                    plane_ctrl.reset()
            if meta is not None:
                meta["plane"] = plane_ctrl.status()

        # Per-drone flocking command. The joystick's desired velocity goes
        # straight through (DJI VS already runs a velocity tracker); the swarm
        # algorithm only contributes a correction (neighbour-velocity consensus
        # + cohesion) that gets added on top.
        resp_map = {}   # per-drone live rotation fit for the GUI, this tick
        for did, ctrl in swarm.drones.items():
            if did not in snap:
                continue  # no fix → DroneController holds last set_velocity
            if did in removed:
                continue  # fenced out: VS disabled, gets no commands
            try:
                self_pos, self_vel, hdg, _self_alt = snap[did]
                if hull_targets is not None:
                    # Hull mode: servo boundary drones onto their hull-derived
                    # heading (no stick feed-forward); interior drones hold
                    # their current heading (rate 0).
                    drone_target = hull_targets.get(did)
                    if drone_target is None:
                        cmd_yaw_rate = 0.0
                        drone_target = hdg  # for the dry-run printout
                    else:
                        # p_scale: --slow must slow the hull servo too, not
                        # just translation (historically it didn't, so a 20%
                        # flight still yawed at the full clamp).
                        cmd_yaw_rate = heading_hold_rate(
                            drone_target, hdg, p_scale=speed_scale)
                elif stitch_targets is not None:
                    # Demostitch: like manual (shared stick feed-forward, the
                    # whole fan rotates rigidly with target_yaw) but each
                    # drone servos onto its own rank-offset target.
                    drone_target = stitch_targets.get(did, target_yaw)
                    cmd_yaw_rate = heading_hold_rate(
                        drone_target, hdg, ff_yaw_rate, p_scale=speed_scale)
                else:
                    # Smooth yaw RATE for this drone: shared stick feed-forward
                    # plus a P term holding its own heading on the shared
                    # target_yaw (both scaled by --slow via speed_scale).
                    drone_target = target_yaw
                    cmd_yaw_rate = heading_hold_rate(
                        target_yaw, hdg, ff_yaw_rate, p_scale=speed_scale)
                # Geofence-breached drones are NOT neighbours: nothing coheres
                # toward (or velocity-matches) a fenced-out hoverer.
                neighbours = [(snap[j][0], snap[j][1]) for j in snap
                              if j != did and j not in removed]
                # Swarm correction (consensus + cohesion) in world frame. On a
                # vertical wall it comes from the SAME law evaluated in the
                # plane's axes (swarm_plane.py) and its vertical part has
                # already been folded into this drone's altitude setpoint; a
                # drone the plane could not place (no altitude reported) falls
                # back to the horizontal path.
                if plane_corr is not None and did in plane_corr:
                    v_n_corr, v_e_corr = plane_corr[did]
                else:
                    v_n_corr, v_e_corr = olfati.GetSwarmAcceleration(
                        self_pos, self_vel, neighbours, d_ref=d_ref,
                    )
                # Virtual obstacles + geofence soft repulsion (β-agent term).
                # Deliberately NOT projected onto the plane: avoidance is the
                # stronger authority and the plane's restoring term is clamped
                # (MAX_PLANE_MPS) so it can never win an argument with it.
                o_n, o_e = 0.0, 0.0
                if avoid is not None and (rects_ne or polys_ne or fence_ne):
                    o_n, o_e = avoid.GetObstacleForce(
                        self_pos, self_vel, rects_ne, fence_ne, polys_ne)
                v_n_total = v_n_des + v_n_corr + o_n
                v_e_total = v_e_des + v_e_corr + o_e
                v_n_total, v_e_total = clamp_mag2(v_n_total, v_e_total, MAX_CMD_MPS)
                # --slow: scale the combined command (joystick desired + swarm
                # correction) uniformly so the whole motion just runs slower —
                # the relative behaviour being tuned keeps its shape.
                v_n_total *= speed_scale
                v_e_total *= speed_scale
                # Altitude: the swarm's shared absolute target, unless the wall
                # (or its exit ramp) owns this drone's altitude. This is the
                # ONLY place a per-drone altitude is commanded.
                alt_cmd = target_alt
                if plane_alt is not None and did in plane_alt:
                    alt_cmd = plane_alt[did]
                # Live command->response rotation fit: feed what is actually
                # sent (post-scale), read back the sliding-window estimate.
                resp = None
                if not dry_run:
                    monitor.note_command(did, now, v_n_total, v_e_total)
                    resp = monitor.fit(did, now)
                resp_map[str(did)] = (
                    None if resp is None
                    else {"rot": round(resp[0], 1), "gain": round(resp[1], 2)})
                # Logging the link next to the control terms is the point: it
                # makes "the link was bad" checkable against the same timeline
                # as the commands that were sent.
                link = link_map[str(did)]
                if logger:
                    logger.log_swarm_debug(
                        did, v_n_des, v_e_des, v_n_corr, v_e_corr,
                        v_n_total, v_e_total, d_ref, len(neighbours),
                        v_n_obs=o_n, v_e_obs=o_e,
                        resp_rot_deg=None if resp is None else round(resp[0], 2),
                        resp_gain=None if resp is None else round(resp[1], 3),
                        link_sq=None if link is None else link["sq"],
                        link_down=None if link is None else link["down"],
                        link_up=None if link is None else link["up"],
                        plane_on=1 if plane_corr is not None else 0,
                        plane_az=(round(plane_ctrl.azimuth_deg, 1)
                                  if plane_corr is not None else None),
                        plane_off=(round(plane_ctrl.offsets[did], 2)
                                   if plane_corr is not None
                                   and did in plane_ctrl.offsets else None),
                        v_up=(round(plane_vup[did], 3)
                              if plane_vup is not None
                              and did in plane_vup else None),
                        alt_cmd=round(alt_cmd, 2))
                # DJI VS is in GROUND/VELOCITY mode (SwarmActivity sets
                # FlightCoordinateSystem.GROUND), so pitch = north m/s and
                # roll = east m/s. We send world-frame velocities directly —
                # the drone does its own world→body rotation internally.
                if dry_run:
                    plane_desc = ""
                    if plane_corr is not None:
                        plane_desc = (
                            f"  off={plane_ctrl.offsets.get(did, 0.0):+5.2f}m "
                            f"v_up={plane_vup.get(did, 0.0):+.2f}")
                    elif plane_alt is not None:
                        plane_desc = "  [ramp]"
                    print(f"  [dry] drone {did}  "
                          f"v_des=({v_n_des:+.2f}N,{v_e_des:+.2f}E)  "
                          f"corr=({v_n_corr:+.2f},{v_e_corr:+.2f})  "
                          f"obs=({o_n:+.2f},{o_e:+.2f})  "
                          f"cmd=({v_n_total:+.2f}N,{v_e_total:+.2f}E)  "
                          f"hdg={hdg:+6.1f}°->{drone_target:+.1f}° "
                          f"yawrate={cmd_yaw_rate:+5.1f}°/s  "
                          f"alt={alt_cmd:.1f}m{plane_desc}  "
                          f"d_ref={d_ref:.3f} (~{d_ref*olfati.scale:.1f}m)")
                else:
                    ctrl.set_velocity(v_n_total, v_e_total, cmd_yaw_rate, alt_cmd)
            except Exception as e:
                print(f"[drone {did}] flocking error: {e}")

        if meta is not None:
            meta["resp"] = resp_map   # replace whole (concurrency rule)

        if now - last_print > 1.0:
            if hull_mode:
                yaw_desc = ("cmdframe=%+6.1f°  hull-boundary=%s"
                            % (cmd_frame_yaw,
                               ",".join(map(str, heading_ctrl.boundary_ids())) or "none"))
            elif stitch_mode:
                yaw_desc = (f"yaw={target_yaw:+6.1f}°  "
                            f"stitch ±{stitch_off:.0f}°/rank  "
                            f"centre={stitch_ctrl.centre_id or '-'}")
            else:
                yaw_desc = f"yaw={target_yaw:+6.1f}°"
            if plane_ctrl is not None and (plane_ctrl.active or plane_ctrl.ramping):
                alt_desc = (f"WALL az={plane_ctrl.azimuth_deg:+6.1f}° "
                            f"alt_ref={(plane_ctrl.alt_ref or 0.0):5.1f}m"
                            + ("  [ramping]" if plane_ctrl.ramping else ""))
            else:
                alt_desc = f"alt={target_alt:5.1f}m"
            print(f"  v_des=({v_n_des:+5.2f}N,{v_e_des:+5.2f}E) world  "
                  f"{yaw_desc}  {alt_desc}  "
                  f"d_ref={d_ref:.3f} (~{d_ref*olfati.scale:.1f}m)  "
                  f"fixes={len(snap)}/{len(swarm.drones)}  "
                  f"SWARM={'ON' if sw else 'off'}  VS={'ON' if vs_on else 'off'}")
            # Per-drone local position + the per-drone world-frame command
            # (which equals v_des once the swarm correction is added).
            for did in sorted(snap.keys()):
                (n_m, e_m), self_vel, hdg, drone_alt = snap[did]
                v_n_self, v_e_self = self_vel
                extra = ""
                if plane_ctrl is not None and did in plane_ctrl.offsets:
                    extra = (f"  off={plane_ctrl.offsets[did]:+5.2f}m"
                             f"  alt*={plane_ctrl.alt_cmd.get(did, 0.0):5.1f}m")
                print(f"    drone {did}  pos=({n_m:+6.2f}N,{e_m:+6.2f}E)  "
                      f"alt={(drone_alt or 0.0):5.1f}m  hdg={hdg:+6.1f}°  "
                      f"vel=({v_n_self:+5.2f}N,{v_e_self:+5.2f}E){extra}")
            # Pairwise distances (physical metres, 3D — same measure the
            # min-separation failsafe uses, so the two never disagree).
            ids = sorted(snap.keys())
            if len(ids) >= 2:
                pairs = []
                for i, a in enumerate(ids):
                    for b in ids[i+1:]:
                        (pa, _, _, alt_a) = snap[a]
                        (pb, _, _, alt_b) = snap[b]
                        d = separation_3d(pa, alt_a, pb, alt_b)
                        pairs.append(f"{a}-{b}={d:5.2f}m")
                print(f"    distances: {'  '.join(pairs)}")
            last_print = now

        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm Olfati-Saber flocking controller")
    ap.add_argument("--drones", type=int, default=1,
                    help="Number of drones (creates IDs 1..N)")
    ap.add_argument("--drone-ips", default="",
                    help="Command-path selector. A comma-separated RC/broker "
                         "IP list ordered by drone id (e.g. "
                         "192.168.100.173,192.168.100.176) pins drone ids to "
                         "those RCs and publishes commands DIRECTLY to each "
                         "RC's MQTT broker (20 Hz capable; needs at least "
                         "--drones addresses, extras ignored). Empty or "
                         "'auto' (default) AUTO-DISCOVERS the RC IPs from "
                         "DroneSwarmServer's established RTSP connections and "
                         "binds them to server slots with the marker probe — "
                         "drone id = server slot, no config needed. 'server' "
                         "forces the legacy path via DroneSwarmServer "
                         "(~4.5 Hz per-command reconnect).")
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
    ap.add_argument("--heading", choices=["manual", "convexhull", "demostitch"],
                    default="manual",
                    help="Initial heading-control mode (live-switchable from the "
                         "GUI afterwards): 'manual' (default) integrates a shared "
                         "target heading from the stick's angular.z; 'convexhull' "
                         "ports the Unity sim's GLOBAL_CONVEXHULL attitude "
                         "algorithm — drones on the swarm's convex hull face "
                         "outward along their vertex bisector, interior drones "
                         "hold heading; the stick yaw then rotates the "
                         "operator's command reference frame instead of "
                         "steering the drones. 'demostitch' points the "
                         "laterally-middle drone at the stick-steered yaw and "
                         "fans each neighbour out by --stitch-offset per rank, "
                         "keeping adjacent camera views overlapped for "
                         "stitching.")
    ap.add_argument("--point-inwards", action="store_true",
                    help="Seed the point-inwards toggle (live-switchable from "
                         "the GUI): in convexhull mode, boundary drones face "
                         "the swarm centroid instead of outward.")
    ap.add_argument("--stitch-offset", type=float, default=DEFAULT_OFFSET_DEG,
                    metavar="DEG",
                    help="demostitch mode: per-rank heading offset between "
                         "laterally adjacent drones (deg). Seeds "
                         "meta['stitch_offset']; live-adjustable from the GUI. "
                         f"Range [{OFFSET_MIN_DEG:.0f}, {OFFSET_MAX_DEG:.0f}], "
                         f"default {DEFAULT_OFFSET_DEG:.0f}.")
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
    ap.add_argument("--no-identity-check", action="store_true",
                    help="Skip the command<->telemetry identity probe run at "
                         "startup and on every swarming Start. The probe sends "
                         "an inert marker through DroneSwarmServer's per-slot "
                         "MQTT path and watches which DroneIPs broker it lands "
                         "on, catching a server-slot/DroneIPs order mismatch "
                         "(cross-wired control loops) before arming. Only "
                         "meaningful on the direct-MQTT path.")
    ap.add_argument("--min-separation", type=float,
                    default=DEFAULT_MIN_SEPARATION_M, metavar="M",
                    help="Failsafe: auto-STOP swarming when any drone pair "
                         f"gets closer than this many metres, measured in 3D "
                         f"(default {DEFAULT_MIN_SEPARATION_M:.1f}; 0 disables)")
    ap.add_argument("--plane-mode", action="store_true",
                    help="Start with vertical-plane ('wall') swarming enabled: "
                         "the swarm re-forms as a wall facing the stick-steered "
                         "heading, on per-drone altitude setpoints (port of the "
                         "Unity sim's SwarmPlaneController). Toggleable live "
                         "from the GUI; this flag only seeds the toggle. Forces "
                         "manual heading (hull/demostitch degenerate on a "
                         "wall) and is refused if the drones' reported "
                         f"altitudes disagree by more than "
                         f"{ALT_SPREAD_GATE_M:.1f} m.")
    ap.add_argument("--plane-gain", type=float, default=DEFAULT_PLANE_GAIN,
                    help="Vertical-plane restoring pull: m/s of horizontal "
                         "command per metre of out-of-plane offset (default "
                         f"{DEFAULT_PLANE_GAIN:.2f}, range "
                         f"[{PLANE_GAIN_MIN:.2f}, {PLANE_GAIN_MAX:.2f}]; the "
                         "term itself is clamped so obstacle/geofence "
                         "avoidance always outranks it). Live-tunable from the "
                         "GUI.")
    ap.add_argument("--plane-leash", type=float,
                    default=DEFAULT_PLANE_LEASH_M, metavar="M",
                    help="Vertical-plane leash: how far one drone's altitude "
                         "setpoint may sit from the wall's reference altitude "
                         f"(default {DEFAULT_PLANE_LEASH_M:.0f} m). Bounds the "
                         "wall's vertical extent and stops a runaway climb.")
    ap.add_argument("--max-alt", type=float, default=MAX_ALT_M, metavar="M",
                    help="Ceiling for every commanded altitude (default "
                         f"{MAX_ALT_M:.0f} m). Raise it for a tall vertical "
                         "wall — N drones at d_ref spacing need roughly "
                         "(N-1)*d_ref of vertical room, centred well above "
                         f"the {MIN_ALT_M:.0f} m floor. Check your site's "
                         "legal ceiling first.")
    ap.add_argument("--airlink-bands", default="", metavar="LIST",
                    help="Per-drone RF band assignment, sent to each RC as a "
                         "one-shot AIRLINK command at startup: comma list in "
                         "drone-id order of 2G4 | 5G8 | DUAL | '-' (leave that "
                         "drone unchanged); a single value applies to every "
                         "drone. Empty (default) sends nothing. With many "
                         "co-located OcuSync links, splitting the fleet "
                         "across 2.4/5.8 GHz halves the contenders per band.")
    ap.add_argument("--airlink-bandwidth", default="", metavar="LIST",
                    help="Per-drone AirLink channel bandwidth in MHz (comma "
                         "list in drone-id order of 40 | 20 | 10 | 5 | '-'; a "
                         "single value applies to every drone). Narrower = "
                         "less spectrum occupied per link and a lower data "
                         "rate, which is the main lever when many co-located "
                         "links share the band — and unlike manual channel "
                         "pinning it works with DJI's AUTO channel selection. "
                         "Empty (default) sends nothing.")
    ap.add_argument("--airlink-mode", default="", choices=["", "auto", "manual"],
                    metavar="auto|manual",
                    help="AirLink channel-selection mode. Empty (default) sends "
                         "nothing and leaves whatever is in force. 'manual' is "
                         "needed for --airlink-bandwidth to stick (in AUTO the "
                         "aircraft accepts a bandwidth then reverts within "
                         "~3 s) but FREEZES the channel and gives up DJI's "
                         "per-link interference adaptation, which DJI "
                         "recommends keeping; it is only sent alongside a "
                         "band/bandwidth request, and AUTO is restored "
                         "automatically for any drone where the value still "
                         "did not hold. 'auto' restores DJI's adaptive "
                         "selection — the way back out, since blanking the "
                         "config does NOT undo a previous manual run.")
    ap.add_argument("--no-link-scan", action="store_true",
                    help="Skip the read-only pre-flight link scan (the "
                         "LINKDIAG round-trip that reports each RC's radio "
                         "config and the aircraft's own interference sweep). "
                         "The scan never changes a setting; skipping it only "
                         f"saves the {LINK_SCAN_TIMEOUT_S:.0f} s wait.")
    ap.add_argument("--video-mode", default="", metavar="WxH@FPS",
                    help="Camera stream cap applied on every RC at startup, "
                         "e.g. 1920x1080@24 (lower encoded bitrate = more "
                         "airlink headroom per link). Empty (default) leaves "
                         "the camera as-is.")
    ap.add_argument("--d-obs", type=float, default=5.0, metavar="M",
                    help="Obstacle/geofence repulsion cutoff in PHYSICAL "
                         "metres (default 5.0). The beta-agent kernel runs at "
                         "d_obs/scale in scaled units so the repulsion stays "
                         "commensurate with the cohesion forces; the push is "
                         "maximal at contact and exactly 0 beyond this range.")
    ap.add_argument("--r0-obs", type=float, default=6.0, metavar="M",
                    help="Obstacle detection radius in PHYSICAL metres "
                         "(default 6.0; must be >= --d-obs). Beyond this an "
                         "obstacle is ignored entirely.")
    ap.add_argument("--c-obs", type=float, default=4.3,
                    help="Obstacle/geofence repulsion gain (default 4.3, "
                         "matching the Unity sim; max push ~= c_obs * 1.45 "
                         "m/s at contact).")
    ap.add_argument("--shapes-file", default=DEFAULT_SHAPES_FILE,
                    metavar="PATH",
                    help="JSON file the GUI-drawn obstacles/geofence persist "
                         f"to (default {DEFAULT_SHAPES_FILE}; relative paths "
                         "resolve against this script's directory). Loaded "
                         "at startup, rewritten on every edit.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print VS commands but do not send to drones")
    ap.add_argument("--no-gui", action="store_true",
                    help="Do not push telemetry to the browser GUI (swarm_gui.py)")
    ap.add_argument("--image-stream", action="store_true",
                    help="Publish 800x450 frames to the DroneFeedSharedMemory "
                         "feed (Unity ImageSharing.cs -> stitcher) in-process, "
                         "reusing the telemetry "
                         "threads' image fetches (replaces running the "
                         "standalone image_stream.py, which contends with this "
                         "controller for the ds_wrapper protocol)")
    ap.add_argument("--image-stream-pose", action="store_true",
                    help="Include a per-frame camera pose (GPS + gimbal attitude -> "
                         "Unity world) in each published block. Required by the sim's "
                         "PLANAR stitcher, which computes its homographies from pose "
                         "and cannot run without it; ignored by STABSTITCH. Implies "
                         "--image-stream")
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
    ap.add_argument("--recording-dir", default=DEFAULT_RECORDING_DIR,
                    help=f"Root for GUI-triggered clip recordings — per-drone "
                         f"1080p MP4 + frame index + the flight data for the "
                         f"window (default {DEFAULT_RECORDING_DIR}/, separate "
                         f"from --log-dir)")
    ap.add_argument("--record-max-s", type=float, default=DEFAULT_MAX_SECONDS,
                    help=f"Hard cap on one clip in seconds; recording "
                         f"auto-stops there (default {DEFAULT_MAX_SECONDS:.0f})")
    args = ap.parse_args()

    if args.drones < 1:
        ap.error("--drones must be >= 1")
    if args.slow <= 0:
        ap.error("--slow SCALE must be > 0")
    if args.min_separation < 0:
        ap.error("--min-separation must be >= 0 (0 disables the failsafe)")
    if args.d_obs <= 0:
        ap.error("--d-obs must be > 0")
    if args.r0_obs < args.d_obs:
        ap.error("--r0-obs must be >= --d-obs (an obstacle must be detected "
                 "at least as far out as it repels)")
    if args.c_obs < 0:
        ap.error("--c-obs must be >= 0")
    if not (GIMBAL_PITCH_MIN <= args.gimbal_pitch <= GIMBAL_PITCH_MAX):
        ap.error(f"--gimbal-pitch must be in "
                 f"[{GIMBAL_PITCH_MIN:.0f}, {GIMBAL_PITCH_MAX:.0f}] (DJI Mini 3 Pro)")
    if not (OFFSET_MIN_DEG <= args.stitch_offset <= OFFSET_MAX_DEG):
        ap.error(f"--stitch-offset must be in "
                 f"[{OFFSET_MIN_DEG:.0f}, {OFFSET_MAX_DEG:.0f}]")
    if not (PLANE_GAIN_MIN <= args.plane_gain <= PLANE_GAIN_MAX):
        ap.error(f"--plane-gain must be in "
                 f"[{PLANE_GAIN_MIN:.2f}, {PLANE_GAIN_MAX:.2f}]")
    if args.plane_leash <= 0:
        ap.error("--plane-leash must be > 0")
    if args.max_alt <= MIN_ALT_M:
        ap.error(f"--max-alt must be > the {MIN_ALT_M:.0f} m floor")
    if not (MAX_SECONDS_MIN <= args.record_max_s <= MAX_SECONDS_MAX):
        ap.error(f"--record-max-s must be in "
                 f"[{MAX_SECONDS_MIN:.0f}, {MAX_SECONDS_MAX:.0f}] seconds")

    def per_drone_list(raw, flag):
        """Expand a comma list to one token per drone (a single token fans
        out to all); [] when the flag wasn't given."""
        toks = [t.strip() for t in raw.split(",") if t.strip()]
        if not toks:
            return []
        if len(toks) == 1:
            return toks * args.drones
        if len(toks) < args.drones:
            ap.error(f"--{flag} has {len(toks)} entries but --drones is "
                     f"{args.drones} (give one value, or one per drone)")
        return toks[:args.drones]

    airlink_bands = [b.upper() for b in
                     per_drone_list(args.airlink_bands, "airlink-bands")]
    for b in airlink_bands:
        if b not in ("2G4", "5G8", "DUAL", "-"):
            ap.error(f"--airlink-bands: unknown band '{b}' "
                     "(expected 2G4, 5G8, DUAL or -)")
    airlink_bw = per_drone_list(args.airlink_bandwidth, "airlink-bandwidth")
    for b in airlink_bw:
        if b not in ("40", "20", "10", "5", "-"):
            ap.error(f"--airlink-bandwidth: '{b}' is not one of "
                     "40, 20, 10, 5 or '-' (MHz)")
    video_mode = args.video_mode.strip()
    if video_mode and not re.match(r"^\d+x\d+@\d+$", video_mode):
        ap.error("--video-mode must look like 1920x1080@24")
    # Command-path mode: 'explicit' (IP list = canonical drone ids),
    # 'auto' (discover IPs from the server, drone id = slot), or
    # 'server' (legacy path through DroneSwarmServer).
    raw_ips = args.drone_ips.strip()
    if raw_ips.lower() in ("", "auto"):
        cmd_mode = "auto"
        drone_ips = []
    elif raw_ips.lower() in ("server", "legacy"):
        cmd_mode = "server"
        drone_ips = []
    else:
        cmd_mode = "explicit"
        drone_ips = [ip.strip() for ip in raw_ips.split(",") if ip.strip()]
        if len(drone_ips) < args.drones:
            ap.error(f"--drone-ips has only {len(drone_ips)} address(es) but "
                     f"--drones is {args.drones} (order = drone id)")
        # Extras are fine: the config lists every switch port; only the first
        # --drones entries are used.

    print("LIS_Swarm Flocking Controller (Olfati-Saber)")
    hw_decode = w.isHWDecoderEnabled()
    print(f"  ds_wrapper HW Decoder: "
          f"{'enabled' if hw_decode == 1 else 'disabled (SW)'}")

    swarm = SwarmController()
    for did in range(1, args.drones + 1):
        swarm.add_drone(did)
        print(f"  Added drone {did}")

    # Direct MQTT command path: one persistent connection per RC broker.
    # Telemetry/video still flow through DroneSwarmServer either way.
    cmd_sender = None
    if cmd_mode == "explicit":
        cmd_sender = MqttCommandSender(
            {did: drone_ips[did - 1] for did in swarm.drones})
        # Give the background network threads a moment to connect so the
        # status print below is meaningful; auto-reconnect keeps trying
        # regardless, so an offline RC does not block launch.
        deadline = time.monotonic() + 3.0
        while (time.monotonic() < deadline and
               not all(cmd_sender.connected(did) for did in swarm.drones)):
            time.sleep(0.1)
        for did in sorted(swarm.drones):
            state = ("connected" if cmd_sender.connected(did)
                     else "NOT connected yet (auto-reconnect active)")
            print(f"  Command path: direct MQTT -> drone {did} "
                  f"@ {drone_ips[did - 1]} [{state}]")
    elif cmd_mode == "auto":
        # Discover the connected RCs from the server itself (its per-slot RTSP
        # control connections), then bind slots to brokers with the marker
        # probe. Drone id = server slot: one identity source, no possible
        # command<->telemetry mismatch.
        print("  Discovering RC IPs from DroneSwarmServer's RTSP connections...")
        candidates = discover_rc_ips()
        if not candidates:
            print("  WARNING: no RC streams found — is DroneSwarmServer "
                  "running with drones connected? Falling back to the server "
                  "command path (~4.5 Hz).")
        else:
            print(f"  Found {len(candidates)} RC(s): {', '.join(candidates)}")
            cmd_sender = auto_bind_command_channels(swarm, candidates)
            if cmd_sender is None:
                print("  WARNING: slot->RC binding failed — falling back to "
                      "the server command path (~4.5 Hz). Fix the brokers/"
                      "server slots and relaunch for 20 Hz commands.")
    else:
        print("  Command path: via DroneSwarmServer (~4.5 Hz max; forced "
              "by --drone-ips server)")
    if cmd_sender is not None:
        for ctrl in swarm.drones.values():
            ctrl.command_sender = cmd_sender
    resolved_ips = ([cmd_sender.ip_of(did) for did in sorted(swarm.drones)]
                    if cmd_sender is not None else [])

    logger = None
    if not args.no_log:
        logger = FlightLogger(base_dir=args.log_dir, meta={
            "script": "swarm_flocking",
            "drones": args.drones, "port": args.port,
            "c_vm": args.c_vm, "r0": args.r0, "scale": args.scale,
            "vel_frame": args.vel_frame, "dry_run": args.dry_run,
            "slow": args.slow, "gimbal_pitch": args.gimbal_pitch,
            "image_stream": args.image_stream or args.image_stream_pose,
            "image_stream_pose": args.image_stream_pose,
            "cmd_mode": cmd_mode,
            "drone_ips": resolved_ips,
            "identity_check": not args.no_identity_check,
            "min_separation": args.min_separation,
            "plane_mode": args.plane_mode, "plane_gain": args.plane_gain,
            "plane_leash": args.plane_leash, "max_alt": args.max_alt,
            "d_obs": args.d_obs, "r0_obs": args.r0_obs, "c_obs": args.c_obs,
            "shapes_file": args.shapes_file,
            "recording_dir": args.recording_dir,
            "record_max_s": args.record_max_s,
            "airlink_bands": airlink_bands,
            "airlink_bandwidth": airlink_bw,
            "video_mode": video_mode,
        })
        swarm.attach_logger(logger)
        print(f"  Flight logging -> {logger.session_dir} (disable with --no-log)")

    # Synchronous probe: call sendWayPointData once per drone from the main
    # thread BEFORE starting any background threads. If a slot is missing in
    # DroneSwarmServer.exe the C extension may block here without releasing
    # the GIL, which would otherwise starve the main thread silently. Only
    # meaningful on the server send path — the direct MQTT path never touches
    # the wrapper for commands (its connect status was printed above).
    if cmd_sender is None:
        for did, drone in sorted(swarm.drones.items()):
            print(f"  Probing drone {did} (sendWayPointData)... ", end="", flush=True)
            try:
                drone.send_vs()
                print("OK", flush=True)
            except Exception as e:
                print(f"FAIL: {e}", flush=True)

    # One-shot AirLink / camera-stream setup ("AIRLINK:band=..:bw=..:video=..",
    # parsed by SwarmActivity.applyAirlinkSettings). Per-drone radio config
    # lives HERE — the RCs run identical APKs, and addressing follows the
    # command channel (drone id), the same identity commands use. QoS 1 on the
    # direct path queues the message for an RC that is still connecting; each
    # RC shows the applied/rejected result on its own status line. Placed
    # after the synchronous probe so a missing server slot surfaces there
    # first (on the server path this send blocks in the wrapper too).
    #
    # Fields are named, so only what was configured is sent — an RC running an
    # older APK reports an unknown field instead of silently applying a value
    # to whatever used to occupy that position.
    airlink_sent = {}    # drone id -> {"band": tok, "bw": tok} actually sent
    if cmd_sender is not None:
        # Drop anything from a previous controller run so the outcomes printed
        # below can only belong to the AIRLINK we are about to send.
        cmd_sender.clear_airlink_results()
    if ((airlink_bands or airlink_bw or video_mode or args.airlink_mode)
            and not args.dry_run):
        for did in sorted(swarm.drones):
            fields = []
            asked = {}
            if airlink_bands and airlink_bands[did - 1] != "-":
                fields.append("band=" + airlink_bands[did - 1])
                asked["band"] = airlink_bands[did - 1]
            if airlink_bw and airlink_bw[did - 1] != "-":
                fields.append("bw=" + airlink_bw[did - 1])
                asked["bw"] = airlink_bw[did - 1]
            # MANUAL is only worth entering to make a RADIO field stick, so it
            # rides along with band/bw and never alone — a bare mode change
            # would freeze the channel for no benefit. AUTO is the opposite: it
            # GIVES adaptivity back, so it goes out unconditionally and is the
            # documented way to undo a manual run.
            if args.airlink_mode == "auto":
                fields.insert(0, "mode=AUTO")
            elif args.airlink_mode == "manual" and asked:
                fields.insert(0, "mode=MANUAL")
            if video_mode:
                fields.append("video=" + video_mode)
            if not fields:
                continue     # this drone was explicitly skipped with '-'
            cmd = "AIRLINK:" + ":".join(fields)
            print(f"  AirLink setup -> drone {did}: {cmd}", flush=True)
            swarm.drones[did].send_command(cmd)
            if asked:
                airlink_sent[did] = asked

    # Pre-flight link scan. DJI's auto channel selection is per-link and has no
    # view of the fleet, so the useful question before flying is what the
    # aircraft themselves see: KeyFrequencyInterference gives an RSSI per
    # frequency bucket, i.e. a band scan from the antenna that matters. Purely
    # read-only, and only available on the direct MQTT path (the app answers on
    # its own broker, which is what cmd_sender is subscribed to).
    if cmd_sender is not None and not args.dry_run and not args.no_link_scan:
        if airlink_sent:
            # See AIRLINK_SETTLE_S: without this the scan reads back the value
            # from BEFORE the one-shot applied, which is indistinguishable from
            # the aircraft rejecting it.
            print(f"  Letting AirLink settle {AIRLINK_SETTLE_S:.0f}s before "
                  f"reading it back...", flush=True)
            time.sleep(AIRLINK_SETTLE_S)
            # What each RC made of the one-shot, reported over the diagnostic
            # channel. The RC's own status line is a single TextView that the
            # next message overwrites, so this is the only place the outcome is
            # readable per drone without adb.
            for did in sorted(airlink_sent):
                results = cmd_sender.airlink_results(did)
                if not results:
                    print(f"    drone {did}: RC reported no AirLink outcome "
                          f"(older APK — check its screen/logcat)", flush=True)
                for text in results:
                    print(f"    drone {did}: {text}", flush=True)
        print("  Requesting link scan from each RC (read-only)...", flush=True)
        cmd_sender.clear_scans()
        for did in sorted(swarm.drones):
            cmd_sender.request_scan(did)
        # One shared window rather than per-drone waits: the requests all went
        # out already, so the answers come back in parallel.
        deadline = time.monotonic() + LINK_SCAN_TIMEOUT_S
        pending = set(swarm.drones)
        while pending and time.monotonic() < deadline:
            for did in sorted(pending):
                if cmd_sender.scan_of(did) is not None:
                    pending.discard(did)
            if pending:
                time.sleep(0.1)
        scans = {}
        stuck = set()   # drones where a requested radio value did not take
        for did in sorted(swarm.drones):
            scan = cmd_sender.scan_of(did)
            scans[did] = scan
            if scan is None:
                print(f"    drone {did}: no answer "
                      f"(older APK, or RC not connected yet)", flush=True)
                continue
            print(f"    drone {did}: {scan}", flush=True)
            # Did the aircraft actually take what we sent it? The set callback
            # only tells the RC's own status line; this is the PC-side check,
            # and it reads the radio back through a different key path than
            # the one that wrote it.
            got = MqttCommandSender.parse_scan(scan)
            for field, tok in sorted(airlink_sent.get(did, {}).items()):
                want = _AIRLINK_EXPECTED.get(field, {}).get(tok)
                have = got.get(field)
                if want is None or have is None:
                    continue
                if have == "?":
                    print(f"      NOTE: asked for {field}={tok}; the aircraft "
                          f"will not report {field} back, so it cannot be "
                          f"confirmed either way", flush=True)
                elif have != want:
                    stuck.add(did)
                    print(f"      WARNING: asked for {field}={tok} ({want}) "
                          f"but the aircraft reports {have} — NOT APPLIED. "
                          f"See this drone's AIRLINK outcome above for the "
                          f"SDK's own verdict.", flush=True)
        # MANUAL was entered only to make a radio value stick. If it did not
        # stick anyway, leaving the aircraft in MANUAL buys nothing and costs
        # DJI's own channel selection, so put it back — an experiment that
        # fails should not silently change how the fleet picks channels.
        if args.airlink_mode == "manual" and stuck:
            print(f"  MANUAL did not make the setting hold on "
                  f"{sorted(stuck)} — restoring AUTO channel selection",
                  flush=True)
            for did in sorted(stuck):
                swarm.drones[did].send_command("AIRLINK:mode=AUTO")
        elif args.airlink_mode == "manual":
            print("  MANUAL channel mode IS IN FORCE: the channel is frozen "
                  "and DJI's per-link interference adaptation is off. "
                  "--airlink-mode auto puts it back.", flush=True)

        if logger:
            # A one-shot pre-flight fact, so it belongs in session.json next to
            # the rest of the run config — not in a per-tick CSV.
            logger.update_meta({"link_scan": scans,
                                "airlink_mode": args.airlink_mode,
                                "airlink_not_applied": sorted(stuck)})
    elif cmd_sender is None and not args.dry_run and not args.no_link_scan:
        # Say so rather than silently skipping: the launcher banner prints
        # linkScan=True, which would otherwise look like it ran and found
        # nothing.
        print("  Link scan n/a: the server command path has no app->PC "
              "return channel (needs DroneIPs auto or an explicit IP list)")

    # Start background threads one drone at a time, with a brief sleep so each
    # thread can do its first iteration and surface any error before we move on.
    print(f"  Starting send + telemetry threads...")
    for did, drone in sorted(swarm.drones.items()):
        print(f"    starting drone {did}... ", end="", flush=True)
        drone.start(send_rate_hz=20, telemetry_rate_hz=20)
        time.sleep(0.3)
        print("done", flush=True)
    print(f"  Started: 20 Hz commands, 20 Hz telemetry")

    # Startup identity probe: verify DroneIPs order matches the server's slot
    # order before anything flies (re-verified on every swarming Start in
    # run()). Failure here is a warning only — arming is where it hard-blocks,
    # so the operator can still fix server slots and press Start. In auto
    # mode the startup probe is redundant (the slot->RC binding above IS the
    # probe), but the Start-edge re-check still runs — it catches an RC or
    # server slot swapped mid-session.
    identity_check = (not args.no_identity_check) and not args.dry_run
    if cmd_sender is not None:
        if not identity_check:
            print("  Identity check DISABLED — trusting the command<->"
                  "telemetry mapping as-is")
        elif cmd_mode == "auto":
            print("  Identity bound by discovery (drone id = server slot); "
                  "re-verified on every swarming Start")
        else:
            print("  Verifying command<->telemetry identity (inert MQTT "
                  "marker probe)...")
            if not run_identity_check(swarm, cmd_sender, logger):
                print("  WARNING: identity unverified — swarming Start will "
                      "re-probe and refuse to arm until it passes "
                      "(--no-identity-check to override)")
    else:
        print("  Identity check n/a: server command path uses one identity "
              "(slot) for commands and telemetry by construction")

    # ONE solver for the whole fleet, SHARED by the live image stream and the
    # clip recorder: it latches a single GPS origin on the first valid fix, and
    # every drone's position — live or recorded — has to be measured from that
    # same origin or the poses are not in a common frame. A second instance
    # would silently put the recording in a different frame from the feed.
    # Always constructed (it holds one lat/lon and does nothing until asked).
    pose_solver = CameraPoseSolver()

    # Optional in-process image streaming to the stitcher pipeline. Fed by the
    # telemetry threads' existing fetches (a DroneController frame sink), so
    # it adds no ds_wrapper calls and cannot slow the cmd/telem rates.
    img_stream = None
    if args.image_stream or args.image_stream_pose:
        img_stream = ImageStreamPublisher(
            swarm.drones, hw_decode,
            pose_solver=pose_solver if args.image_stream_pose else None)
        img_stream.start()
        print(f"  Image stream -> DroneFeedSharedMemory "
              f"({args.drones} x 800x450, <=20 Hz per drone"
              f"{', with camera pose' if args.image_stream_pose else ''})")

    # GUI-triggered clip recording. Always available (the button is in the
    # controls bar); constructing it touches no disk and starts no threads, so
    # a run where Record is never pressed costs nothing. Like the image stream
    # it rides the telemetry threads' existing fetches — both can be active at
    # once now that frame sinks are a list. It gets the pose solver
    # unconditionally (not gated on --image-stream-pose): a clip without a
    # per-frame camera pose cannot drive the sim's PLANAR stitcher, and the
    # solve is a handful of flops on data already in hand.
    recording_dir = args.recording_dir
    if not os.path.isabs(recording_dir):
        recording_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), recording_dir)
    recorder = ClipRecorder(swarm.drones, hw_decode, base_dir=recording_dir,
                            max_seconds=args.record_max_s, logger=logger,
                            meta={"script": "swarm_flocking",
                                  "drones": args.drones,
                                  "cmd_mode": cmd_mode,
                                  "drone_ips": resolved_ips,
                                  "dry_run": args.dry_run,
                                  "slow": args.slow,
                                  "gimbal_pitch": args.gimbal_pitch,
                                  "plane_mode": args.plane_mode,
                                  "flight_log": (logger.session_dir
                                                 if logger else None)},
                            pose_solver=pose_solver,
                            # The live meta, so a clip records the facades the
                            # operator had drawn AT CAPTURE TIME and can derive its
                            # own scene plane. The `meta=` dict above is a snapshot
                            # and cannot see a wall picked mid-flight.
                            meta_source=lambda: swarm_meta)
    print(f"  Clip recording -> {recording_dir} "
          f"(GUI Record button, max {args.record_max_s:.0f} s per clip, "
          f"with per-frame camera pose)")

    # angular.x is repurposed for d_ref in swarm mode, so the gimbal would
    # otherwise stay at the DroneController default (-90°). Park it at the
    # launch value (--gimbal-pitch) as soon as the send threads are running; the
    # GUI slider seeds to the same value and can retarget it live afterwards
    # (see command_listener / run).
    for d in swarm.drones.values():
        d.set_gimbal(args.gimbal_pitch, 0.0)
    print(f"  Gimbal pitch set to {args.gimbal_pitch:+.0f}° (yaw 0°) for all drones")

    olfati = OlfatiSaber(r0_coh=args.r0, c_vm=args.c_vm, scale=args.scale)

    # Virtual obstacles + geofence repulsion (β-agent term; params in physical
    # metres, converted to the cohesion math's scaled units internally).
    # c_vm: the C# GetObstacleForce uses the SAME velocity-matching gain as
    # the cohesion consensus, so the swarm's --c-vm is passed here too
    # (inert at the default c_vm = 0).
    avoid = ObstacleAvoidance(d_obs_m=args.d_obs, r0_obs_m=args.r0_obs,
                              c_obs=args.c_obs, c_vm=args.c_vm,
                              scale=args.scale)

    # GUI-drawn shapes persist across controller restarts in a JSON file next
    # to this script (unless --shapes-file points elsewhere).
    shapes_path = args.shapes_file
    if not os.path.isabs(shapes_path):
        shapes_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), shapes_path)
    obstacles, geofence = load_shapes(shapes_path)
    facades = load_facades(shapes_path)
    n_poly = sum(1 for ob in obstacles if ob.get("kind") == "poly")
    print(f"  Shapes: {len(obstacles)} obstacle(s)"
          f"{f' ({n_poly} building footprint(s))' if n_poly else ''}, "
          f"{len(facades)} facade(s), "
          f"geofence {'with ' + str(len(geofence)) + ' vertices' if geofence else 'none'} "
          f"({shapes_path})")

    # Always instantiated: the GUI can switch heading modes at runtime, so
    # both controllers must exist even when starting in manual mode. run()
    # only consults each while meta["heading_mode"] selects it.
    heading_ctrl = ConvexHullHeading(point_inwards=args.point_inwards)
    stitch_ctrl = DemoStitchHeading()

    # Vertical-plane ("wall") controller. Always instantiated so the GUI toggle
    # works at runtime; it stays inert (and the altitude channel stays the
    # shared scalar) until run() enters plane mode.
    plane_ctrl = SwarmPlane(gain=args.plane_gain, leash_m=args.plane_leash,
                            min_alt=MIN_ALT_M, max_alt=args.max_alt)

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
        # Demostitch fan offset (deg/rank), seeded from --stitch-offset and
        # then owned by the GUI; stitch_centre is the live centre drone id
        # (odd drone counts only) for the GUI's CENTRE pill.
        "stitch_offset": args.stitch_offset,
        "stitch_centre": None,
        # Vertical-plane ("wall") swarming: plane_mode/plane_gain are seeded
        # from the CLI and then owned by the GUI toggle + gain input;
        # meta["plane"] is the live wall state run() publishes (azimuth, the
        # reference altitude, and each drone's out-of-plane offset + commanded
        # altitude). "downwash" lists [upper, lower] pairs stacked closer than
        # DOWNWASH_RADIUS_M horizontally — advisory only, never a stop.
        "plane_mode": args.plane_mode,
        "plane_gain": args.plane_gain,
        "plane": plane_ctrl.status(),
        "downwash": [],
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
        # Virtual obstacles + geofence, seeded from shapes.json and then owned
        # by the GUI (command_listener replaces the lists wholesale on every
        # edit and persists them; run() converts to local metres each tick).
        # "removed" is the live list of geofence-breached (VS-disabled) drone
        # ids, refreshed each tick by run() for the GUI's FENCED OUT badges.
        "obstacles": obstacles,
        "geofence": geofence,
        # PLANAR inspection walls picked off a building footprint. READ EVERY TICK
        # by run(), which auto-picks the one the formation is filming and publishes
        # its standoff to the Unity stitcher (see _announce_scene_plane). They repel
        # nothing — a facade is a measurement surface, not a keep-out shape — but
        # they are no longer inert, and one added mid-flight moves the scene plane.
        "facades": facades,
        "removed": [],
        "obstacle_params": {
            "d_obs_m": args.d_obs,
            "r0_obs_m": args.r0_obs,
            "c_obs": args.c_obs,
        },
    }

    swarming = threading.Event()   # cleared = held (do nothing); set = flocking
    threading.Thread(target=command_listener,
                     args=(swarming, swarm_meta, args.cmd_host, args.cmd_port,
                           shapes_path),
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
            speed_scale=args.slow, meta=swarm_meta, heading_ctrl=heading_ctrl,
            stitch_ctrl=stitch_ctrl, cmd_sender=cmd_sender,
            identity_check=identity_check,
            min_separation=args.min_separation, avoid=avoid,
            plane_ctrl=plane_ctrl, max_alt=args.max_alt, recorder=recorder,
            # Both needed for the PLANAR scene plane: the solver defines the frame
            # the standoff must be measured in (and latches it), the publisher owns
            # the mapping it is written to. Passed as arguments rather than through
            # swarm_meta, which TelemetryFeedPublisher JSON-encodes at 5 Hz.
            pose_solver=pose_solver, img_stream=img_stream)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if gui_feed is not None:
            gui_feed.stop()
        if img_stream is not None:
            img_stream.stop()
        # BEFORE logger.close(): this finalises any clip in progress (an MP4
        # whose writer is never released has no moov atom and won't play) and
        # detaches + closes the flight-log mirror it owns.
        recorder.close()
        receiver.stop()
        swarm.stop_all()
        if cmd_sender is not None:
            # After stop_all(): its final DISABLE_VS must still go out
            # through the MQTT connections.
            cmd_sender.stop()
        if logger is not None:
            logger.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
