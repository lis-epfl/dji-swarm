"""
LIS_Swarm PC Joystick Controller
=================================
Uses the existing ds_wrapper (DroneSwarmServer shared memory bridge) to send
virtual stick commands to drones and receive image + telemetry data back.

The data path is:
  Python -> ds_wrapper.sendWayPointData() -> SharedMem -> DroneSwarmServer.exe
    -> MQTT -> Android App (SwarmActivity) -> DJI Virtual Stick API

Telemetry comes back via:
  Android App -> native setTelemetryData() -> RTSP/SharedMem -> DroneSwarmServer.exe
    -> SharedMem -> ds_wrapper.getImageAndTelemetryData()

Usage:
    python joystick_controller.py              # UDP joystick mode (default)
    python joystick_controller.py --slow       # slow test mode (30% speed)
    python joystick_controller.py --slow 0.5   # slow test mode at 50% speed
    python joystick_controller.py --cli        # fallback keyboard CLI

Joystick input comes from readController.py over UDP :5055 (see
udp_joystick_receiver.py). The drone-side VS timer in SwarmActivity runs yaw in
angular-velocity (rate) mode and altitude in absolute-position mode; the UDP
loop keeps an absolute target heading (for the heading-hold) but emits a yaw
RATE, and integrates the vertical-rate stick into an absolute altitude target.

Commands sent via sendWayPointData():
    "VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw"
        pitch/roll = world N/E velocity m/s, yaw = yaw RATE deg/s,
        throttle = absolute altitude m, gimbal angles = absolute deg.
    "ENABLE_VS"  - Enable virtual stick mode
    "DISABLE_VS" - Disable virtual stick mode
    "TAKEOFF"    - Auto takeoff
    "LAND"       - Auto land

Telemetry format (from getImageAndTelemetryData, after image bytes):
    lat:lon:alt:heading:gimbal_pitch:gimbal_roll:gimbal_yaw:sat_count:
    drone_pitch:drone_roll:drone_yaw:vx:vy:vz:waypoint_done:value_check:vs_on_off
"""

import argparse
import time
import threading
from collections import deque
import ds_wrapper as w

from udp_joystick_receiver import JoystickReceiver
from flight_logger import FlightLogger
from swarm_telemetry_feed import (
    TelemetryFeedPublisher,
    DEFAULT_GUI_HOST,
    DEFAULT_GUI_PORT,
)


# Image data is 1920*1080*1.5 = 3,110,400 bytes (YUV), telemetry starts at offset 3,110,408
TELEMETRY_OFFSET = 3110408

# === UDP joystick mapping ===
# linear.x, linear.y, linear.z are unit-scaled stick deflections in [-1, 1]
# (apart from linear.z which is quadratic-scaled in readController.py).
# angular.x is centered around 1.0 with ±0.4 swing (see readController.py),
# i.e. it ranges roughly [0.6, 1.4] — used here for gimbal pitch.
# angular.z is a yaw-rate stick in [-1, 1].
MAX_PITCH_MPS    = 3.0     # forward velocity at full stick (m/s)
MAX_ROLL_MPS     = 3.0     # right velocity at full stick (m/s)
YAW_RATE_DEG_S   = 60.0    # yaw rate at full angular.z (deg/s) — stick feed-forward
VERT_RATE_MPS    = 2.0     # climb/descent rate at full linear.z (m/s) — integrated
MAX_ALT_M        = 30.0    # cap on the integrated altitude target
INITIAL_ALT_M    = 4.0     # seed altitude when VS mode starts (drones never target 0 m)
MIN_ALT_M        = 1.0     # floor for the integrated altitude target in VS mode
GIMBAL_PITCH_MIN = -90.0
GIMBAL_PITCH_MAX = 30.0
JOYSTICK_DEADZONE = 0.05

# EMA smoothing factor for direct-stick channels (pitch, roll, gimbal pitch).
# 0.0 = frozen, 1.0 = raw stick (no smoothing). At the ~50 Hz UDP loop rate,
# alpha=0.25 gives a time-constant of ~60 ms — kills jitter without feeling laggy.
STICK_SMOOTHING_ALPHA = 0.25

# Yaw heading-hold controller. The DJI VS yaw channel is now angular-velocity
# (rate) mode — see SwarmActivity.startVsSendLoop. We keep an absolute target
# heading and convert it into a smooth yaw RATE: stick feed-forward turns the
# drone while a P term servos the measured heading back onto the target. ANGLE
# mode (absolute heading straight to the FC) was the source of the old choppy,
# stepwise yaw. KP_YAW is the main tuning knob: too high + laggy telemetry
# oscillates, too low feels sluggish returning to heading. The loop response
# time-constant is ~1/KP_YAW seconds. Two structural guards keep the P loop
# well-behaved regardless of KP_YAW (see integrate_target_heading and
# heading_hold_rate): MAX_TARGET_LEAD_DEG and YAW_ERR_DEADBAND_DEG.
# 0.8 (was 1.5): closed-loop sims with the command+telemetry transport delay
# modelled at 0.15-0.5 s each way showed 1.5 sitting near the stability margin
# — after a turn the drone wagged for 4-12 s (the reported behaviour), while
# 0.8 settles in ~0.5-3 s with at most one small correction swing across the
# whole delay range. Raise it back only if the telemetry path gets faster.
KP_YAW             = 0.8     # heading error (deg) -> yaw rate (deg/s)
MAX_YAW_RATE_DEG_S = 100.0   # clamp on the commanded yaw rate (deg/s)

# Anti-windup on the stick-integrated target heading: the target may lead the
# MEASURED heading by at most this many degrees (integrate_target_heading).
# Without it a sustained turn lets the target run 40°+ ahead of the aircraft
# (P-tracking lag = ff/KP_YAW, plus telemetry latency, plus any FC rate
# shortfall), so on stick release the drone keeps turning to burn off that
# debt, overshoots on stale telemetry, and wags for several seconds — and past
# 180° of lag the wrapped error flips sign and the turn reverses outright.
MAX_TARGET_LEAD_DEG = 25.0

# Heading errors inside this band contribute no P correction (subtracted, not
# hard-cut, so the response stays continuous). The compass heading jitters
# ~1-2°; chasing that noise keeps the nose twitching indefinitely after every
# turn instead of coming to a visible stop.
YAW_ERR_DEADBAND_DEG = 1.5

# Velocity scale applied by the --slow test flag when given with no value.
# The flag multiplies all commanded motion (pitch/roll velocity, yaw rate, climb
# rate) by this factor so the drone(s) can be flown and tuned at a safe, low
# speed without changing the shape of the behaviour. 1.0 = full speed.
SLOW_DEFAULT_SCALE = 0.3


def _deadzone(v, dz=JOYSTICK_DEADZONE):
    return 0.0 if abs(v) < dz else v


def _normalize_heading_180(h):
    """Wrap any heading into [-180, 180]."""
    return ((h + 180.0) % 360.0) - 180.0


def heading_hold_rate(target_heading, current_heading, ff_rate=0.0):
    """Yaw RATE (deg/s) that drives current_heading toward target_heading.

    The DJI VS yaw channel is angular-velocity mode, so instead of sending an
    absolute heading we send a rate: ``ff_rate`` (stick feed-forward) plus a
    proportional correction on the wrapped heading error. During a sustained
    turn the error settles small (the FF carries the turn); when the stick is
    centred FF=0 and the P term smoothly holds/returns to target_heading.

    current_heading may be None (no telemetry/GPS yet) → feed-forward only.
    Errors within ±YAW_ERR_DEADBAND_DEG produce no correction (heading noise
    would otherwise keep the nose dithering around the setpoint forever).
    Result is clamped to ±MAX_YAW_RATE_DEG_S.
    """
    if current_heading is None:
        rate = ff_rate
    else:
        err = _normalize_heading_180(target_heading - current_heading)
        if abs(err) <= YAW_ERR_DEADBAND_DEG:
            err = 0.0
        elif err > 0:
            err -= YAW_ERR_DEADBAND_DEG
        else:
            err += YAW_ERR_DEADBAND_DEG
        rate = ff_rate + KP_YAW * err
    return max(-MAX_YAW_RATE_DEG_S, min(MAX_YAW_RATE_DEG_S, rate))


def integrate_target_heading(target_yaw, ff_rate, dt, current_heading):
    """Advance the stick-integrated target heading by ff_rate*dt, with
    anti-windup: the result is clamped to within ±MAX_TARGET_LEAD_DEG of the
    measured heading.

    Naive integration (target += rate*dt) lets the setpoint run arbitrarily
    far ahead of the aircraft during a sustained turn. All that accumulated
    error is still owed when the stick is centred: the drone keeps turning
    long after release, then overshoots (its heading measurement is stale)
    and oscillates while the debt decays. Clamping the lead makes the
    setpoint track the aircraft, so at most MAX_TARGET_LEAD_DEG of catch-up
    remains on release.

    The clamp only acts while the stick is deflected (ff_rate != 0): windup
    can only accumulate while integrating, and skipping the clamp at centre
    stick preserves the hold's full authority — an external disturbance
    (wind, a bump) never drags the setpoint along with the drone.

    current_heading None (no telemetry/GPS yet) → integrate unclamped.
    """
    target_yaw = _normalize_heading_180(target_yaw + ff_rate * dt)
    if ff_rate != 0.0 and current_heading is not None:
        err = _normalize_heading_180(target_yaw - current_heading)
        if err > MAX_TARGET_LEAD_DEG:
            target_yaw = _normalize_heading_180(current_heading + MAX_TARGET_LEAD_DEG)
        elif err < -MAX_TARGET_LEAD_DEG:
            target_yaw = _normalize_heading_180(current_heading - MAX_TARGET_LEAD_DEG)
    return target_yaw


def parse_telemetry(raw_data):
    """Parse telemetry from the image+telemetry data array returned by ds_wrapper."""
    try:
        telemetry_str = bytearray(raw_data[TELEMETRY_OFFSET:]).decode().strip('\x00')
        parts = telemetry_str.split(':')
        if len(parts) >= 17:
            return {
                'lat': float(parts[0]),
                'lon': float(parts[1]),
                'alt': float(parts[2]),
                'heading': float(parts[3]),
                'gimbal_pitch': float(parts[4]),
                'gimbal_roll': float(parts[5]),
                'gimbal_yaw': float(parts[6]),
                'sat_count': int(parts[7]),
                'drone_pitch': float(parts[8]),
                'drone_roll': float(parts[9]),
                'drone_yaw': float(parts[10]),
                'vx': float(parts[11]),
                'vy': float(parts[12]),
                'vz': float(parts[13]),
                'waypoint_done': int(parts[14]),
                'value_check': float(parts[15]),
                'vs_enabled': float(parts[16]) > 0,
            }
    except Exception as e:
        print(f"Telemetry parse error: {e}")
    return None


class _RateMeter:
    """Sliding-window estimate of how often tick() is called, in Hz.

    tick() runs in the producer thread (the send or telemetry loop); hz() is read
    from the GUI-feed thread, hence the lock. The window auto-prunes, so a stalled
    or stopped loop decays toward 0 Hz rather than reporting a stale rate — which
    is exactly what makes this useful for spotting a blocking ds_wrapper call."""

    def __init__(self, window=2.0):
        self._window = window
        self._ts = deque()
        self._lock = threading.Lock()

    def tick(self):
        now = time.perf_counter()
        with self._lock:
            self._ts.append(now)
            cutoff = now - self._window
            while self._ts and self._ts[0] < cutoff:
                self._ts.popleft()

    def hz(self):
        now = time.perf_counter()
        with self._lock:
            cutoff = now - self._window
            while self._ts and self._ts[0] < cutoff:
                self._ts.popleft()
            n = len(self._ts)
            if n < 2:
                return 0.0
            span = self._ts[-1] - self._ts[0]
        return round((n - 1) / span, 1) if span > 0 else 0.0


class DroneController:
    """Controls a single drone via ds_wrapper."""

    def __init__(self, drone_id=1):
        """
        Args:
            drone_id: Drone number (1-based, matching DroneSwarmServer slots)
        """
        self.drone_id = drone_id

        # Current joystick state
        self.pitch = 0.0       # forward/back velocity m/s (-15 to 15)
        self.roll = 0.0        # left/right velocity m/s (-15 to 15)
        self.yaw = 0.0         # yaw RATE deg/s (±MAX_YAW_RATE_DEG_S)
        self.throttle = 0.0    # altitude meters (absolute)
        self.gimbal_pitch = -90.0
        self.gimbal_yaw = 0.0

        # Latest telemetry
        self.telemetry = {}

        # Optional FlightLogger (set by the launch script); None = no logging.
        self.logger = None

        # Measured loop rates (Hz): how fast commands actually go out and
        # telemetry actually comes back. Surfaced to the GUI.
        self._send_meter = _RateMeter()
        self._recv_meter = _RateMeter()

        # Optional frame consumer (set by ImageStreamPublisher): called from
        # the telemetry thread as frame_sink(data, telem) with the raw wrapper
        # array so the image bytes it already carries aren't thrown away. The
        # sink must copy immediately — `data` aliases the wrapper's shared
        # memory. None = telemetry-only (default).
        self.frame_sink = None

        # Optional MqttCommandSender (set by the launch script): when set,
        # commands publish straight to the RC's broker over its persistent
        # connection instead of via ds_wrapper -> DroneSwarmServer, whose
        # per-command MQTT reconnect cycle caps sends at ~4.5 Hz. None =
        # legacy wrapper path (default).
        self.command_sender = None

        # Background threads
        self._running = False
        self._send_thread = None
        self._telem_thread = None

    def send_command(self, command):
        """Send a raw command string to the drone.

        One-shot commands (ENABLE_VS/TAKEOFF/...) use QoS 1 on the direct
        MQTT path: queued while disconnected, guaranteed once connected.
        """
        if self.command_sender is not None:
            self.command_sender.send(self.drone_id, command, qos=1)
        else:
            w.sendWayPointData(command, self.drone_id)
        if self.logger:
            self.logger.log_drone_command(self.drone_id, "EVENT", cmd=command)

    def send_vs(self):
        """Send the current virtual stick state (QoS 0: latest-wins stream)."""
        cmd = (f"VS:{self.pitch:.2f}:{self.roll:.2f}:{self.yaw:.2f}:"
               f"{self.throttle:.2f}:{self.gimbal_pitch:.2f}:{self.gimbal_yaw:.2f}")
        if self.command_sender is not None:
            self.command_sender.send(self.drone_id, cmd, qos=0)
        else:
            w.sendWayPointData(cmd, self.drone_id)
        if self.logger:
            self.logger.log_drone_command(
                self.drone_id, "VS", self.pitch, self.roll, self.yaw,
                self.throttle, self.gimbal_pitch, self.gimbal_yaw, cmd)

    def enable_vs(self):
        self.send_command("ENABLE_VS")

    def disable_vs(self):
        self.send_command("DISABLE_VS")

    def takeoff(self):
        self.send_command("TAKEOFF")

    def land(self):
        self.send_command("LAND")

    def set_velocity(self, pitch, roll, yaw, throttle):
        """Set joystick values.
        pitch:    forward (+) / backward (-) m/s  (world north in GROUND mode)
        roll:     right (+) / left (-) m/s        (world east  in GROUND mode)
        yaw:      yaw RATE deg/s (DJI VS angular-velocity yaw mode; + = CW)
        throttle: target altitude in meters
        """
        self.pitch = max(-15, min(15, pitch))
        self.roll = max(-15, min(15, roll))
        self.yaw = max(-MAX_YAW_RATE_DEG_S, min(MAX_YAW_RATE_DEG_S, yaw))
        self.throttle = max(0, throttle)

    def set_gimbal(self, pitch, yaw=0):
        """Set gimbal angles in degrees."""
        self.gimbal_pitch = pitch
        self.gimbal_yaw = yaw

    def send_hz(self):
        """Measured rate (Hz) at which VS commands are actually being sent."""
        return self._send_meter.hz()

    def recv_hz(self):
        """Measured rate (Hz) at which telemetry is actually being read back."""
        return self._recv_meter.hz()

    def get_image_and_telemetry(self):
        """Get raw image + telemetry data from the drone.
        Returns the full numpy array from ds_wrapper.
        Image: data[0:3110400] (YUV 1920x1080)
        Telemetry: data[3110408:] (colon-separated string)
        """
        return w.getImageAndTelemetryData(self.drone_id)

    def update_telemetry(self):
        """Fetch and parse the latest telemetry."""
        data = self.get_image_and_telemetry()
        t = parse_telemetry(data)
        if t:
            self.telemetry = t
            if self.logger:
                self.logger.log_telemetry(self.drone_id, t)
            sink = self.frame_sink
            if sink is not None:
                try:
                    sink(data, t)
                except Exception:
                    # Frame publishing is non-critical; never let it break
                    # the telemetry loop.
                    pass
        return t

    def start(self, send_rate_hz=20, telemetry_rate_hz=20):
        """Start background threads for sending commands and reading telemetry.

        Both loops use drift-compensated scheduling: each ds_wrapper call
        blocks on a server round-trip (a telemetry fetch waits ~30 ms for the
        next video packet), so a naive sleep(interval) after the call would
        cap the loop well below its target rate.
        """
        self._running = True

        def _paced_loop(interval, body, label):
            next_t = time.monotonic() + interval
            while self._running:
                try:
                    body()
                except Exception as e:
                    print(f"[drone {self.drone_id}] {label} error: {e}", flush=True)
                now = time.monotonic()
                if next_t > now:
                    time.sleep(next_t - now)
                    next_t += interval
                else:
                    # Fell behind (slow wrapper call); resync instead of
                    # bursting to catch up.
                    next_t = now + interval

        def _send_loop():
            def body():
                self.send_vs()
                self._send_meter.tick()
            _paced_loop(1.0 / send_rate_hz, body, "send_vs")

        def _telem_loop():
            def body():
                self.update_telemetry()
                self._recv_meter.tick()
            _paced_loop(1.0 / telemetry_rate_hz, body, "update_telemetry")

        self._send_thread = threading.Thread(target=_send_loop, daemon=True, name=f"VS_Send_{self.drone_id}")
        self._telem_thread = threading.Thread(target=_telem_loop, daemon=True, name=f"Telem_{self.drone_id}")
        self._send_thread.start()
        self._telem_thread.start()

    def stop(self):
        """Stop background threads and disable virtual stick."""
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=2)
        if self._telem_thread:
            self._telem_thread.join(timeout=2)
        self.disable_vs()


class SwarmController:
    """Controls multiple drones as a swarm."""

    def __init__(self):
        self.drones = {}

    def add_drone(self, drone_id=1):
        ctrl = DroneController(drone_id)
        self.drones[drone_id] = ctrl
        return ctrl

    def attach_logger(self, logger):
        """Route command/telemetry logging for every drone to `logger`."""
        for d in self.drones.values():
            d.logger = logger

    def start_all(self, send_rate_hz=20, telemetry_rate_hz=20):
        for d in self.drones.values():
            d.start(send_rate_hz, telemetry_rate_hz)

    def stop_all(self):
        for d in self.drones.values():
            d.stop()

    def enable_vs_all(self):
        for d in self.drones.values():
            d.enable_vs()

    def disable_vs_all(self):
        for d in self.drones.values():
            d.disable_vs()

    def set_all_velocity(self, pitch, roll, yaw, throttle):
        for d in self.drones.values():
            d.set_velocity(pitch, roll, yaw, throttle)

    def get_all_telemetry(self):
        return {did: d.telemetry for did, d in self.drones.items()}


def interactive_mode(controller):
    """Simple interactive CLI for testing."""
    print("\n--- LIS_Swarm Joystick Controller ---")
    print("Commands:")
    print("  enable          - Enable virtual stick")
    print("  disable         - Disable virtual stick")
    print("  takeoff         - Auto takeoff")
    print("  land            - Auto land")
    print("  vs P R Y T      - Set velocity (pitch roll yaw-rate°/s throttle)")
    print("  gimbal P [Y]    - Set gimbal (pitch [yaw])")
    print("  telem           - Show latest telemetry")
    print("  stop            - Zero all velocities")
    print("  quit            - Exit")
    print()

    while True:
        try:
            cmd = input("> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        if controller.logger:
            controller.logger.log_user_command(None, source="cli", raw=" ".join(cmd))

        c = cmd[0].lower()

        if c == "quit":
            break
        elif c == "enable":
            controller.enable_vs()
            print("Sent ENABLE_VS")
        elif c == "disable":
            controller.disable_vs()
            print("Sent DISABLE_VS")
        elif c == "takeoff":
            controller.takeoff()
            print("Sent TAKEOFF")
        elif c == "land":
            controller.land()
            print("Sent LAND")
        elif c == "stop":
            controller.set_velocity(0, 0, 0, controller.throttle)
            print("Velocities zeroed (yaw rate 0, holding altitude)")
        elif c == "vs" and len(cmd) >= 5:
            p, r, y, t = float(cmd[1]), float(cmd[2]), float(cmd[3]), float(cmd[4])
            controller.set_velocity(p, r, y, t)
            print(f"Set VS: pitch={p} roll={r} yaw={y} throttle={t}")
        elif c == "gimbal" and len(cmd) >= 2:
            gp = float(cmd[1])
            gy = float(cmd[2]) if len(cmd) >= 3 else 0
            controller.set_gimbal(gp, gy)
            print(f"Set gimbal: pitch={gp} yaw={gy}")
        elif c == "telem":
            t = controller.telemetry
            if t:
                print(f"  Lat: {t.get('lat', 0):.6f}  Lon: {t.get('lon', 0):.6f}")
                print(f"  Alt: {t.get('alt', 0):.1f}m  Hdg: {t.get('heading', 0):.1f}")
                print(f"  Gimbal P: {t.get('gimbal_pitch', 0):.1f}  Y: {t.get('gimbal_yaw', 0):.1f}")
                print(f"  Sats: {t.get('sat_count', 0)}  VS: {'ON' if t.get('vs_enabled') else 'OFF'}")
                print(f"  Vel: x={t.get('vx', 0):.1f} y={t.get('vy', 0):.1f} z={t.get('vz', 0):.1f}")
            else:
                print("  No telemetry received yet")
        else:
            print("Unknown command. Type 'quit' to exit.")


def udp_joystick_mode(controller, receiver, speed_scale=1.0):
    """Drive the drone from UDP joystick data sent by readController.py.

    Stick → drone mapping:
      linear.x  -> pitch  (forward velocity, m/s)
      linear.y  -> roll   (right velocity, m/s)
      linear.z  -> climb  (integrated into target altitude, m)
      angular.z -> yaw    (integrated into target heading, deg)
      angular.x -> gimbal pitch (axis is centered around 1.0)
      switch s1 -> toggle ENABLE_VS / DISABLE_VS  (rising edge)
      switch s2 -> LAND                            (rising edge)

    speed_scale (--slow) uniformly multiplies the commanded velocity, yaw rate
    and climb rate so the drone moves slowly for testing/tuning. 1.0 = full speed.
    """
    print("\n--- UDP Joystick Mode ---")
    print("Listening for readController.py on :5055")
    print("  s1: toggle VS    s2: LAND    Ctrl+C: stop and disable VS")
    if speed_scale != 1.0:
        print(f"  SLOW TEST MODE: all velocities/rates scaled to {speed_scale:.0%}")
    print()

    target_yaw = 0.0
    target_alt = INITIAL_ALT_M
    smooth_pitch = 0.0
    smooth_roll  = 0.0
    smooth_gimbal_pitch = GIMBAL_PITCH_MIN
    last_s1 = 0
    last_s2 = 0
    last_t = time.time()
    vs_local = False
    last_print = 0.0

    while True:
        now = time.time()
        dt = now - last_t
        last_t = now

        state = receiver.get_state()
        if state is None:
            # No fresh joystick data: hold last command, don't keep integrating
            time.sleep(0.05)
            continue

        # s1: toggle VS on rising edge. On enable, snapshot current heading/alt
        # so the drone doesn't snap to (north, ground).
        if state.s1 == 1 and last_s1 == 0:
            if vs_local:
                controller.disable_vs()
                vs_local = False
                print("[s1] DISABLE_VS")
            else:
                t = controller.telemetry
                if t:
                    target_yaw = _normalize_heading_180(t.get('heading', 0.0))
                    target_alt = max(t.get('alt', INITIAL_ALT_M), INITIAL_ALT_M)
                else:
                    target_alt = INITIAL_ALT_M
                controller.enable_vs()
                vs_local = True
                print(f"[s1] ENABLE_VS  start heading={target_yaw:+.1f}°  alt={target_alt:.1f} m")
        last_s1 = state.s1

        # s2: emergency LAND on rising edge
        if state.s2 == 1 and last_s2 == 0:
            controller.land()
            print("[s2] LAND")
        last_s2 = state.s2

        # Stick mapping
        lin_x = _deadzone(state.linear_x)
        lin_y = _deadzone(state.linear_y)
        lin_z = _deadzone(state.linear_z)
        ang_z = _deadzone(state.angular_z)

        raw_pitch = lin_x * MAX_PITCH_MPS
        raw_roll  = lin_y * MAX_ROLL_MPS
        smooth_pitch = STICK_SMOOTHING_ALPHA * raw_pitch + (1.0 - STICK_SMOOTHING_ALPHA) * smooth_pitch
        smooth_roll  = STICK_SMOOTHING_ALPHA * raw_roll  + (1.0 - STICK_SMOOTHING_ALPHA) * smooth_roll
        # Yaw: integrate the stick into an absolute heading target (the hold
        # setpoint), then command a smooth yaw RATE toward it — stick feed-forward
        # leads the turn, the P term holds heading when the stick is centred.
        ff_yaw_rate = ang_z * YAW_RATE_DEG_S * speed_scale
        cur_heading = controller.telemetry.get('heading') if controller.telemetry else None
        target_yaw = integrate_target_heading(target_yaw, ff_yaw_rate, dt, cur_heading)
        cmd_yaw_rate = heading_hold_rate(target_yaw, cur_heading, ff_yaw_rate)
        target_alt = max(MIN_ALT_M, min(MAX_ALT_M, target_alt + lin_z * VERT_RATE_MPS * speed_scale * dt))

        # Gimbal pitch from angular.x (centered 1.0, swing ±0.4 → [0.6, 1.4])
        ax = max(0.6, min(1.4, state.angular_x))
        raw_gimbal_pitch = GIMBAL_PITCH_MIN + ((ax - 0.6) / 0.8) * (GIMBAL_PITCH_MAX - GIMBAL_PITCH_MIN)
        smooth_gimbal_pitch = STICK_SMOOTHING_ALPHA * raw_gimbal_pitch + (1.0 - STICK_SMOOTHING_ALPHA) * smooth_gimbal_pitch

        # --slow scales the translation command (yaw/climb already scaled above).
        cmd_pitch = smooth_pitch * speed_scale
        cmd_roll  = smooth_roll * speed_scale
        controller.set_velocity(cmd_pitch, cmd_roll, cmd_yaw_rate, target_alt)
        controller.set_gimbal(smooth_gimbal_pitch, 0)

        if now - last_print > 1.0:
            print(f"  P={cmd_pitch:+5.2f}m/s  R={cmd_roll:+5.2f}m/s  "
                  f"Y={target_yaw:+6.1f}°→{cmd_yaw_rate:+5.1f}°/s  "
                  f"Alt={target_alt:5.1f}m  Gim={smooth_gimbal_pitch:+5.1f}°  "
                  f"VS={'ON' if vs_local else 'off'}")
            last_print = now

        time.sleep(0.02)  # ~50 Hz local loop; DroneController's send_thread relays at its own rate


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm joystick controller")
    ap.add_argument("--drone", type=int, default=1, help="Drone ID (default 1)")
    ap.add_argument("--port",  type=int, default=5055, help="UDP port for joystick (default 5055)")
    ap.add_argument("--cli",   action="store_true", help="Use keyboard CLI instead of UDP joystick")
    ap.add_argument("--slow", nargs="?", const=SLOW_DEFAULT_SCALE, type=float, default=1.0,
                    metavar="SCALE",
                    help="Test mode: scale all commanded velocities and yaw/climb "
                         f"rates for slow, controlled tuning. Bare --slow uses "
                         f"{SLOW_DEFAULT_SCALE}; pass a value (e.g. --slow 0.5) to "
                         "override. Default 1.0 (full speed).")
    ap.add_argument("--no-gui", action="store_true",
                    help="Do not push telemetry to the browser GUI (swarm_gui.py)")
    ap.add_argument("--gui-host", default=DEFAULT_GUI_HOST,
                    help=f"GUI telemetry UDP host (default {DEFAULT_GUI_HOST})")
    ap.add_argument("--gui-port", type=int, default=DEFAULT_GUI_PORT,
                    help=f"GUI telemetry UDP port (default {DEFAULT_GUI_PORT})")
    ap.add_argument("--no-log", action="store_true",
                    help="Disable background flight-data logging to flight_logs/")
    ap.add_argument("--log-dir", default="flight_logs",
                    help="Directory for flight-log session folders (default flight_logs)")
    args = ap.parse_args()

    if args.slow <= 0:
        ap.error("--slow SCALE must be > 0")

    print("LIS_Swarm Joystick Controller")
    print("Using ds_wrapper via DroneSwarmServer")
    if args.slow != 1.0:
        print(f"SLOW TEST MODE: velocities/rates scaled to {args.slow:.0%}")
    print()

    decode = w.isHWDecoderEnabled()
    print(f"HW Decoder: {'enabled' if decode == 1 else 'disabled (SW)'}")

    logger = None
    if not args.no_log:
        logger = FlightLogger(base_dir=args.log_dir, meta={
            "script": "joystick_controller",
            "drone": args.drone, "port": args.port, "cli": args.cli,
            "slow": args.slow,
        })
        print(f"Flight logging -> {logger.session_dir} (disable with --no-log)")

    drone = DroneController(drone_id=args.drone)
    drone.logger = logger
    drone.start(send_rate_hz=20, telemetry_rate_hz=20)
    print(f"Background threads started (drone {args.drone}, 20Hz commands, 20Hz telemetry)")

    receiver = None
    if not args.cli:
        receiver = JoystickReceiver(port=args.port, logger=logger)
        receiver.start()
        print(f"UDP joystick receiver listening on :{args.port}")

    gui_feed = None
    if not args.no_gui:
        gui_feed = TelemetryFeedPublisher(
            source=lambda: {drone.drone_id: drone.telemetry},
            stats_source=lambda: {drone.drone_id: {
                "send_hz": drone.send_hz(), "recv_hz": drone.recv_hz()}},
            host=args.gui_host, port=args.gui_port)
        gui_feed.start()
        print(f"GUI telemetry feed -> {args.gui_host}:{args.gui_port} "
              f"(open swarm_gui.py; disable with --no-gui)")

    try:
        if args.cli:
            interactive_mode(drone)
        else:
            udp_joystick_mode(drone, receiver, speed_scale=args.slow)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if gui_feed is not None:
            gui_feed.stop()
        if receiver is not None:
            receiver.stop()
        drone.stop()
        if logger is not None:
            logger.close()
        print("Stopped.")


if __name__ == "__main__":
    main()
