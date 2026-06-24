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
    python joystick_controller.py --cli        # fallback keyboard CLI

Joystick input comes from readController.py over UDP :5055 (see
udp_joystick_receiver.py). The drone-side timer in SwarmActivity expects
absolute heading (deg) and absolute altitude (m); the UDP loop integrates
yaw-rate and vertical-rate stick inputs to produce those targets.

Commands sent via sendWayPointData():
    "VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw"
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
import ds_wrapper as w

from udp_joystick_receiver import JoystickReceiver
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
YAW_RATE_DEG_S   = 60.0    # yaw rate at full angular.z (deg/s) — integrated
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


def _deadzone(v, dz=JOYSTICK_DEADZONE):
    return 0.0 if abs(v) < dz else v


def _normalize_heading_180(h):
    """Wrap any heading into [-180, 180]."""
    return ((h + 180.0) % 360.0) - 180.0


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
        self.yaw = 0.0         # heading angle degrees (-180 to 180)
        self.throttle = 0.0    # altitude meters (absolute)
        self.gimbal_pitch = -90.0
        self.gimbal_yaw = 0.0

        # Latest telemetry
        self.telemetry = {}

        # Background threads
        self._running = False
        self._send_thread = None
        self._telem_thread = None

    def send_command(self, command):
        """Send a raw command string to the drone."""
        w.sendWayPointData(command, self.drone_id)

    def send_vs(self):
        """Send the current virtual stick state."""
        cmd = (f"VS:{self.pitch:.2f}:{self.roll:.2f}:{self.yaw:.2f}:"
               f"{self.throttle:.2f}:{self.gimbal_pitch:.2f}:{self.gimbal_yaw:.2f}")
        w.sendWayPointData(cmd, self.drone_id)

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
        pitch:    forward (+) / backward (-) m/s
        roll:     right (+) / left (-) m/s
        yaw:      heading angle in degrees
        throttle: target altitude in meters
        """
        self.pitch = max(-15, min(15, pitch))
        self.roll = max(-15, min(15, roll))
        self.yaw = max(-180, min(180, yaw))
        self.throttle = max(0, throttle)

    def set_gimbal(self, pitch, yaw=0):
        """Set gimbal angles in degrees."""
        self.gimbal_pitch = pitch
        self.gimbal_yaw = yaw

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
        return t

    def start(self, send_rate_hz=20, telemetry_rate_hz=10):
        """Start background threads for sending commands and reading telemetry."""
        self._running = True

        def _send_loop():
            interval = 1.0 / send_rate_hz
            while self._running:
                try:
                    self.send_vs()
                except Exception as e:
                    print(f"[drone {self.drone_id}] send_vs error: {e}", flush=True)
                time.sleep(interval)

        def _telem_loop():
            interval = 1.0 / telemetry_rate_hz
            while self._running:
                try:
                    self.update_telemetry()
                except Exception as e:
                    print(f"[drone {self.drone_id}] update_telemetry error: {e}", flush=True)
                time.sleep(interval)

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

    def start_all(self, send_rate_hz=20, telemetry_rate_hz=10):
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
    print("  vs P R Y T      - Set velocity (pitch roll yaw throttle)")
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
            controller.set_velocity(0, 0, controller.yaw, controller.throttle)
            print("Velocities zeroed (holding altitude & heading)")
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


def udp_joystick_mode(controller, receiver):
    """Drive the drone from UDP joystick data sent by readController.py.

    Stick → drone mapping:
      linear.x  -> pitch  (forward velocity, m/s)
      linear.y  -> roll   (right velocity, m/s)
      linear.z  -> climb  (integrated into target altitude, m)
      angular.z -> yaw    (integrated into target heading, deg)
      angular.x -> gimbal pitch (axis is centered around 1.0)
      switch s1 -> toggle ENABLE_VS / DISABLE_VS  (rising edge)
      switch s2 -> LAND                            (rising edge)
    """
    print("\n--- UDP Joystick Mode ---")
    print("Listening for readController.py on :5055")
    print("  s1: toggle VS    s2: LAND    Ctrl+C: stop and disable VS")
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
        target_yaw = _normalize_heading_180(target_yaw + ang_z * YAW_RATE_DEG_S * dt)
        target_alt = max(MIN_ALT_M, min(MAX_ALT_M, target_alt + lin_z * VERT_RATE_MPS * dt))

        # Gimbal pitch from angular.x (centered 1.0, swing ±0.4 → [0.6, 1.4])
        ax = max(0.6, min(1.4, state.angular_x))
        raw_gimbal_pitch = GIMBAL_PITCH_MIN + ((ax - 0.6) / 0.8) * (GIMBAL_PITCH_MAX - GIMBAL_PITCH_MIN)
        smooth_gimbal_pitch = STICK_SMOOTHING_ALPHA * raw_gimbal_pitch + (1.0 - STICK_SMOOTHING_ALPHA) * smooth_gimbal_pitch

        controller.set_velocity(smooth_pitch, smooth_roll, target_yaw, target_alt)
        controller.set_gimbal(smooth_gimbal_pitch, 0)

        if now - last_print > 1.0:
            print(f"  P={smooth_pitch:+5.2f}m/s  R={smooth_roll:+5.2f}m/s  Y={target_yaw:+6.1f}°  "
                  f"Alt={target_alt:5.1f}m  Gim={smooth_gimbal_pitch:+5.1f}°  "
                  f"VS={'ON' if vs_local else 'off'}")
            last_print = now

        time.sleep(0.02)  # ~50 Hz local loop; DroneController's send_thread relays at its own rate


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm joystick controller")
    ap.add_argument("--drone", type=int, default=1, help="Drone ID (default 1)")
    ap.add_argument("--port",  type=int, default=5055, help="UDP port for joystick (default 5055)")
    ap.add_argument("--cli",   action="store_true", help="Use keyboard CLI instead of UDP joystick")
    ap.add_argument("--no-gui", action="store_true",
                    help="Do not push telemetry to the browser GUI (swarm_gui.py)")
    ap.add_argument("--gui-host", default=DEFAULT_GUI_HOST,
                    help=f"GUI telemetry UDP host (default {DEFAULT_GUI_HOST})")
    ap.add_argument("--gui-port", type=int, default=DEFAULT_GUI_PORT,
                    help=f"GUI telemetry UDP port (default {DEFAULT_GUI_PORT})")
    args = ap.parse_args()

    print("LIS_Swarm Joystick Controller")
    print("Using ds_wrapper via DroneSwarmServer")
    print()

    decode = w.isHWDecoderEnabled()
    print(f"HW Decoder: {'enabled' if decode == 1 else 'disabled (SW)'}")

    drone = DroneController(drone_id=args.drone)
    drone.start(send_rate_hz=20, telemetry_rate_hz=10)
    print(f"Background threads started (drone {args.drone}, 20Hz commands, 10Hz telemetry)")

    receiver = None
    if not args.cli:
        receiver = JoystickReceiver(port=args.port)
        receiver.start()
        print(f"UDP joystick receiver listening on :{args.port}")

    gui_feed = None
    if not args.no_gui:
        gui_feed = TelemetryFeedPublisher(
            source=lambda: {drone.drone_id: drone.telemetry},
            host=args.gui_host, port=args.gui_port)
        gui_feed.start()
        print(f"GUI telemetry feed -> {args.gui_host}:{args.gui_port} "
              f"(open swarm_gui.py; disable with --no-gui)")

    try:
        if args.cli:
            interactive_mode(drone)
        else:
            udp_joystick_mode(drone, receiver)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if gui_feed is not None:
            gui_feed.stop()
        if receiver is not None:
            receiver.stop()
        drone.stop()
        print("Stopped.")


if __name__ == "__main__":
    main()
