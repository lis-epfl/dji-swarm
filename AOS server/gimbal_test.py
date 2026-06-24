"""
LIS_Swarm Gimbal Sweep Test
===========================
Sweeps the gimbal pitch through a range in fixed-degree increments and verifies
each move against telemetry. No flight motion is commanded — pitch/roll/yaw/
throttle in the VS string are all zero.

CAUTION — read this before running:
- The Android app (SwarmActivity) only fires `sendGimbalCommand` from inside its
  20 Hz Virtual-Stick timer. So gimbal commands require VS to be ENABLED.
  This script enables VS for the duration of the sweep and disables it on exit
  (including on Ctrl+C).
- While VS is enabled, the 20 Hz timer also sends VirtualStickAdvancedParam
  with vertical control = POSITION, throttle = 0, and yaw control = ANGLE,
  yaw = 0. That means: target altitude 0, target heading North.
  If the drone is airborne it WILL try to descend and rotate to north.
  RUN THIS ON THE GROUND OR A BENCH ONLY.
- Physical RC sticks override virtual stick — keep a thumb on them.

Usage:
    python gimbal_test.py
    python gimbal_test.py --drone 1 --start 0 --end -90 --step 10 --dwell 2
    python gimbal_test.py --no-return       # sweep one direction only
    python gimbal_test.py --yes             # skip confirmation prompt
"""

import argparse
import time

import ds_wrapper as w


TELEMETRY_OFFSET = 3_110_408
GIMBAL_PITCH_FIELD = 4   # index into the colon-separated telemetry string


def send(drone_id, command):
    w.sendWayPointData(command, drone_id)


def vs_gimbal(drone_id, gimbal_pitch, gimbal_yaw=0.0):
    """Send a VS: command that keeps all flight values at zero and only sets the gimbal."""
    cmd = f"VS:0.00:0.00:0.00:0.00:{gimbal_pitch:.2f}:{gimbal_yaw:.2f}"
    send(drone_id, cmd)


def read_gimbal_pitch(drone_id):
    """Best-effort telemetry read; returns None if unavailable."""
    try:
        data = w.getImageAndTelemetryData(drone_id)
        s = bytearray(data[TELEMETRY_OFFSET:]).decode().strip('\x00').strip()
        parts = s.split(':')
        if len(parts) > GIMBAL_PITCH_FIELD:
            return float(parts[GIMBAL_PITCH_FIELD])
    except Exception:
        pass
    return None


def build_sequence(start, end, step):
    direction = -abs(step) if end < start else abs(step)
    seq = []
    p = start
    if direction < 0:
        while p >= end - 1e-6:
            seq.append(round(p, 3))
            p += direction
    else:
        while p <= end + 1e-6:
            seq.append(round(p, 3))
            p += direction
    return seq


def confirm():
    print()
    print("=" * 64)
    print(" GIMBAL SWEEP TEST")
    print("=" * 64)
    print(" This will:")
    print("   1. ENABLE virtual stick on the drone (takes flight authority)")
    print("   2. Sweep the gimbal pitch in fixed-degree steps")
    print("   3. DISABLE virtual stick")
    print()
    print(" While VS is enabled, the app commands altitude 0 / heading 0.")
    print(" The drone MUST be on the ground or bench. Do not run in flight.")
    print(" Physical RC sticks override VS — keep a thumb on them.")
    print("=" * 64)
    print()
    return input("Type 'go' to proceed, anything else to abort: ").strip().lower() == "go"


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm gimbal pitch sweep test")
    ap.add_argument("--drone", type=int,   default=1,     help="Drone ID, 1-based (default 1)")
    ap.add_argument("--start", type=float, default=0.0,   help="Start pitch in deg (default 0)")
    ap.add_argument("--end",   type=float, default=-90.0, help="End pitch in deg (default -90)")
    ap.add_argument("--step",  type=float, default=10.0,  help="Step size in deg (default 10)")
    ap.add_argument("--dwell", type=float, default=2.0,   help="Seconds to hold each step (default 2)")
    ap.add_argument("--no-return", action="store_true",   help="Do not sweep back to start")
    ap.add_argument("--yes",       action="store_true",   help="Skip confirmation prompt")
    args = ap.parse_args()

    if not args.yes and not confirm():
        print("Aborted.")
        return

    drone = args.drone

    sequence = build_sequence(args.start, args.end, args.step)
    if not args.no_return and len(sequence) > 1:
        # Sweep back, skipping the duplicate end point
        sequence = sequence + list(reversed(sequence))[1:]
    print(f"\nDrone {drone}: {len(sequence)} steps, "
          f"{args.start:+.0f}° → {args.end:+.0f}° in {args.step:.0f}° increments, "
          f"dwell {args.dwell}s\n")

    try:
        print("Enabling virtual stick…")
        send(drone, "ENABLE_VS")
        time.sleep(2.0)   # give the DJI SDK time to take flight authority

        for i, pitch in enumerate(sequence, 1):
            vs_gimbal(drone, pitch)
            time.sleep(args.dwell)
            actual = read_gimbal_pitch(drone)
            if actual is None:
                report = "(no telemetry)"
            else:
                err = actual - pitch
                report = f"actual {actual:+6.1f}°   Δ {err:+5.1f}°"
            print(f"  [{i:>2}/{len(sequence)}]  cmd {pitch:+6.1f}°   →   {report}")

        print("\nSweep complete.")

    except KeyboardInterrupt:
        print("\nInterrupted — cleaning up.")

    finally:
        print("Parking gimbal at -90° and disabling virtual stick…")
        vs_gimbal(drone, -90.0)
        time.sleep(1.0)
        send(drone, "DISABLE_VS")
        print("Done.")


if __name__ == "__main__":
    main()
