"""
LIS_Swarm Receive Test
======================
Read-only test of the telemetry + image pipeline.

  Drone -> Android App (SwarmActivity) -> RTSP/SharedMem -> DroneSwarmServer.exe
    -> SharedMem -> ds_wrapper.getImageAndTelemetryData()

This script does NOT call sendWayPointData, ENABLE_VS, TAKEOFF, or any other
command. It is safe to run with the drone powered on a bench, on the ground,
or in flight. Use it to confirm the receive path is healthy before touching
joystick_controller.py.

Usage:
    python receive_test.py
    python receive_test.py --drone 1 --rate 4
    python receive_test.py --save-frame frame.yuv
"""

import argparse
import hashlib
import os
import time

import ds_wrapper as w


IMAGE_LEN = 1920 * 1080 * 3 // 2          # 3,110,400 bytes YUV 4:2:0
TELEMETRY_OFFSET = 3_110_408


def parse_telemetry(raw_data):
    """Decode the colon-separated telemetry string after the image bytes."""
    try:
        telemetry_str = bytearray(raw_data[TELEMETRY_OFFSET:]).decode().strip('\x00').strip()
        parts = telemetry_str.split(':')
        if len(parts) < 17:
            return None
        return {
            'lat':           float(parts[0]),
            'lon':           float(parts[1]),
            'alt':           float(parts[2]),
            'heading':       float(parts[3]),
            'gimbal_pitch':  float(parts[4]),
            'gimbal_roll':   float(parts[5]),
            'gimbal_yaw':    float(parts[6]),
            'sat_count':     int(parts[7]),
            'drone_pitch':   float(parts[8]),
            'drone_roll':    float(parts[9]),
            'drone_yaw':     float(parts[10]),
            'vx':            float(parts[11]),
            'vy':            float(parts[12]),
            'vz':            float(parts[13]),
            'waypoint_done': int(parts[14]),
            'value_check':   float(parts[15]),
            'vs_enabled':    float(parts[16]) > 0,
        }
    except Exception:
        return None


def image_signature(raw_data):
    """Cheap fingerprint of the image: (nonzero count in first 1KB, md5 prefix).
    Used to detect 'no frames' (all zeros) and 'frames frozen' (unchanging hash)."""
    head = bytes(raw_data[:1024])
    nonzero = sum(1 for b in head if b != 0)
    digest = hashlib.md5(head).hexdigest()[:8]
    return nonzero, digest


def render(state, telem, img_nonzero, img_sig):
    os.system('cls' if os.name == 'nt' else 'clear')
    print(f"=== LIS_Swarm Receive Test — drone {state['drone']} ===")
    print(f"Polls: {state['polls']:>5}   Elapsed: {state['elapsed']:.0f}s   "
          f"Telemetry OK: {state['telemetry_ok']}/{state['polls']}   "
          f"Image OK: {state['image_ok']}/{state['polls']}   "
          f"Frame changes: {state['frame_changes']}")
    print()

    if telem:
        print("--- Telemetry ---")
        print(f"  GPS:     {telem['lat']:.6f}, {telem['lon']:.6f}   alt {telem['alt']:+.1f} m")
        print(f"  Sats:    {telem['sat_count']:>2}   Heading: {telem['heading']:.1f}°")
        print(f"  Aircraft P/R/Y:  {telem['drone_pitch']:+7.1f} / {telem['drone_roll']:+7.1f} / {telem['drone_yaw']:+7.1f}")
        print(f"  Gimbal   P/R/Y:  {telem['gimbal_pitch']:+7.1f} / {telem['gimbal_roll']:+7.1f} / {telem['gimbal_yaw']:+7.1f}")
        print(f"  Velocity X/Y/Z:  {telem['vx']:+7.2f} / {telem['vy']:+7.2f} / {telem['vz']:+7.2f}  m/s")
        print(f"  Virtual stick:   {'ENABLED' if telem['vs_enabled'] else 'disabled'}")
    else:
        print("--- Telemetry: NO DATA ---")
        print("  Raw string did not parse. Likely causes:")
        print("    * Drone not connected to RC Pro")
        print("    * AOSManager.setRunning(true) not called in SwarmActivity")
        print("    * DroneSwarmServer.exe not running or wrong drone slot")

    print()
    print("--- Image stream ---")
    print(f"  First-1KB nonzero bytes: {img_nonzero}/1024     fingerprint: {img_sig}")
    if img_nonzero == 0:
        print("  NO image bytes — RTSP / feedFrame path appears dead.")
    elif state['frame_changes'] == 0 and state['polls'] > 5:
        print("  Image bytes present but UNCHANGING — frames may be stuck.")
    else:
        print("  Image bytes present and changing — frames are flowing.")


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm telemetry + image receive-only test")
    ap.add_argument("--drone", type=int, default=1, help="Drone ID, 1-based (default 1)")
    ap.add_argument("--rate", type=float, default=2.0, help="Poll rate in Hz (default 2)")
    ap.add_argument("--save-frame", metavar="PATH",
                    help="Save the first non-zero YUV frame to PATH. View with:\n"
                         "  ffmpeg -f rawvideo -pix_fmt nv21 -s 1920x1080 -i PATH PATH.png")
    args = ap.parse_args()

    print(f"LIS_Swarm Receive Test — drone {args.drone}, polling at {args.rate} Hz")
    print(f"HW Decoder: {'enabled' if w.isHWDecoderEnabled() == 1 else 'disabled (SW)'}")
    print("Press Ctrl+C to stop.\n")
    time.sleep(0.5)

    interval = 1.0 / args.rate
    state = {
        'drone': args.drone,
        'polls': 0,
        'telemetry_ok': 0,
        'image_ok': 0,
        'frame_changes': 0,
        'elapsed': 0.0,
    }
    last_sig = None
    saved = False
    start = time.time()

    try:
        while True:
            state['polls'] += 1
            state['elapsed'] = time.time() - start

            data = w.getImageAndTelemetryData(args.drone)
            telem = parse_telemetry(data)
            nonzero, sig = image_signature(data)

            if telem is not None:
                state['telemetry_ok'] += 1
            if nonzero > 0:
                state['image_ok'] += 1
                if last_sig is not None and sig != last_sig:
                    state['frame_changes'] += 1
                last_sig = sig
                if args.save_frame and not saved:
                    with open(args.save_frame, "wb") as f:
                        f.write(bytes(data[:IMAGE_LEN]))
                    saved = True

            render(state, telem, nonzero, sig)
            if saved:
                print(f"\n[saved YUV frame to {args.save_frame}]")

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
