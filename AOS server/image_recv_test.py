"""STANDALONE RATE BENCHMARK — do NOT run alongside a live controller.

Polls ds_wrapper.getImageAndTelemetryData in a tight loop, doing NO image
decode, resize, or shared-memory writes — just enough parsing to prove the
telemetry came through. The point is to measure the raw rate at which
telem/image blocks arrive from DroneSwarmServer.exe.

Like image_stream.py, this contends for the wrapper's shared-memory protocol,
so run it only with NO controller (joystick_controller.py / swarm_flocking.py)
running.
"""

import time
import threading
from collections import deque

import ds_wrapper as w

# How often (seconds) to print a rate summary. No sleep in the fetch loop —
# we poll as fast as the wrapper will return.
REPORT_INTERVAL_S = 1.0

# Sliding window (seconds) over which the per-drone receive rate is averaged.
FREQ_WINDOW_S = 5.0

# Telemetry string starts at this byte offset in the returned array (see
# CLAUDE.md / joystick_controller.parse_telemetry).
TELEM_OFFSET = 3110408

num_drones = 1

print("Starting image/telemetry receive-rate benchmark...")
print("WARNING: standalone debug tool - do not run while a controller "
      "(joystick_controller.py / swarm_flocking.py) is flying; it contends "
      "for the ds_wrapper protocol.")


def measure_drone(drone_id):
    """Poll a single drone as fast as possible and report the receive rate."""
    print(f"[Drone {drone_id}] Receive thread started")

    # Timestamps of fetches within the last FREQ_WINDOW_S seconds.
    frame_times = deque()
    total = 0
    last_report = time.monotonic()

    try:
        while True:
            # Fetch the current telem/image block (blocks until the wrapper
            # returns; no artificial sleep so we measure the true arrival rate).
            image_telemetry_data = w.getImageAndTelemetryData(drone_id)

            now = time.monotonic()
            total += 1

            # Roll the averaging window.
            frame_times.append(now)
            while frame_times and now - frame_times[0] > FREQ_WINDOW_S:
                frame_times.popleft()

            # Touch the telemetry so we don't measure a no-op the wrapper could
            # optimise away, and so a malformed block surfaces as an error.
            heading = float(bytearray(image_telemetry_data[TELEM_OFFSET:])
                            .decode().split(':')[3])

            if now - last_report >= REPORT_INTERVAL_S:
                span = now - frame_times[0]
                freq = (len(frame_times) - 1) / span if span > 0 else 0.0
                print(f"[Drone {drone_id}] {freq:6.2f} Hz "
                      f"(avg over {FREQ_WINDOW_S:.0f}s)  "
                      f"total={total}  heading={heading:.1f}")
                last_report = now

    except KeyboardInterrupt:
        print(f"[Drone {drone_id}] Stopped by user.")
    except Exception as e:
        print(f"[Drone {drone_id}] An error occurred: {e}")
    finally:
        print(f"[Drone {drone_id}] Receive thread finished. total={total}")


threads = []
try:
    for drone_id in range(1, num_drones + 1):
        thread = threading.Thread(target=measure_drone, args=(drone_id,),
                                  daemon=True)
        threads.append(thread)
        thread.start()
        print(f"Started thread for Drone {drone_id}")

    # Keep main thread alive.
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Main thread interrupted by user.")
finally:
    print("Receive-rate benchmark finished.")
