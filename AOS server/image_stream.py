"""STANDALONE DEBUG TOOL — do NOT run alongside a live controller.

This polls ds_wrapper.getImageAndTelemetryData from its own process. The
wrapper's shared-memory protocol has one status byte per drone slot and no
mutex, so a second process polling it starves a running controller's
command/telemetry loops (they collapse to ~1 Hz). The normal path is now
in-process streaming: swarm_flocking.py --image-stream (image_stream_feed.py),
enabled via the ImageStream key in flocking.config.psd1.

Use this script only for debugging the video path with NO controller running.
"""

import time
import cv2
import ds_wrapper as w
import threading
from collections import deque

# Imports for image sharing to memory mapped files
import mmap
import utils.imageSharingUtil as imageSharingUtil

# Ceiling on the wrapper poll rate so this tool never spins the shared-memory
# protocol flat-out.
POLL_INTERVAL_S = 0.05

# Sliding window (seconds) over which the per-drone frame frequency is averaged.
FREQ_WINDOW_S = 5.0

print("Starting Image test...")
print("WARNING: standalone debug tool - do not run while a controller "
      "(joystick_controller.py / swarm_flocking.py) is flying; it contends "
      "for the ds_wrapper protocol. Use swarm_flocking.py --image-stream "
      "instead.")

num_drones = 1
decode = w.isHWDecoderEnabled()

if decode == 1:
    decoding = 'hardware'
elif decode == 0:
    decoding = 'software'
else:
    print('Invalid decoding method')

# Shared memory configuration
width = 800
height = 450
depth = 3
processedImageSize = width * height * depth
# Owned by utils.imageSharingUtil (the module that writes the bytes). This tool has
# no camera pose to publish, so its blocks carry poseStatus 0 and are simply not
# usable by the PLANAR stitcher -- STABSTITCH is unaffected.
metadataSize = imageSharingUtil.BLOCK_HEADER_BYTES
blockSize = metadataSize + processedImageSize
totalMMFSize = num_drones * blockSize

# Create shared memory mapped file once
try:
    processedMMF = mmap.mmap(-1, totalMMFSize, "BlockSharedMemory")
except Exception as e:
    print(f"Error creating shared memory: {e}")
    processedMMF = None

def process_drone(drone_id):
    """Process image stream for a single drone in a separate thread"""
    print(f"[Drone {drone_id}] Processing thread started")

    # Timestamps of the last FREQ_WINDOW_S seconds of frame fetches, used to
    # report a rolling-average frame frequency.
    frame_times = deque()

    try:
        while True:
            time.sleep(POLL_INTERVAL_S)
            print(f"[Drone {drone_id}] Fetching telemetry data...")

            # get the current telemetry data
            image_telemetry_data = w.getImageAndTelemetryData(drone_id)

            # Record this fetch and drop samples older than the averaging window.
            now = time.monotonic()
            frame_times.append(now)
            while frame_times and now - frame_times[0] > FREQ_WINDOW_S:
                frame_times.popleft()
            # Average over the elapsed span (up to FREQ_WINDOW_S); the first
            # frame bounds the interval, so frequency uses count-1 gaps.
            span = now - frame_times[0]
            freq = (len(frame_times) - 1) / span if span > 0 else 0.0

            print(f"[Drone {drone_id}] Telemetry data fetched. "
                  f"Frequency (avg over {FREQ_WINDOW_S:.0f}s): {freq:.2f} Hz")

            telemetry_data = bytearray(image_telemetry_data[3110408:]).decode()
            telemetry_elements = telemetry_data.split(':')

            print(f"[Drone {drone_id}] Telemetry: {telemetry_elements}")
            latitude = telemetry_elements[0]
            longitude = telemetry_elements[1]
            altitude = telemetry_elements[2]
            heading = float(telemetry_elements[3])
            curr_pitch = float(telemetry_elements[4])
            gimbal_yaw = telemetry_elements[6]
            waypoint_check = telemetry_elements[14]
            next_waypoint = telemetry_elements[15]
        
            ####################################### Image Processing #############################################

            # Get the image data from the drones
            if decoding == 'software':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV420p2RGB)
            elif decoding == 'hardware':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV2BGR_NV12)

            ######################################### Image Sharing to Memory Mapped Files ############################################

            # Resize the image
            Image = cv2.resize(Image, (width, height))

            # Write the image to shared memory
            if processedMMF is not None:
                try:
                    print(f"[Drone {drone_id}] Writing processed image to shared memory")
                    
                    # Compute the block offset for this droneId
                    blockOffset = (drone_id - 1) * blockSize

                    # Optionally flip the image vertically
                    # Image = cv2.flip(Image, 0)

                    # Write the memory block (header and image data)
                    imageSharingUtil.write_memory(processedMMF, blockOffset, processedImageSize, Image, drone_id - 1, heading, enable_debug=True)
                    
                    # Clean up image
                    del Image
                except Exception as e:
                    print(f"[Drone {drone_id}] Problem writing to shared memory: {e}")

            ############################################################################################################################

    except KeyboardInterrupt:
        print(f"[Drone {drone_id}] Processing stopped by user.")
    except Exception as e:
        print(f"[Drone {drone_id}] An error occurred: {e}")
    finally:
        print(f"[Drone {drone_id}] Processing thread finished.")

# Create and start threads for each drone
threads = []
try:
    for drone_id in range(1, num_drones + 1):
        thread = threading.Thread(target=process_drone, args=(drone_id,), daemon=True)
        threads.append(thread)
        thread.start()
        print(f"Started thread for Drone {drone_id}")

    # Keep main thread alive
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("Main thread interrupted by user.")
finally:
    print("Image test finished.")
    if processedMMF is not None:
        processedMMF.close()