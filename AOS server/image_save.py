import time
import cv2
import base64
import ds_wrapper as w
import threading
import os
from datetime import datetime

print("Starting Image Capture...")

num_drones = 3
decode = w.isHWDecoderEnabled()

if decode == 1:
    decoding = 'hardware'
elif decode == 0:
    decoding = 'software'
else:
    print('Invalid decoding method')

# Output directory for saved images with timestamp
session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_dir = f"saved_streams/captured_drone_images_{session_timestamp}"
os.makedirs(output_dir, exist_ok=True)

# Configuration
CAPTURE_INTERVAL = 0.05  # Time between saves in seconds
MAX_CAPTURES = 150       # Maximum images per drone (set to None for unlimited)

capture_count = {i: 0 for i in range(1, num_drones + 1)}

def process_drone(drone_id):
    """Process and save image stream for a single drone in a separate thread"""
    print(f"[Drone {drone_id}] Processing thread started")
    
    # Create drone-specific subdirectory
    drone_dir = os.path.join(output_dir, f"drone_{drone_id}")
    os.makedirs(drone_dir, exist_ok=True)
    
    try:
        while True:
            # Check if we've reached max captures
            if MAX_CAPTURES and capture_count[drone_id] >= MAX_CAPTURES:
                print(f"[Drone {drone_id}] Reached maximum captures ({MAX_CAPTURES})")
                break
            
            print(f"[Drone {drone_id}] Fetching telemetry data...")

            # Get the current telemetry data
            image_telemetry_data = w.getImageAndTelemetryData(drone_id)

            print(f"[Drone {drone_id}] Telemetry data fetched.")

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
        
            # Get the image data from the drones
            if decoding == 'software':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV420p2RGB)
            elif decoding == 'hardware':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV2BGR_NV12)
            
            # Generate timestamp for filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            
            # Save image with metadata in filename
            filename = f"drone{drone_id}_{timestamp}_h{heading:.1f}_lat{latitude}_lon{longitude}_alt{altitude}.jpg"
            filepath = os.path.join(drone_dir, filename)
            
            # Convert RGB to BGR for cv2.imwrite if using software decoding
            if decoding == 'software':
                Image = cv2.cvtColor(Image, cv2.COLOR_RGB2BGR)
            
            cv2.imwrite(filepath, Image)
            
            capture_count[drone_id] += 1
            print(f"[Drone {drone_id}] Saved image {capture_count[drone_id]}: {filename}")
            
            # Clean up
            del Image
            
            # Wait before next capture
            time.sleep(CAPTURE_INTERVAL)

    except KeyboardInterrupt:
        print(f"[Drone {drone_id}] Processing stopped by user.")
    except Exception as e:
        print(f"[Drone {drone_id}] An error occurred: {e}")
    finally:
        print(f"[Drone {drone_id}] Processing thread finished. Total images: {capture_count[drone_id]}")

# Create and start threads for each drone
threads = []
try:
    print("=" * 60)
    print("Drone Image Capture - Direct Streaming")
    print("=" * 60)
    print(f"Output directory: {output_dir}")
    print(f"Monitoring {num_drones} drones...")
    print(f"Capture interval: {CAPTURE_INTERVAL}s")
    print(f"Max captures per drone: {MAX_CAPTURES if MAX_CAPTURES else 'Unlimited'}")
    print("Press Ctrl+C to stop capturing\n")
    
    for drone_id in range(1, num_drones + 1):
        thread = threading.Thread(target=process_drone, args=(drone_id,), daemon=True)
        threads.append(thread)
        thread.start()
        print(f"Started capture thread for Drone {drone_id}")

    # Keep main thread alive
    while True:
        time.sleep(1)
        
        # Check if all threads have finished (reached max captures)
        if MAX_CAPTURES and all(capture_count[i] >= MAX_CAPTURES for i in range(1, num_drones + 1)):
            print("\nAll drones reached maximum captures.")
            break

except KeyboardInterrupt:
    print("\n\nCapture stopped by user.")
finally:
    print("\nCapture summary:")
    for drone_id, count in capture_count.items():
        print(f"  Drone {drone_id}: {count} images saved")
    print(f"\nImages saved to: {output_dir}")
    print("Image capture finished.")