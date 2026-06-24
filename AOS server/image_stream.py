import time
import cv2
import base64
import ds_wrapper as w
import threading
from queue import Queue

# Imports for image sharing to memory mapped files
import struct
import mmap
import utils.imageSharingUtil as imageSharingUtil

print("Starting Image test...")

num_drones = 1
decode = w.isHWDecoderEnabled()

if decode == 1:
    decoding = 'hardware'
elif decode == 0:
    decoding = 'software'
else:
    print('Invalid decoding method')

# Shared memory configuration
width = 640
height = 360
depth = 3
processedImageSize = width * height * depth
metadataSize = 12
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
    
    try:
        while True:
            print(f"[Drone {drone_id}] Fetching telemetry data...")

            # get the current telemetry data
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
        
            ####################################### Image Processing #############################################

            # Get the image data from the drones
            if decoding == 'software':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV420p2RGB)
            elif decoding == 'hardware':
                Image = cv2.cvtColor(image_telemetry_data[0:3110400].reshape(1080*3//2, 1920), cv2.COLOR_YUV2BGR_NV12)
                            
            # Convert the image to base64
            retval, buffer = cv2.imencode('.jpg', Image)
            imgBase64 = base64.b64encode(buffer).decode('utf-8')
            
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