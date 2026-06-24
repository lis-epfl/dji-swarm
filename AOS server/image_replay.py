import time
import cv2
import os
import mmap
import glob
import threading
import utils.imageSharingUtil as imageSharingUtil

# --- Configuration (Matching image_stream.py and image_save.py) ---
num_drones = 3  
width = 640
height = 360
depth = 3
processedImageSize = width * height * depth
metadataSize = 12
blockSize = metadataSize + processedImageSize
totalMMFSize = num_drones * blockSize

print(f"processedImageSize: {processedImageSize}, blockSize: {blockSize}, totalMMFSize: {totalMMFSize}")

# The directory where image_save.py dumped the files
TARGET_SESSION_DIR = "saved_streams\captured_drone_images_20260121_165127" 

# --- Shared Memory Initialization ---
try:
    processedMMF = mmap.mmap(-1, totalMMFSize, "BlockSharedMemory")
    print("Connected to Shared Memory: BlockSharedMemory")
except Exception as e:
    print(f"Error creating shared memory: {e}")
    processedMMF = None

def parse_heading_from_filename(filename):
    """Extracts heading from 'drone1_timestamp_h123.4_lat...' format"""
    try:
        # Splits at '_h' and takes the number before the next '_'
        return float(filename.split('_h')[1].split('_')[0])
    except:
        return 0.0

def process_drone_files(drone_id):
    """Monitor and replay images for a specific drone (Threaded)"""
    print(f"[Drone {drone_id}] Replay thread started")
    drone_dir = os.path.join(TARGET_SESSION_DIR, f"drone_{drone_id}")
    processed_files = set()
    wait_time = 0
    max_wait = 3  # Exit if directory doesn't exist after 30 seconds
    
    # Count total images before starting replay
    total_images = 0
    images_replayed = 0

    try:
        while True:
            if not os.path.exists(drone_dir):
                wait_time += 1
                if wait_time > max_wait:
                    print(f"[Drone {drone_id}] Directory not found after {max_wait}s. Exiting thread.")
                    break
                print(f"[Drone {drone_id}] Waiting for directory... ({wait_time}s)")
                time.sleep(1)
                continue
            
            wait_time = 0  # Reset counter once directory exists
            
            # Count total images on first iteration
            if total_images == 0:
                all_files = sorted(glob.glob(os.path.join(drone_dir, "*.jpg")))
                total_images = len(all_files)
                print(f"[Drone {drone_id}] Found {total_images} images to replay")

            # Get all images and sort by timestamp in filename to maintain order
            files = sorted(glob.glob(os.path.join(drone_dir, "*.jpg")))

            if not files:
                print(f"[Drone {drone_id}] No files found, waiting...")
                time.sleep(0.5)
                continue

            for filepath in files:
                # print(f"[Drone {drone_id}] Processing file: {filepath}")
                if filepath not in processed_files:
                    # Read the saved image
                    image = cv2.imread(filepath)
                    if image is None:
                        continue

                    # Extract heading metadata from the filename
                    heading = parse_heading_from_filename(os.path.basename(filepath))

                    # Standardize size for shared memory
                    if image.shape[0] != height or image.shape[1] != width:
                        image = cv2.resize(image, (width, height))

                    if processedMMF is not None:
                        try:
                            # Calculate offset exactly like image_stream.py
                            blockOffset = (drone_id - 1) * blockSize
                            
                            # Flip image to match streaming logic
                            # image = cv2.flip(image, 0)

                            # Write to shared memory
                            imageSharingUtil.write_memory(
                                processedMMF, 
                                blockOffset, 
                                processedImageSize, 
                                image, 
                                drone_id - 1, 
                                heading, 
                                enable_debug=False
                            )
                            images_replayed += 1
                            print(f"[Drone {drone_id}] Replayed {os.path.basename(filepath)} ({images_replayed}/{total_images})")
                        except Exception as e:
                            print(f"[Drone {drone_id}] MMF Write Error: {e}")

                    processed_files.add(filepath)
                    # Small delay to simulate the original CAPTURE_INTERVAL
                    time.sleep(0.05)
            
            # Loop back through folder after all images processed
            if len(processed_files) == total_images and total_images > 0:
                print(f"[Drone {drone_id}] Completed {images_replayed} images. Looping back...")
                processed_files.clear()
                images_replayed = 0
                time.sleep(1)
            else:
                time.sleep(0.5)  # Poll for new files

    except Exception as e:
        print(f"[Drone {drone_id}] Thread Error: {e}")

# --- Main Thread Logic ---
threads = []
try:
    print(f"Starting Replay from: {TARGET_SESSION_DIR}")
    for d_id in range(1, num_drones + 1):
        # Create threads for each drone ID
        thread = threading.Thread(target=process_drone_files, args=(d_id,), daemon=True)
        threads.append(thread)
        thread.start()
        print(f"Started replay thread for Drone {d_id}")

    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nReplay stopped by user.")
finally:
    if processedMMF is not None:
        processedMMF.close()
    print("Replay finished.")