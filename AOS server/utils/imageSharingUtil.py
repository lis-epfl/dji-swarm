import struct
import mmap
import time

def write_memory(processedMMF, blockOffset, processedImageSize, image_data, droneId, heading, enable_debug=False):
    """
    Write an image block to shared memory with Unity.

    Block layout:
      - int flag (4 bytes)
      - int droneId (4 bytes)
      - float heading (4 bytes)
      - image data (processedImageSize bytes)
    
    Args:
        processedMMF: Memory-mapped file object
        blockOffset: Offset in bytes for this drone's block
        processedImageSize: Size of image data in bytes
        image_data: Image data as numpy array
        droneId: Drone ID (int)
        heading: Heading angle (float)
        enable_debug: Enable debug logging (default: False)
    """
    if enable_debug:
        print(f"[DEBUG] write_memory called: blockOffset={blockOffset}, droneId={droneId}, heading={heading:.2f}, imageSize={processedImageSize}")
    
    metadataSize = 12  # 4 bytes flag, 4 bytes droneId, 4 bytes heading
    blockSize = metadataSize + processedImageSize
    max_retries = 100
    retry_count = 0

    while retry_count < max_retries:
        # Check if Unity is ready (flag == 0)
        processedMMF.seek(blockOffset)
        flag_bytes = processedMMF.read(4)
        if len(flag_bytes) != 4:
            print("Error: Buffer for flag is less than 4 bytes. Buffer length:", len(flag_bytes))
            time.sleep(0.001)
            retry_count += 1
            continue
            
        flag = struct.unpack('i', flag_bytes)[0]
        if enable_debug:
            print(f"[DEBUG] Flag value: {flag}")

        if flag == 0:
            # Set flag to 1 (busy writing)
            if enable_debug:
                print(f"[DEBUG] Writing to offset {blockOffset}")
            processedMMF.seek(blockOffset)
            processedMMF.write(struct.pack('i', 1))

            # Write droneId (int) at offset blockOffset + 4
            processedMMF.seek(blockOffset + 4)
            processedMMF.write(struct.pack('i', droneId))
            if enable_debug:
                print(f"[DEBUG] Wrote droneId: {droneId}")

            # Write heading (float) at offset blockOffset + 8
            processedMMF.seek(blockOffset + 8)
            processedMMF.write(struct.pack('f', heading))
            if enable_debug:
                print(f"[DEBUG] Wrote heading: {heading}")

            # Convert image to bytes and check size
            image_bytes = image_data.tobytes()
            if len(image_bytes) != processedImageSize:
                raise ValueError(f"Image size mismatch: expected {processedImageSize}, got {len(image_bytes)}")

            # Write image data at offset blockOffset + 12
            if enable_debug:
                print(f"[DEBUG] Writing image data ({len(image_bytes)} bytes) at offset {blockOffset + 12}")
            processedMMF.seek(blockOffset + 12)
            processedMMF.write(image_bytes)

            # Reset flag to 0 (writing complete)
            processedMMF.seek(blockOffset)
            processedMMF.write(struct.pack('i', 0))
            if enable_debug:
                print(f"[DEBUG] Write complete for droneId {droneId}")
            
            # Give Unity time to read before next write
            time.sleep(0.06)  # Slightly longer than Unity's readInterval (0.05s)
            break
        else:
            # Flag is busy, wait a bit before retrying
            time.sleep(0.01)
            retry_count += 1
            if enable_debug and retry_count % 10 == 0:
                print(f"[DEBUG] Waiting for Unity to read... (retry {retry_count})")
    
    if retry_count >= max_retries:
        print(f"[WARNING] Max retries reached for droneId {droneId}")