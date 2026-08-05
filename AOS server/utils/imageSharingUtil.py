import struct
import mmap
import time

# ---------------------------------------------------------------------------------
# Block header layout. This module owns it: it is the only place that writes the
# bytes, so the sizes live here and every other module imports them rather than
# repeating the number. Mirrors PyUniSharingFast.cs's blockLegacyHeaderSize /
# blockPoseHeaderSize and ImageSharing.cs's MetadataSize -- change one, change all
# of them, and run vr_swarm_simulation/Assets/Scripts/ImageStitching/tools/
# check_wire_layout.py.
#
#   v1 (12) : int flag | int droneId | float heading
#   v2 (48) : ... plus float camPos[3] | float camRot[4] xyzw
#                 | float captureTime | int poseStatus
#
# The feed map is v2 throughout. The pose fields are what the sim's PLANAR stitcher
# needs; a producer with no pose to give writes zeros and poseStatus 0, which the
# consumer treats as "unposed" and drops from the planar solve while STABSTITCH
# ignores it entirely. One layout for every producer beats two, because a mismatch
# is silent -- it reads image bytes as a header rather than failing.
# ---------------------------------------------------------------------------------
BLOCK_HEADER_V1_BYTES = 12
BLOCK_HEADER_V2_BYTES = 48
BLOCK_HEADER_BYTES = BLOCK_HEADER_V2_BYTES

# Offsets inside the header, from the start of the block.
BLOCK_CAM_POS_OFFSET = 12       # float32 x, y, z   (Unity world, LEFT-handed)
BLOCK_CAM_ROT_OFFSET = 24       # float32 x, y, z, w (Unity Transform.rotation)
BLOCK_CAPTURE_TIME_OFFSET = 40  # float32, seconds since the publisher started
BLOCK_POSE_STATUS_OFFSET = 44   # int32 bitfield
POSE_VALID = 1 << 0


def write_memory(processedMMF, blockOffset, processedImageSize, image_data, droneId, heading,
                 enable_debug=False, pace_s=0.06, pose=None, capture_time=0.0):
    """
    Write an image block to shared memory with Unity.

    Block layout: see BLOCK_HEADER_* above. Always writes the v2 (48-byte) header.

    Args:
        processedMMF: Memory-mapped file object
        blockOffset: Offset in bytes for this drone's block
        processedImageSize: Size of image data in bytes
        image_data: Image data as numpy array
        droneId: Drone ID (int)
        heading: Heading angle (float)
        enable_debug: Enable debug logging (default: False)
        pace_s: Post-write sleep giving the consumer time to read before the
            next overwrite; caps the write rate at ~1/pace_s (default 0.06,
            slightly longer than Unity's 0.05 s readInterval)
        pose: optional ((x, y, z), (qx, qy, qz, qw)) in Unity world, from
            dji_camera_pose.CameraPoseSolver. None writes an all-zero pose with
            poseStatus 0, i.e. "this producer has no pose" -- which costs that
            view in the planar mosaic and nothing anywhere else.
        capture_time: seconds since the publisher started, NOT a wall clock. The
            field is float32, in which time.time() (~1.75e9) has about 128 s of
            resolution -- it would silently destroy the consumer's frame-skew
            gate rather than fail. Only ever read as a difference.
    """
    if enable_debug:
        print(f"[DEBUG] write_memory called: blockOffset={blockOffset}, droneId={droneId}, heading={heading:.2f}, imageSize={processedImageSize}")

    metadataSize = BLOCK_HEADER_BYTES
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

            # Camera pose. Written in one contiguous run so a torn read can only
            # ever mix a pose with itself, and always written -- an unwritten tail
            # would leave whatever the previous frame's drone put there.
            if pose is None:
                pos, quat, status = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), 0
            else:
                pos, quat = pose
                status = POSE_VALID
            processedMMF.seek(blockOffset + BLOCK_CAM_POS_OFFSET)
            processedMMF.write(struct.pack('<3f', *pos))
            processedMMF.write(struct.pack('<4f', *quat))
            processedMMF.write(struct.pack('<f', float(capture_time)))
            processedMMF.write(struct.pack('<i', status))
            if enable_debug:
                print(f"[DEBUG] Wrote pose: pos={pos} quat={quat} status={status}")

            # Convert image to bytes and check size
            image_bytes = image_data.tobytes()
            if len(image_bytes) != processedImageSize:
                raise ValueError(f"Image size mismatch: expected {processedImageSize}, got {len(image_bytes)}")

            # Write image data after the header
            if enable_debug:
                print(f"[DEBUG] Writing image data ({len(image_bytes)} bytes) at offset {blockOffset + metadataSize}")
            processedMMF.seek(blockOffset + metadataSize)
            processedMMF.write(image_bytes)

            # Reset flag to 0 (writing complete)
            processedMMF.seek(blockOffset)
            processedMMF.write(struct.pack('i', 0))
            if enable_debug:
                print(f"[DEBUG] Write complete for droneId {droneId}")
            
            # Give Unity time to read before next write
            time.sleep(pace_s)
            break
        else:
            # Flag is busy, wait a bit before retrying
            time.sleep(0.01)
            retry_count += 1
            if enable_debug and retry_count % 10 == 0:
                print(f"[DEBUG] Waiting for Unity to read... (retry {retry_count})")
    
    if retry_count >= max_retries:
        print(f"[WARNING] Max retries reached for droneId {droneId}")