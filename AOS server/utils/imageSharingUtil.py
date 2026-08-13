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

# ---------------------------------------------------------------------------------
# Geometry of the STITCHER's section, "BlockSharedMemory".
#
# This is a different map from the feed map this repo normally writes
# ("DroneFeedSharedMemory", MAX_DRONES x [48 + 800*450*3], owned by
# image_stream_feed.py). In the normal architecture nothing here touches
# BlockSharedMemory at all -- Unity's ImageSharing.cs is its sole producer. The two
# legacy debug tools image_stream.py and image_replay.py bypass Unity and write it
# directly, and these constants are what let them do that without corrupting it.
#
# The section is a FIXED array of FIXED-stride slots, and both numbers must match
# PyUniSharingFast.blockSlotCapacity / blockSlotStride, ImageSharing.cs's
# StitchSlotCapacity / StitchSlotStride, and StitcherThreading.py's
# BLOCK_SLOT_CAPACITY / BLOCK_SLOT_STRIDE, exactly. This is not a convention:
#
#   * A named Windows section cannot be resized. mmap.mmap(-1, size, tagname) is
#     CreateFileMapping underneath, and when the name already exists it opens the
#     EXISTING section -- a larger request fails with ERROR_ACCESS_DENIED, and a
#     smaller one silently succeeds with a partial view. So a tool here that asks
#     for a differently-sized BlockSharedMemory either denies Unity its mapping
#     (if it gets there first) or quietly maps a prefix of Unity's.
#   * The stride is sized from the sim's 1280x720 image envelope, NOT from the
#     800x450 feed this repo produces. An 800x450 payload is written as a prefix of
#     the slot and the rest is padding. Computing the stride from the local image
#     size, as these tools used to, puts every slot after the first in the middle
#     of its predecessor's pixels.
#
# vr_swarm_simulation/Assets/Scripts/ImageStitching/tools/check_wire_layout.py
# asserts all four copies against each other when this repo is checked out beside
# the sim. Run it after touching any of them.
STITCH_SLOT_CAPACITY = 24
STITCH_MAX_IMAGE_WIDTH = 1280
STITCH_MAX_IMAGE_HEIGHT = 720
# Kept on one line each: check_wire_layout.py parses this file with a line-oriented
# regex, and a constant it cannot evaluate is silently skipped rather than checked.
STITCH_SLOT_STRIDE = BLOCK_HEADER_V2_BYTES + STITCH_MAX_IMAGE_WIDTH * STITCH_MAX_IMAGE_HEIGHT * 3
STITCH_SECTION_BYTES = STITCH_SLOT_CAPACITY * STITCH_SLOT_STRIDE

# ---------------------------------------------------------------------------------
# Geometry of the FEED section, "DroneFeedSharedMemory" -- the map this repo writes
# and Unity's ImageSharing.cs reads. It lives here rather than in image_stream_feed.py
# for one reason: check_wire_layout.py parses THIS file and not that one, so a
# constant stated here is machine-checked against the C# and a constant stated there
# is not. image_stream_feed.py re-exports these.
#
# Layout: FEED_MAX_DRONES blocks of FEED_BLOCK_STRIDE, then a fixed trailer.
#
#   block i   at i * FEED_BLOCK_STRIDE     (48-byte v2 header + 800x450 BGR)
#   trailer   at FEED_TRAILER_OFFSET       (the PLANAR scene-plane standoff)
# ---------------------------------------------------------------------------------
FEED_MAX_DRONES = 10
FEED_IMAGE_WIDTH = 800
FEED_IMAGE_HEIGHT = 450
FEED_IMAGE_BYTES = FEED_IMAGE_WIDTH * FEED_IMAGE_HEIGHT * 3
FEED_BLOCK_STRIDE = BLOCK_HEADER_V2_BYTES + FEED_IMAGE_BYTES
FEED_BLOCKS_BYTES = FEED_MAX_DRONES * FEED_BLOCK_STRIDE
FEED_TRAILER_OFFSET = FEED_BLOCKS_BYTES

# ---------------------------------------------------------------------------------
# The scene-plane trailer: the PC's answer to "how far in front of the formation is
# the surface", which is the one PLANAR input the field cannot measure and which was
# previously typed into Unity's planarStandoffMetres by hand. Getting it wrong is the
# dominant mosaic error -- 30 m typed against a true 34.3 m on the 2026-08-11 MED
# clips is ~21 px of seam, four times everything else combined.
#
# Producer: this repo (image_stream_feed.py live, clip_replay.py for a recorded clip).
# Consumer: ImageSharing.cs, which hands it to PyUniSharingFast to republish into
# MetadataSharedMemory. One producer, one consumer, as everywhere else on this wire.
# Unity NEVER writes these bytes, and must not extend its block-init loop over them.
#
# WHY IT FITS HERE RATHER THAN IN A SECTION OF ITS OWN. A named Windows section
# cannot be resized, so growing a map that another process may already have created
# is normally how you earn ERROR_ACCESS_DENIED. It is safe here only because Windows
# compares PAGE-ROUNDED sizes: the block array is 10,800,480 B, which rounds up to
# 10,801,152 (2637 x 4 KB), leaving 672 bytes that are already backed and already
# mapped. Measured on this machine: create at the block size and open at +672 both
# succeed in either order, and +673 fails with ERROR_ACCESS_DENIED. So an old
# producer and a new Unity interoperate, in both start orders, and the trailer simply
# reads zero.
#
# That headroom is the whole basis of the design, so FEED_TRAILER_BYTES <= 672 is
# asserted by check_wire_layout.py rather than left as a comment. Past it, both cross
# orders become a hard failure and the feed dies.
FEED_TRAILER_BYTES = 64
FEED_SECTION_BYTES = FEED_TRAILER_OFFSET + FEED_TRAILER_BYTES

# 'PSO1' -- a fresh section is zero-filled, so a magic that is not 0 is what tells
# Unity "a PC has written here" apart from "this value happens to be 0". Without it
# an untouched trailer reads as a legal-looking 0.0 m standoff.
FEED_TRAILER_MAGIC = 0x50534F31
FEED_TRAILER_VERSION = 1

# Offsets within the trailer, from FEED_TRAILER_OFFSET.
FEED_TR_MAGIC_OFFSET = 0
FEED_TR_VERSION_OFFSET = 4
FEED_TR_SEQ_OFFSET = 8
FEED_TR_HEARTBEAT_OFFSET = 12
FEED_TR_STANDOFF_OFFSET = 16
FEED_TR_STATUS_OFFSET = 20
FEED_TR_FACADE_ID_OFFSET = 24
FEED_TR_LOOK_OFF_OFFSET = 28
FEED_TR_SPREAD_OFFSET = 32
FEED_TR_PX_PER_M_OFFSET = 36
FEED_TR_TILT_OFFSET = 40
FEED_TR_VIEW_COUNT_OFFSET = 44
FEED_TR_END = 48

# status bits
FEED_TR_STATUS_LOCKED = 1
FEED_TR_STATUS_DWELLING = 2
FEED_TR_STATUS_NO_FACADE = 4
FEED_TR_STATUS_NO_ORIGIN = 8


def open_stitch_map(map_name="BlockSharedMemory"):
    """
    Map the stitcher's BlockSharedMemory at its fixed size and retire every slot.

    Returns the mmap, or raises. Callers address slot i at i * STITCH_SLOT_STRIDE.

    Every slot is initialised to flag = 0, droneId = -1, poseStatus = 0 — not just
    the ones this tool intends to fill. A fresh section is zero-filled and 0 is a
    legal drone id, so an untouched slot otherwise advertises a ready block from
    drone 0 carrying an all-zero (degenerate) quaternion, which the sim's PLANAR
    solve reports as a geometry error rather than as an empty slot. That used to be
    unreachable here because the section was sized to exactly the slots the tool
    filled; against a fixed 24-slot capacity most of them stay empty for the whole
    run, so this loop is now the only thing standing between one replayed drone and
    23 phantom ones.
    """
    mmf = mmap.mmap(-1, STITCH_SECTION_BYTES, map_name)
    for slot in range(STITCH_SLOT_CAPACITY):
        base = slot * STITCH_SLOT_STRIDE
        mmf.seek(base)
        mmf.write(struct.pack('<i', 0))                    # flag: ready
        mmf.write(struct.pack('<i', -1))                   # droneId: no view here
        mmf.seek(base + BLOCK_POSE_STATUS_OFFSET)
        mmf.write(struct.pack('<i', 0))                    # poseStatus: unposed
    return mmf


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


# ---------------------------------------------------------------------------------
# Scene-plane trailer I/O. Same argument as write_memory: one module writes these
# bytes, so the layout is stated once and every producer goes through this.
# ---------------------------------------------------------------------------------

def open_feed_map(map_name="DroneFeedSharedMemory"):
    """
    Map DroneFeedSharedMemory, preferring the size that includes the trailer.

    Returns ``(mmf, trailer_ok)``. ``trailer_ok`` False means the section already
    existed at the pre-trailer size, i.e. a producer or a Unity built before the
    standoff trailer is holding it: the feeds work exactly as before and the standoff
    falls back to Unity's inspector field.

    The fallback is not decoration. This call sites in ``ImageStreamPublisher``'s
    constructor, which ``swarm_flocking.main()`` runs before the aircraft are armed --
    an unhandled ``OSError`` here does not degrade the mosaic, it takes down the whole
    flight controller. Given the page-rounding headroom (see FEED_TRAILER_BYTES) the
    grown request should never actually fail, which is precisely why the failure path
    has to be one that keeps flying rather than one nobody has ever exercised.
    """
    try:
        return mmap.mmap(-1, FEED_SECTION_BYTES, map_name), True
    except OSError as e:
        mmf = mmap.mmap(-1, FEED_BLOCKS_BYTES, map_name)
        print("[imageSharing] {} exists at the pre-trailer size ({} B): {}. "
              "Feeds are unaffected; the PLANAR standoff will fall back to Unity's "
              "planarStandoffMetres. Restart the other side to get it back."
              .format(map_name, FEED_BLOCKS_BYTES, e))
        return mmf, False


def write_standoff_trailer(mmf, seq, heartbeat, standoff_m, status,
                           facade_id=-1, look_off_deg=0.0, spread_m=0.0,
                           px_per_m=0.0, tilt_deg=0.0, view_count=0):
    """
    Publish the scene-plane trailer. Returns the sequence counter to pass in next
    time (it advances by 2 per write, staying even between writes).

    Seqlocked in the shape PyUniSharingFast.WriteDynamicState uses: bump to odd,
    write, bump to even. ``standoff_m`` is a naturally-aligned float32 and so cannot
    tear on its own -- the lock is there because the READER must never be told a
    standoff that belongs to one facade alongside another facade's id, and because
    the 16 spare bytes at the end are where a plane normal would go, which is exactly
    the torn-vector case the seqlock pattern exists for.

    No flag handshake and no retry loop: nothing else writes these bytes, so this
    cannot block. That is what lets it be called straight from a control loop.

    ``heartbeat`` must advance on EVERY call, including calls that publish an
    unchanged standoff -- it is the only thing distinguishing a stationary formation
    from a dead producer, and closing this handle does not clear the section.
    """
    base = FEED_TRAILER_OFFSET
    seq = (int(seq) + 1) & 0x7FFFFFFF          # odd: in flux
    struct.pack_into('<i', mmf, base + FEED_TR_SEQ_OFFSET, seq)
    struct.pack_into('<i', mmf, base + FEED_TR_MAGIC_OFFSET, FEED_TRAILER_MAGIC)
    struct.pack_into('<i', mmf, base + FEED_TR_VERSION_OFFSET, FEED_TRAILER_VERSION)
    struct.pack_into('<i', mmf, base + FEED_TR_HEARTBEAT_OFFSET,
                     int(heartbeat) & 0x7FFFFFFF)
    struct.pack_into('<f', mmf, base + FEED_TR_STANDOFF_OFFSET, float(standoff_m))
    struct.pack_into('<i', mmf, base + FEED_TR_STATUS_OFFSET, int(status))
    struct.pack_into('<i', mmf, base + FEED_TR_FACADE_ID_OFFSET, int(facade_id))
    struct.pack_into('<f', mmf, base + FEED_TR_LOOK_OFF_OFFSET, float(look_off_deg))
    struct.pack_into('<f', mmf, base + FEED_TR_SPREAD_OFFSET, float(spread_m))
    struct.pack_into('<f', mmf, base + FEED_TR_PX_PER_M_OFFSET, float(px_per_m))
    struct.pack_into('<f', mmf, base + FEED_TR_TILT_OFFSET, float(tilt_deg))
    struct.pack_into('<i', mmf, base + FEED_TR_VIEW_COUNT_OFFSET, int(view_count))
    seq = (seq + 1) & 0x7FFFFFFF               # even: settled
    struct.pack_into('<i', mmf, base + FEED_TR_SEQ_OFFSET, seq)
    return seq


def read_standoff_trailer(mmf, retries=3):
    """
    Read the trailer back. Returns a dict, or None when no PC has ever written it.

    Used by the self-checks and by any tool that wants to see what Unity is being
    told; the real consumer is ImageSharing.cs, which implements the same read.
    Mirrors that reader's degradation rule: on a torn read the standoff and heartbeat
    are taken anyway and only the diagnostics are marked stale, because a busy writer
    must never cost the mosaic its plane.
    """
    base = FEED_TRAILER_OFFSET
    if len(mmf) < base + FEED_TRAILER_BYTES:
        return None
    if struct.unpack_from('<i', mmf, base + FEED_TR_MAGIC_OFFSET)[0] != FEED_TRAILER_MAGIC:
        return None
    torn = True
    for _ in range(retries):
        s0 = struct.unpack_from('<i', mmf, base + FEED_TR_SEQ_OFFSET)[0]
        if s0 & 1:
            continue
        out = {
            "version": struct.unpack_from('<i', mmf, base + FEED_TR_VERSION_OFFSET)[0],
            "heartbeat": struct.unpack_from('<i', mmf, base + FEED_TR_HEARTBEAT_OFFSET)[0],
            "standoff_m": struct.unpack_from('<f', mmf, base + FEED_TR_STANDOFF_OFFSET)[0],
            "status": struct.unpack_from('<i', mmf, base + FEED_TR_STATUS_OFFSET)[0],
            "facade_id": struct.unpack_from('<i', mmf, base + FEED_TR_FACADE_ID_OFFSET)[0],
            "look_off_deg": struct.unpack_from('<f', mmf, base + FEED_TR_LOOK_OFF_OFFSET)[0],
            "spread_m": struct.unpack_from('<f', mmf, base + FEED_TR_SPREAD_OFFSET)[0],
            "px_per_m": struct.unpack_from('<f', mmf, base + FEED_TR_PX_PER_M_OFFSET)[0],
            "tilt_deg": struct.unpack_from('<f', mmf, base + FEED_TR_TILT_OFFSET)[0],
            "view_count": struct.unpack_from('<i', mmf, base + FEED_TR_VIEW_COUNT_OFFSET)[0],
        }
        if struct.unpack_from('<i', mmf, base + FEED_TR_SEQ_OFFSET)[0] == s0:
            out["torn"] = False
            return out
    return {
        "version": struct.unpack_from('<i', mmf, base + FEED_TR_VERSION_OFFSET)[0],
        "heartbeat": struct.unpack_from('<i', mmf, base + FEED_TR_HEARTBEAT_OFFSET)[0],
        "standoff_m": struct.unpack_from('<f', mmf, base + FEED_TR_STANDOFF_OFFSET)[0],
        "status": struct.unpack_from('<i', mmf, base + FEED_TR_STATUS_OFFSET)[0],
        "torn": torn,
    }