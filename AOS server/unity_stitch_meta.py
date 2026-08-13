"""
What the Unity stitcher is *currently* set to — read from its own metadata mapping.

Why this is read-only
---------------------
There is no clean way to push a setting into Unity from here, and the reason is
structural rather than missing work: the settings live in `PyUniSharingFast`'s
serialized inspector fields, and `WriteMetadata` republishes the whole mapping
every Unity frame. Anything written into `MetadataSharedMemory` from this side is
gone within ~16 ms, and the inspector — the thing that actually holds the value —
never sees it. `PyUniSharingFast` owns that mapping; the rest of the pipeline is
built on that being true (it is the only writer of the wire version, intrinsics
and scene plane, which is exactly why `planar_inputs_ready()` can trust them).

What is possible, and is what this module does, is the other direction: read the
live values and say precisely which inspector fields disagree with the clip being
replayed. A short diff of "this is what it is, this is what it needs to be" beats
a static checklist — a checklist tells you what to type, a diff tells you what you
got wrong.

Reading without creating
------------------------
`mmap.mmap(-1, size, name)` *creates* a named section when none exists, which
would make "Unity is not running" indistinguishable from "Unity is running with
everything zeroed", and — if this side's size were ever smaller than Unity's —
would make Unity's own `CreateFileMapping` fail with ERROR_ACCESS_DENIED against
the section we left behind. So the view is opened with `OpenFileMappingW`, which
returns nothing at all when the producer is absent. That also makes the mapping
read-only at the OS level, so the paragraph above cannot be violated by accident.

Layout is mirrored from `PyUniSharingFast.cs`'s `meta*Offset` constants (which
`StitcherThreading.py` mirrors too, asserted equal by that repo's
`tools/check_wire_layout.py`). A third copy can drift, so `parse` cross-checks the
mapping's self-describing fields — block header size, block image size, wire
version — against what this repo already knows from `image_stream_feed`, and
`plausible()` reports false when they disagree rather than printing numbers read
from the middle of some other field.

Pure module: stdlib + `image_stream_feed`'s wire constants. Windows only (the
whole pipeline is). `python unity_stitch_meta.py` runs a self-check against a
synthetic mapping, plus a live read if a scene happens to be up.
"""

import struct
import sys
import time

# The wire this repo writes. Imported, not restated: if the two ever disagree the
# comparison below is the place it should surface.
from image_stream_feed import BLOCK_HEADER_BYTES

__all__ = ["read", "parse", "plausible", "compare", "describe", "watch_fields",
           "INSPECTOR_NAMES", "MAP_NAME", "METADATA_SIZE"]


MAP_NAME = "MetadataSharedMemory"

# MUST equal PyUniSharingFast.metadataSize exactly. Too small here and a stale
# section of ours would break Unity's create; the OpenFileMapping path above
# means we never create one, and this stays a mirror to be checked, not trusted.
METADATA_SIZE = 412

# Wire resolution of the block section (ImageSharing.cs ImageWidth/Height).
WIRE_W, WIRE_H = 800, 450
PLANAR_WIRE_VERSION = 2

# --- v1 prefix (written sequentially by WriteMetadata) ---
OFF_BLOCK_W = 0            # int32
OFF_BLOCK_H = 4            # int32
OFF_BLOCK_COUNT = 8        # int32
OFF_PANO_W = 12            # int32
OFF_PANO_H = 16            # int32
OFF_STITCHER = 20          # 64-byte UTF-8, zero padded
# --- v2 static tail (absolute offsets) ---
OFF_BLOCK_HEADER_SIZE = 256   # int32
OFF_WIRE_VERSION = 260        # int32
OFF_FX = 264                  # float32 fx, fy, cx, cy
OFF_POSE_SOURCE = 308         # uint8
OFF_BLEND_MODE = 310          # uint8
OFF_PLANE_VALID = 332         # uint8, then uint8 mode
OFF_SWEEP_ENABLED = 344       # uint8 sweep, then uint8 pose-refine
OFF_SWEEP_RANGE = 348         # float32, then int32 steps
OFF_STANDOFF = 364            # float32
OFF_CANVAS_MODE = 380         # uint8
OFF_HEARTBEAT = 384           # uint32, bumped every Unity frame
OFF_SLOT_CAPACITY = 388       # int32, then int32 stride
# Which source the standoff at 364 ACTUALLY came from this frame. Published because
# the value alone cannot say -- it is a float either way -- so without it a tool that
# supplies a standoff over the feed trailer cannot tell whether the scene took it or
# is ignoring it in favour of a hand-typed one (planarStandoffSource = Inspector).
OFF_STANDOFF_SOURCE = 396     # uint8

# Enum mirrors. Values are explicit on the C# side precisely so they can be
# mirrored by number.
PLANE_MODES = {0: "Auto", 1: "Facade", 2: "Nadir", 3: "Manual",
               4: "FormationRelative"}
PLANE_MODE_FORMATION_RELATIVE = 4
BLEND_MODES = {0: "Feather", 1: "Nearest"}
BLEND_NEAREST = 1
POSE_SOURCES = {0: "GroundTruth", 1: "NoisyState", 2: "GroundTruthPlusGnss"}
CANVAS_MODES = {0: "Fixed", 1: "AutoFit"}
# The RESOLVED source, not the requested one: PyUniSharingFast.planarStandoffSource is
# Auto/Inspector, but Auto still reads "inspector" whenever no live value is arriving.
STANDOFF_SOURCES = {0: "inspector", 1: "PC"}
STANDOFF_SOURCE_PC = 1

# How closely a live value has to match before it is left alone. The standoff one
# is deliberately loose enough that hand-stepping it under --loop does not get
# flagged the moment the operator nudges it.
FX_TOL_PX = 2.0
STANDOFF_TOL_M = 0.05

# Fields worth reporting a mid-replay change of, and how to render them.
watch_fields = ("stitcher", "plane_mode", "standoff", "blend_mode",
                "sweep_enabled", "sweep_range", "standoff_source")

# Parsed key -> the name of the field as it appears in the Unity inspector. Every
# message aimed at the operator uses these: the parsed names are this module's
# business, the inspector's are what has to be found and edited.
INSPECTOR_NAMES = {
    "stitcher": "typeOfStitcher",
    "plane_mode": "scenePlaneMode",
    "standoff": "planarStandoffMetres",
    "standoff_source": "standoff in force",
    "blend_mode": "planarBlendMode",
    "sweep_enabled": "planarPlaneSweep",
    "sweep_range": "planarSweepRange",
    "fx": "manualVerticalFovDeg",
}


def _open_view():
    """`(bytes, handle_pair)` of the live mapping, or `(None, None)` if absent.

    Opens an existing section read-only. Absence is the normal case (no scene
    playing), not an error.
    """
    try:
        import ctypes
        from ctypes import wintypes
    except ImportError:                                   # not Windows
        return None, None
    try:
        k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    except (OSError, AttributeError):
        return None, None
    FILE_MAP_READ = 0x0004
    k32.OpenFileMappingW.argtypes = (wintypes.DWORD, wintypes.BOOL,
                                     wintypes.LPCWSTR)
    k32.OpenFileMappingW.restype = wintypes.HANDLE
    k32.MapViewOfFile.argtypes = (wintypes.HANDLE, wintypes.DWORD,
                                  wintypes.DWORD, wintypes.DWORD,
                                  ctypes.c_size_t)
    k32.MapViewOfFile.restype = wintypes.LPVOID
    k32.UnmapViewOfFile.argtypes = (wintypes.LPCVOID,)
    k32.CloseHandle.argtypes = (wintypes.HANDLE,)

    h = k32.OpenFileMappingW(FILE_MAP_READ, False, MAP_NAME)
    if not h:
        return None, None
    view = k32.MapViewOfFile(h, FILE_MAP_READ, 0, 0, METADATA_SIZE)
    if not view:
        k32.CloseHandle(h)
        return None, None
    try:
        import ctypes as _c
        raw = _c.string_at(view, METADATA_SIZE)
    finally:
        k32.UnmapViewOfFile(view)
        k32.CloseHandle(h)
    return raw, True


def parse(raw):
    """The mapping's bytes -> a settings dict. No interpretation, no verdicts."""
    u = lambda fmt, off: struct.unpack_from(fmt, raw, off)
    name = raw[OFF_STITCHER:OFF_STITCHER + 64]
    try:
        stitcher = name.decode('utf-8').rstrip('\x00').strip()
    except UnicodeDecodeError:
        stitcher = ""
    fx, fy, cx, cy = u('<ffff', OFF_FX)
    plane_valid, plane_mode = u('<BB', OFF_PLANE_VALID)
    sweep_enabled, refine_enabled = u('<BB', OFF_SWEEP_ENABLED)
    return {
        "stitcher": stitcher,
        "block_w": u('<i', OFF_BLOCK_W)[0],
        "block_h": u('<i', OFF_BLOCK_H)[0],
        "block_count": u('<i', OFF_BLOCK_COUNT)[0],
        "block_header_size": u('<i', OFF_BLOCK_HEADER_SIZE)[0],
        "wire_version": u('<i', OFF_WIRE_VERSION)[0],
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "pose_source": u('<B', OFF_POSE_SOURCE)[0],
        "blend_mode": u('<B', OFF_BLEND_MODE)[0],
        "plane_valid": bool(plane_valid),
        "plane_mode": plane_mode,
        "sweep_enabled": bool(sweep_enabled),
        "refine_enabled": bool(refine_enabled),
        "sweep_range": u('<f', OFF_SWEEP_RANGE)[0],
        "sweep_steps": u('<i', OFF_SWEEP_RANGE + 4)[0],
        "standoff": u('<f', OFF_STANDOFF)[0],
        "standoff_source": u('<B', OFF_STANDOFF_SOURCE)[0],
        "canvas_mode": u('<B', OFF_CANVAS_MODE)[0],
        "heartbeat": u('<I', OFF_HEARTBEAT)[0],
        "slot_capacity": u('<i', OFF_SLOT_CAPACITY)[0],
        "slot_stride": u('<i', OFF_SLOT_CAPACITY + 4)[0],
    }


def plausible(meta):
    """`(ok, why)` — do the self-describing fields match what this repo knows?

    Guards against reading a stale layout: if the block geometry and header size
    at these offsets are not the ones this repo's publisher writes, the offsets
    are wrong (the two repos were built from different revisions) and every other
    number in the dict is a float taken from the middle of some other field.
    """
    if meta is None:
        return False, "no mapping"
    if meta["wire_version"] == 0 and meta["heartbeat"] == 0 and not meta["stitcher"]:
        return False, "producer has not written yet"
    if (meta["block_w"], meta["block_h"]) != (WIRE_W, WIRE_H):
        return False, ("block image size {}x{} is not this repo's {}x{} — the "
                       "two sides were built from different revisions"
                       .format(meta["block_w"], meta["block_h"], WIRE_W, WIRE_H))
    if meta["block_header_size"] not in (12, BLOCK_HEADER_BYTES):
        return False, ("block header size {} is neither v1's 12 nor this repo's "
                       "{}".format(meta["block_header_size"], BLOCK_HEADER_BYTES))
    return True, ""


def read(liveness_wait_s=0.35):
    """Read the live settings.

    Returns `None` when no producer has the mapping open — the normal "Unity is
    not playing" answer, and unambiguous because this never creates the section.
    Otherwise a settings dict plus:

    * `live` — the heartbeat advanced during `liveness_wait_s`. A section whose
      handle is still held by a paused or crashed editor keeps its last bytes
      readable forever, so every downstream gate would go on passing on stale
      settings; only movement proves a producer.
    * `plausible` / `why` — from `plausible()`.
    """
    raw, _ = _open_view()
    if raw is None:
        return None
    meta = parse(raw)
    if liveness_wait_s > 0:
        time.sleep(liveness_wait_s)
        raw2, _ = _open_view()
        beat2 = parse(raw2)["heartbeat"] if raw2 is not None else meta["heartbeat"]
        meta["live"] = (beat2 != meta["heartbeat"])
    else:
        meta["live"] = None
    ok, why = plausible(meta)
    meta["plausible"] = ok
    meta["why"] = why
    return meta


def compare(meta, want):
    """`(mismatches, info)` for a clip's requirements against the live settings.

    `want` keys: `stitcher`, `fx`, `plane_mode`, `standoff`, `blend_mode`,
    `wire_version`. Each mismatch is `(inspector_field, current, needed)` using
    the names as they appear in the Unity inspector, because that is what the
    operator has to go and find.
    """
    bad, info = [], []

    if "stitcher" in want and meta["stitcher"] != want["stitcher"]:
        bad.append(("typeOfStitcher", meta["stitcher"] or "(unset)",
                    want["stitcher"]))
    if "wire_version" in want and meta["wire_version"] != want["wire_version"]:
        bad.append(("(producer wire version — rebuild/update the scene, not an "
                    "inspector field)", str(meta["wire_version"]),
                    str(want["wire_version"])))
    if "fx" in want and abs(meta["fx"] - want["fx"]) > FX_TOL_PX:
        # fx is published, the vfov is what gets typed: quote both.
        bad.append(("useManualIntrinsics + manualVerticalFovDeg",
                    "fx {:.1f} px".format(meta["fx"]),
                    "fx {:.1f} px (vfov {:.1f} deg at {}x{})".format(
                        want["fx"], want.get("vfov_deg") or 0.0, WIRE_W, WIRE_H)))
    if "plane_mode" in want and meta["plane_mode"] != want["plane_mode"]:
        bad.append(("scenePlaneMode",
                    PLANE_MODES.get(meta["plane_mode"], str(meta["plane_mode"])),
                    PLANE_MODES.get(want["plane_mode"], str(want["plane_mode"]))))
    if want.get("standoff") and abs(meta["standoff"] - want["standoff"]) > STANDOFF_TOL_M:
        bad.append(("planarStandoffMetres", "{:.2f}".format(meta["standoff"]),
                    "{:.2f}".format(want["standoff"])))
    if "blend_mode" in want and meta["blend_mode"] != want["blend_mode"]:
        bad.append(("planarBlendMode",
                    BLEND_MODES.get(meta["blend_mode"], str(meta["blend_mode"])),
                    BLEND_MODES.get(want["blend_mode"], str(want["blend_mode"]))))

    # Notes label themselves, so `describe` can print them outside the "change
    # these" block without them reading as more fields to change.
    if meta["sweep_enabled"] and meta["sweep_range"] > 0.0:
        info.append("note: planarPlaneSweep is ON (range {:.2f} m, {} steps): it "
                    "moves the plane off the standoff you set, so turn it off "
                    "while stepping the standoff by hand."
                    .format(meta["sweep_range"], meta["sweep_steps"]))
    if meta["block_count"] and meta["block_count"] < want.get("drones", 0):
        info.append("note: blockImageCount is {} but the clip has {} drones — "
                    "the count is only a scan hint now, but check the scene "
                    "expects the whole fleet.".format(meta["block_count"],
                                                      want["drones"]))
    info.append("live: stitcher {}, plane {} (valid={}), standoff {:.2f} m (from the {}), "
                "blend {}, canvas {}, poseSource {}, wire v{} hdr {} B"
                .format(meta["stitcher"] or "(unset)",
                        PLANE_MODES.get(meta["plane_mode"], meta["plane_mode"]),
                        meta["plane_valid"], meta["standoff"],
                        STANDOFF_SOURCES.get(meta.get("standoff_source"), "?"),
                        BLEND_MODES.get(meta["blend_mode"], meta["blend_mode"]),
                        CANVAS_MODES.get(meta["canvas_mode"], meta["canvas_mode"]),
                        POSE_SOURCES.get(meta["pose_source"], meta["pose_source"]),
                        meta["wire_version"], meta["block_header_size"]))
    return bad, info


def describe(meta, want, indent="  ", compact=False):
    """Printable lines for the live check.

    `compact` drops the live-state summary while everything matches — there is
    nothing to act on then — but keeps it as soon as a field disagrees, which is
    when knowing the whole published state is what saves a second look.
    """
    if meta is None:
        if compact:
            return [indent + "scene: not publishing {} (press Play, then "
                             "--check-unity)".format(MAP_NAME)]
        return [indent + "Unity is not publishing {} — nothing to check against. "
                         "Press Play, then re-run with --check-unity."
                         .format(MAP_NAME)]
    if not meta.get("plausible"):
        return [indent + "{} is open but unreadable: {}.".format(
            MAP_NAME, meta.get("why") or "?")]
    lines = []
    if meta.get("live") is False:
        lines.append(indent + "! {} exists but its heartbeat is not advancing — "
                              "the editor is paused or was stopped, so these are "
                              "last frame's settings.".format(MAP_NAME))
    bad, info = compare(meta, want)
    if bad:
        lines.append(indent + "CHANGE THESE IN THE INSPECTOR "
                              "(PyUniSharingFast):")
        width = max(len(f) for f, _c, _n in bad)
        for field, cur, needed in bad:
            lines.append("{}  {:<{w}}  {}  ->  {}".format(
                indent, field, cur, needed, w=width))
    else:
        lines.append(indent + ("scene: matches this clip" if compact else
                               "Live scene matches this clip — nothing to change."))
    for note in info:
        if compact and note.startswith("live:") and not bad:
            continue
        lines.append(indent + note)
    return lines


def _selftest():
    """Parse and compare against a synthetic mapping — no Unity needed."""
    ok = [True]

    def check(name, cond, detail=""):
        print("  {:<46} {}{}".format(name, "PASS" if cond else "FAIL",
                                     "" if cond else "  <- " + str(detail)))
        if not cond:
            ok[0] = False

    buf = bytearray(METADATA_SIZE)
    struct.pack_into('<iiiii', buf, 0, WIRE_W, WIRE_H, 3, 1920, 1080)
    buf[OFF_STITCHER:OFF_STITCHER + 7] = b"PLANAR\x00"
    struct.pack_into('<ii', buf, OFF_BLOCK_HEADER_SIZE, BLOCK_HEADER_BYTES, 2)
    struct.pack_into('<ffff', buf, OFF_FX, 525.0, 525.0, 400.0, 225.0)
    struct.pack_into('<BB', buf, OFF_PLANE_VALID, 1,
                     PLANE_MODE_FORMATION_RELATIVE)
    struct.pack_into('<B', buf, OFF_BLEND_MODE, BLEND_NEAREST)
    struct.pack_into('<f', buf, OFF_STANDOFF, 33.88)
    struct.pack_into('<B', buf, OFF_STANDOFF_SOURCE, STANDOFF_SOURCE_PC)
    struct.pack_into('<I', buf, OFF_HEARTBEAT, 1234)

    m = parse(bytes(buf))
    check("stitcher name parses", m["stitcher"] == "PLANAR", m["stitcher"])
    check("standoff parses", abs(m["standoff"] - 33.88) < 1e-4, m["standoff"])
    check("standoff source parses", m["standoff_source"] == STANDOFF_SOURCE_PC,
          m["standoff_source"])
    # The source byte lives at 396, inside what used to be metadataReservedGap. If it
    # ever collides with the two section-size fields the mapping ends with, the sizes
    # are what get corrupted -- and those are what Python refuses to run on.
    check("the source byte sits clear of the trailing size fields",
          OFF_STANDOFF_SOURCE + 1 <= 404 and METADATA_SIZE == 412,
          (OFF_STANDOFF_SOURCE, METADATA_SIZE))
    check("plane mode parses",
          m["plane_mode"] == PLANE_MODE_FORMATION_RELATIVE, m["plane_mode"])
    good, why = plausible(m)
    check("a matching layout is plausible", good, why)

    want = {"stitcher": "PLANAR", "fx": 525.0, "vfov_deg": 46.4,
            "plane_mode": PLANE_MODE_FORMATION_RELATIVE, "standoff": 33.88,
            "blend_mode": BLEND_NEAREST, "wire_version": 2, "drones": 3}
    bad, _info = compare(m, want)
    check("a correctly configured scene reports nothing", not bad, bad)

    # Every mismatch the operator can cause, one at a time.
    m2 = dict(m, stitcher="STABSTITCH")
    check("wrong stitcher is caught",
          any(f == "typeOfStitcher" for f, _c, _n in compare(m2, want)[0]))
    m3 = dict(m, standoff=30.0)
    check("wrong standoff is caught",
          any(f == "planarStandoffMetres" for f, _c, _n in compare(m3, want)[0]))
    m4 = dict(m, plane_mode=2)
    check("wrong plane mode is caught",
          any(f == "scenePlaneMode" for f, _c, _n in compare(m4, want)[0]))
    m5 = dict(m, blend_mode=0)
    check("Feather blend is caught",
          any(f == "planarBlendMode" for f, _c, _n in compare(m5, want)[0]))
    m6 = dict(m, fx=337.0)
    check("sim-derived intrinsics are caught",
          any(f.startswith("useManualIntrinsics") for f, _c, _n
              in compare(m6, want)[0]))
    check("a standoff nudge inside tolerance is not caught",
          not compare(dict(m, standoff=33.90), want)[0])
    check("the sweep is reported as info",
          any("planarPlaneSweep" in n for n in
              compare(dict(m, sweep_enabled=True, sweep_range=4.0), want)[1]))

    # A layout mismatch must be refused rather than reported as settings.
    bad_layout = bytearray(buf)
    struct.pack_into('<i', bad_layout, OFF_BLOCK_W, 1920)
    good2, why2 = plausible(parse(bytes(bad_layout)))
    check("a stale layout is refused", not good2, why2)
    check("an all-zero mapping is refused",
          not plausible(parse(bytes(METADATA_SIZE)))[0])

    print("\n{}".format("SELF-CHECK PASSED" if ok[0] else "SELF-CHECK FAILED"))
    return 0 if ok[0] else 1


if __name__ == "__main__":
    print("unity_stitch_meta self-check\n")
    rc = _selftest()
    print("\nLive read of {}:".format(MAP_NAME))
    live = read()
    if live is None:
        print("  not open — no Unity scene is publishing it.")
    else:
        for line in describe(live, {"stitcher": "PLANAR"}, indent="  "):
            print(line)
    sys.exit(rc)
