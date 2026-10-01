# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A DJI drone-swarm control system. A PC operator drives one or more DJI Mini 3 Pro
drones with a joystick; commands flow to the drones and live video + telemetry flow
back. There is **no waypoint navigation in the active path** — control is direct
virtual-stick (joystick) commands. This is a rewrite of an older waypoint-based
"AOS" (Aerial Observation System) app into a leaner app called **LIS_Swarm**.

The repo has three independently-built components (a fourth, `rc-joystick/`, is in progress —
see below) plus the end-to-end data flow that ties them together. Understanding the flow is the key to working here:

```
joystick → Python script ──ds_wrapper.sendWayPointData()──► [shared memory]
                                                                   │
                                              DroneSwarmServer.exe ─┘ (reads shmem, posts to)
                                                                   │ MQTT publish
                                                                   ▼
                                       Android app (SwarmActivity) embedded MQTT broker :1883
                                                                   │
                                                      DJI VirtualStickManager → drone
   ◄───────────────────────────────────────────────────────────────────
   video (RTSP :8554) + telemetry ──► DroneSwarmServer.exe ──► [shared memory]
                                                                   │
   Python ◄──ds_wrapper.getImageAndTelemetryData()────────────────┘
```

**Command-path exception:** `swarm_flocking.py` normally skips the whole top row for
commands and publishes them **directly** to each RC's MQTT broker over a persistent
paho-mqtt connection (`mqtt_command_sender.py`). Three modes via `DroneIPs`
(flocking.config.psd1) / `--drone-ips`: an **explicit IP list** pins drone ids to switch
ports (stable numbering, verified/remapped by the identity probe); **empty/`auto`**
(default) auto-discovers the RC IPs from the running server's per-slot RTSP control
connections (`discover_rc_ips`, `Get-NetTCPConnection` to :8554) and binds them to slots
with the marker probe (`auto_bind_command_channels`) — drone id = server slot, one
identity source, mismatch impossible; **`server`** forces the legacy path through
DroneSwarmServer, which reconnects per command (~220 ms each, ~4.5 Hz max — worse with
more drones). Auto mode falls back to the server path with a loud warning when discovery
or binding fails. Video + telemetry always flow through `DroneSwarmServer.exe`: telemetry
is **not** sent over MQTT — it rides inside each drone's RTSP session as a non-video data
stream that the server's decode thread splits out. So the server is **required even when
commands bypass it**; without it the controllers have no GPS/heading/altitude, not just
no video.

## Components

### 1. `AOS server/` — PC side
- **`DroneSwarm_Wrapper/`** — C++/pybind11 module `ds_wrapper`. The Python⇄C++ bridge.
  It is **only** a shared-memory poke: it writes command bytes into a named Windows
  file-mapping (`dllmemfilemap`), `PostMessage`s the `DroneSwarmServer` window, and
  busy-waits on a status byte. See `DroneSwarm_Wrapper.cpp` for the exact byte layout
  (per-drone slots of `SHMEMSLOTSIZE`; drone N uses offset `(N-1)*SHMEMSLOTSIZE`).
- **`DroneSwarmServer/`** — MFC C++ Windows app (`DroneSwarmServer.exe`). Owns the RTSP
  video ingest **and the telemetry embedded in it** (per-drone decode thread in
  `Dialog1Dlg.cpp`: non-video RTSP packets ARE the telemetry), the per-drone session
  setup (IP entry / Connect), and the shared-memory protocol the wrapper talks to; its
  MQTT client to the drones is now only the fallback command path. The wrapper's
  `PostMessage` calls target this app's `WM_PYWRAPPER_*` message handlers.
- **Python control scripts** (run against the built `ds_wrapper.*.pyd`):
  - `joystick_controller.py` — primary single-drone joystick driver (UDP joystick or `--cli`).
  - `swarm_flocking.py` — multi-drone Olfati-Saber flocking from one joystick. Runs a
    **read-only pre-flight link scan** at startup (`LINKDIAG` per RC, `--no-link-scan` /
    `LinkScan` config key to skip): each aircraft reports its live radio config plus its own
    per-frequency interference sweep, printed to the console and stored in the flight log's
    `session.json`. It changes nothing — it is the band scan DJI's per-link AUTO channel
    selection is silently working against. Also
    hosts `RotationProbe` — the GUI's **Rotation check** button: while swarming is HELD,
    each drone in turn is VS-armed and flies a 0.6 m/s open-loop pulse north then east
    at its current altitude; the GPS displacement gives a per-drone rotation/gain
    verdict (OK / SKEWED / ROTATED / DEAD / VS FAIL, in `meta["rotation_check"]` and the
    flight log). Run it before Start — any verdict other than OK means that aircraft
    executes velocity commands in the wrong direction (2026-07-05 flight: several
    aircraft flew commands rotated 90–180°, suspected FC compass/yaw error, so the
    cohesion loop closed with flipped sign → one drone ejected + pair collapse to the
    min-sep failsafe). Deliberately NOT `--slow`-scaled (needs ~2 m displacement to
    clear GPS noise); refuses to run if any pair is closer than min-separation + ~5 m.
  - `olfati_saber.py` — the flocking math, extracted from `swarm_flocking.py`: the
    `OlfatiSaber` cohesion/velocity-consensus class, plus `ObstacleAvoidance` — a port of
    the Unity sim's `GetObstacleForce` β-agent term for 2D **virtual obstacles** (axis-
    aligned rectangles *and* real building-footprint polygons, `poly_force`) and one
    **geofence polygon** whose edges repel inward. Shapes are
    **drawn on the GUI map** ("Add obstacle" click-drag / "Pick building" click /
    "Add geofence" vertex clicks →
    `/command` POST → UDP :5098 → `command_listener` → `meta["obstacles"]`/`meta["geofence"]`/
    `meta["facades"]`, persisted to `AOS server/shapes.json`, gitignored). Shape validation +
    shapes.json persistence live in this module and are shared with `swarm_gui.py`, whose own
    `ShapesStore` also applies/saves every edit — so drawing works and shapes stay visible
    with NO controller running (the GUI serves them as `data.shapes`; a live controller's
    meta echo remains authoritative). See the
    [shapes.json compatibility gotcha](#critical-gotchas) before touching the file format.
    `d_obs`/`r0_obs` params are PHYSICAL
    metres (divided by `scale` internally — Unity's raw d_obs=5.0 scaled would be 50 m
    physical). Geofence is also a hard cutoff: a drone outside it is braked, gets a
    per-drone DISABLE_VS, and leaves the flock (not a neighbour, excluded from hull
    heading, still covered by min-separation) until swarming is Stop→Started, which clears
    the breach list (`meta["removed"]` shows it in the GUI). Pure Python, no `ds_wrapper`
    import, testable standalone.
  - `heading_convexhull.py` — GLOBAL_CONVEXHULL heading control (port of the Unity VR sim's
    `AttitudeAlgorithm.cs`), used by `swarm_flocking.py`: drones on the swarm's convex hull
    face outward along their vertex bisector (point-inwards flips it), interior drones hold
    heading, stick yaw is ignored. Emits target headings in compass deg; `swarm_flocking.py`
    converts them to yaw rates via `heading_hold_rate`. Mode + point-inwards are **runtime
    settings owned by the GUI** (Heading selector in `swarm_gui.py`'s controls bar →
    `/command` POST → UDP :5098 → `command_listener` → `meta["heading_mode"]`); the
    `--heading`/`--point-inwards` CLI flags only seed them. Pure Python, no `ds_wrapper` import.
  - `swarm_plane.py` — VERTICAL-PLANE ("wall") swarming, port of the Unity sim's
    `SwarmPlaneController.cs`: a GUI toggle (`--plane-mode` / `PlaneMode` seeds it) that
    swaps the plane the cohesion law is constrained to, from horizontal to a **vertical
    wall** facing the stick-steered `target_yaw`, so the operator faces a billboard of
    drones. Binary, like the sim — there is no tilt angle. It calls
    `OlfatiSaber.GetSwarmAcceleration` **unmodified**, on coordinates projected into the
    wall's axes (`n`/`e1` horizontal, `e2` = up); the potential is isotropic, so the basis
    is all that changes. Its vertical component becomes a **per-drone altitude setpoint**
    (`alt += v_up·dt`, leashed to a reference altitude the climb stick moves) — the only
    per-drone altitude anywhere in this repo, because DJI VS gives absolute-altitude
    control and no vertical-velocity channel. Deviations from the sim, all deliberate:
    azimuth comes from the PC-owned `target_yaw` not a drone's compass; the plane is
    centroid-pinned (the sim's anchor drone is pilot-flown, which has no analogue here);
    the restoring gain is in physical m/s-per-m and clamped so obstacle/geofence
    repulsion always outranks it; **entry seeds an alternating vertical stagger** because
    a wall seeded flat is a horizontal line in its own axes — the saddle this fleet
    already gets stuck in; and exit **ramps** the per-drone setpoints together at a
    bounded rate instead of stepping back to the shared scalar. Pure Python, no
    `ds_wrapper` import; `python swarm_plane.py` runs a self-check that flies a
    kinematic 3-drone swarm into a wall. See the [vertical-plane gotchas](#critical-gotchas).
  - `heading_demostitch.py` — DEMOSTITCH heading control, the third Heading-selector mode
    (between manual and convexhull), used by `swarm_flocking.py`: the laterally-middle drone
    (positions projected perpendicular to the stick-steered global yaw) points exactly at the
    global yaw and each neighbour fans out by `meta["stitch_offset"]`° per rank
    (`--stitch-offset`/GUI Offset input, default 30), so adjacent camera views keep partial
    overlap for image stitching. Stick yaw integrates the shared `target_yaw` exactly like
    manual (world-frame translation too); centre re-election is continuous as the yaw rotates,
    guarded by a 1.5 m lateral rank-hysteresis margin plus a 0.5 s low-pass on the offset
    component only (the full target would lag the stick). `meta["stitch_centre"]` feeds the
    GUI's CENTRE pill (odd drone counts only). Pure Python, no `ds_wrapper` import;
    `python heading_demostitch.py` runs a self-check.
  - `response_monitor.py` — pure module: `ResponseMonitor`, a sliding-window
    least-squares fit of the rotation+gain between the velocity commands actually sent
    to each drone (post `--slow`) and its GPS-derived velocity (healthy ≈ 0° / gain 1).
    Fed by the flocking loop while swarming, logged to `swarm_debug.csv`
    (`resp_rot_deg`/`resp_gain`) and shown live in the GUI drone cards
    (`meta["resp"]`), so a drone executing commands in the wrong frame is visible
    in-flight instead of only in offline log analysis. No `ds_wrapper` import.
  - `udp_joystick_receiver.py` — receives joystick JSON over UDP :5055 (used by the above).
    Also owns `JoystickWatchdog`, the pure policy for what an ARMED swarm flies when that
    feed goes stale (see the [arm-gate gotcha](#critical-gotchas));
    `python udp_joystick_receiver.py --selftest` checks it, bare it's a live listener.
  - `joyreporter.py` — pygame joystick debug readout.
  - `building_footprint.py` — the **"Pick building"** lookup: one map click → a real
    building outline, from **OpenStreetMap via Overpass** (`lookup()`, plus a
    `FootprintCache` keyed by GEOMETRY — a click inside a ring already on file hits, so a
    site surveyed at the office answers in the field with no connectivity). Proxied through
    `swarm_gui.py`'s `/footprint` route rather than fetched by the browser, for that cache.
    OSM was chosen after the Swiss federal alternatives were checked and **do not work**:
    `ch.swisstopo.vec25-gebaeude` (api3 identify) answers at EPFL with ONE generalized
    117 425 m² MultiPolygon — the whole block merged at 1:25 000 — and
    `ch.kantone.cadastralwebmap-farbe` is identify-queryable and ground-true but returns
    the cadastral **parcel** (`egris_egrid`), not a building; swissTLM3D / swissBUILDINGS3D
    have the accuracy but are download-only (STAC). So swisstopo enters this feature as a
    **basemap** (the cadastral layer is the ground-true visual cross-check) and OSM supplies
    the geometry. **OSM is crowd-sourced, not surveyed** — `clip_scene_plane.analyse()`
    warns when a facade's source is `osm:` and prints the clip's own `tolerance_m`, which is
    what actually decides whether an outline was good enough. Pure stdlib (`urllib`), no
    `ds_wrapper`; `python building_footprint.py` self-checks offline,
    `python building_footprint.py LAT LON` does a live probe.
  - `swarm_gui.py` — browser GUI server: a satellite map (default EPFL Lausanne) showing
    each drone's position + heading, a complete graph of inter-drone distance lines
    (**3D** metres labelled, split into horizontal/vertical when the pair is stacked), and
    a per-drone status panel (including an **RF link** row fed by
    `meta["link"]` — see the [link diagnostics protocol](#two-protocols-you-will-touch-constantly),
    plus **Alt cmd** / **Off plane** rows while a vertical-plane wall is up). The controls
    bar owns the live runtime settings (heading mode, stitch offset, gimbal, obstacles/
    geofence, and the **Vertical plane** toggle + gain — `meta["plane_mode"]`/
    `meta["plane_gain"]`, with the controller free to refuse the toggle and echo it back off)
    plus the **Record clip** button and its REC chip (`meta["recording"]`, see
    `clip_recorder.py`; the button hides itself if the controller doesn't publish that key).
    A **basemap switcher** (the layer control, top right) offers Esri World Imagery
    (default, worldwide) plus **SWISSIMAGE** and the **swisstopo cadastral webmap** — the
    latter renders the official cadastral survey, so its outlines are TRUE GROUND outlines
    with none of the roof lean that makes a satellite roofline worth 5-7 m (see the
    [satellite-parallax gotcha](#critical-gotchas)); both swisstopo layers are
    Switzerland/Liechtenstein only, hence Esri staying the default. The choice persists in
    `localStorage`. **"Pick building"** clicks a building → `/footprint`
    (`building_footprint.py`) → its outline previews → **Add obstacle** stores it as a
    polygon obstacle, or clicking one **wall** of that outline stores the wall as a
    **facade** (amber on the map; the popup carries a ready-to-paste
    `--set-plane-from-facade <id>`). That is the accurate route to a PLANAR scene plane —
    see `clip_scene_plane.py`. **Does NOT import `ds_wrapper`** — it
    only LISTENS on UDP :5099 for telemetry pushed by a running controller (so it runs
    unprivileged, in its own terminal, on any Python ≥3.7; it does import the pure
    `olfati_saber` module for the shared shapes helpers, and owns a `ShapesStore` that
    loads/saves `shapes.json` so obstacle/geofence drawing works without a controller).
    Frontend assets in `gui/`
    (Leaflet + Esri World Imagery tiles, needs internet, no API key).
  - `swarm_telemetry_feed.py` — `TelemetryFeedPublisher` embedded by `joystick_controller.py`
    and `swarm_flocking.py`; UDP-pushes a JSON telemetry snapshot to `swarm_gui.py` at 5 Hz
    (on by default; `--no-gui` to disable, `--gui-host/--gui-port` to redirect). Best-effort
    and fire-and-forget so it can never stall the control loop.
  - `image_stream_feed.py` — `ImageStreamPublisher`, embedded by `swarm_flocking.py`
    (`--image-stream`, seeded by the `ImageStream` config key): publishes each drone's
    live frame (800×450 BGR + heading) into the `DroneFeedSharedMemory` mapping read by
    the Unity DJI scene (`ImageSharing.cs`, which feeds the stitcher). **No `ds_wrapper` import
    and zero extra wrapper calls** — it consumes the image bytes the telemetry threads
    already fetch (via `DroneController.add_frame_sink`), copies them into a latest-wins
    mailbox, and does the convert/resize/handshake on per-drone worker threads so it
    can never slow the cmd/telem rates. (In real-drone mode the Unity component's
    `enableImageWriting` must be off — its writer would fight this one.)
    The block header it writes is the **48-byte v2** `flag|droneId|heading|camPos[3]|
    camRot[4] xyzw|captureTime|poseStatus` (layout owned by `utils/imageSharingUtil.py`).
    It also owns the map's **scene-plane trailer** (`set_standoff`/`clear_standoff`): the
    PLANAR standoff the control loop computes, written at 10 Hz on its own mmap — never a
    per-drone one, whose `seek()->write()` is not GIL-atomic — and never from a thread,
    so a wedged control loop stops the heartbeat and Unity falls back, which is the whole
    point of having one. See the [scene-plane trailer gotcha](#critical-gotchas).
    The pose fields are what the sim's pose-driven **`PLANAR`** stitcher needs and a
    compass heading cannot give (one scalar is no position and one of three rotation
    DoF); `dji_camera_pose.CameraPoseSolver` supplies them, gated behind
    `--image-stream-pose`/`ImageStreamPose`. **Without that flag the blocks go out with
    `poseStatus 0`** and `PLANAR` drops every view — `StitcherThreading.planar_inputs_ready()`
    prints one `[PLANAR] unavailable: …` line and falls back to the individual feeds, so
    the symptom is a blank panorama, not a crash. `STABSTITCH` ignores pose either way.
  - `clip_recorder.py` — `ClipRecorder`, embedded by `swarm_flocking.py`: the GUI's
    **Record clip** button saves a short section of a flight WITH pictures — one
    1080p MP4 per drone plus a per-frame index CSV (`t_epoch`, raw telemetry, and the
    **solved Unity-world camera pose** `pos_*`/`quat_*` xyzw/`pose_status` from
    `dji_camera_pose.CameraPoseSolver`), and the flight data for exactly that window, in
    `recordings/clip_YYYYMMDD_HHMMSS/` (`--recording-dir`/`RecordingDir`, gitignored,
    **separate from `flight_logs/`**, which is untouched and keeps running). Auto-stops
    at `--record-max-s`/`RecordMaxSeconds` (default 120 s) so a forgotten recording can't
    fill the disk. Like the image stream it adds **zero `ds_wrapper` calls** — it registers
    another `DroneController.add_frame_sink()` consumer (frame sinks are now a **list**, so
    `image_stream_feed.py` no longer owns the hook exclusively and both can run at once),
    does one ~3 MB copy on the telemetry thread onto a 4-deep bounded queue, and encodes on
    per-drone worker threads. The windowed flight data comes from the `FlightLogger` mirror
    tee (`logger.mirror`, see `flight_logger.py`): a second logger rooted in the clip folder
    receives every row **already stamped by the primary**, so clip rows are byte-identical
    to their `flight_logs/` counterparts and join across the two. **`drone*_frames.csv`
    `t_epoch` is the authoritative timebase, not the MP4** — see the
    [video-rate gotcha](#critical-gotchas).
    **Clips are the offline feed for the sim's `PLANAR` stitcher**, which is pose-driven
    and cannot use pixels alone. The pose is solved on the telemetry thread from the same
    `ds_wrapper` fetch that produced the frame (tightest available pairing — pose/video
    skew is the dominant mosaic error term), using the **same fleet-wide
    `CameraPoseSolver` instance as the live image stream**, so a clip and the feed share
    one latched GPS origin. That origin is recorded in `session.json`
    (`pose_origin_latlon`) because it exists nowhere else once the controller exits, along
    with nominal camera intrinsics for the clip's own 1920×1080 (`camera.fx` ≈ 1260 px at
    the Mini 3 Pro's 46.4° vfov — a working default, **not** a calibration; the pinhole
    model has no distortion term, so **DJI dewarping must be ON at capture**). Frames
    taken without a GPS fix get `pose_status` 0 and are counted + warned about at stop.
    A `meta_source` callable (the same pattern `TelemetryFeedPublisher` uses) gives it the
    controller's LIVE meta, so the **facades the operator had drawn** land in `session.json`
    — the constructor's `meta=` dict is a snapshot and structurally cannot see a wall picked
    mid-flight — and at stop it derives and stores `meta["scene_plane"]` through the same
    `pick_facade` + `analyse` path `clip_replay --set-plane-from-facade` uses, warnings and
    all. Best-effort throughout: no facade, no fix or an unusable trace just means a clip
    without a stored plane, exactly as before. Its **no-flight-log** `session.json` now
    nests under `'meta'` like `FlightLogger`'s does; it used to write flat, which every
    reader (`_session_meta` does `doc.get('meta')`) got wrong the same silent way.
    `session.json` also records `meta["pose_yaw_source"]` — which telemetry field the
    stored `quat_*` took its bearing from — because a clip predating the
    [gimbal-yaw fix](#critical-gotchas) is numerically indistinguishable from a corrected
    one when that aircraft's slip happened to be small.
  - `dji_camera_pose.py` — telemetry → Unity-world camera pose (`CameraPoseSolver`,
    `quat_from_gimbal`, `camera_bearing`), the pose behind `PLANAR` on both the live
    feed and recorded clips. One instance per **fleet**, not per drone: a shared latched
    GPS origin is the entire point. The camera bearing comes from telemetry `heading`,
    **not** `gimbal_yaw` — see the [gimbal-yaw gotcha](#critical-gotchas);
    `yaw_source='gimbal'` restores the old reading for re-measuring the slip and nothing
    else. Pitch/roll still come off the gimbal. Pure module, no `ds_wrapper`;
    `python dji_pose_selftest.py` checks it against an independent axis construction
    that shares no code, and its section 8 pins the bearing *source* with the real
    measured slips as its fixture.
  - `migrate_clip_poses.py` — one-shot: re-solves the `quat_*` columns of already-recorded
    clips after the gimbal-yaw fix, since `clip_replay.py` publishes the stored pose
    verbatim and cannot correct one at replay time. Rebuilds through the same
    `dji_camera_pose.quat_from_gimbal` the recorder calls, so a migrated clip is
    byte-identical to what the fixed recorder would write. Backs each CSV up to
    `*.pre_yawfix`, writes via temp + `os.replace`, and stamps
    `meta["pose_yaw_source"]` so it is **idempotent by marker, never by inspecting the
    numbers**. **Dry run by default** — `--apply` to write; the dry run prints the
    per-drone slip it would remove, which is the number to check against the flight log
    before committing. No `ds_wrapper`, no drones.
  - `clip_replay.py` — replays a recorded clip into `DroneFeedSharedMemory` **as if the
    drones were flying**, so the Unity DJI scene + stitcher run with no change from a live
    flight: same map, same 48-byte v2 header, same 800×450 BGR payload, same per-frame
    pose, same cross-drone capture skew, real-time paced off the recorded `t_epoch`.
    Every byte goes through the same `utils.imageSharingUtil.write_memory` the live
    publisher uses, and the map name / capacity / header size are **imported from
    `image_stream_feed`** rather than restated — a live/replay wire mismatch would be
    silent (the consumer reads image bytes as a header), so it must not be possible to
    introduce one. Poses are taken from the CSV, never re-derived: re-deriving would
    relatch the GPS origin and put the replay in a different frame from the recording.
    The corollary is that a clip recorded before the
    [gimbal-yaw fix](#critical-gotchas) still carries the bad bearing in `quat_*` and no
    amount of replaying fixes it — `pose_convention_note` checks
    `meta["pose_yaw_source"]` and both warns loudly at startup and fails `--check-unity`,
    pointing at `migrate_clip_poses.py`.
    `--loop` for tuning against fixed footage, `--speed`, and `--pose-lead-s` to re-pair
    frames with earlier/later poses — the one knob for pose/video skew, which the sim's
    error budget makes the dominant term. It also **publishes the clip's standoff** into
    the feed map's scene-plane trailer, recomputed per frame from the clip's stored
    `plane_n`/`plane_d` and the replayed poses rather than republished as the stored mean
    (which is wrong at both ends of any clip where the formation translates);
    `--no-standoff` hands the value back to Unity's inspector. While it is driving,
    `standoff` is dropped from the `--check-unity` checklist and the `[unity]` watch —
    otherwise the watch reports the replayer's own writes as operator edits.
    **`--loop` runs every drone on ONE pass clock** whose period is the longest drone's
    span; shorter feeds hold their last frame to the shared boundary. Each thread used to
    re-latch its own epoch per pass, so a pass lasted that drone's own span and the
    cross-drone `capture_time` skew grew ~35 ms per loop until the stitcher dropped the
    slowest drone's view as stale — see the
    [replay-loop gotcha](#critical-gotchas). Clips are addressed by **label**
    (`--clip grass_nadir_long`): the folder keeps the recorder's `clip_<timestamp>` name —
    that timestamp is what joins a clip back to `flight_logs/`, and `session.json`'s own
    `clip` field references it — while the human name is stored **inside** the clip as
    `meta.label` in its `session.json`, so it travels with the data instead of living in a
    side index that can drift from the folders. `--list` prints the label→folder table
    (duration, posed-frame counts, and a **video-health flag**: any drone below 80% of the
    clip's nominal frame rate is `WARN`, below 50% `*BAD*`, with the starved drones named —
    a replay of a flagged clip repeats the warning at startup, because one aircraft's link
    starving its view costs the mosaic that panel and reads as a stitcher fault),
    `--clip <c> --set-label <name>` writes one and
    **refuses a name another clip already carries** (two clips answering to one `--clip`
    would silently replay the wrong footage). `--clip` still accepts a path or folder name,
    and `--recordings-dir` matches a non-default `RecordingDir`. Needs **no `ds_wrapper`,
    no DroneSwarmServer, no admin, no drones**. Supersedes `image_replay.py` for clips (that one reads
    `saved_streams/` JPEGs and writes `poseStatus 0`, so it can't drive PLANAR).
    `PyUniSharingFast` must still be in the scene — it is the only writer of the wire
    version, intrinsics and scene plane, all three of which
    `StitcherThreading.planar_inputs_ready()` requires; the settings to match are printed
    at startup, including the **measured standoff** when the clip carries one
    (`--set-plane-from-altitude` for nadir; `--set-plane-from-facade` /
    `--set-plane-from-line` / `--set-plane-from-shape` / `--set-plane-standoff`, see
    `clip_scene_plane.py`) and then **checked against what the scene is actually
    publishing** (`unity_stitch_meta.py`): the startup banner lists the inspector fields
    that disagree, `--check-unity` does only that check and exits (0 = ready, 1 = not), and
    while replaying, edits made in the inspector are echoed as `[unity]` lines
    (`--no-unity-watch` to silence) so hand-stepping the standoff under `--loop` leaves a
    record of what was tried. The startup banner is one line per topic; `-v`/`--verbose`
    gives the long form (per-drone counts, full checklist, whole plane report) and nothing
    is only in the long form except wording. GUI state rides on `meta["recording"]`
    (`{on, finalizing, elapsed, remaining, frames, dropped, last, …}`); the button hides
    itself when a controller doesn't publish it. Recording is deliberately **independent of
    swarming**: a Stop, a min-separation auto-STOP or a geofence DISABLE_VS does not cut the
    clip, and it works in `--dry-run`. No `ds_wrapper` import; `python clip_recorder.py`
    runs a self-check that records a synthetic 2-drone fleet through a full auto-stop cycle.
  - `clip_scene_plane.py` — the **scene plane** for a recorded clip: the one PLANAR input
    the field cannot measure. `ScenePlaneMode.FormationRelative` takes the plane's normal
    from the published poses and asks the operator for one scalar,
    `planarStandoffMetres` — the perpendicular distance from the formation to the surface.
    Nothing records it at capture time, but a clip does record every camera's position in a
    georeferenced frame (`meta["pose_origin_latlon"]` + the `pos_*` columns), so the
    standoff is a subtraction as soon as the **surface** has a lat/lon — computable
    **after** the flight, which is what makes it retro-fittable to footage flown with no
    obstacle drawn. **A NADIR clip needs none of that** — the surface is the ground, the
    pose frame's `y` already IS height above the launch pad, so
    **`--set-plane-from-altitude`** (`ground_plane`) takes the standoff straight from the
    flying height with no trace, no origin and no `shapes.json`, and — being a real plane
    rather than a scalar — it is re-measured per frame, so a survey that climbs is still
    described (`nadir_6drone_40m` replays 39.72→41.70 m rather than one 40.93 m average).
    Its tilt reads ~0° because `formation_normal` returns a vertical normal for drones at
    one altitude, which is the same geometry that makes those clips useless against a
    *vertical* wall. `--plane-offset` carries the one site fact it cannot know: how far the
    imaged ground lies below the pad. For a FACADE, three georeferenced sources plus an
    escape hatch, wired into
    `clip_replay.py`: **`--set-plane-from-facade [ID]`** (one wall of a real building
    footprint, picked on the GUI map with "Pick building" — it routes to
    `facade_from_line`, so the azimuth is the wall's TRUE bearing: **the accurate route,
    and the easy one**), `--set-plane-from-line LAT1,LON1,LAT2,LON2` (two points along the
    wall, any bearing — the same accuracy, typed by hand), `--set-plane-from-shape [ID]` (an
    obstacle rectangle from `shapes.json`, which the GUI draws and persists with **no
    controller running**, so no flight is needed to place one; a rectangle has no rotation
    to report, so the wall's azimuth is snapped to N/S/E/W — the last resort), and
    `--set-plane-standoff M` for a number measured any other way. `--plane-offset M` moves the surface M metres beyond the traced
    line, which is what makes a **drone hover track** (or a cadastral outline with a ledge
    in front of it) usable as the trace — see the
    [satellite-parallax gotcha](#critical-gotchas). The result is stored as `meta["scene_plane"]` **inside the
    clip** — same reasoning as `meta.label`: it is a measurement of that footage in that
    clip's latched pose frame. `clip_recorder.py` now stores one **at capture time** from
    the facade the operator had drawn, so these flags are the way to re-measure or
    retro-fit rather than a mandatory step.
    Also owns **`pick_facade`**, the auto-pick that answers "which traced wall is this
    formation filming?" — used live by `swarm_flocking.run()`, by the recorder, and by the
    offline checker, so there is one rule rather than three. It is **pure**: `now` is an
    argument and all carry-over is in a `state` the caller hands back, which is what lets
    `python clip_scene_plane.py --clip <dir> --pick [-v]` replay a clip's own clock and
    reproduce exactly what the live loop would have decided on that footage — printing
    which wall won, for what fraction, how many switches, and every candidate's reject
    reason. That is the offline half of verifying a rule whose entire job is to not flap. It refuses a trace it cannot describe the clip with (the
    formation behind the wall, or the cameras pointing away from it) and reports what the
    number is worth: the standoff's spread over the clip, the angle between the traced wall
    and the normal `PlanarStitcher._plane_from_formation` will actually derive, and the
    **seam cost in mosaic pixels per metre of plane error** (`f·B/Z²`) — see the
    [scene-plane gotcha](#critical-gotchas). Pure module: no `ds_wrapper`, no numpy, no
    OpenCV; `python clip_scene_plane.py` runs a self-check.
  - `unity_stitch_meta.py` — read-only view of **what the Unity stitcher is currently set
    to**, from `PyUniSharingFast`'s own `MetadataSharedMemory` mapping (412 B; the
    `meta*Offset` layout is mirrored here, and `plausible()` cross-checks the mapping's
    self-describing fields — block image size, header size, wire version — against this
    repo's own so a revision skew is refused instead of misread). There is **no way to push
    a setting into Unity through THIS mapping**, and that is structural, not missing work:
    the values live in serialized inspector fields and `WriteMetadata` republishes the whole
    mapping every Unity frame, so anything written here is gone in ~16 ms and the inspector
    never sees it. What is possible is the diff, which is what `clip_replay.py` prints.
    **One setting is nevertheless now driven from the PC — `planarStandoffMetres` — and it
    does not contradict the above**: it does not go through this mapping at all. It rides
    the `DroneFeedSharedMemory` trailer, which flows PC→Unity like the pixels do, and
    `PyUniSharingFast` *republishes* it here. So this module still reads a value Unity
    wrote — which is why `clip_replay.py` drops `standoff` from the checklist while it is
    the one driving. Unity also publishes the **resolved source** at offset 396
    (`standoff_source`, 0 = its own inspector field, 1 = us), because the value at 364 is a
    float either way and the scene can override us outright with
    `planarStandoffSource = Inspector`. It is reported on the live line
    (`standoff 34.26 m (from the PC)`) and is in `watch_fields`, so a mid-replay takeover
    prints an `[unity]` line rather than leaving the replayer claiming a plane it has lost. Opens the
    section with `OpenFileMappingW` rather than `mmap`, deliberately: `mmap.mmap(-1, …)`
    **creates** a missing named section, which would make "Unity is not running"
    indistinguishable from "running with everything zeroed" — and a leftover section of the
    wrong size would make Unity's own `CreateFileMapping` fail. Liveness is the heartbeat
    advancing, not the mapping existing: a stopped editor's bytes stay readable forever.
    Pure module, stdlib + `image_stream_feed`'s wire constants;
    `python unity_stitch_meta.py` self-checks the parser and every catchable mis-setting
    against a synthetic mapping, then reports on a live scene if one is up.
  - `mqtt_command_sender.py` — `MqttCommandSender`, used by `swarm_flocking.py` when
    `DroneIPs`/`--drone-ips` is set: one **persistent** paho-mqtt connection per RC broker
    (`tcp://<rc-ip>:1883`), publishing the command strings directly (the app's Moquette
    intercept fires on any publish, topic irrelevant). Auto-reconnects; QoS 0 for the
    20 Hz `VS:` stream, QoS 1 for one-shots. Exists because the server's send path
    reconnects per command (~220 ms → 1.5 Hz/drone at 3 drones). No `ds_wrapper` import.
    Also hosts the identity-probe capture (`probe_start`/`probe_wait_for`/`probe_stop`):
    the per-IP connections subscribe to `MQTTWayPoints` so
    `swarm_flocking.run_identity_check` can see which RC each server slot's marker lands
    on (see the [drone-identity gotcha](#critical-gotchas)).
    The same connections are the **app→PC return path for link diagnostics**: each
    permanently subscribes to `LISSwarmDiag` (resubscribed in `on_connect`, so it survives
    an RC power-cycle) and keeps the latest `LINK:`/`LINKSCAN:` per drone —
    `link_of()` (age-limited, `LINK_STALE_S`), `scan_of()`, `request_scan()`,
    `clear_scans()`. Messages are routed by topic, so diagnostics never pollute the
    identity-probe buffer.
  - `image_stream.py` — **standalone debug tool only; never run alongside a live
    controller.** (`clip_replay.py` is safe by contrast — it reads files, not the wrapper.) It polls the wrapper from its own process, and the shared-memory
    protocol (one status byte per drone slot, no mutex) lets a second process starve a
    running controller's cmd/telem loops down to ~1 Hz — the bug that motivated
    `image_stream_feed.py`. `image_save.py` / `image_replay.py` + utils in
    `utils/imageSharingUtil.py` complete the video/image shared-memory pipeline.
  - Standalone manual tests (no test framework): `receive_test.py` (read-only, safe),
    `test_telem.py`, `gimbal_test.py`, `altitude_test.py`, `altitude_iterator.py`.
- **`*.ps1` launchers (`dji-joystick.ps1`, `dji-flocking.ps1`, `dji-gui.ps1`)** — these are
  how the operator *actually* starts everything (Windows Terminal panes). They wrap the
  `python ...` invocations and expose params that map onto the scripts' CLI flags, so they
  **must be kept in sync** with the scripts. See [the launchers gotcha](#critical-gotchas)
  and [Launchers](#launchers-ps1-files--how-the-operator-actually-starts-things) below.

### 2. `lis-swarm-app/` — Android app (the drone-side controller)
Runs on the DJI RC (RC Pro). Package `com.lisswarm`, DJI SDK v5 (`5.3.0`), arm64-v8a only.
> **Force-stop DJI Fly first.** On each RC, force-stop the stock **DJI Fly** app before
> launching this app — if DJI Fly is running it holds the drone/SDK connection and this
> app fails to connect (SDK registration/aircraft link never comes up). Force-stop it in
> Android Settings → Apps → DJI Fly (not just background it).
- `com.lisswarm.ConnectionActivity` — launcher; permissions + DJI SDK registration, then opens SwarmActivity.
- `com.lisswarm.SwarmActivity` — the real workhorse. Embedded MQTT broker, command parsing,
  DJI VirtualStick send loop (20 Hz), telemetry listeners, and the video surface.
- `com.lisswarm.LISApplication` — DJI SDK install hook.
- `at.jku.icg.aos_dji_sdkv5.*` — code **kept verbatim from the original AOS app** (do not
  rewrite casually): `AOSManager`, `DroneSwarmStreamData` (RTSP + telemetry native bridge),
  `MQTTEmbedded` (Moquette broker wrapper), `DJIManager`, `NativeLib`, `LiveInfo*`.
- Native `.so` libs live in `app/src/main/jniLibs/` and `app/src/main/lib/` (RtspServer,
  ffmpeg_ext, DJI libs, etc.) — these are prebuilt; there is no NDK source here.

### 3. `rc-joystick/` — a DJI RC Pro as the swarm joystick (in progress)
It replaces the Taranis X9D + `readController.py` with a **dedicated spare RC Pro that is never
linked to an aircraft**. An app on the RC streams its sticks, dials and buttons over the RC's
Ethernet (UDP :5070, the RC is the server): sticks and dials from the RC's built-in gamepad
(~70 Hz), everything else, plus the sticks' fallback, from MSDK. `rcjoy bridge` re-emits
readController.py's exact JSON on :5055, so no consumer changes. The folder imports nothing from
`AOS server/` or `lis-swarm-app/`. Contract: `rc-joystick/PROTOCOL.md`; status, checks and the
MSDK 5.3.0 RC key inventory: `rc-joystick/README.md`.
- `pc/rcjoy/` — **done**: stdlib, Python ≥ 3.7 (`python -m rcjoy selftest|monitor|bridge|fake-rc`,
  run from `rc-joystick/pc`).
  - It times everything with `time.perf_counter` (`protocol.now`). `time.monotonic` is
    `GetTickCount64` on Windows, 15.6 ms per tick, which read RTTs as 0 or 15.6 ms.
  - The monitor's `rx Hz` is only the RC's send clock (the requested `--rate`). The gamepad's rate
    at the PC is `fresh`: states whose sticks or dials changed, read at `--rate 100`.
  - `--csv` logs every state and never overwrites a file.
  - `bridge --profile sim` is for the Unity sim: spacing 0.4–1.6, readController.py's range, because
    the sim takes `angular.x` unclamped as its `d_ref`. The default flocking profile's 0.6–1.4 is
    where `swarm_flocking.py` clamps. `--ax-range` overrides either.
  - Under `--profile sim` C2 is the experiment's **identify** button, not a knob reset: it counts
    into an extra `marks` int (cumulative, so a lost datagram loses no press) that the sim's
    `ExperimentRecorder` logs as the moment of each identify. Only that profile sends the field,
    so the AOS consumers still get readController.py's exact JSON.
  - Without `--rc`, the bridge and the monitor broadcast, and also unicast to the last RC they locked
    onto (`%LOCALAPPDATA%\rcjoy\last_rc.txt`). One `--rc` is enough for every later launch, even
    when the firewall blocks broadcast replies. A loopback `--rc` (fake RC) is never saved.
  - When several RCs answer, `rcjoy/chooser.py` shows a pop-up with one button per RC. It is
    PowerShell + WinForms, because this PC's Python 3.7 has no tkinter. The fleet's RCs never
    answer :5070, so flying the swarm doesn't trigger it.
- `android/` — the **LIS_CONTROLLER** app, package **`com.liscontroller`**. Its DJI App Key in the
  manifest is bound to that package, so they change together. It is its own Gradle project with
  MSDK 5.3.0 and lis-swarm-app's build config minus Moquette; the build steps are in the README.
  - `StreamService` (foreground) owns MSDK init, the read-only `RcInputReader`, and
    `UdpStreamServer`. `MainActivity` renders the screen and feeds `GamepadInput` (joystick events
    reach only the focused window); `RcInputReader.snapshot()` merges the two sources, so the
    stream, `info` and the screen always agree.
    - 1.6's screen is minimal: a status line, chips for the IP, PC link and battery, the stick
      gates and dial sliders, the stick source and gamepad mode, C1/C2, and alerts only when
      something needs doing. Every diagnostic is behind **Details**.
    - An app resource silently replaces a library's of the same name, so check new names against
      the dependencies' `R.txt` (`DEBUGGING.md` §7). 1.6's `string/details` clashed with MSDK's.
  - **Version 1.6** (2026-09-28) is 1.5's stream on that screen, with a new icon. 1.5 ran on the
    RC Pro (Android 10) from 2026-09-25, and **over Ethernet from 2026-09-28**: 100 states/s,
    0 lost, and the IP stable across a replug. Its protocol code, gamepad path included, passed a
    JVM run against the real `RcJoystickClient`. 1.0 crashed on open (the `sdkclasses.bangcle`
    gotcha below). 1.1 was the first to run. Two MSDK facts surfaced then that any MSDK code here
    must respect:
    - **`ProductKey.KeyConnection` is TRUE on an RC Pro with no aircraft.** MSDK counts the RC's
      own link as product `UNRECOGNIZED` (`onProductConnect(0)`). An aircraft link is
      `FlightControllerKey.KeyConnection`, or a product type naming an aircraft. lis-swarm-app
      waits for product **and** FC for the same reason.
    - **MSDK's analytics replaces the process-wide crash handler**
      (`dji.v5.inner.analytics.handler`). LIS_CONTROLLER's `CrashLog.reassert` puts its own back
      on top after each SDK step.

    The async `getValue` of the RC's `KeyConnection` is rejected at first, so the liveness probe
    tries candidate keys (`PROTOCOL.md`, `rc_ok`); `KeyConnection` then answers in 0–30 ms.
  - **MSDK serves the sticks with no aircraft**: pushed on change at 6–10 per second per axis,
    ±660. Their signs match the wire's convention (+ = up / right), checked against the gamepad.
  - **The RC Pro has a built-in gamepad**: USB "DJI DJI Virtual Joystick" (`0x2ca3:0x1501`,
    Xbox 360 protocol, `xpad`), which Android names "DJI embedded joystick". It reports at **~70 Hz**
    in its fast mode; the HID's stick "up" is negative. **DJI's framework hides it from
    third-party apps** unless three global settings are on (see the
    [RC Pro gamepad gotcha](#critical-gotchas)). 1.5 takes the sticks and dials from it while the
    app is in front, with MSDK as the witness that it still delivers, the fallback when it
    doesn't, and a sign cross-check that withholds an axis rather than fly it inverted.
    `stick_src` in every `state` says which source is live.
  - **At the PC the gamepad arrives at ~60 fresh samples/s, not ~70.** The RC's screen runs at
    60 Hz, and Android hands joystick input to the app once per frame. The consumers are 20 Hz, so
    it costs nothing.
  - Open: the gamepad's **slow mode** (~100 ms between reports, so only MSDK's rate), **trigger
    unknown**.
    - On 2026-09-28 it lasted a whole morning and survived everything but a reboot. Every boot on
      record came up fast.
    - Ruled out: Wi-Fi, USB vs Ethernet, charging, battery, DJI's AndroidStatus message, MSDK's
      connection, force-stops, and DJI Fly (`DEBUGGING.md` §8).
    - The screen turns amber with "reboot the RC". Check the mode before flying.

    Also open: S5's broadcast discovery, and the zero-code RF / idle / pairing checks (README).
  - **DJI Fly must not run alongside it.**
    - The RC's home screen launches DJI Fly at every boot, as the default app of the RC's
      **role** (`RoleMgr` in `com.dpad.service`).
    - DJI Fly then takes DJI's single app link to the RC (`dji_link`) from MSDK every ~6 s. That
      makes `rc_ok` false, and the PC gets no joystick.
    - Force-stop it in Settings once it is up. A swipe restarts it within 26 ms, and
      `pm disable-user` does not survive a reboot.
    - Never change the role to get rid of it: that can factory-reset the RC.
    - The user keeps DJI Fly installed (2026-09-28).
  - Crash handling: `CrashLog` writes `crash.txt` plus a step `trace.txt` that survives native
    crashes. After a crash the app opens in safe mode, whose stage buttons bisect stream → MSDK →
    keys. `collect-debug.ps1` gathers it all over USB or network adb (`DEBUGGING.md`).
  - The APK was audited against lis-swarm-app's (`DEBUGGING.md` §7): native `DT_NEEDED`,
    SDK dex classes, assets, merged manifest, dependency versions and resource shadowing all
    match. The theme is AppCompat like lis-swarm-app's; keep it so.
  - The launchers have no `-Controller` switch yet.
- The bridge **sends only while the RC is fresh and usable**. `aircraft_linked` blocks it: a
  linked RC's sticks would also be flying that aircraft. It never pads a gap with neutral packets,
  so a dead RC reaches the swarm's [dropout failsafe](#critical-gotchas) exactly like an unplugged
  Taranis. Output is ≤ 20 Hz because Unity's `UDPReceiverManager` drains one datagram per frame.
  `s2` is clamped to ±0.999 because `joystick_controller.py` reads `int(s2) == 1` as LAND.

## Two protocols you will touch constantly

**Command string** (Python → app; MQTT payload to the RC's broker, published either
directly by `mqtt_command_sender.py` (normal, persistent connection, QoS 0 for the `VS:`
stream / QoS 1 for one-shots) or via `sendWayPointData` → `DroneSwarmServer` (fallback,
~4.5 Hz — its Paho client reconnects per command)):
```
VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw
ENABLE_VS | DISABLE_VS | TAKEOFF | LAND
AIRLINK:mode=<MANUAL|AUTO>:band=<2G4|5G8|DUAL>:bw=<40|20|10|5>:video=<WxH@fps>
                                                              (named, any subset)
LINKDIAG                                                       (read-only)
```
Parsed in `SwarmActivity.onCommandReceived`. Fields: `pitch`/`roll` = velocity m/s,
`yaw` = **yaw RATE deg/s** (DJI VS angular-velocity mode; + = clockwise),
`throttle` = **absolute altitude m**, gimbal angles abs deg. The PC keeps an absolute
target heading and runs a heading-hold P controller (`joystick_controller.heading_hold_rate`)
that emits this rate — see the [yaw gotcha](#critical-gotchas).

`AIRLINK:` is a one-shot radio/camera-stream setup (`SwarmActivity.applyAirlinkSettings`)
sent per drone by `swarm_flocking.py` at startup when `AirlinkBands`/`AirlinkBandwidth`/
`VideoMode` (config) or the matching `--airlink-*`/`--video-mode` flags are set. Its fields
are **named, not positional**, and any subset may be sent (`-` = leave unchanged) — so an RC
left on an older APK reports an unknown field instead of silently applying a new value to
whatever used to occupy that slot. `bw=` (`AirLinkKey.KeyBandwidth`) is the highest-value
knob for a crowded site: it narrows the spectrum each link **occupies**, whereas `video=`
only lowers the bitrate carried inside the existing channel width — and it works under DJI's
AUTO channel selection. There is deliberately **no manual-channel field** — see the
[AirLink gotcha](#critical-gotchas). Every set is read back onto the RC status line.
The AirLink keys are inherited from MSDK 5.3.0's internal `co_b` base class (not on the
public `AirLinkKey` docs for 5.3); `KeyBandwidth`/`KeyFrequencyBand`/`KeyFrequencyInterference`
etc. are all verified present in the 5.3.0 jar and the app compiles against them, but
whether the *aircraft* accepts a given set is firmware-dependent — read the RC status line.

`LINKDIAG` asks the RC for one `LINKSCAN:` reply and changes nothing.

**Link diagnostics** (app → Python; MQTT, topic `LISSwarmDiag` = `MQTTEmbedded.DIAG_TOPIC`)
are the **only** app→PC channel besides the RTSP telemetry string, which cannot carry new
fields — its producer is the native `setTelemetryData()` whose 17-argument signature is fixed
and has no source in-tree. The app publishes onto its own embedded broker via Moquette's
`Server.internalPublish`; `mqtt_command_sender.py` subscribes on every (re)connect:
```
LINK:<signal>:<down>:<up>              1 Hz, unsolicited, -1 = not reported yet
LINKSCAN:band=..:mode=..:bw=..:freq=..:sq=..:down=..:up=..:if=<from>-<to>@<rssi>,...
AIRLINKRES:<text>                      outcome of each AIRLINK: field
```
`AIRLINKRES:` exists because the RC's `tvStatus` is a **single TextView** — the next
message overwrites the previous one, and during the 2026-08-07 bench run the bandwidth
verdict was gone within 3 s, replaced by the `LINKDIAG` line. Reading ten RC screens (or ten
`adb logcat`s) to learn whether a set took is not workable, so the PC that asked for the
change is told directly: `swarm_flocking.py` prints each drone's outcomes after the settle.
Three outcomes are distinguished, and the difference matters: `REJECTED: <reason>` (the SDK
refused), `reads <v> — asked <w>, NOT APPLIED` (the set *succeeded* and the aircraft ignored
it), and `(applied)`.
`LINK:` is served purely from the app's `KeyManager.listen()` cache (**no** `getValue()`
round-trips — that polling is what got removed from `DroneSwarmStreamData`); `LINKSCAN:` is
on-demand only, does five one-shot reads, and prints `?` for any key the firmware locks —
which on this airframe is always `if=`, see the [AirLink gotchas](#critical-gotchas).
A scan taken right after an `AIRLINK:` waits `AIRLINK_SETTLE_S` first and flags any requested
value the aircraft did not adopt. Quality scales are 0-100, DJI's reading: **<40 poor,
40-60 normal, >60 good**. Available only on the direct MQTT command path (the server path has
no return channel).
Surfaced as `meta["link"]` → the GUI's per-drone **RF link** row, logged to
`swarm_debug.csv` (`link_sq`/`link_down`/`link_up`), and the startup scan lands in
`session.json` under `link_scan`.

**RC-side operator lockout:** the app's on-screen **Disable VS** button latches out all PC
motion commands (`VS:`/`ENABLE_VS`/`TAKEOFF`/`LAND` are dropped) until the on-screen
Enable VS button clears it; only `AIRLINK:`, `LINKDIAG` and `DISABLE_VS` pass through while
latched (none move the aircraft). PC-sent `DISABLE_VS` (GUI Stop, min-sep/geofence
failsafes) does NOT latch — see the [lockout gotcha](#critical-gotchas).

**Telemetry string** (app → Python; travels inside the drone's RTSP session as a
non-video data stream — NOT over MQTT — then lands in shared memory appended after the
image bytes). Colon-separated, 17 fields, produced by `DroneSwarmStreamData.setTelemetryData(...)`
and parsed by `joystick_controller.parse_telemetry`:
```
lat:lon:alt:heading:gimbal_pitch:gimbal_roll:gimbal_yaw:sat_count:
drone_pitch:drone_roll:drone_yaw:vx:vy:vz:waypoint_done:value_check:vs_on_off
```
In `getImageAndTelemetryData(droneN)`'s returned array: image YUV is `[0:3110400]`
(1920×1080), telemetry string starts at offset **3110408**.

## Critical gotchas

- **Coordinate frame (read before touching VS math).** `SwarmActivity.initVirtualStick`/
  the send loop set `FlightCoordinateSystem.GROUND` + `RollPitchControlMode.VELOCITY` +
  `YawControlMode.ANGULAR_VELOCITY` + `VerticalControlMode.POSITION`. So the `VS:` `pitch`
  field is **world-frame north velocity** and `roll` is **world-frame east velocity** — NOT
  body forward/right. The drone does world→body rotation internally. **Never rotate
  world→body in Python before filling pitch/roll** — doing so double-rotates and the
  drone scrambles directions at any heading ≠ 0. Telemetry `vx/vy/vz` are NED world-frame
  too. If anyone changes the app back to BODY mode, this whole assumption breaks.
  **DJI axis transpose (do NOT "fix" it back):** on this Mini 3 Pro / MSDK v5 combo the
  GROUND-frame `param.setPitch(...)` actually drives the aircraft **EAST** and `setRoll(...)`
  drives it **NORTH** — the transpose of the obvious mapping. So `SwarmActivity`'s send loop
  deliberately feeds the protocol's **east** field to `setPitch` and its **north** field to
  `setRoll`. Verified by the 2026-07-05 rotation-check: with the naive pitch←north/roll←east,
  every drone flew east on a north command and north on an east command, *identically at west/
  north/south headings* (heading-independent ⇒ still world frame, just N/E swapped — not a
  compass/body-frame problem). The Python side and the `VS:` protocol keep pitch=north/
  roll=east; only the app's DJI binding is transposed. Reverting `setPitch`/`setRoll` to the
  "matching" fields reintroduces the 90° swap.
- **Yaw is a RATE, not an angle (read before touching yaw).** The app's VS yaw channel is
  `YawControlMode.ANGULAR_VELOCITY`, so the `VS:` `yaw` field is **deg/s**, not an absolute
  heading. ANGLE mode (the old default) fed the FC's position controller a stepped heading
  setpoint and was visibly choppy both while turning and while holding. Absolute-heading hold
  now lives on the PC: `joystick_controller.heading_hold_rate(target_yaw, current_heading,
  ff_rate)` = stick feed-forward + `KP_YAW`×heading-error, clamped to `±MAX_YAW_RATE_DEG_S`.
  `joystick_controller` and `swarm_flocking` both keep integrating an absolute `target_yaw`
  and convert it through this helper per send. `KP_YAW` is the main tuning knob (too high +
  laggy telemetry → oscillation). Two structural guards protect the loop: the integration
  goes through `integrate_target_heading`, which clamps the target to ±`MAX_TARGET_LEAD_DEG`
  of the measured heading (anti-windup — without it a sustained turn banks up 40°+ of error
  and the drone wags for seconds after stick release), and `heading_hold_rate` has a
  ±`YAW_ERR_DEADBAND_DEG` deadband so heading noise doesn't keep the nose dithering.
  `--slow` scales the P term too (callers pass `p_scale=speed_scale`) — before 2026-07 it
  only scaled the stick feed-forward, so "slow" flights still yawed at the full clamp; keep
  the scale threaded through if you add a caller. If you
  revert the app to `ANGLE`, also send absolute heading again and drop the heading-hold helper.
- **Gimbal needs VS enabled.** The app only sends gimbal commands inside the 20 Hz VS
  timer (`startVsSendLoop`). Gimbal moves do nothing unless VS is ENABLED.
- **The app's Disable VS button is a PC lockout latch.** Pressing it on the RC drops every
  PC motion command (`VS:`, `ENABLE_VS`, `TAKEOFF`, `LAND`) and keeps re-disabling VS at
  1 Hz until the RC's Enable VS button is pressed (`SwarmActivity.pcLockout`). It exists
  because the PC streams `VS:` at 20 Hz for the controller's lifetime and a QoS-1
  `ENABLE_VS` queued during a link blip can re-arm VS *after* an operator disable — a bare
  `disableVirtualStick()` was not an override. Consequences: a GUI **Start won't re-arm a
  latched drone** (its GUI card shows VS off) and the rotation probe reports VS FAIL for
  it — both correct; clear the latch on the RC. PC-sent `DISABLE_VS` (GUI Stop,
  min-separation/geofence failsafes) never latches, so Stop→Start cycles are unaffected.
  `AIRLINK:` and `DISABLE_VS` still pass through while latched.
- **AirLink settings are STICKY, and "send nothing" ≠ "restore defaults".** An empty
  `AirlinkBands`/`AirlinkBandwidth`/`VideoMode` means no `AIRLINK:` is sent at all — the
  radios keep whatever a *previous* run (or a bench test) last wrote, because band,
  bandwidth and camera mode all persist in the RC/aircraft/camera firmware. Blanking the
  config does NOT undo an experiment; you must explicitly set the value back
  (`AirlinkBands = @('DUAL')` is the DJI default). This is the first thing to check when
  the link "got worse" after an AirLink change. `LINKDIAG`/the startup link scan reports
  what is actually in force per RC.
- **What the Mini 3 Pro actually does with AirLink sets** (bench-verified 2026-08-07 on
  MSDK 5.3.0; the docs are contradictory, so trust this table):

  | Set | Result |
  | --- | --- |
  | `band` in AUTO mode | applies (DJI Fly exposes 2.4/5.8/dual with channel on auto) |
  | `bw` in AUTO mode | **accepted, then reverts** to 40 MHz within ~3 s |
  | `mode=MANUAL` | accepted — DJI's "unsupported" answer is about the **DJI Fly UI**, not the SDK |
  | `bw` in MANUAL mode | applies and holds (`mode=MANUAL:bw=BANDWIDTH_10MHZ`) |
  | `KeyFrequencyInterference` | never served — `if=?` always, so **no on-aircraft band scan** |

  So narrower bandwidth is only reachable via `--airlink-mode manual`, and it is bought with
  DJI's per-link interference adaptation: MANUAL **freezes the channel** (the scan's `freq=`),
  and with no interference measurement there is nothing to plan an assignment from. Entering
  MANUAL without choosing a frequency point pins the aircraft to whatever channel it was on —
  no adaptivity *and* no plan, worse than either alternative. Justify it against the logged
  `link_sq`/`link_down`/`link_up`, the only outcome metric this fleet has. `--airlink-mode
  auto` is the way back out. If channel planning is ever wanted, use the documented
  `KeyFrequencyPoint`/`KeyFrequencyPointRange` — **not** the `KeyChannelNumber` the removed
  `AirlinkChannels` path used, which is absent from the public MSDK 5 AirLink docs.
- **An AirLink set is ASYNC — never read it back immediately.** `applyAirlinkSettings` fires
  `KeyManager.setValue` and returns, and a bandwidth change renegotiates the link over
  seconds, so a `LINKDIAG` sent straight after reports the value from *before* the set —
  indistinguishable from a rejection, and the mistake that made the AUTO-mode revert above
  look like a refusal. `swarm_flocking.py` sleeps `AIRLINK_SETTLE_S` first, then compares
  requested vs reported. Two independent reports, and neither replaces the other because a
  set can be accepted and still not take: the app's `AIRLINKRES:` sees the SDK's accept/reject
  *reason*, the PC's scan re-reads the radio through a different key path later.
- **A polygon obstacle in `shapes.json` MUST also carry its bounding box.**
  `olfati_saber.load_shapes` reads `float(ob["lat_min"])` inside a loop wrapped in
  `except (OSError, ValueError, KeyError, TypeError)` that returns `([], None)` for the
  **whole file** — so one record missing those keys does not degrade one shape, it
  discards every obstacle *and the geofence*, which is a hard flight failsafe. The file is
  read by three processes (`swarm_gui.py`, `swarm_flocking.py`, `clip_scene_plane.py`) that
  can be at different versions on one machine. Hence the v2 contract: a `kind: "poly"`
  obstacle stores `vertices` **and** `lat_min/lat_max/lon_min/lon_max`, always recomputed
  from the ring by `normalize_polygon_obstacle` and never trusted from the wire. An old
  reader then sees a plain rectangle — degraded but strictly **conservative**, since the
  bbox contains the footprint, so avoidance is never weaker than the file describes. Do
  not add a shape type that cannot state a bbox. Two consequences that bite:
  `save_shapes(path, obstacles, geofence)` with no 4th argument means "**preserve the
  facades on disk**" (a read-modify-write) so pre-v2 call sites cannot erase them — pass
  `[]` to clear; and the control loop must **partition obstacles by `kind`** so a polygon
  goes to `polys_ne` XOR `rects_ne` (it satisfies both, and feeding both counts its
  repulsion twice — `olfati_saber.py`'s self-check asserts that double-count is real).
  `python olfati_saber.py` carries the version-skew regression test; run it after any
  change to the format.
- **Multi-drone = 1-based `drone_id`** everywhere, mapping to `DroneSwarmServer` shared-memory slots.
- **Drone identity has TWO independent sources when `DroneIPs` is an explicit list — keep
  them reconciled.** Commands go to the N-th `DroneIPs` entry (RC/switch-port = the
  canonical drone N); telemetry comes from DroneSwarmServer slot N (whatever IP was
  entered/scanned into that slot). If the orders disagree, every control loop closes
  across the WRONG aircraft — the 2026-07-04 4-drone flight had a 3-cycle mismatch and
  three drones spun at the yaw-rate clamp while pair spacing collapsed to 2.9 m.
  `swarm_flocking.py` probes this at startup and on every swarming Start
  (`run_identity_check`: an inert `IDCHECK:` marker is sent through the server's per-slot
  MQTT path while the per-IP command connections subscribe to `MQTTWayPoints` and watch
  where it lands), auto-remaps by pointing `DroneController.telemetry_slot` at the right
  server slot (GUI/logs/flocking all follow), and refuses to arm when unresolvable
  (`--no-identity-check` / `IdentityCheck` config key to override). **Auto-discovery mode
  (empty `DroneIPs`, the script default) sidesteps the dual source entirely**: RC IPs are
  read off the running server and bound to slots by the same probe, so drone id = server
  slot and a mismatch cannot exist (numbering then follows the server's slot order for
  that session, not the switch ports). The legacy `server` path has a single identity by
  construction. The old AOS broker was immune for the same reason — it used the slot for
  both directions.
- **Fresh telemetry is only ~5 Hz per drone** even though the fetch loop runs at 20 Hz —
  ~60% of fetches return a byte-identical repeat of the previous sample (measured across
  2026-07 flight logs). Per-field fresh rates: GPS lat/lon ~5 Hz, heading/attitude ~4–5 Hz,
  gimbal ~5–6 Hz, and velocity `vx/vy/vz` only ~1–2 Hz — near-useless for control; derive
  velocity from GPS positions instead (as `response_monitor.py` does). Root cause is
  app-side: `DroneSwarmStreamData.getTelemetryData()` runs once per video frame but fires
  *async* `KeyManager.getValue()` reads and immediately publishes the previous cached
  values, so freshness is capped by the DJI SDK key-update rate, not the sampling loop
  (switching to `KeyManager.listen()` push listeners is the fix if more is ever needed).
  Budget control gains accordingly: at the current 40 °/s yaw-rate clamp a drone turns up
  to 8° between fresh heading samples. Don't raise `MAX_YAW_RATE_DEG_S`/`KP_YAW` without
  checking this rate first.
- **Min-separation failsafe:** `swarm_flocking.py` auto-STOPs swarming (zero velocities →
  brake → DISABLE_VS, same as GUI Stop) when any pair with a GPS fix gets closer than
  `--min-separation` (default 3 m, `MinSeparation` config key, 0 disables). Distance is
  **3D** (`swarm_plane.separation_3d`), not horizontal — vertical-plane mode stacks drones
  on purpose, and a horizontal-only check reads a forming wall as 0 m apart and stops the
  swarm the moment it starts working. The GUI's distance labels and cohesion plot use the
  same 3D measure so they can never disagree with the failsafe.
- **Vertical-plane mode owns the altitude channel, and altitude frames are per-aircraft.**
  With the wall up, each drone gets its OWN `VS:` throttle value instead of the shared
  `target_alt`; everywhere else in the repo altitude is one shared scalar. Two consequences
  bite: (1) telemetry `alt` is **takeoff-relative per aircraft**, so the true vertical gap
  between two drones is `(alt_i − alt_j) + (ground_i − ground_j)` — a wall built in altitude
  space is skewed by launch-pad height differences and the 3D separation check is wrong by
  the same amount. **Launch every drone from one flat pad.** Entry is gated on the spread of
  reported altitudes (`ALT_SPREAD_GATE_M`, 3 m) since all drones were holding the same shared
  target, which makes that spread a direct measure of the mismatch; the toggle pops back off
  in the GUI when it refuses. (2) A 90° wall puts drones directly above one another, so the
  upper one's **rotor downwash** lands on the lower — a 249 g airframe's worst case, and
  something the Unity sim (independent rigid bodies) does not model at all. There is a
  non-blocking DOWNWASH advisory (chip + log) for it; the 3D min-separation check is the only
  hard stop. Also note plane mode forces heading `manual` (`convexhull`'s hull collapses to a
  line on a wall, `demostitch` ranks drones laterally and stacked drones cannot be), and
  `MAX_ALT_M` (30 m) is now a `--max-alt` / `MaxAlt` setting because a wall needs roughly
  `(N-1)·d_ref` of vertical room — check the site's legal ceiling before raising it.
- **Joystick arm gate:** swarming Start refuses to arm unless a fresh joystick packet
  arrived on :5055 inside the receiver's staleness window (readController.py running +
  controller connected). The GUI mirrors it via `meta["joystick"]`: Start greys out and a
  NO JOYSTICK banner chip shows. Stop / 'q' still work with no joystick; `--dry-run`
  skips the gate. **A joystick lost while armed** (`udp_joystick_receiver.JoystickWatchdog`):
  the swarm flies **neutral sticks** — velocity and climb zeroed, yaw on pure heading-hold,
  the spacing knob `angular_x` kept so `d_ref` doesn't jump — with flocking and every
  failsafe still running, for `JOYSTICK_LOST_STOP_S` (3 s, counted after the receiver's
  0.5 s staleness window), then **auto-STOPs** through the min-separation path (zero →
  brake → DISABLE_VS). EVENTs `JOYSTICK_LOST` / `JOYSTICK_RESTORED:<s>` /
  `JOYSTICK_LOST_STOP:<s>`; the NO JOYSTICK chip shows throughout. Until 2026-09 this
  branch just skipped the tick, so every drone's send thread kept repeating its last
  velocity **and yaw rate** with no min-separation or geofence check — a swarm that lost
  the joystick mid-turn kept turning. "Neutral" means `angular_x = 1.0` (`NEUTRAL_JS`),
  NOT `JoystickState()`'s 0.0, which `d_ref_from_ax` clamps to the tightest spacing
  (4 m); a `--dry-run` that never saw a joystick flies `NEUTRAL_JS` (8 m) and never
  auto-stops. `python udp_joystick_receiver.py --selftest` checks the policy.
- **Python must be 3.7.** The wrapper is built as `ds_wrapper.cp37-win_amd64.pyd`; a
  different Python won't load it. Run scripts from `AOS server/` so the `.pyd` and
  `python37.dll` resolve.
- **`DroneSwarmServer.exe` runs as Administrator**, so anything calling the wrapper
  (VSCode/terminal running the Python scripts) must also run **elevated**, or the shared
  memory / window messaging won't connect.
- **`ds_wrapper.*.pyd` and `DroneSwarmServer.exe` must sit in the same folder** (currently `AOS server/`).
- **The wrapper MUST release the GIL while it busy-waits.** Every `ds_wrapper` call spins
  on a shared-memory status byte until `DroneSwarmServer` acks: a telemetry/image fetch
  blocks ~30 ms (the decode thread only services it on its next `av_read_frame`
  iteration), and a `sendWayPointData` blocks for a full server-side MQTT
  connect→publish→disconnect. `DroneSwarm_Wrapper.cpp` wraps these waits in
  `py::gil_scoped_release` (+ `YieldProcessor()` in the spin). Removing that reintroduces
  the starvation where the 20 Hz send thread froze every other Python thread and the
  telemetry/image rate collapsed to ~3.5 Hz. The Python loops in
  `joystick_controller.DroneController.start` are drift-compensated for the same reason —
  a naive `sleep(interval)` after a ~30 ms blocking fetch can't hold 20 Hz.
- The `joystick_controller.py` "VS_Send" thread relays at 20 Hz; the app re-sends to DJI
  at its own 20 Hz. Telemetry/image fetches run at 20 Hz per drone. **Video is NOT a
  uniform 30 fps** — the app never sets a frame rate; each aircraft streams at whatever
  its camera's recording frame rate is configured to in the DJI menus, and the 2026-07
  fleet measured a mix of ~24 / 25 / ~30 fps per aircraft. All rates are still above the
  20 Hz fetch loop so nothing starves, but standardize the camera settings if the stitcher
  needs uniform frame ages. (The live stream codec is H.265 end-to-end regardless of the
  camera's recording-codec setting — the server's decoder options are `hevc_cuvid`/`hevc`
  only.) This is also why **a recorded clip's MP4 timing is only nominal**:
  `cv2.VideoWriter` takes one constant header rate and has no per-frame timestamp input, so
  `clip_recorder.py` writes a 20 fps header (the fetch rate) and records the *measured* rate
  in the clip's `session.json`. Anything that needs true frame times must join
  `frame` → `t_epoch` through `drone*_frames.csv` — frames are never duplicated or dropped
  to force a constant rate, because that would destroy the frame↔telemetry correspondence.
  Stale-command handling matters — see the UDP staleness window in
  `udp_joystick_receiver.py`.
- **Telemetry `gimbal_yaw` is NOT the camera bearing — it slips at every takeoff.** The
  field (the app's `GimbalKey.KeyGimbalAttitude` yaw) really is world-referenced: regress
  it on `heading` over the 2026-08-11 clips' 187° of rotation and the slope is 1.0000 with
  a 0.22° residual. But it carries a per-aircraft offset that is **re-rolled every
  fast yaw**: 0 on the ground pre-takeoff on every airframe on every date on file, nonzero
  airborne, and different every flight — across 30 airborne flights in `flight_logs/`,
  drone 1 spans −22° to +158°, drone 2 −15° to +50°, drone 3 −14° to +48°. It is a
  **random walk that steps during fast yaw and holds between steps**: bucketing every
  fresh sample in `flight_logs/` by yaw rate, the offset moves a median 1.30° (p95 18°)
  per sample above 25 °/s against 0.10° (p95 2.0°) below it. Drone 2 in
  `flight_20260811_161828` sat at −9°, stepped to −15.5° in 0.2 s while the aircraft
  yawed 6° — at a constant 14.7 m, **not** a takeoff — then held −15.5 ± 0.1° for nine
  minutes and 620° of cumulative yaw travel. It slips because the gimbal's yaw has **no
  absolute reference**: pitch and roll are corrected continuously by gravity (which is
  why those fields are faithful — commanded −90.0 reads −90.0), and there is no
  magnetometer in the gimbal to do the same for yaw, so an error made during a slew just
  persists. It is the gimbal that moved, not the compass — a magnetometer-referenced
  heading cannot hold a constant 15.5° error through 620° of rotation without a
  heading-dependent signature, and the 1.0000 slope says there is none. It differs *per
  aircraft* because the **maneuver history** does, not the hardware. So
  **`dji_camera_pose` takes the bearing from `heading`**. Consequences: **no stored
  constant can fix this** — not per-fleet (one derived on 2026-08-11 applied to a
  2026-07-13 flight is off by 50°) and not even per-flight, since the offset can step
  mid-flight as drone 2 did between two clips of one hover; and nothing is lost by ignoring
  the field, because `SwarmActivity.sendGimbalCommand` already issues an `ABSOLUTE_ANGLE`
  yaw of 0 at 20 Hz and the gimbal never goes there — the Mini 3 Pro's pan axis is not
  user-controllable, so there is no genuine pan for `gimbal_yaw` to carry. Why it matters
  more than the plane: the error is **differential and rotational**, so two cameras whose
  bearings disagree by `δθ` land `Z·tan(δθ)` apart on the scene plane, which no per-view
  translation can absorb and the PLANAR pose refiner is structurally unable to help with.
  The MED fleet's 18.2° spread was **11.3 m of seam at a 34.255 m standoff**, or 173 px —
  bigger than every other term in the budget combined, and the reason those clips would
  not stitch. Recorded clips bake the bearing into `quat_*`, so old ones need
  `migrate_clip_poses.py` once; `clip_replay.py` refuses to replay an unmarked clip
  quietly. `dji_pose_selftest.py` section 8 is the regression test.
- **Whether a traced scene plane is good enough is decided by the standoff, not the trace.**
  PLANAR's plane error displaces a view by `f·B·δZ/Z²` px — **quadratic in standoff**,
  linear in baseline. The 2026-08-11 MED facade clips
  (`planarStandoffMetres` **34.3 / 33.9 m**, B ≈ 10.8 m, f = 525 px at the 800×450 wire
  size) tolerate 1.0 m for a 5 px seam, so the swisstopo trace + DJI's ~2 m absolute GPS
  lands at ~10 px — usable, with the last of it closed by stepping the standoff by hand
  under `--loop`. The same 2 m at a 12 m standoff would be ~70 px, and there the stitcher's
  own plane sweep cannot rescue it either (its basin is ≈ `L·Z/B`, ±0.2–0.6 m — narrower
  than the trace error, so it starts outside the basin). Hence **film facades from 30 m+**;
  the tolerance scales as `Z²/B`, so range and a tight formation both buy more than a
  better trace. `clip_scene_plane.py` prints the pixel cost for each clip's own geometry
  rather than a verdict, because the same trace is fine at 34 m and useless at 12.
- **The scene-plane trailer grows `DroneFeedSharedMemory`, and it fits only because of page
  rounding.** The PLANAR standoff is no longer typed into Unity's inspector; the PC computes it
  and writes it into a 64-byte trailer after the 10 feed blocks, which `ImageSharing.cs` reads
  and `PyUniSharingFast` republishes into its metadata. Growing an already-sized named section
  is normally how you earn `ERROR_ACCESS_DENIED` — Windows never resizes one. It is safe here
  *only* because Windows compares **page-rounded** sizes: 10 × 1,080,048 = 10,800,480 B rounds
  to 10,801,152 (2637 × 4 KB), leaving **672 already-backed bytes**. Measured: create at the
  block size, open at +672, succeeds in either order; +673 is denied. So a new Unity and an old
  DJI_Swarm interoperate both ways round and the trailer reads zero.
  `check_wire_layout.py` asserts `page(FeedSectionBytes) == page(FeedBlocksBytes)`, because past
  672 bytes **both cross orders become fatal and the feed dies** — that bound is machine-checked,
  not a comment. Every `FEED_*` constant lives in `utils/imageSharingUtil.py`, not
  `image_stream_feed.py`, for the same reason: that checker parses the former and asserts it
  against the C#, and does not parse the latter. All five writers of the map
  (`image_stream_feed.py`, `clip_replay.py`, `image_feed_test.py`, the sim's
  `planar_feed_bench.py`, `ImageSharing.cs`) must request the same size or the first one to
  create it wins. Both sides carry a fallback to the pre-trailer size; on this side that is
  **not optional**, because `ImageStreamPublisher.__init__` runs before the aircraft arm and an
  unhandled `OSError` there takes down the flight controller, not merely the mosaic.
  Consequence: **`clip_replay.py` and a live controller must not run together any more.** They
  already fought over the blocks; now they would also fight over the trailer, with no handshake
  to arbitrate.
- **`--loop` needs one pass clock for the whole clip, or it starves a view.** Each
  `_DroneReplay` thread used to re-latch `epoch0 = time.monotonic()` at the top of every pass,
  so a pass lasted that drone's OWN recorded span. On the MED clips those are 6.4503 / 6.4850 /
  6.4655 s, so drone 2 fell **34.6 ms further behind drone 1 every loop**; after ~7 loops (~47 s)
  the cross-drone spread in `capture_time` passed `PlanarStitcher.MAX_CAPTURE_SKEW_S` (0.25 s)
  and the stitcher dropped that view as stale — for most of a ~20 minute beat cycle, with
  nothing in either process saying why, and looking fine for the first 45 s of every fresh run.
  Now every drone is scheduled against one shared epoch and one period (the longest drone's span
  plus `LOOP_GAP_S`), shorter feeds hold their last frame to the boundary, and `capture_time` is
  **monotonic across passes** rather than restarting — a reset made the first drone to wrap read
  ~0 while the others still held ~6.45, which dropped the view that had just been refreshed.
  Measured over an hour of replay: worst spread 0.12 s, zero breaches. `LOOP_GAP_S` is kept small
  (0.08 s) because it is paid as a hold, and a held frame is genuinely stale by that long, so it
  lands directly in the spread the stitcher gates on.
- **Satellite tiles are orthorectified to the ground, not to buildings — never trace a
  roofline.** The GUI's Esri World Imagery is off-nadir, so anything with height is
  displaced away from the tile's nadir point by `height × tan(off-nadir)` (~5–7 m for a
  20 m building at 15°, which dwarfs every other term in the plane budget). Ground-level
  detail — the wall/ground junction, kerbs, road markings — is where it belongs, so trace
  **only** that. Consequences when drawing a facade on the map: all buildings in one tile
  lean the same way, so the lean is measurable off any corner where a wall face is visible;
  the base of the facade *facing* the sensor is visible and traceable, while the far side's
  base is hidden under its own displaced roof and cannot be traced at all. Three ways round
  it, all better than tracing the tile:
  **(1)** the GUI's **"Pick building"** — an OSM footprint is a ground outline, so it
  carries no lean at all; pick the wall you filmed and use `--set-plane-from-facade`. This
  is the default route now. Sanity-check the outline against the **Cadastral (swisstopo)**
  basemap in the layer control, which draws the official cadastral survey; OSM is
  crowd-sourced and `analyse()` says so in a warning.
  **(2)** Swiss cadastral footprints read by hand (`map.geo.admin.ch`, true ground
  outlines, right-click gives WGS84) → `--set-plane-from-line`.
  **(3)** fly a drone along the facade a metre or two off it, trace **its GPS track**, and
  pass that offset as `clip_replay.py --plane-offset`. Still the most accurate available,
  because the aircraft's absolute GPS bias is then common-mode with the clip's and cancels
  out of the subtraction, leaving differential error (~0.5 m) instead of absolute (~2 m) —
  no map, OSM or cadastral, can do that.
- **The `.ps1` launchers are the real entry points — keep them in sync.** The operator does
  not run `python …` by hand; they run `.\dji-joystick.ps1` / `.\dji-flocking.ps1` /
  `.\dji-gui.ps1` (all in `AOS server/`). Each launcher hard-codes the `python` command line
  and maps its own params onto the scripts' CLI flags: `-Drones`→`--drones`, `-Slow`→`--slow`,
  `-NoGui`→`--no-gui`, `-HttpPort`→`--http-port`, `-Lan`→`--http-host 0.0.0.0`. **If you rename
  a script, change a CLI flag/default, or change how a script is invoked, update the matching
  launcher(s) in the same change** — otherwise the operator's normal launch path silently
  breaks even though the script "works" when run directly. The flocking/joystick launchers
  also spawn `readController.py` (the UDP joystick source on :5055) from an *external* repo
  (`vr_swarm_simulation/Assets/Scripts/Control`) under conda env `stitching` — those paths and
  the env name are hard-coded near the top of the `.ps1` files; flag them if they need editing.
- **Every MSDK app must ship `assets/sdkclasses.bangcle`, or it dies on launch.** MSDK 5.3.0's
  protected classes are that file: 2,328,116 bytes, inside the aircraft AAR's `classes.jar`.
  `Helper.install()` loads it before anything else runs, so an APK without it crashes on open
  ("keeps stopping") with nothing of the app's own code executed.
  - lis-swarm-app's `build.gradle` `exclude 'assets/sdkclasses.bangcle'` strips the jar copy.
    That is safe **only** because lis-swarm-app keeps a byte-identical duplicate in
    `app/src/main/assets/`. That file looks like an AOS leftover but is load-bearing: deleting it
    breaks lis-swarm-app exactly this way.
  - `rc-joystick/android` must **not** exclude it. Version 1.0 copied the exclude without the
    duplicate and crashed on open.
  - That build now **fails** if the APK lacks the file (`app/build.gradle`), and the app traces the
    payload size as its first step. `rc-joystick/android/DEBUGGING.md` is the crash procedure.
- **DJI's firmware hides the RC Pro's built-in gamepad from third-party apps.** The kernel sees
  the "DJI embedded joystick" at ~70 Hz, but `com.dji.comkey.ComKeyManager.interceptMotion`
  (system_server, `services.jar`) drops its events before they are queued for any app, unless:
  - global `dji_lab_game_mode` is on (a hidden "Lab" switch; system apps are exempt), and
  - for stick axes, `dji_motion_via_left_joystick_enabled` / `_right_` is `"1"`.

  DJI's flight apps (`config_FocusOffApps`: DJI Fly, Pilot, Agras…) never receive it. Nothing is
  logged when it drops: an app just sees no joystick events at all, however it wraps its window.
  `rc-joystick/android/enable-gamepad.ps1` sets the three settings (they persist).
  - **Only on the dedicated joystick RC, never a fleet RC.** With game mode on, stick motion
    reaches whatever app is in front, and Android turns unconsumed joystick motion into D-pad
    focus navigation, on a fleet RC that would be lis-swarm-app's screen. The script refuses an
    RC with `com.lisswarm` installed.
  - LIS_CONTROLLER consumes the events for the same reason.

## Building & running

### Launchers (`*.ps1` files) — how the operator actually starts things

> **Do not skip these when editing the Python scripts.** The day-to-day way the operator
> runs the system is the PowerShell launchers in `AOS server/`, **not** bare `python …`
> commands. Each launcher hard-codes the command line, the working directories, and the
> conda env, and opens the panes in Windows Terminal (`wt.exe`). The raw `python …` lines
> in the sections below are what the launchers run under the hood / for debugging.

| Launcher | Starts | Params → script flags |
| --- | --- | --- |
| `.\dji-joystick.ps1` | `joystick_controller.py` + `readController.py` | `-Slow`→`--slow` |
| `.\dji-flocking.ps1` | `swarm_flocking.py` + `readController.py` + `swarm_gui.py` | reads **`flocking.config.psd1`** for defaults; CLI flags override it. `-Drones`→`--drones`, `-Slow`→`--slow`, `-GimbalPitch`→`--gimbal-pitch`, `-ConvexHull`→`--heading convexhull`, `-PointInwards`→`--point-inwards`, `-DemoStitch`→`--heading demostitch` / `-StitchOffset`→`--stitch-offset` (demostitch fan offset deg/rank, config key `StitchOffset`), `-Cvm`→`--c-vm`, `-R0`→`--r0`, `-Scale`→`--scale`, `-NoGui`→`--no-gui` (also drops the GUI pane), `-ImageStream`→`--image-stream` (in-process stitcher feed; **no** separate image_stream.py pane), `-DroneIPs`→`--drone-ips` (explicit RC IPs in drone-id order, needs ≥ Drones entries, extras ignored; **empty config `@()` = auto-discover from the running server**, `-DroneIPs server` = force legacy server path), `-NoIdentityCheck`→`--no-identity-check` (skip the command↔telemetry identity probe; config key `IdentityCheck`), `-MinSeparation`→`--min-separation` (auto-STOP distance, m; config key `MinSeparation`), `-DObs`→`--d-obs` / `-R0Obs`→`--r0-obs` / `-CObs`→`--c-obs` (virtual-obstacle/geofence repulsion cutoff, detection radius [physical m] and gain; config keys `DObs`/`R0Obs`/`CObs`; the shapes themselves are drawn in the GUI and persist in `shapes.json`), `-AirlinkBands`→`--airlink-bands` / `-AirlinkBandwidth`→`--airlink-bandwidth` / `-VideoMode`→`--video-mode` (per-drone RF band + channel bandwidth in MHz `40\|20\|10\|5` + camera-stream cap, sent to each RC as an `AIRLINK:` one-shot at startup; config keys `AirlinkBands`/`AirlinkBandwidth`/`VideoMode`; empty = send nothing, which leaves whatever was last applied — **not** a reset, see the [AirLink gotcha](#critical-gotchas)), `-NoLinkScan`→`--no-link-scan` (skip the read-only pre-flight link/interference scan; config key `LinkScan`), `-AirlinkMode auto\|manual`→`--airlink-mode` (channel-selection mode; `manual` is the only way `-AirlinkBandwidth` sticks but freezes the channel, `auto` restores DJI's adaptation and is the way back out; config key `AirlinkMode`), `-RecordingDir`→`--recording-dir` / `-RecordMaxSeconds`→`--record-max-s` (root folder and per-clip duration cap for the GUI's **Record clip** button — per-drone 1080p MP4 + frame index + the flight-data CSVs for that window; config keys `RecordingDir`/`RecordMaxSeconds`; separate from `--log-dir` and gitignored), `-PlaneMode`→`--plane-mode` / `-PlaneGain`→`--plane-gain` / `-PlaneLeash`→`--plane-leash` / `-MaxAlt`→`--max-alt` (vertical-plane "wall" swarming seed, its restoring gain [m/s per m of out-of-plane offset] and vertical leash, plus the ceiling every commanded altitude is clamped to; config keys `PlaneMode`/`PlaneGain`/`PlaneLeash`/`MaxAlt`; the toggle and gain are live in the GUI — this only seeds them), `-HttpPort`→`swarm_gui.py --http-port`, `-Config`→alternate config path |
| `.\dji-gui.ps1` | `swarm_gui.py` only | `-HttpPort`→`--http-port`, `-Lan`→`--http-host 0.0.0.0`, `-NoBuildingLookup`→`--no-footprint-lookup` (kills the map's outbound OSM Overpass queries; map tiles are fetched by the browser and are unaffected) |

`dji-flocking.ps1`'s launch settings live in **`AOS server/flocking.config.psd1`** (a
code-free PowerShell data file read with `Import-PowerShellDataFile`). Precedence is
baked-in launcher defaults < `flocking.config.psd1` < an explicitly-passed CLI flag (resolved
via `$PSBoundParameters`). Only the launcher reads the config — `swarm_flocking.py` itself is
unchanged, so running `python swarm_flocking.py …` directly ignores the config. Keep the
config keys and the launcher's arg-forwarding in sync with `swarm_flocking.py`'s flags.

**If you change a script's CLI flags, defaults, filename, or how it's invoked, update the
matching launcher in the same change** (see the [critical gotcha](#critical-gotchas)). Note
`dji-joystick.ps1`/`dji-flocking.ps1` also launch `readController.py` — the UDP joystick
source on :5055 — from an **external** repo (`vr_swarm_simulation/Assets/Scripts/Control`)
under conda env `stitching`; those paths/env are hard-coded near the top of each `.ps1`.

### C++ `ds_wrapper` (Python module) and `DroneSwarmServer.exe`
Windows + Visual Studio 2022 (MFC, C++/CLI) only. Full toolchain (CUDA, FFmpeg 7.0.1 w/
NVIDIA decode, Npcap SDK, Paho MQTT C static libs) and step-by-step build is in
[AOS server/README.md](AOS%20server/README.md). Summary:
```
# from AOS server/DroneSwarm_Wrapper, in an x64 Native Tools Command Prompt for VS 2022
mkdir build && cd build
cmake -G "Visual Studio 17 2022" -A x64 ..\.
# open DroneSwarmWrapper.sln, build the `ds_wrapper` target
# copy build\<Release|Debug>\ds_wrapper.lib into DroneSwarmServer\, then build DroneSwarmServer.sln
```
The wrapper itself has no source-level tests; verify it by importing in Python (`import ds_wrapper as w`).

### Python scripts
Python 3.7. Deps: `numpy opencv-python matplotlib keyboard paho-mqtt` (and `pygame` for
`joyreporter.py`). In normal use these are started by the [`.ps1` launchers](#launchers-ps1-files--how-the-operator-actually-starts-things)
above — the raw commands below are the equivalents the launchers run (and what to use for
debugging). Run from `AOS server/` **as Administrator**, with `DroneSwarmServer.exe` already running:
```
cd "AOS server"
python receive_test.py                 # safe read-only health check of the telemetry/video path
python joystick_controller.py          # UDP joystick (default); --cli for keyboard; --drone N
python swarm_flocking.py --drones 3    # multi-drone; --dry-run to print VS without transmitting
```
There is no unit-test suite. The `*_test.py` scripts are manual hardware checks — start with
`receive_test.py` (does not command motion) before running anything that flies the drone.

### Browser GUI (`swarm_gui.py`)
Runs in its **own** terminal (no admin, any Python ≥3.7) — it never touches `ds_wrapper`,
it just renders telemetry a controller pushes to it over UDP. Start a controller first
(both push by default), then:
```
cd "AOS server"
python swarm_gui.py --open        # serves 127.0.0.1:8000, opens a browser
python swarm_gui.py --http-host 0.0.0.0   # expose on the LAN (e.g. a tablet)
# or: .\dji-gui.ps1
```
The banner reads "Feed connected" only while a controller is publishing; "No telemetry feed"
means no controller is running (or it was started with `--no-gui`).

### Android app
Gradle 8.11.1, but **no `gradlew` wrapper script is committed** — use Android Studio or a
locally installed Gradle 8.11.x. Single module `:app`. Min SDK 26, target/compile SDK 34, arm64-v8a.
```
cd lis-swarm-app
gradle :app:assembleDebug      # or build/run from Android Studio
gradle :app:installDebug       # to a connected RC
```
DJI app key is in `AndroidManifest.xml` (`com.dji.sdk.API_KEY`).

## Version control

Single git repo rooted at `DJI_Swarm/` (monorepo); `AOS server/` and `lis-swarm-app/` are
plain subfolders. `lis-swarm-app/` was flattened in from its own repo — that history still
lives at `github.com/lis-epfl/lis-swarm-app`. The root has no remote configured yet.

The root `.gitignore` excludes build output (`build/`, `.gradle/`, `x64/`, NuGet `packages/`),
regenerable C++ binaries (`*.exe`, `*.lib`, `*.a`, `*.pyd`, `python37.dll` — rebuild via
`AOS server/README.md`), heap dumps (`*.hprof`), `__pycache__/`, the bundled WebView2
runtime, and the field-session artifacts (`flight_logs/`, `recordings/`, `saved_streams/`,
`shapes.json`, `footprint_cache.json`). The prebuilt Android `.so` libs and assets under `lis-swarm-app/app/src/main/`
**are** committed (no source exists for them); the largest, `libdjisdk_jni.so` (~55 MB),
trips GitHub's >50 MB warning but is under the 100 MB hard limit.
