# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A DJI drone-swarm control system. A PC operator drives one or more DJI Mini 3 Pro
drones with a joystick; commands flow to the drones and live video + telemetry flow
back. There is **no waypoint navigation in the active path** — control is direct
virtual-stick (joystick) commands. This is a rewrite of an older waypoint-based
"AOS" (Aerial Observation System) app into a leaner app called **LIS_Swarm**.

The repo has three independently-built components plus the end-to-end data flow that
ties them together. Understanding the flow is the key to working here:

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
  - `swarm_flocking.py` — multi-drone Olfati-Saber flocking from one joystick. Also
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
    aligned rectangles) and one **geofence polygon** whose edges repel inward. Shapes are
    **drawn on the GUI map** ("Add obstacle" click-drag / "Add geofence" vertex clicks →
    `/command` POST → UDP :5098 → `command_listener` → `meta["obstacles"]`/`meta["geofence"]`,
    persisted to `AOS server/shapes.json`, gitignored). Shape validation + shapes.json
    persistence live in this module and are shared with `swarm_gui.py`, whose own
    `ShapesStore` also applies/saves every edit — so drawing works and shapes stay visible
    with NO controller running (the GUI serves them as `data.shapes`; a live controller's
    meta echo remains authoritative). `d_obs`/`r0_obs` params are PHYSICAL
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
  - `response_monitor.py` — pure module: `ResponseMonitor`, a sliding-window
    least-squares fit of the rotation+gain between the velocity commands actually sent
    to each drone (post `--slow`) and its GPS-derived velocity (healthy ≈ 0° / gain 1).
    Fed by the flocking loop while swarming, logged to `swarm_debug.csv`
    (`resp_rot_deg`/`resp_gain`) and shown live in the GUI drone cards
    (`meta["resp"]`), so a drone executing commands in the wrong frame is visible
    in-flight instead of only in offline log analysis. No `ds_wrapper` import.
  - `udp_joystick_receiver.py` — receives joystick JSON over UDP :5055 (used by the above).
  - `joyreporter.py` — pygame joystick debug readout.
  - `swarm_gui.py` — browser GUI server: a satellite map (default EPFL Lausanne) showing
    each drone's position + heading, a complete graph of inter-drone distance lines
    (metres labelled), and a per-drone status panel. **Does NOT import `ds_wrapper`** — it
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
    already fetch (via `DroneController.frame_sink`), copies them into a latest-wins
    mailbox, and does the convert/resize/handshake on per-drone worker threads so it
    can never slow the cmd/telem rates. (In real-drone mode the Unity component's
    `enableImageWriting` must be off — its writer would fight this one.)
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
  - `image_stream.py` — **standalone debug tool only; never run alongside a live
    controller.** It polls the wrapper from its own process, and the shared-memory
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
- `com.lisswarm.ConnectionActivity` — launcher; permissions + DJI SDK registration, then opens SwarmActivity.
- `com.lisswarm.SwarmActivity` — the real workhorse. Embedded MQTT broker, command parsing,
  DJI VirtualStick send loop (20 Hz), telemetry listeners, and the video surface.
- `com.lisswarm.LISApplication` — DJI SDK install hook.
- `at.jku.icg.aos_dji_sdkv5.*` — code **kept verbatim from the original AOS app** (do not
  rewrite casually): `AOSManager`, `DroneSwarmStreamData` (RTSP + telemetry native bridge),
  `MQTTEmbedded` (Moquette broker wrapper), `DJIManager`, `NativeLib`, `LiveInfo*`.
- Native `.so` libs live in `app/src/main/jniLibs/` and `app/src/main/lib/` (RtspServer,
  ffmpeg_ext, DJI libs, etc.) — these are prebuilt; there is no NDK source here.

## Two protocols you will touch constantly

**Command string** (Python → app; MQTT payload to the RC's broker, published either
directly by `mqtt_command_sender.py` (normal, persistent connection, QoS 0 for the `VS:`
stream / QoS 1 for one-shots) or via `sendWayPointData` → `DroneSwarmServer` (fallback,
~4.5 Hz — its Paho client reconnects per command)):
```
VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw
ENABLE_VS | DISABLE_VS | TAKEOFF | LAND
AIRLINK:band:channel:res@fps
```
Parsed in `SwarmActivity.onCommandReceived`. Fields: `pitch`/`roll` = velocity m/s,
`yaw` = **yaw RATE deg/s** (DJI VS angular-velocity mode; + = clockwise),
`throttle` = **absolute altitude m**, gimbal angles abs deg. The PC keeps an absolute
target heading and runs a heading-hold P controller (`joystick_controller.heading_hold_rate`)
that emits this rate — see the [yaw gotcha](#critical-gotchas).
`AIRLINK:` is a one-shot radio/camera-stream setup (`SwarmActivity.applyAirlinkSettings`)
sent per drone by `swarm_flocking.py` at startup when `AirlinkBands`/`AirlinkChannels`/
`VideoMode` (config) or the matching `--airlink-*`/`--video-mode` flags are set: band
`2G4|5G8|DUAL|-`, channel int (`-1` = auto mode, `-` = skip), optional `1920x1080@24`-style
camera cap. Every set is read back and shown on the RC status line; a rejected MANUAL
channel reverts that RC to AUTO. The AirLink keys are inherited from MSDK 5.3.0's internal
`co_b` base class (not on the public `AirLinkKey` docs for 5.3) — bench-verify on the
Mini 3 Pro before relying on them in the field.

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
  `--min-separation` (default 3 m, `MinSeparation` config key, 0 disables).
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
  only.) Stale-command handling matters — see the UDP staleness window in
  `udp_joystick_receiver.py`.
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
| `.\dji-flocking.ps1` | `swarm_flocking.py` + `readController.py` + `swarm_gui.py` | reads **`flocking.config.psd1`** for defaults; CLI flags override it. `-Drones`→`--drones`, `-Slow`→`--slow`, `-GimbalPitch`→`--gimbal-pitch`, `-ConvexHull`→`--heading convexhull`, `-PointInwards`→`--point-inwards`, `-Cvm`→`--c-vm`, `-R0`→`--r0`, `-Scale`→`--scale`, `-NoGui`→`--no-gui` (also drops the GUI pane), `-ImageStream`→`--image-stream` (in-process stitcher feed; **no** separate image_stream.py pane), `-DroneIPs`→`--drone-ips` (explicit RC IPs in drone-id order, needs ≥ Drones entries, extras ignored; **empty config `@()` = auto-discover from the running server**, `-DroneIPs server` = force legacy server path), `-NoIdentityCheck`→`--no-identity-check` (skip the command↔telemetry identity probe; config key `IdentityCheck`), `-MinSeparation`→`--min-separation` (auto-STOP distance, m; config key `MinSeparation`), `-DObs`→`--d-obs` / `-R0Obs`→`--r0-obs` / `-CObs`→`--c-obs` (virtual-obstacle/geofence repulsion cutoff, detection radius [physical m] and gain; config keys `DObs`/`R0Obs`/`CObs`; the shapes themselves are drawn in the GUI and persist in `shapes.json`), `-AirlinkBands`→`--airlink-bands` / `-AirlinkChannels`→`--airlink-channels` / `-VideoMode`→`--video-mode` (per-drone RF band/channel assignment + camera-stream cap, sent to each RC as an `AIRLINK:` one-shot at startup; config keys `AirlinkBands`/`AirlinkChannels`/`VideoMode`; empty = leave the radios on DJI auto), `-HttpPort`→`swarm_gui.py --http-port`, `-Config`→alternate config path |
| `.\dji-gui.ps1` | `swarm_gui.py` only | `-HttpPort`→`--http-port`, `-Lan`→`--http-host 0.0.0.0` |

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
`AOS server/README.md`), heap dumps (`*.hprof`), `__pycache__/`, and the bundled WebView2
runtime. The prebuilt Android `.so` libs and assets under `lis-swarm-app/app/src/main/`
**are** committed (no source exists for them); the largest, `libdjisdk_jni.so` (~55 MB),
trips GitHub's >50 MB warning but is under the 100 MB hard limit.
