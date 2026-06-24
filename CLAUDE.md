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

## Components

### 1. `AOS server/` — PC side
- **`DroneSwarm_Wrapper/`** — C++/pybind11 module `ds_wrapper`. The Python⇄C++ bridge.
  It is **only** a shared-memory poke: it writes command bytes into a named Windows
  file-mapping (`dllmemfilemap`), `PostMessage`s the `DroneSwarmServer` window, and
  busy-waits on a status byte. See `DroneSwarm_Wrapper.cpp` for the exact byte layout
  (per-drone slots of `SHMEMSLOTSIZE`; drone N uses offset `(N-1)*SHMEMSLOTSIZE`).
- **`DroneSwarmServer/`** — MFC C++ Windows app (`DroneSwarmServer.exe`). Owns the MQTT
  client to the drones, the RTSP video ingest, and the shared-memory protocol the
  wrapper talks to. The wrapper's `PostMessage` calls target this app's `WM_PYWRAPPER_*`
  message handlers (`DroneSwarmServerDlg.cpp`).
- **Python control scripts** (run against the built `ds_wrapper.*.pyd`):
  - `joystick_controller.py` — primary single-drone joystick driver (UDP joystick or `--cli`).
  - `swarm_flocking.py` — multi-drone Olfati-Saber flocking from one joystick.
  - `udp_joystick_receiver.py` — receives joystick JSON over UDP :5055 (used by the above).
  - `joyreporter.py` — pygame joystick debug readout.
  - `swarm_gui.py` — browser GUI server: a satellite map (default EPFL Lausanne) showing
    each drone's position + heading, a complete graph of inter-drone distance lines
    (metres labelled), and a per-drone status panel. **Does NOT import `ds_wrapper`** — it
    only LISTENS on UDP :5099 for telemetry pushed by a running controller (so it runs
    unprivileged, in its own terminal, on any Python ≥3.7). Frontend assets in `gui/`
    (Leaflet + Esri World Imagery tiles, needs internet, no API key).
  - `swarm_telemetry_feed.py` — `TelemetryFeedPublisher` embedded by `joystick_controller.py`
    and `swarm_flocking.py`; UDP-pushes a JSON telemetry snapshot to `swarm_gui.py` at 5 Hz
    (on by default; `--no-gui` to disable, `--gui-host/--gui-port` to redirect). Best-effort
    and fire-and-forget so it can never stall the control loop.
  - `image_stream.py` / `image_save.py` / `image_replay.py` — video/image shared-memory pipeline + utils in `utils/imageSharingUtil.py`.
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

**Command string** (Python → app, via `sendWayPointData` → MQTT payload):
```
VS:pitch:roll:yaw:throttle:gimbal_pitch:gimbal_yaw
ENABLE_VS | DISABLE_VS | TAKEOFF | LAND
```
Parsed in `SwarmActivity.onCommandReceived`. Fields: `pitch`/`roll` = velocity m/s,
`yaw` = **absolute heading deg**, `throttle` = **absolute altitude m**, gimbal angles abs deg.

**Telemetry string** (app → Python), appended after the image bytes in the shared-memory
array. Colon-separated, 17 fields, produced by `DroneSwarmStreamData.setTelemetryData(...)`
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
  `YawControlMode.ANGLE` + `VerticalControlMode.POSITION`. So the `VS:` `pitch` field is
  **world-frame north velocity** and `roll` is **world-frame east velocity** — NOT body
  forward/right. The drone does world→body rotation internally. **Never rotate
  world→body in Python before filling pitch/roll** — doing so double-rotates and the
  drone scrambles directions at any heading ≠ 0. Telemetry `vx/vy/vz` are NED world-frame
  too. If anyone changes the app back to BODY mode, this whole assumption breaks.
- **Gimbal needs VS enabled.** The app only sends gimbal commands inside the 20 Hz VS
  timer (`startVsSendLoop`). Gimbal moves do nothing unless VS is ENABLED.
- **Multi-drone = 1-based `drone_id`** everywhere, mapping to `DroneSwarmServer` shared-memory slots.
- **Python must be 3.7.** The wrapper is built as `ds_wrapper.cp37-win_amd64.pyd`; a
  different Python won't load it. Run scripts from `AOS server/` so the `.pyd` and
  `python37.dll` resolve.
- **`DroneSwarmServer.exe` runs as Administrator**, so anything calling the wrapper
  (VSCode/terminal running the Python scripts) must also run **elevated**, or the shared
  memory / window messaging won't connect.
- **`ds_wrapper.*.pyd` and `DroneSwarmServer.exe` must sit in the same folder** (currently `AOS server/`).
- The `joystick_controller.py` "VS_Send" thread relays at 20 Hz; the app re-sends to DJI
  at its own 20 Hz. Stale-command handling matters — see the UDP staleness window in `udp_joystick_receiver.py`.
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
| `.\dji-flocking.ps1` | `swarm_flocking.py` + `readController.py` + `swarm_gui.py` | `-Drones`→`--drones`, `-Slow`→`--slow`, `-NoGui`→`--no-gui` (also drops the GUI pane), `-HttpPort`→`swarm_gui.py --http-port` |
| `.\dji-gui.ps1` | `swarm_gui.py` only | `-HttpPort`→`--http-port`, `-Lan`→`--http-host 0.0.0.0` |

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
