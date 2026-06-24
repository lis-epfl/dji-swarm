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
  - `image_stream.py` / `image_save.py` / `image_replay.py` — video/image shared-memory pipeline + utils in `utils/imageSharingUtil.py`.
  - Standalone manual tests (no test framework): `receive_test.py` (read-only, safe),
    `test_telem.py`, `gimbal_test.py`, `altitude_test.py`, `altitude_iterator.py`.

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

## Building & running

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
`joyreporter.py`). Run from `AOS server/` **as Administrator**, with `DroneSwarmServer.exe` already running:
```
cd "AOS server"
python receive_test.py                 # safe read-only health check of the telemetry/video path
python joystick_controller.py          # UDP joystick (default); --cli for keyboard; --drone N
python swarm_flocking.py --drones 3    # multi-drone; --dry-run to print VS without transmitting
```
There is no unit-test suite. The `*_test.py` scripts are manual hardware checks — start with
`receive_test.py` (does not command motion) before running anything that flies the drone.

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
