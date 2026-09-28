# Debugging LIS_CONTROLLER on the RC

The procedure for when the app crashes, hangs, or misbehaves on the RC Pro. The RC is on the PC's
USB-C cable, or on Ethernet with adb over the network (§5). One command, `collect-debug.ps1`,
gathers everything.

## 0. The "keeps stopping" crash of version 1.0 — fixed in 1.1

**Cause.** DJI MSDK 5.3.0 ships its protected classes as `assets/sdkclasses.bangcle`, a 2.3 MB
file inside the AAR's `classes.jar`. `Helper.install()` loads it at startup. lis-swarm-app's
`build.gradle` excludes that jar copy, but only because lis-swarm-app carries a byte-identical
duplicate in its own `src/main/assets/`. Version 1.0 of this app copied the exclude without the
duplicate. Its APK therefore had no MSDK classes, and the process died before anything else ran.

**Fix and guard.**
- 1.1 no longer excludes the file.
- `app/build.gradle` now **fails the build** if an APK ever lacks it.
- The app writes the payload size as its very first trace line (`MSDK payload
  assets/sdkclasses.bangcle: 2328116 bytes`).

## 1. One-time setup

1. On the RC: enable **Developer options** (Settings → About → tap *Build number* 7 times; DJI's
   menu names may differ slightly). Then turn on **USB debugging**.
2. Connect the RC's USB-C to the PC and accept **"Allow USB debugging"** on the RC screen.
3. `adb devices` must list the **RC's serial**. On 2026-09-25 this PC also had an Android
   emulator running (`emulator-5554`). The collector skips emulators, but a plain `adb` command
   needs `-s <rc-serial>` while both are attached.
4. Force-stop DJI Fly: `adb -s <rc-serial> shell am force-stop dji.go.v5`. Make sure
   lis-swarm-app is **not** installed on this RC. Two MSDK apps contend for the RC's USB link,
   and the collector warns if it finds it.

## 2. Install and reproduce (one command)

From `rc-joystick\android`, after building (README, "Build"):

```
.\collect-debug.ps1 -Install -Launch
```

This installs the fresh APK and force-stops any running copy. It clears logcat, starts the app,
waits 20 s (answer any permission prompt on the RC), and captures everything into
`debug-logs\<timestamp>\` (gitignored). A summary prints at the end: device, installed version,
DJI Fly or lis-swarm-app warnings, the crash lines if any, and the app's last steps.

| File | What it is |
|---|---|
| `logcat-crash.txt` | Android's crash buffer: the stack trace of any Java **or native** crash |
| `app-crash.txt` | the app's own report: stack trace, the steps just before the crash, the build |
| `app-trace.txt` / `app-trace.prev.txt` | the app's step trace for this run and the previous one. Written line by line, so it survives even a native crash. |
| `logcat-app.txt` | the app's log (`LIS_CONTROLLER`) plus the runtime's crash tags |
| `logcat-all.txt` | everything, for the rest |
| `package-liscontroller.txt` | installed version, granted permissions |
| `getprop.txt`, `packages.txt` | the RC's Android build, and what else is installed |

Other forms: `.\collect-debug.ps1` alone takes a snapshot without touching the app. `-Launch -Safe`
starts it in safe mode. `-Seconds 60` waits longer. `-Serial <id>` picks a device.

## 3. Reading the result

| What the summary / files show | Meaning | Next |
|---|---|---|
| `FATAL EXCEPTION` + `at com.liscontroller...` | a Java bug in this app | send the folder; the stack names the line |
| `FATAL EXCEPTION` inside `dji.*`/`com.secneo.*` only | MSDK threw | step 4 decides where; check DJI Fly / lis-swarm-app warnings |
| `Fatal signal 11 (SIGSEGV)` / `Abort message` | a **native** crash, almost always inside MSDK's `.so` files; no `app-crash.txt` | the last line of `app-trace.prev.txt` is the step it died in |
| crash right after `Helper.install ...` in the trace | MSDK could not load its classes | the first trace line must show the payload's size, not `MISSING` |
| no crash; app shows red **SDK FAILED** (1.5: **SDK: registration FAILED**) | not a crash: the first registration needs internet. An uninstall wipes it; `install -r` keeps it. | give the RC internet once, press *Retry registration* |
| red **AIRCRAFT LINKED** banner with no aircraft on | 1.1 read `ProductKey.KeyConnection`, which is true on an RC Pro alone (product `UNRECOGNIZED`). Fixed in 1.2. | 1.2's banner names the signal. If it says *flight controller connected* with every aircraft off, send the folder: the trace records every link-key change. |
| **NOT OK - liveness probe failing** | no async read of the RC answering yet | the trace says which candidate keys were tried and why each was dropped. After all four, 1.2 falls back to `KeyConnection` alone by itself. |
| no crash; red **NOT OK - …** | the app runs; the reason is on the status line | see README, "The hardware session" |
| red **NOT OK · MSDK reports the RC not connected**, 1.6's alert **MSDK has lost the RC**, the trace flipping `key product_conn: true -> false` every ~6 s | **DJI Fly is running** and taking DJI's one app link to the RC from MSDK, in turns. The home screen starts it at every boot (README, "DJI Fly on the joystick RC"). While it holds the link, the PC gets no joystick. Seen 2026-09-28 11:41. | Settings → Apps → DJI Fly → **Force stop**, once its screen is fully up. Don't swipe it away: that restarts it at once. The alert and the flipping stop. |
| amber **sticks via MSDK, ~10 Hz · DJI game mode is off …**, 1.6's alert **DJI's gamepad gate is closed**, trace `DJI gamepad gate: … false` | DJI's framework drops the RC's built-in gamepad for this app (README, "The RC Pro's built-in gamepad"). The stream still works, at MSDK's ~10 Hz. | `.\enable-gamepad.ps1` on the joystick RC; the app picks it up within a second |
| amber **sticks via MSDK (the app is not in front)** / **(no gamepad report yet)** | by design: joystick events reach only the focused window, so the sticks come from MSDK until the app is in front **and** the gamepad has reported since | bring the app to the front and move a stick |
| trace `gamepad reports/s: …, median gap ~100 ms (SLOW mode)`, 1.6's centre readout **SLOW** and amber header **gamepad in slow mode · reboot the RC** | the RC's virtual joystick is in its slow mode (~10 reports/s, against ~14 ms apart in its fast mode). The kernel sees the same, so nothing is being lost. Seen 2026-09-25 15:58 and for a whole morning on 2026-09-28, through restarts, reinstalls, Wi-Fi, USB/Ethernet, charging and DJI Fly. Trigger unknown (README). | **reboot the RC**: every boot on record came up fast. Note what came before the slow mode, and send the folder. |
| PC monitor `fresh` ~60/s while the app's `gamepad` says ~70 | the RC's screen runs at 60 Hz, and Android hands joystick input to the app once per frame, so two reports in one frame arrive as one. Nothing is lost on the wire (`seq loss 0`). | nothing: expected. Every consumer runs at 20 Hz. |
| trace `CROSS-CHECK: MSDK reads <axis> INVERTED vs the gamepad` | MSDK and the gamepad disagree in sign while both are held still. The MSDK fallback then withholds that axis (the PC sees no joystick) rather than fly it backwards. On 2026-09-25 all six axes **agreed**. | flip that axis in `STICK_SIGN` / `DIAL_SIGN` (`RcInputReader.java`) only after confirming on the PC monitor which source is wrong |
| `run-as: unknown package` | the app is not installed on this device | wrong device (`-Serial`) or install failed (`install.txt`) |

## 4. Bisect with safe mode

After a Java crash, the next launch opens in **SAFE MODE** automatically. `-Launch -Safe` forces
it. Nothing starts by itself. The Details panel opens by itself, with the crash report, the
previous run's last steps, and a row of stage buttons:

| Button | Starts | If *this* step crashes, the fault is in … |
|---|---|---|
| **1 · stream only** | the foreground service + UDP :5070. No DJI code at all. | this app / Android (service, notification, socket) |
| **2 · + MSDK** | `SDKManager.init` + registration, no key listeners | MSDK start-up: payload, native libraries, USB link to the RC, DJI Fly contention |
| **3 · + keys** | the read-only key listeners and the liveness probe: normal operation | a key or a callback. Since 1.1 one bad key is caught, shown as `FAILED`/`ERR` in the KEYS table and logged, instead of killing the app. |

Press them in order. After each one, watch the status line (close Details to see it) and **THIS
RUN, last steps**. If one crashes, open the app again: the report and trace show that step. Run
`.\collect-debug.ps1` (no `-Launch`) to capture the files. **Clear report & start** leaves safe
mode.

The service no longer restarts itself after a crash (`START_NOT_STICKY`). One bug used to turn
into an endless "keeps stopping" loop.

## 5. Logcat while the RC is on Ethernet

USB-C and the Ethernet adapter compete for the RC's port. Once it no longer crashes, switch adb to
the network:

```
adb -s <rc-serial> tcpip 5555            # while still on USB
# unplug USB, plug the Ethernet adapter into the swarm switch, read the IP off the app's header chip
adb connect <rc-eth-ip>:5555
adb -s <rc-eth-ip>:5555 logcat -s LIS_CONTROLLER
```

The once-a-second `rates/s` lines are MSDK's listener rates. The gamepad's side is in `trace.txt`
(`gamepad reports/s`, with the median report gap that tells the virtual joystick's fast mode from its
slow one). `adb -s <rc-eth-ip>:5555 usb` switches back.

- `.\collect-debug.ps1 -Serial <rc-eth-ip>:5555` works unchanged over the network.
- **Network adb does not survive a reboot of the RC.** It needs `adb tcpip 5555` over USB again. The
  app and `rcjoy monitor` need no adb, so a reboot's effect can be read off the monitor, and the
  traces collected over USB afterwards: `app-trace.prev.txt` keeps the previous run.
- The RC kept its address across an Ethernet replug (2026-09-28), so `--rc` stays valid.

**The kernel's own report rate** is the ground truth for the gamepad's mode, independent of the
app. Capture 10 s on the RC and pull the file, which also avoids PowerShell re-encoding the text:

```
adb -s <serial> shell getevent -pl /dev/input/event8        # must name "DJI embedded joystick"
adb -s <serial> shell "timeout 10 getevent -lt /dev/input/event8 > /sdcard/ge.txt"
adb -s <serial> pull /sdcard/ge.txt ge.txt
```

Keep a stick moving throughout. The gaps between consecutive `SYN_REPORT` lines are the report
interval: 14–16 ms in fast mode, ~100 ms in slow mode. On 2026-09-28 the kernel and the app agreed
exactly in both modes.

## 6. What to send

The `debug-logs\<timestamp>` folder, and a photo of the RC screen. That covers every case above.

## 7. Audited against lis-swarm-app (2026-09-25)

lis-swarm-app is the MSDK app known to run on these RCs. With both debug APKs built, every
packaging-level difference was checked against it:

| Checked | Result |
|---|---|
| Native libraries: every `DT_NEEDED` of all 46 `.so` files, resolved through virtual addresses as the dynamic linker does. DJI's libraries have deliberately inconsistent file offsets, so a naive reader fails on 25 of them. | All present in the APK or among public NDK libraries. Nothing needs lis-swarm-app's three extra libraries (its own RTSP / ffmpeg_ext / lf_interface). |
| MSDK / secneo classes in the dex (`dexdump`) | identical: 47 `dji.*` + 2 `com.secneo.*` stubs |
| Assets | identical, apart from lis-swarm-app's own AOS/map leftovers; `sdkclasses.bangcle` was the one real gap (§0) |
| Merged manifest | identical permissions, features and SDK components. MSDK declares **no** activities. Ours adds only the stream service and its three permissions. |
| Resolved runtime dependencies | ours is a strict subset. The only version difference is `slf4j-api` 1.7.25 (MSDK's own) vs 1.7.36 (lis-swarm-app pins it for Moquette). |
| Resource names shadowing a library's | only `string/app_name` and `xml/accessory_filter`, the same two as lis-swarm-app. MSDK's own accessory filter lists the same five DJI identities. Rechecked for 1.6's new screen against every dependency's `R.txt`: its one clash, `string/details` (MSDK has one), was renamed. **Recheck after adding any resource**: an app resource silently replaces a library's of the same name. |

**Deliberate behaviour differences:**
- MSDK init runs in a foreground service, with the same application context lis-swarm-app passes.
- Nothing waits for an aircraft.
- RC keys are listened to at once.
- The activity is `singleTask`.
- `CrashLog` runs before `Helper.install`.

**Aligned in 1.1 after the audit:**
- The theme is **AppCompat**, like lis-swarm-app. MSDK showing an AppCompat dialog over a
  Material-themed activity would throw.
- The crash handler is **re-asserted** after `Helper.install`, the SDK's content providers,
  `SDKManager.init` and registration, because an SDK that installs its own default handler would
  otherwise silently switch `crash.txt` off.

**So if 1.1 still fails on the RC, packaging is not the cause.** Look at the runtime: §3, the
trace, and safe mode.

## 8. Dead ends: already ruled out

So nobody spends a session on them again:

- **The gamepad reaching no app (1.0–1.4):** not the Gboard IME, not the window-callback wrapper
  (a tap on it saw nothing either). It was DJI's framework gate (README, "DJI's gate").
  `dumpsys input`'s RecentQueue also lists events the framework **dropped**, so it proves nothing
  about delivery.
- **The gamepad's slow mode (2026-09-28):** not Wi-Fi, USB or Ethernet, charging, battery level,
  DJI's AndroidStatus message to the controller (identical payloads in both modes), MSDK's
  connection, a force-stop of the app, or DJI Fly. The controller never re-enumerated between
  modes. Unsolved; a reboot recovers it.
- **Disabling DJI Fly:** `pm disable-user` did not survive a reboot. Don't change the RC's role
  instead: it can factory-reset the RC (README, "DJI Fly on the joystick RC").
