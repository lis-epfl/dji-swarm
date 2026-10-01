# rc-joystick — a DJI RC Pro as the swarm joystick

This replaces the Taranis X9D with a **dedicated spare DJI RC Pro that is never linked to an
aircraft**. The **LIS_CONTROLLER** app (package `com.liscontroller`) on the RC streams its sticks,
dials and buttons over the RC's Ethernet:

- the sticks and dials from the RC's **built-in gamepad** (~70 reports/s) while the app is in front;
- the buttons, RC health and the aircraft interlock from **DJI MSDK**, which is also the sticks'
  fallback (~10 Hz).

On the PC, `rcjoy bridge` re-emits the exact JSON `readController.py` sends, so
`swarm_flocking.py`, `joystick_controller.py` and the Unity sim run unchanged.

The folder imports nothing from `AOS server/` or `lis-swarm-app/`. The wire contract is
[PROTOCOL.md](PROTOCOL.md).

## Status

| Part | State |
|---|---|
| PC package `pc/rcjoy/` (client, bridge, monitor, fake RC) | **done**; `python -m rcjoy selftest` passes all 122 checks |
| Swarm dropout failsafe (`AOS server/udp_joystick_receiver.JoystickWatchdog`) | **done**; it protects the Taranis path too |
| Android app `android/` (LIS_CONTROLLER, App Key in the manifest) | **1.6** (2026-09-28): 1.5's stream on a new minimalist screen and icon (below). 1.5 ran on the RC Pro from 2026-09-25, over Ethernet from 2026-09-28: sticks and dials from the built-in gamepad at ~70 Hz, MSDK as witness and fallback, every axis cross-checked. History: 1.0 crashed on open ([DEBUGGING.md](android/DEBUGGING.md) §0); 1.1 showed a false AIRCRAFT LINKED and its liveness read was rejected (fixed in 1.2); 1.3 retried registration by itself; 1.4 found DJI's gate on the gamepad. The protocol code passed a PC-side JVM run against the real `RcJoystickClient`, gamepad path included. |
| Ethernet (2026-09-28) | **passes**: 100 states/s, 0 lost, IP stable across a replug, back ~3 s after one. The PC gets **~60 fresh samples/s** in fast mode, capped by the RC's 60 Hz screen (below). Open: the gamepad's **slow mode** (below). |
| Zero-code checks Z1–Z3, hardware checks S1–S6 (below) | S1–S4 answered on the bench, S5's unicast half on 2026-09-28; S5's broadcast discovery, S6's FC value and Z1–Z3 still to record |
| Swarm software | [test procedure below](#testing-with-the-swarm-software): the simulator first, then AOS server |
| Launcher switch `-Controller taranis\|rcpro` | not started |

## The Android app (`android/`)

`StreamService`, a foreground service, owns:

- MSDK init and registration, without waiting for an aircraft.
- The **read-only** `RcInputReader`. It only calls `listen` and `getValue`, never `setValue` or
  `performAction`.
- The UDP stream on :5070.
- A wake lock.

`MainActivity` renders the screen, and feeds **`GamepadInput`**, because Android delivers joystick
events only to the focused window:

- **Sticks and dials come from the RC's built-in gamepad** while all of these hold: the app is in
  front, the gamepad has reported since it came to the front, the screen is live, and DJI's gate is
  open (below). Otherwise they come from MSDK, at ~10 Hz. Every `state` says which (`stick_src`).
- **MSDK is the witness.** The gamepad reports only on change, so its silence proves nothing. A
  real MSDK stick or dial change that the gamepad did not report means the gamepad is not
  delivering, and the sticks fall back to MSDK until it reports again.
- **MSDK is cross-checked** against the gamepad whenever both are held still. An axis MSDK reads
  with the opposite sign is withheld on the MSDK path (the PC then has no joystick) rather than
  flown backwards. On 2026-09-25 all six axes agreed.
- The activity **consumes** the gamepad's events, so Android never turns stick motion into D-pad
  navigation of the screen. Immersive mode keeps a stray edge swipe from taking focus away.

**DJI's gate on the gamepad.** DJI's firmware drops the gamepad's events for every non-system app
unless three global settings are on. Run this **once**, with the joystick RC on USB:

```
.\android\enable-gamepad.ps1            # -Status to look, -Disable to undo
```

It refuses an RC that has lis-swarm-app installed, because on a fleet RC it would also hand the
flying sticks to whatever app is in front. The settings survive a reboot. A DJI firmware update
may reset them; the screen and trace then say so, and the sticks fall back to MSDK.

Swiping the app away from recents, or pressing **Stop**, stops the stream with a `bye`, and the PC
goes stale at once. Leaving it any other way (swipe up from the bottom edge for the navigation
bar) keeps it streaming, with the sticks from MSDK until it is back in front. The RC's front
circle button did nothing when tried (2026-09-25).

**What the screen shows** (1.6; everything diagnostic is behind **Details**):
- **One status line:** a coloured word and why.
  - green **STREAMING**: sticks from the gamepad in fast mode, with the rate and how many PCs
  - amber **STREAMING**: degraded, still usable. Either the sticks are on MSDK (~10 Hz, and
    why), or the gamepad is in its **slow mode**, which says "reboot the RC".
  - amber **READY** (no PC subscribed yet) or **STARTING**
  - red **NOT OK** / **BLOCKED** / **SDK FAILED**, with the reason
- **Three chips:** the RC's Ethernet address (what `--rc` needs), the PC link, the battery.
- **The sticks** as round gates and **the dials** as sliders. A control the RC does not serve is
  drawn dashed and empty: a missing stick is not a centred one.
- **The source:** GAMEPAD (FAST / SLOW, reports/s) or MSDK (10 Hz), and **C1/C2**, which light up
  while held and flash on each press.
- **Alerts, only when something needs doing:** Wi-Fi or Bluetooth (or their scanning) on; MSDK
  has lost the RC, with the DJI Fly fix ([below](#dji-fly-on-the-joystick-rc)); DJI's gamepad
  gate closed; an axis MSDK reads inverted; battery below 25%. And a large red banner if an
  aircraft ever links: the stream is then blocked.
- **Details** (a panel over the screen; it opens by itself in safe mode, with the stage
  buttons): SDK state, RC info and link keys, the values on the wire, every button's level and
  press count, both stick sources' values and the cross-check verdict per axis, the liveness
  probe, the radios, the stream and its subscribers, the **KEYS** table (callbacks, callbacks/s,
  age, cached seed, last value), Android's input devices and recent input events, and the
  trace's last steps.

**Build** with the same toolchain as lis-swarm-app. A `gradlew` is committed, but it pins
Gradle **9.0.0**, and the app has only been built with the local 8.11.1 below:

```
cd rc-joystick\android
$env:JAVA_HOME = "C:\Program Files\Android\Android Studio\jbr"
& "$env:USERPROFILE\.gradle\wrapper\dists\gradle-8.11.1-bin\bpt9gzteqjrbo1mjrsomdt32c\gradle-8.11.1\bin\gradle.bat" :app:assembleDebug
adb install -r app\build\outputs\apk\debug\app-debug.apk      # USB debugging on the RC
```

**If a control reads backwards,** flip its entry in `STICK_SIGN` / `DIAL_SIGN` at the top of
`RcInputReader.java`. The wire convention is + = up / right / clockwise, and the PC never flips a
sign.

**If the app crashes or misbehaves,** follow [android/DEBUGGING.md](android/DEBUGGING.md). With
the RC on USB-C, `.\collect-debug.ps1 -Install -Launch` installs, reproduces and gathers
everything into one folder.
- Every run writes a step trace, and a crash report (`crash.txt`) whenever it crashes. After a
  crash the app reopens in **safe mode**.
- In safe mode its stage buttons start the service one piece at a time (stream → MSDK → keys) to
  find the step that fails.

## DJI Fly on the joystick RC

DJI Fly stays installed on this RC; that was decided on 2026-09-28. But it must not run alongside
LIS_CONTROLLER:

- **The RC's home screen starts it at every boot.** It is the default app of the RC's **role**.
  DJI's `RoleMgr` (in `com.dpad.service`) maps role 4 to `dji.go.v5`, and the launcher logs
  `startAppByRole defaultapp:dji.go.v5`.
- **It then fights LIS_CONTROLLER for DJI's one app link to the RC** (`dji_link`). Every ~6 s one
  of them connects and the other drops. While DJI Fly holds the link, MSDK reports the RC not
  connected, `rc_ok` is false, and the PC gets **no joystick**. The app's alert says so, and the
  trace shows `product_conn` flipping every few seconds.
- **After every boot:** wait until DJI Fly's screen has fully come up, then **Settings → Apps →
  DJI Fly → Force stop**, then open LIS_CONTROLLER. It must read STREAMING and stay that way.
  - Don't swipe DJI Fly away from recents: Android restarts it within 26 ms.
  - Don't force-stop it within the first seconds of boot: at 11:41 on 2026-09-28 it came back
    1.5 s later. Once it is up, a force-stop holds (13:15:24: zero link drops afterwards).
- **What does not work:**
  - `pm disable-user` did not survive a reboot. DJI's service checks DJI Fly's state at every
    boot (`updateFlyAppState boot check`).
  - Changing the RC's role would change the default app. But the role also selects the RC's
    firmware and button mapping, and switching it can **factory-reset the RC** (`MASTER_CLEAR
    after 15s`) and download firmware. Don't.

## The hardware session (answers S1–S6)

1. On the spare RC Pro, **force-stop DJI Fly** (Settings → Apps; see
   [DJI Fly on the joystick RC](#dji-fly-on-the-joystick-rc)). Install the APK. **Never** install
   lis-swarm-app on this RC.
2. **Register once online (S1).** Give the RC internet (lab Wi-Fi is fine for this one step), open
   LIS_CONTROLLER, grant the permissions, and wait for `SDK: registered`.
3. **Make it RF-quiet:**
   - Wi-Fi off and Bluetooth off.
   - Settings → Location → **Wi-Fi scanning and Bluetooth scanning off**.
   - Airplane mode on, if offered.
   - Reboot, open the app again, and confirm it still reaches `SDK: registered` **offline**
     (S1). The red RF banner must be gone.
4. **Connect it:** plug the RC's USB-Ethernet into the swarm switch. On the PC, from
   `rc-joystick\pc`, run `python -m rcjoy monitor --rate 100 --csv s5.csv`. **Leave out
   `--rc`**, so this also proves broadcast discovery (S5). For the gamepad's rate, circle the
   right stick about once a second and read the `rates/s` line (below, "Reading the rates").
5. **Move every control, one at a time**, and read the answers:
   - S2: the monitor's bars and raw values, and the app's KEYS table (in **Details**).
     - Does each stick read + for up/right?
     - Do the dials spring back to 0?
     - Does the right-hand control show up on `r` or on `wheel`?
   - S3: the table's callbacks/s while sweeping a stick. Also `adb logcat -s LIS_CONTROLLER`
     prints a per-key rate line every second.
   - S4: the BUTTONS line in Details (level and press count per button), and the monitor's
     press counts.
   - The **PROBE** lines show whether an async read of the RC is a real round-trip or a cache hit.
   - The **ANDROID INPUT DEVICES** section shows whether a gamepad is exposed.
6. **Check the link (S6):** the LINK line in Details must read `fc false` or `fc null`.
   `product true (type UNRECOGNIZED)` is normal: it is the RC Pro's own link (S6 below).
   `still_linked` and `pairing` are recorded for the findings.
7. **Photograph the Details panel** and paste the monitor output. That fills in the table
   below.

## PC side (`pc/`, stdlib only, Python ≥ 3.7)

Run from `rc-joystick/pc`:

```
python -m rcjoy selftest                       # loopback checks, no hardware, ~14 s
python -m rcjoy monitor  [--rc IP] [--rate 100] [--csv PATH]   # bench: every input + link health
python -m rcjoy bridge   [--rc IP] [--out HOST:PORT]... [--profile flocking|joystick|sim] [--ax-range LO,HI]
python -m rcjoy fake-rc  [--pattern steps|sweep|still] [--input-hz 70] [--aircraft-linked] [--null lv,r]
```

- **Reading the rates.** The monitor's `rates/s` line shows three numbers, and they are different
  things:
  - `wire` is the RC's **send clock**. The app samples its latest input at the `--rate` asked for
    (10–100), whatever the sticks do. It reads 50 by default even if the gamepad is at 70.
  - `fresh at this PC` counts the states whose sticks or dials changed. The gamepad reports only
    on change, so while a stick moves, this is the rate at which its reports **reach the PC**.
    Held still it reads 0.
  - `app gamepad` is the app's own count of the reports it received, with the median gap that
    tells fast mode (~14 ms) from slow (~100 ms). It reads the same over USB or Ethernet.

  To measure the gamepad, ask for `--rate 100`: at 50, reports that arrive between two ticks
  overwrite each other before they leave the RC. `fresh` can then still fall a little short of
  the app's figure, because two reports sometimes land in one tick, which costs nothing on the
  wire (`seq loss` stays 0). The CSV's `t_ms` (the RC's send clock) shows whether the RC's ticks
  were regular. `fake-rc --pattern sweep --input-hz 70` reproduces all of this on loopback.
- **Every rcjoy module times with `time.perf_counter`** (`protocol.now`). `time.monotonic` is
  `GetTickCount64` on Windows, 15.625 ms per tick, which read a loopback RTT as 0 and a 2 ms one
  as 0 or 15.6.

- **Without `--rc`** the bridge and the monitor find the RC by themselves. They broadcast, and they
  also send a unicast to the last RC they locked onto, which is saved in
  `%LOCALAPPDATA%\rcjoy\last_rc.txt`.
  - The broadcast's reply relies on Windows' default "allow unicast response to broadcast"
    firewall setting. The unicast to the saved RC does not, so one `--rc IP` is enough for every
    later launch.
  - **If several RCs answer, a pop-up asks which one.** It shows one button per RC: its IP (the
    chip at the top of the RC's screen), then type, serial, battery and app version. The RC used
    last is marked, and Enter picks it. The pick is saved. "Not now" leaves the bridge waiting,
    and it asks again only when the set of answering RCs changes. Where no pop-up can be shown,
    the saved RC wins, else `--rc` is required.
  - Only RCs running LIS_CONTROLLER answer. The fleet's RCs (lis-swarm-app) never do, so a swarm
    on the switch does not bring up the pop-up.
  - A `--rc` on this PC (a loopback fake RC) is never saved.
- **The bridge sends only while the RC is fresh and usable:**
  - `rc_ok`
  - no aircraft link
  - all four sticks served

  Silence looks like an unplugged Taranis, so the swarm's arm gate and its dropout failsafe
  (neutral sticks for 3 s, then auto-STOP) act on the truth.
- **Mapping** (Mode 2, `--stick-mode` for 1/3):
  - Right stick: forward (up) and right.
  - Left stick: climb (`v·|v|`, as readController does) and yaw (right = clockwise).
  - The dials spring back, so each drives an integrated knob. Full deflection sweeps its range in
    3 s. **The knobs reset if the bridge restarts.**
  - `--profile flocking`: right dial = spacing (`angular.x`, 0.6–1.4), left dial = `s2` (Unity
    gimbal).
  - `--profile sim`: flocking's dials, with spacing 0.4–1.6. The Unity sim uses `angular.x`
    **unclamped as its spread** (Olfati-Saber `d_ref`), and readController.py's pot spans
    0.4–1.6. Flocking's 0.6–1.4 is where `swarm_flocking.py` clamps `angular.x`.
    `--ax-range LO,HI` overrides any profile's range.
  - `--profile joystick`: the left (gimbal) dial drives `angular.x`, which `joystick_controller.py`
    reads as the gimbal.
  - C1 toggles `s1` (Unity panorama). C2 resets both knobs, **except under `--profile sim`**,
    where it is the experiment's identify button: it adds a `marks` field, a JSON int counting
    C2 presses since the bridge started, and leaves the knobs alone (a reset would snap the
    spread and the gimbal at the moment the pilot reports a target). A count rather than a
    level, as the RC sends it, so a press survives a lost or unread datagram. The flocking and
    joystick profiles never send `marks`.
  - `s2` never reaches ±1, because `joystick_controller.py` would read `int(s2) == 1` as LAND.
- **Hardware-free end-to-end check:** run `fake-rc` → `bridge --rc 127.0.0.1` → the receiver
  smoke test. **Use spare ports**, e.g. `fake-rc --bind 127.0.0.1 --port 5170` and
  `bridge --rc 127.0.0.1 --port 5170 --out 127.0.0.1:5155`. Never point a test bridge at
  `:5055` while a controller is running: it would fly the swarm.

## Testing with the swarm software

The bridge replaces `readController.py` one for one: the same JSON on the same port
(`127.0.0.1:5055`), at the same 20 Hz. The command that replaces it, from `rc-joystick\pc`, is:

```
python -m rcjoy bridge --profile sim     # for the Unity sim (spacing 0.4-1.6, like the Taranis)
python -m rcjoy bridge                   # for AOS server (spacing 0.6-1.4)
```

**No IP needed.** Without `--rc` the bridge (and the monitor) tries two things at once: a
broadcast on the switch, and a unicast to the RC it used last. Every RC it locks onto is saved in
`%LOCALAPPDATA%\rcjoy\last_rc.txt`. So if the broadcast's reply is blocked (Windows' firewall can
do that), `--rc <RC-IP>` **once** is enough, and every later launch finds the RC by itself. The
RC shows its address in a chip at the top of its screen. If several RCs running LIS_CONTROLLER
answer, a pop-up asks which one. The fleet's RCs never answer, so flying the swarm doesn't bring
it up. A `--rc` on this PC (a loopback fake RC) is never saved.

It is plain Python ≥ 3.7, stdlib only, so it needs no conda env and no pygame. To launch it from
another repo, run it with `rc-joystick\pc` as the working directory, or set `PYTHONPATH` to that
folder. **Never run it alongside `readController.py`**: both would send to :5055.

What the simulator does with each field (`vr_swarm_simulation`, `InputManager.cs` and
`UDPReceiverManager.cs`):

| Field | Sim input | Bridge (Mode 2) | Different from the Taranis |
|---|---|---|---|
| `linear.x` / `linear.y` | pitch / roll | right stick up / right | – |
| `linear.z` | throttle | left stick up, `v·abs(v)` | – |
| `angular.z` | yaw | left stick right | – |
| `angular.x` | **spread = Olfati-Saber `d_ref`**, used unclamped | right dial drives a knob | the dial springs back, so the knob **holds** its value when released. `--profile sim` gives readController.py's 0.4–1.6 |
| `switches.s1` | `userSwitch`: panorama on (+1) / feeds (−1), acted on when it **changes** | C1 toggles it; starts at +1 | – |
| `switches.s2` | FPV gimbal pitch, `SetGimbalPitchNormalized` over −1..+1 | left dial drives a knob, ±0.999, starts at 0 | the pitch moves **while** the dial is turned and holds when released. It starts at mid-range, not wherever a Taranis dial happened to be |
| `marks` (sim profile only) | `ExperimentRecorder`: each increase is an **identify** (the moment the pilot reports a target; the experimenter's 1/2/3 key then gives its outcome) | C2 adds 1; starts at 0 | new. The other profiles send no `marks`, and there C2 resets both knobs (spread 1.0, pitch mid) |

Two things the simulator does that matter for these tests:

- **It keeps the last packet.** `sharedJoystickData` is only overwritten by a new datagram. When
  the bridge stops sending (RC dropped, app stopped, cable pulled), the sim **keeps flying the
  last stick values**. The Taranis path behaved the same way. A sim-side timeout that zeroes the
  input after ~0.5 s would fix it.
- **It reads one datagram per frame.** At 20 Hz from the bridge, the scene must run at ≥ 20 fps,
  or a backlog builds and the input lags more and more.

### Before every session

1. RC: boot, force-stop DJI Fly ([as above](#dji-fly-on-the-joystick-rc)), open LIS_CONTROLLER.
   Circle a stick: the screen must read **GAMEPAD FAST**. If it reads SLOW, reboot the RC.
2. PC on the swarm switch.

### A. The bridge on its own (5 min)

Run `python -m rcjoy bridge --profile sim`, adding `--rc <RC-IP>` the first time if it keeps
saying `searching for the RC`. It prints `SENDING to
127.0.0.1:5055`, then one status line, e.g. `f+0.00 r+0.00 c+0.00 y+0.00 ax1.00 s2+0.00 s1+1`.

1. Right stick up → `f+1.00`; right → `r+1.00`. Left stick up → `c+1.00` (half up → `c+0.25`);
   left stick right → `y+1.00`.
2. Right dial turned right → `ax` climbs to `ax1.60` over ~3 s and stays there when released;
   turned left → down to `ax0.40`.
3. Left dial → `s2` moves toward ±0.999 and stays.
4. C1 → `s1` flips between `s1+1` and `s1-1`. C2 → a `[bridge] C2 mark #N` line, and `ax` /
   `s2` stay where they were (without `--profile sim`: `ax1.00`, `s2+0.00`).
5. Pull the RC's Ethernet → `NOT SENDING - stale …` within 0.3 s. Plug it back in → `SENDING`
   again within a few seconds.

Any other sign or field than listed is a mapping error: note it and tell me. The fix belongs in
the bridge's mapping (`STICK_MODES`, `PROFILES`), not in Unity.

### B. The Unity simulator

In the scene, `InputManager`'s Input Mode must be **JOYSTICK** (or ANY), and
`UDPReceiverManager.port` must be 5055. Start the bridge as in A, then press Play.

1. **Translation:** right stick up flies forward and right flies right, in the scene's command
   frame, as with the Taranis. Release: the swarm stops.
2. **Climb:** left stick up climbs, gently near the centre (the curve is quadratic).
3. **Yaw:** left stick right turns clockwise.
4. **Spread:** right dial right → the swarm spreads out, and holds its spacing when you let go;
   left → it tightens. The inspector's `d_ref` follows 0.4–1.6.
5. **Gimbal:** left dial → the FPV cameras pitch while you turn it, and stay put when you let go.
   C2 leaves both the spread and the pitch where they are.
6. **Panorama:** C1 switches between the panorama and the feeds each press. No change at start.
7. **Lag:** fly for 2–3 minutes, then release everything. The swarm must stop at once. If the
   response lags more and more, the scene is below 20 fps (see above).
8. **Dropout:** while flying forward, pull the RC's Ethernet. The sim keeps flying the last
   command (see above). Plug it back in: control returns. Also try Stop on the RC.
9. **Bridge restart:** stop and restart the bridge. The spread goes back to 1.0 and the pitch to
   mid-range: the knobs are not saved.
10. **MSDK fallback:** on the RC, swipe up from the bottom edge and tap Home. The bridge warns
    `sticks via MSDK, ~10 Hz`, and the sim still flies, more coarsely. Bring the app back: the
    warning clears.

### C. AOS server, afterwards

Use the bridge's default profile (`python -m rcjoy bridge`), since `swarm_flocking.py` clamps
`angular.x` to 0.6–1.4.

1. With no controller running, run `python udp_joystick_receiver.py` in `AOS server/`. The same
   moves as in A must show on its printout. Then with `--profile joystick`: the **left** dial now
   drives `angular.x`, which `joystick_controller.py` reads as the gimbal.
2. Run `swarm_flocking.py --dry-run` directly, not through `dji-flocking.ps1`, which would also
   start `readController.py`. It needs DroneSwarmServer running and an elevated shell, plus
   `dji-gui.ps1`. With the bridge streaming, the GUI shows no NO JOYSTICK chip: press Start. Pull
   the RC's Ethernet: `JOYSTICK_LOST`, then `JOYSTICK_LOST_STOP` about 3.8 s later. Repeat,
   plugging it back within 3 s: `back after …`, and no stop.

## Checks still to run (go/no-go)

Do these with the spare RC Pro. Force-stop DJI Fly first, and turn Wi-Fi and Bluetooth off
(airplane mode if the RC exposes it).

| # | Check | Pass | Result |
|---|---|---|---|
| Z1 | **RF emission while unlinked.** Use a spectrum analyser at 2.4/5.8 GHz. Or, during a hover flown with the Taranis, power the RC Pro on and off beside the RC cluster at least 3 times, 2 min each, and compare `link_sq/down/up` in `swarm_debug.csv` (only logged while swarming). | no measurable change | not run |
| Z2 | **Idle.** Leave the RC on, unlinked and untouched for 20 min. | no alert, power-off or sleep | not run |
| Z3 | **Pairing.** Power each non-fleet airframe near the RC one at a time. | none links | not run |
| S1 | Registers once online, then again offline after a reboot | offline OK | **pass** (2026-09-25: `SDK registered` with Wi-Fi off) |
| S2 | Sticks and dials served with no link. Record ranges and signs, and whether the dials spring to 0. **Check whether the right control reports via `KeyRightDial` or `KeyScrollWheel`, and whether it is a spring-return dial or an incremental wheel.** | 4 sticks + 2 dials | **Pass, both ways.** MSDK pushes all four sticks and both dials (`KeyRightDial`; `KeyScrollWheel` stays silent) with no aircraft, ±660. The built-in gamepad serves them too. Signs cross-checked in 1.5 with each control held at full deflection: all six agree, + = up / right / turned right, so `STICK_SIGN` / `DIAL_SIGN` stay +1. Both dials are spring-return. |
| S3 | Listener rate while sweeping a stick; whether unchanged values repeat; async `getValue` round-trip | ≥ 20 Hz (10–20 workable) | **MSDK: 6–10 changes/s per axis** (on change only). **Gamepad: ~70 reports/s** (14–16 ms apart, all axes per report, nothing while still), in its fast mode; in its **slow mode ~10/s** (~100 ms apart), for a whole morning on 2026-09-28 (below). **At the PC over Ethernet** (2026-09-28, `monitor --rate 100`, stick circling): **57–60 fresh samples/s** while the app counted 65–68, with 0 lost. The ceiling is the RC's 60 Hz screen: Android hands input to the app once per frame. In slow mode it is ~10/s end to end, and the kernel says the same. Async `getValue` of the RC's keys: `REQUEST_HANDLER_NOT_FOUND` at first, then `KeyConnection` answers in 0–30 ms. |
| S4 | Which buttons reach MSDK | C1 + C2 | **C1 and C2** (MSDK, with it registered; the gamepad reports them only while MSDK is not registered). DJI's own `com.dpad.service` also sees them and rebroadcasts `dpad-service.keys.ACTION_KEY_C1`/`C2`. |
| S5 | Ethernet in airplane mode: unicast echo **and** a PC broadcast received | both | **Unicast: pass** (2026-09-28): 100.0 states/s, 0 lost in every run, the RC sending every 10 ms, the PC's worst gap 20–28 ms. The RC kept its IP across an Ethernet replug, and the stream was back ~3 s after the adapter was. **Broadcast discovery: not recorded yet.** |
| S6 | `FlightControllerKey.KeyConnection` false; log `KeyPairingStatus` and `KeyIsStillLinked` | false | partly answered. **`ProductKey.KeyConnection` is TRUE with no aircraft**: MSDK counts the RC's own link as product `UNRECOGNIZED` (`onProductConnect(0)`). That caused 1.1's false AIRCRAFT LINKED, so the interlock no longer uses it. `FlightControllerKey.KeyConnection` and `KeyIsStillLinked` are **never served** with no aircraft (null, read as not linked); `KeyPairingStatus` = `UNPAIRED`. The RC's last pairing partner, per `persist.dji.uav`, is a **`wm260`** (Mavic 3), which matters for Z3. |

## The RC Pro's built-in gamepad

The RC Pro also presents its controls to Android as a gamepad:

- USB device **"DJI DJI Virtual Joystick"**, vendor `0x2ca3`, product `0x1501`, on the RC's
  internal USB hub next to **"DJI pigeon"** (the radio module). Full speed, Xbox 360 protocol
  (Linux `xpad` driver, 2 ms interrupt endpoints). Android names it **"DJI embedded joystick"**
  (`/dev/input/event8`, generic key layout).
- It reports with **no aircraft**, with or without MSDK (`adb shell "getevent -lt
  /dev/input/event8"`, 2026-09-25).
- Android delivers it to the **focused** window only, as joystick `MotionEvent`s and gamepad
  `KeyEvent`s, so an app must be **in the foreground** to read it.

**DJI's gate.** On this firmware (RM510, 03.02.0300) DJI's modified framework filters the device's
events before any app sees them: `com.dji.comkey.ComKeyManager.interceptMotion`, inside
system_server, read from `services.jar`. An event reaches an app only if:

- for a stick axis, `dji_motion_via_left_joystick_enabled` / `_right_` is `"1"` (the 5D pad has
  `dji_motion_via_five_dimen_enabled`; the dials are not gated here);
- **and** `DjiServicesHelper.enableMotion(<focused package>)` is true: the package is not in the
  framework's `config_FocusOffApps` list (DJI Fly, DJI Pilot, Agras…) **and** either it is a system
  app or the global setting **`dji_lab_game_mode`** is on. That is a "Lab → game mode" switch with
  no screen in this firmware's Settings.

All three were off on this RC (game mode unset), so 1.0–1.4 received nothing at all while the
kernel saw ~70 reports/s. `android\enable-gamepad.ps1` turns them on. With them off, the app works
from MSDK alone.

**Two report modes.** The virtual joystick sends a report on every change, either ~14 ms apart
(**fast mode**, ~70/s) or ~100 ms apart (**slow mode**, ~10/s). It is the controller behind the
USB device that chooses: the kernel's own timestamps show the same two rates, and the app,
Ethernet and the PC pass on every report either way. In slow mode the gamepad is only as fast as
MSDK, never slower.

Observations:

- **2026-09-25:** fast 15:11–15:38 across several app restarts; slow 15:58–16:02, in the run
  after an `install -r`; fast again at 16:46, after an app restart that followed 36 min
  untouched. All three were in one boot.
- **2026-09-28:** slow from the first reading at 10:53 (on USB, before Ethernet), through app
  restarts, screen off/on, the C/N/S switch, toggling DJI's gate, re-plugging the Ethernet
  adapter, DJI Fly, a reinstall and a reboot. Fast at 11:25, after the app had been uninstalled,
  reinstalled and registered again. Fast after **every** later boot (11:41, 13:15), from the
  first stick movement.

Ruled out as the switch: Wi-Fi, USB versus Ethernet, charging, battery level (slow at 70%, fast at
31% and 58%), DJI's status message to the controller (the same payload in both modes), MSDK's
connection (both fast and slow with it up), a force-stop of the app, and DJI Fly. The controller
never re-enumerated between modes, so it switches at runtime. **The trigger is still unknown.**

**Operating rule until it is found:** check the mode before a flight. Circle a stick and read the
app (GAMEPAD **FAST**, or an amber "slow mode · reboot the RC") or the monitor's `rates/s`. If it
is slow, **reboot the RC**: every boot on record came up fast. Every trace records the mode
(`gamepad reports/s: …, median gap … ms (fast mode)`), and so do `info.gamepad.gap_ms`, the
monitor and its CSV.

**~60 fresh samples/s is the ceiling at the PC, not ~70.** The RC's screen runs at 60 Hz
(`HWDeviceDRM … FPS: 60` at boot), and Android delivers joystick input to the app once per
frame. When two reports land in one frame, the app sees only the newer one. Every consumer runs at
20 Hz, so this costs nothing: each 20 Hz command's input is at most ~17 ms old.

| Control | Android axis / key (kernel code) | Raw range | Direction |
|---|---|---|---|
| Left stick H / V | `AXIS_X` / `AXIS_Y` (`ABS_X` / `ABS_Y`) | ±32767, flat ±128 | right = **+**, up = **−** (HID convention; `GamepadInput` flips V for the wire's up = +) |
| Right stick H / V | `AXIS_RX` / `AXIS_RY` (`ABS_RX` / `ABS_RY`) | ±32767, flat ±128 | right = **+**, up = **−** |
| Left dial | `AXIS_Z` (`ABS_Z`) | 0–255, rest 127 | turned right → 254 |
| Right dial | `AXIS_RZ` (`ABS_RZ`) | 0–255, rest 127 | turned right → 254 |
| C1 / C2 | `BUTTON_L1` / `BUTTON_R1` (`BTN_TL` / `BTN_TR`) | key | press / release; with MSDK registered they go to MSDK instead |
| 5D up / down / left / right | `AXIS_HAT_Y` −1 / +1, `AXIS_HAT_X` −1 / +1 | −1..1 | |
| 5D press | `BUTTON_THUMBL` (`BTN_THUMBL`) | key | |

Resolution is 16-bit on the sticks and 8-bit on the dials. On the wire both become ±660, the
dials reaching ±655 at the 8-bit end stop. Record, shutter, RTH and Pause produce no gamepad
events.

## MSDK 5.3.0 `RemoteControllerKey` inventory

Read on 2026-09-24 with `javap` from `dji-sdk-v5-aircraft-provided-5.3.0.jar` (on Maven Central).
As with the AirLink keys, almost all of these are declared on an **obfuscated base class**
(`co_v`) and reached through inheritance. The public `RemoteControllerKey` itself declares only
`KeyControlMode` and `KeyMultiControlChannel`.

| Purpose | Keys (value type) |
|---|---|
| Sticks | `KeyStickLeftHorizontal`, `KeyStickLeftVertical`, `KeyStickRightHorizontal`, `KeyStickRightVertical` (Integer) |
| Dials | `KeyLeftDial`, `KeyRightDial`, `KeyScrollWheel` (Integer) |
| Buttons | `KeyCustomButton1Down`/`2`/`3`, `KeyRCCustomButton4Down`, `KeyShutterButtonDown`, `KeyRecordButtonDown`, `KeyGoHomeButtonDown`, `KeyPauseButtonDown`, `KeyRCSwitchButtonDown`, `KeyRCPlaybackButtonDown`, `KeyRCRightWheelButtonDown` (Boolean); `KeyFiveDimensionPressedStatus` (up/down/left/right/middle) |
| Aggregate | `KeyRcHardwareState` (`RcCustomButtonHardwareStatus`: C1–C4 clicks + the 5D pad; **buttons only, no sticks**) |
| C/N/S switch | `KeyFlightModeSwitchState` (`RCFlightModeSwitch`: `SWITCH_ONE`/`TWO`/`THREE`), **RC-side**, so possibly served while unlinked |
| Health / link | `KeyConnection`, `KeyIsLeft…/Right…StickNormal` ×4, `KeyIsStillLinked`, `KeyPairingStatus` (`PairingState`) |
| Info | `KeyRemoteControllerType`, `KeySerialNumber`, `KeyFirmwareVersion`, `KeyBatteryInfo`, `KeyControlMode` |

The app only **reads** these keys. It never calls `setValue` or `performAction`, so it can never
change the RC's radio or pairing.

**MSDK itself, though, attempts a few internal writes at startup** (SDR link-loss prevention,
LTE). On the first RC Pro run with no aircraft, they failed with `REQUEST_HANDLER_NOT_FOUND`, as
did async reads of the RC's own keys (`FirmwareVersion`, `SerialNumber`): MSDK 5.3.0 has no
request handlers for component 4 (the RC) while the product is `UNRECOGNIZED` (id 0). The
**pushed** values arrive regardless (S2): sticks, dials, C1/C2, the 5D pad, the C/N/S switch,
battery, serial and firmware, at 6–10 changes a second while a stick moves.
