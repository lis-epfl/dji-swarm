# LIS_CONTROLLER — wire protocol v1

A dedicated DJI RC Pro, never linked to an aircraft, runs the **LIS_CONTROLLER** app (package
`com.liscontroller`). It reads the sticks and dials from the RC's built-in gamepad while it can,
and everything else (and the sticks' fallback) through DJI MSDK. The app streams them to any PC
that asks. This document is the contract between that app
(`rc-joystick/android/`, `Protocol.java`) and the PC package (`rc-joystick/pc/rcjoy/`,
`protocol.py`). Keep all three in step.

## Transport

- UDP, one UTF-8 JSON object per datagram.
- The **RC is the server.** It listens on **UDP 5070** on all interfaces, which in practice means
  eth0, the same switch the swarm RCs use. It sends only to addresses that subscribed. This gives:
  - nothing to configure on the RC
  - no inbound Windows-firewall rule on the PC, because the replies ride the PC's own outbound flow
  - zero traffic when nobody is listening
- Every message carries `"type"` and `"v": 1`. A receiver **rejects** any other `v` with a clear
  message; it never guesses at an unknown version's fields. Within v1, unknown extra fields are
  ignored, so fields may be added without a version bump. Removing or retyping one needs `v: 2`.

## PC → RC

### `subscribe` — sent every 1 s

```json
{"type":"subscribe","v":1,"client":"rcjoy-bridge","rate_hz":50,"t":12345.678}
```

| field | meaning |
|---|---|
| `client` | free-text name shown on the RC's subscriber list |
| `rate_hz` | requested `state` rate; the RC clamps it to 10–100 |
| `t` | the PC's clock at send time, echoed back in `info.echo_t` for RTT |

- The RC keys subscribers by source `ip:port`.
- A subscription **expires 3 s** after its last `subscribe`.
- The RC replies to every `subscribe` at once with an `info`.
- **Discovery** is the same message sent to `255.255.255.255:5070`, once from each local IPv4
  address. Whoever answers with `info` is an RC.

### `unsubscribe`

```json
{"type":"unsubscribe","v":1}
```

Removes the sender's `ip:port` at once. This is optional; the TTL covers a PC that just vanishes.

## RC → PC

### `state` — the stream

The RC sends one `state` to every live subscriber, at the highest `rate_hz` any live subscriber
asked for. On a button press it may send early, which is why `presses` counts are used rather
than button levels.

```json
{"type":"state","v":1,"seq":1234,"t_ms":5551234,
 "rc_ok":true,"aircraft_linked":false,"stick_src":"gamepad",
 "sticks":{"lh":0,"lv":-12,"rh":660,"rv":330},
 "dials":{"l":0,"r":null},
 "buttons":{"c1":false,"c2":false,"c3":null,"shutter":false,"record":false,
            "rth":null,"pause":null,"rc_switch":null,
            "5d_up":null,"5d_down":null,"5d_left":null,"5d_right":null,"5d_press":null},
 "presses":{"c1":3,"c2":0,"shutter":1,"record":0},
 "mode_switch":null}
```

| field | meaning |
|---|---|
| `seq` | +1 per state **tick**, not per input sample; the same value goes to every subscriber. It restarts at 0 when the app starts. A tick carries whatever input is current, so consecutive states can repeat one sample, and a sample replaced before the next tick is never sent. |
| `t_ms` | the RC's monotonic clock (`SystemClock.elapsedRealtime`); for diagnostics only |
| `rc_ok` | `true` only when **all** of: MSDK reports the RC connected (`RemoteControllerKey.KeyConnection`); `aircraft_linked` is false; and a read-only async `getValue` of the RC succeeded within the last 500 ms. That last one is liveness: change-driven listeners are silent while a stick is held still. Which keys the firmware answers asynchronously is not documented, and on the RC Pro `KeyConnection`'s read was rejected. So the app tries candidates in order (`KeyConnection`, `KeySerialNumber`, `KeyBatteryInfo`, `KeyStickLeftHorizontal`) and keeps the first that answers. If **none** ever answers, liveness is dropped, and `KeyConnection` alone decides (the app's screen and trace say so). |
| `aircraft_linked` | This RC's sticks are flying an aircraft right now: `FlightControllerKey.KeyConnection` is true, **or** the product is connected **and** its type names an actual aircraft. **Not** `ProductKey.KeyConnection` alone: on an RC Pro with no aircraft at all, MSDK reports the RC's own link as a connected product of type `UNRECOGNIZED` (`onProductConnect(0)`, seen 2026-09-25). It forces `rc_ok = false`, and the PC must treat it as **blocked**, never as input. |
| `stick_src` | where this packet's `sticks` and `dials` came from: `"gamepad"` (the RC's built-in gamepad, up to ~70 reports/s) or `"msdk"` (~10 changes/s: the app is not in front, DJI's gamepad gate is closed, or the gamepad went silent while MSDK saw motion). Both are **usable**; `msdk` is only slower, and the PC warns. Added in app 1.5; absent (read as unknown) from older apps. |
| `sticks` | physical sticks `lh` `lv` `rh` `rv` (left/right, horizontal/vertical), raw **±660**. **+ = up / right.** The app converts both sources to this convention (checked against each other on the RC Pro, 2026-09-25), so the PC never flips a sign. On the MSDK path the app sends `null` for an axis its cross-check found reading **inverted** against the gamepad: the PC then has no joystick, rather than one axis flying backwards. |
| `dials` | `l` (the RC's gimbal dial) and `r`, raw ±660, spring-return. **+ = turned right / clockwise.** |
| `buttons` | current level, `true` = held |
| `presses` | **cumulative rising-edge counts** since the app started, so a press that falls between two packets is never lost. The PC acts on deltas. It takes a baseline on the first packet after lock-on or after a stale period (so a press is never applied late), and re-baselines when a count goes down (the app restarted). |
| `mode_switch` | the C/N/S switch as `"ONE"`/`"TWO"`/`"THREE"`, or `null`. Source: the RC-side `RemoteControllerKey.KeyFlightModeSwitchState` (`RCFlightModeSwitch`, present in MSDK 5.3.0), so it may be served while unlinked; the spike decides. It is informational only: the operator starts and stops the swarm from the GUI, and no switch is ever wired to swarm state. |

**`null` means the key is not served**, as distinct from zero or false. A PC must treat a `null`
**stick** as "no valid input". A missing stick is not a centred stick. A `null` dial or button only
disables what it drives.

### `info` — the reply to every `subscribe`

This doubles as the discovery answer and the RTT echo, and must fit in **1200 bytes** so it is
never fragmented.

```json
{"type":"info","v":1,"echo_t":12345.678,"app":"1.0",
 "rc_type":"RM510","sn":"5YSZK...","fw":"01.02.0300","battery":87,"stick_mode":"MODE_2",
 "eth_ip":"192.168.100.50","port":5070,"rate_hz":50,"subscribers":2,
 "keys":{"lh":{"n":8123,"age_ms":12},"r":{"n":0,"age_ms":null}},
 "rf":{"wifi":false,"wifi_scan":false,"bt":false,"ble_scan":false,"airplane":true,
       "aircraft_linked":false},
 "gamepad":{"hz":69,"gap_ms":14,"why":null,"gate":true,
            "verified":["lh","lv","rh","rv","l","r"],"inverted":[]}}
```

| field | meaning |
|---|---|
| `echo_t` | the `t` of the `subscribe` this answers |
| `battery` | RC battery, percent, or `null` |
| `stick_mode` | what MSDK reports, for display only. The **PC's** `--stick-mode` decides the mapping, since the sticks are reported physically. |
| `keys` | one entry per MSDK key the app listens to, by short name: `n` = listener callbacks since the app started, `age_ms` = time since the last one (`null` if never). This shows which keys the firmware actually serves. Names are the wire field names where one exists (`lh` … `rc_switch`, `mode_switch`), plus `5d` for the one MSDK key behind the five `5d_*` buttons, `rc_conn` for the RC's `KeyConnection`, and `wheel` for `KeyScrollWheel`. The last is diagnostic: it may be what the RC Pro's right-hand control reports through. |
| `rf` | Android's radios: `wifi`, `bt`, plus `wifi_scan` / `ble_scan`, the Location-settings scanning modes. Those transmit (Wi-Fi probe requests, BLE scans) even with Wi-Fi or Bluetooth "off". Also `airplane` and the aircraft link. Airplane mode does **not** cover the RC's OcuSync radio. RF quiet is measured, not assumed (see README). |
| `gamepad` | the built-in gamepad path (app 1.5+): `hz` = reports in the last second (0 while nothing moves); `gap_ms` = median gap between reports while moving, which tells the device's fast mode (~14) from its slow one (~100), `null` until known; `why` = why the sticks are on MSDK, `null` when they are on the gamepad; `gate` = DJI's gate settings all on (`true`), one off (`false`), unreadable (`null`); `verified` / `inverted` = axes the cross-check has found agreeing / disagreeing in sign with MSDK while both were held still. |

### `bye` — on shutdown

```json
{"type":"bye","v":1,"reason":"app stopped"}
```

The PC goes stale at once instead of waiting for its timeout.

## PC-side semantics, fixed by this contract

- **Fresh** = the locked RC's last `state` is younger than 0.3 s (`stale_after`) and no `bye` has
  arrived since.
- Input is **usable** only while it is fresh, `rc_ok` is true, `aircraft_linked` is false, and all
  four sticks are non-null. Anything else is exactly "no joystick": the bridge **sends nothing**.
  It never pads a gap with neutral packets, so the swarm's arm gate and its dropout policy
  (neutral sticks for 3 s, then auto-STOP) see the truth.
- After locking onto an RC, the PC accepts `state`, `info` and `bye` **only from that RC's
  `ip:5070`**. Anything else on the LAN is dropped and counted.
- **Input freshness is measured on the PC, not sent.** v1 has no per-sample counter. A `state`
  whose `sticks` or `dials` differ from the previous one is **fresh**. The gamepad reports only
  on change, so while a stick moves the fresh rate is the rate at which input reaches the PC. It
  is bounded by the tick rate, so measuring the gamepad's ~70/s needs `rate_hz` 100. In practice
  it tops out at **~60/s** (2026-09-28): the RC's Android hands joystick input to the app once per
  60 Hz screen frame, so two reports in one frame arrive as one.
