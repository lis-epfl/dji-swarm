"""FakeRc: the RC side of protocol v1, driven by synthetic inputs.

    python -m rcjoy fake-rc [--bind IP] [--port 5070] [--pattern steps|sweep|still]
                            [--input-hz 70] [--aircraft-linked] [--no-rc-ok]
                            [--null lv,r,...] [--msdk-sticks]

Stands in for the Android app so the PC package, the bridge and the launchers
can be tested with no hardware. The `steps` pattern moves one physical control at
a time and prints what it did, so the bridge's output can be checked for the
right field and sign. The `sweep` pattern moves every input continuously, stepping
--input-hz times a second like the RC's gamepad reporting, so `monitor --rate 100`
can be checked for reading that rate back as `fresh`. It mirrors what the real app
must do (see PROTOCOL.md), so keep the two in step.
"""

import math
import socket
import threading
import time

from . import protocol as P


def sweep_inputs(t):
    """The sweep pattern's raw inputs at time t: the right stick circling once a second,
    as in the Ethernet rate check, so every sample moves it; the rest on slow sines."""
    a = 2.0 * math.pi * t
    return {"rh": int(660 * math.cos(a)), "rv": int(660 * math.sin(a)),
            "lh": int(660 * math.sin(t * 0.7)), "lv": int(660 * math.sin(t * 0.5)),
            "l": int(660 * math.sin(t * 0.3)), "r": int(660 * math.sin(t * 0.4))}


class FakeRc:
    def __init__(self, bind="0.0.0.0", port=P.DEFAULT_PORT, version=P.VERSION):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.ip, self.port = self.sock.getsockname()
        self.version = version          # a wrong one exercises the PC's rejection
        self.lock = threading.Lock()
        self.subscribers = {}           # (ip, port) -> {"client", "rate", "last"}
        self.seq = 0
        self.sticks = {k: 0 for k in P.STICKS}
        self.dials = {k: 0 for k in P.DIALS}
        self.buttons = {k: False for k in P.BUTTONS}
        self.presses = {k: 0 for k in P.BUTTONS}
        self.null_keys = set()          # protocol field names reported as null
        self.rc_ok = True
        self.aircraft_linked = False
        self.stick_src = "gamepad"      # "msdk" = the app's ~10 Hz fallback
        self.gamepad_why = None         # why the gamepad is not in use (with "msdk")
        self.inverted = []              # axes the app's cross-check found inverted
        self.muted = False              # stop streaming state, no bye (cable pulled)
        # Set: the inputs follow sweep_inputs(), stepping sweep_hz times a second the
        # way the gamepad reports, and each state carries whichever sample is current,
        # as the app's tx thread does. None: the inputs are whatever set() last put.
        self.sweep_hz = None
        self.gamepad_hz = 70            # what info.gamepad.hz claims
        self.fresh_sent = 0             # states sent whose sticks/dials changed
        self._last_inputs = None
        self.battery = 87
        self.rf = {"wifi": False, "wifi_scan": False, "bt": False, "ble_scan": False,
                   "airplane": True}
        self.on_subscriber_change = None
        self._t0 = P.now()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._threads = []

    # --- inputs --------------------------------------------------------------------

    def set(self, **raw):
        """set(rv=660, l=-330, ...): raw +-660 values by protocol field name."""
        with self.lock:
            for k, v in raw.items():
                if k in self.sticks:
                    self.sticks[k] = v
                elif k in self.dials:
                    self.dials[k] = v
                else:
                    raise KeyError(k)

    def centre(self):
        with self.lock:
            for d in (self.sticks, self.dials):
                for k in d:
                    d[k] = 0

    def press(self, name, n=1):
        with self.lock:
            self.presses[name] += n
        self._wake.set()                # the real app sends early on an edge

    # --- messages ------------------------------------------------------------------

    def _val(self, k, v):
        return None if k in self.null_keys else v

    def build_state(self):
        with self.lock:
            if self.sweep_hz:
                n = int((P.now() - self._t0) * self.sweep_hz)
                for k, v in sweep_inputs(n / float(self.sweep_hz)).items():
                    (self.sticks if k in self.sticks else self.dials)[k] = v
            linked = self.aircraft_linked
            return {
                "type": "state", "v": self.version, "seq": self.seq,
                "t_ms": int((P.now() - self._t0) * 1000),
                "rc_ok": self.rc_ok and not linked, "aircraft_linked": linked,
                "stick_src": self.stick_src,
                "sticks": {k: self._val(k, v) for k, v in self.sticks.items()},
                "dials": {k: self._val(k, v) for k, v in self.dials.items()},
                "buttons": {k: self._val(k, v) for k, v in self.buttons.items()},
                "presses": {k: v for k, v in self.presses.items()
                            if k not in self.null_keys},
                "mode_switch": None,
            }

    def build_info(self, echo_t):
        with self.lock:
            rate = self._rate_locked()
            keys = {k: ({"n": 0, "age_ms": None} if k in self.null_keys
                        else {"n": 1234, "age_ms": 20})
                    for k in P.STICKS + P.DIALS + P.BUTTONS}
            return {
                "type": "info", "v": self.version, "echo_t": echo_t, "app": "fake",
                "rc_type": "FAKE", "sn": "FAKE0001", "fw": "00.00.0000",
                "battery": self.battery, "stick_mode": "MODE_2",
                "eth_ip": self.ip, "port": self.port, "rate_hz": rate,
                "subscribers": len(self.subscribers), "keys": keys,
                "rf": dict(self.rf, aircraft_linked=self.aircraft_linked),
                "gamepad": {
                    "hz": self.gamepad_hz if self.stick_src == "gamepad" else 0,
                    "gap_ms": 14 if self.stick_src == "gamepad" else None,
                    "why": self.gamepad_why if self.stick_src != "gamepad" else None,
                    "gate": True, "verified": ["lh", "lv", "rh", "rv"],
                    "inverted": list(self.inverted)},
            }

    def _rate_locked(self):
        rates = [s["rate"] for s in self.subscribers.values()]
        return max(rates) if rates else P.RATE_DEFAULT_HZ

    # --- lifecycle -----------------------------------------------------------------

    def start(self):
        for target, name in ((self._rx, "fake-rc-rx"), (self._tx, "fake-rc-tx")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self, bye=True):
        self._stop.set()
        self._wake.set()
        # As the app does: no state may follow the bye, or the PC takes it as the
        # stream resuming and waits out stale_after instead of going stale at once.
        for t in self._threads:
            if t.name == "fake-rc-tx":
                t.join(timeout=1.0)
        if bye:
            with self.lock:
                subs = list(self.subscribers)
            msg = P.bye("fake rc stopped")
            msg["v"] = self.version
            for addr in subs:
                try:
                    self.sock.sendto(P.encode(msg), addr)
                except OSError:
                    pass
        try:
            self.sock.close()
        except OSError:
            pass
        for t in self._threads:
            t.join(timeout=1.0)

    def _rx(self):
        self.sock.settimeout(0.2)
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(P.RECV_BUFFER)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue        # Windows: ICMP unreachable from a subscriber that left
            except OSError:
                return
            try:
                msg = P.decode(data)
            except P.ProtocolError:
                continue
            if msg["type"] == "subscribe":
                with self.lock:
                    new = addr not in self.subscribers
                    self.subscribers[addr] = {
                        "client": str(msg.get("client", "?")),
                        "rate": P.clamp_rate(msg.get("rate_hz")),
                        "last": P.now()}
                if new and self.on_subscriber_change:
                    self.on_subscriber_change("+", addr, msg.get("client"))
                try:
                    self.sock.sendto(P.encode(self.build_info(msg.get("t"))), addr)
                except OSError:
                    pass
            elif msg["type"] == "unsubscribe":
                with self.lock:
                    gone = self.subscribers.pop(addr, None)
                if gone and self.on_subscriber_change:
                    self.on_subscriber_change("-", addr, gone["client"])

    def _tx(self):
        next_t = P.now()
        while not self._stop.is_set():
            now = P.now()
            with self.lock:
                for addr in [a for a, s in self.subscribers.items()
                             if now - s["last"] > P.SUBSCRIPTION_TTL_S]:
                    gone = self.subscribers.pop(addr)
                    if self.on_subscriber_change:
                        self.on_subscriber_change("x", addr, gone["client"])
                subs = list(self.subscribers)
                rate = self._rate_locked()
                muted = self.muted
            if subs and not muted:
                st = self.build_state()
                payload = P.encode(st)
                for addr in subs:
                    try:
                        self.sock.sendto(payload, addr)
                    except OSError:
                        pass
                inputs = (st["sticks"], st["dials"])
                with self.lock:
                    self.seq += 1
                    # Counted as the PC counts `fresh`, so the two can be compared.
                    if self._last_inputs is not None and inputs != self._last_inputs:
                        self.fresh_sent += 1
                    self._last_inputs = inputs
            next_t += 1.0 / rate
            delay = next_t - P.now()
            if delay > 0:
                self._wake.wait(delay)
                self._wake.clear()
            else:
                next_t = P.now()


# --- CLI patterns -------------------------------------------------------------------

STEP_S = 2.0
STEPS = (
    ("centre", {}, None),
    ("right stick UP     (Mode 2: forward  -> linear.x +)", {"rv": 660}, None),
    ("right stick RIGHT  (Mode 2: right    -> linear.y +)", {"rh": 660}, None),
    ("left stick UP half (Mode 2: climb    -> linear.z +0.25)", {"lv": 330}, None),
    ("left stick RIGHT   (Mode 2: yaw CW   -> angular.z +)", {"lh": 660}, None),
    ("right dial RIGHT   (flocking: angular.x rises to 1.4)", {"r": 660}, None),
    ("left dial LEFT     (flocking: s2 falls)", {"l": -660}, None),
    ("C1 press           (s1 flips)", {}, "c1"),
    ("C2 press           (knobs back to ax 1.0 / s2 0.0)", {}, "c2"),
)


def main(args):
    if args.input_hz <= 0:
        print("--input-hz must be > 0")
        return 2
    rc = FakeRc(bind=args.bind, port=args.port)
    rc.null_keys = set(k.strip() for k in (args.null or "").split(",") if k.strip())
    rc.aircraft_linked = args.aircraft_linked
    rc.rc_ok = not args.no_rc_ok
    if args.msdk_sticks:
        rc.stick_src = "msdk"
        rc.gamepad_why = "the app is not in front (fake)"
    if args.pattern == "sweep":
        rc.sweep_hz = args.input_hz
        rc.gamepad_hz = int(round(args.input_hz))   # the app sends an int
    else:
        rc.gamepad_hz = 0           # the real app reads 0 while nothing moves
    rc.on_subscriber_change = lambda how, addr, client: print(
        "[fake-rc] subscriber %s %s:%d (%s)" % (
            {"+": "joined", "-": "left", "x": "expired"}[how], addr[0], addr[1], client),
        flush=True)
    rc.start()
    print("[fake-rc] listening on %s:%d  pattern=%s%s%s%s%s%s  (Ctrl+C to quit)" % (
        rc.ip, rc.port, args.pattern,
        ("  input %g/s" % rc.sweep_hz) if rc.sweep_hz else "",
        "  AIRCRAFT-LINKED" if rc.aircraft_linked else "",
        "  rc_ok=false" if not rc.rc_ok else "",
        ("  null=" + ",".join(sorted(rc.null_keys))) if rc.null_keys else "",
        "  sticks=MSDK" if rc.stick_src == "msdk" else ""),
        flush=True)
    t0 = P.now()
    step = -1
    try:
        while True:
            t = P.now() - t0
            if args.pattern == "steps":
                i = int(t // STEP_S) % len(STEPS)
                if i != step:
                    step = i
                    label, raw, press = STEPS[i]
                    rc.centre()
                    if raw:
                        rc.set(**raw)
                    if press:
                        rc.press(press)
                    print("[fake-rc] %s" % label, flush=True)
            time.sleep(0.02)            # sweep: the tx thread samples sweep_inputs()
    except KeyboardInterrupt:
        pass
    finally:
        rc.stop(bye=True)
        print("\n[fake-rc] stopped (sent bye)")
    return 0
