"""FakeRc: the RC side of protocol v1, driven by synthetic inputs.

    python -m rcjoy fake-rc [--bind IP] [--port 5070] [--pattern steps|sweep|still]
                            [--aircraft-linked] [--no-rc-ok] [--null lv,r,...]
                            [--msdk-sticks]

Stands in for the Android app so the PC package, the bridge and the launchers
can be tested with no hardware. The `steps` pattern moves one physical control at
a time and prints what it did, so the bridge's output can be checked for the
right field and sign. It mirrors what the real app must do (see PROTOCOL.md), so
keep the two in step.
"""

import math
import socket
import threading
import time

from . import protocol as P


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
        self.battery = 87
        self.rf = {"wifi": False, "wifi_scan": False, "bt": False, "ble_scan": False,
                   "airplane": True}
        self.on_subscriber_change = None
        self._t0 = time.monotonic()
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
            linked = self.aircraft_linked
            return {
                "type": "state", "v": self.version, "seq": self.seq,
                "t_ms": int((time.monotonic() - self._t0) * 1000),
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
                    "hz": 70 if self.stick_src == "gamepad" else 0,
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
        self._stop.set()
        self._wake.set()
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
                        "last": time.monotonic()}
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
        next_t = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
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
                payload = P.encode(self.build_state())
                for addr in subs:
                    try:
                        self.sock.sendto(payload, addr)
                    except OSError:
                        pass
                with self.lock:
                    self.seq += 1
            next_t += 1.0 / rate
            delay = next_t - time.monotonic()
            if delay > 0:
                self._wake.wait(delay)
                self._wake.clear()
            else:
                next_t = time.monotonic()


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
    rc = FakeRc(bind=args.bind, port=args.port)
    rc.null_keys = set(k.strip() for k in (args.null or "").split(",") if k.strip())
    rc.aircraft_linked = args.aircraft_linked
    rc.rc_ok = not args.no_rc_ok
    if args.msdk_sticks:
        rc.stick_src = "msdk"
        rc.gamepad_why = "the app is not in front (fake)"
    rc.on_subscriber_change = lambda how, addr, client: print(
        "[fake-rc] subscriber %s %s:%d (%s)" % (
            {"+": "joined", "-": "left", "x": "expired"}[how], addr[0], addr[1], client),
        flush=True)
    rc.start()
    print("[fake-rc] listening on %s:%d  pattern=%s%s%s%s%s  (Ctrl+C to quit)" % (
        rc.ip, rc.port, args.pattern,
        "  AIRCRAFT-LINKED" if rc.aircraft_linked else "",
        "  rc_ok=false" if not rc.rc_ok else "",
        ("  null=" + ",".join(sorted(rc.null_keys))) if rc.null_keys else "",
        "  sticks=MSDK" if rc.stick_src == "msdk" else ""),
        flush=True)
    t0 = time.monotonic()
    step = -1
    try:
        while True:
            t = time.monotonic() - t0
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
            elif args.pattern == "sweep":
                rc.set(lh=int(660 * math.sin(t * 0.7)), lv=int(660 * math.sin(t * 0.5)),
                       rh=int(660 * math.sin(t * 0.9)), rv=int(660 * math.sin(t * 1.1)),
                       l=int(660 * math.sin(t * 0.3)), r=int(660 * math.sin(t * 0.4)))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        rc.stop(bye=True)
        print("\n[fake-rc] stopped (sent bye)")
    return 0
