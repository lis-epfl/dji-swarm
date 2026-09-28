"""RcJoystickClient: subscribe to the LIS RC Joystick app and keep its latest state.

    client = RcJoystickClient()            # auto-discover by broadcast
    client = RcJoystickClient("192.168.100.50")
    client.start()
    s = client.get_state()                 # RcState, or None = "no joystick"

get_state() has the same contract as AOS server's JoystickReceiver.get_state():
None unless the input is fresh AND usable (rc_ok, no aircraft link, all four
sticks served). not_ok_reason() says why in words.
"""

import collections
import select
import socket
import threading
import time

from . import protocol as P

RELOCK_AFTER_S = 10.0       # auto mode: a locked RC silent this long -> rediscover
DISCOVERY_WAIT_S = 0.5      # listen this long for answers after each discovery send
BATTERY_WARN_PCT = 25
RATE_WINDOW_S = 2.0
GAP_WINDOW_S = 10.0


def local_ipv4_addresses():
    """This host's non-loopback IPv4 addresses. Best effort, stdlib only."""
    addrs = set()
    try:
        for _fam, _t, _p, _c, sa in socket.getaddrinfo(
                socket.gethostname(), None, socket.AF_INET):
            addrs.add(sa[0])
    except OSError:
        pass
    return sorted(a for a in addrs if not a.startswith("127."))


class RcJoystickClient:
    def __init__(self, rc=None, port=P.DEFAULT_PORT, rate_hz=P.RATE_DEFAULT_HZ,
                 stale_after=0.3, client_name="rcjoy", local_addrs=None,
                 discovery_targets=None):
        """
        rc           None or 'auto' = discover by broadcast; else the RC's IP or host name.
        port         the RC's UDP port.
        rate_hz      requested state rate (the RC clamps to 10..100).
        stale_after  seconds without a state before get_state() returns None.
        local_addrs, discovery_targets
                     auto-mode overrides, mainly for the selftest: the local IPv4s to
                     open a discovery socket on, and the (ip, port) list every
                     discovery subscribe goes to (default: 255.255.255.255:port).
        """
        self.auto = rc is None or str(rc).lower() == "auto"
        self.port = int(port)
        self.rate_hz = P.clamp_rate(rate_hz)
        self.stale_after = float(stale_after)
        self.client_name = client_name
        self._rc_ip = None if self.auto else socket.gethostbyname(str(rc))
        self._local_addrs = local_addrs
        self._targets = discovery_targets

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._threads = []
        self._socks = []
        self._locked = None             # RC ip we accept packets from
        self._lock_sock = None          # the socket that reaches it
        self._candidates = {}           # discovery round: ip -> (sock, info, t)
        self._multi = []                # several RCs answered the last round
        self._state = None
        self._info = None
        self._bye = None                # (t, reason) of the last bye
        self._last_rx = 0.0
        self._last_seq = None
        self._arrivals = collections.deque()
        self._gaps = collections.deque()
        self._packets = 0
        self._lost = 0
        self._foreign = 0
        self._bad = 0
        self._rtt = None
        self._warned = set()
        self._events = collections.deque(maxlen=50)
        self._press = P.PressCounter()
        self._press_cbs = []

    # --- lifecycle ---------------------------------------------------------------

    def start(self):
        if self.auto:
            addrs = (self._local_addrs if self._local_addrs is not None
                     else local_ipv4_addresses())
            for a in addrs:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    s.bind((a, 0))
                    self._socks.append(s)
                except OSError as e:
                    self._event("cannot open a discovery socket on %s: %s" % (a, e))
            if not self._socks:
                # No usable local address list: one wildcard socket, OS routing.
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                s.bind(("0.0.0.0", 0))
                self._socks.append(s)
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("0.0.0.0", 0))
            self._socks.append(s)
            with self._lock:
                self._locked, self._lock_sock = self._rc_ip, s
                self._last_rx = time.monotonic()
        for target, name in ((self._rx_loop, "rcjoy-rx"),
                             (self._keepalive_loop, "rcjoy-sub")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self):
        with self._lock:
            locked, sock = self._locked, self._lock_sock
        if locked is not None and sock is not None:
            try:
                sock.sendto(P.encode(P.unsubscribe()), (locked, self.port))
            except OSError:
                pass
        self._stop_evt.set()
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass
        for t in self._threads:
            t.join(timeout=1.0)

    # --- the contract ------------------------------------------------------------

    def get_state(self):
        """The latest RcState if fresh and usable, else None."""
        now = time.monotonic()
        with self._lock:
            s = self._state
        if s is None or now - s.received_at > self.stale_after:
            return None
        if not s.rc_ok or s.aircraft_linked or s.missing_sticks():
            return None
        return s

    def latest(self):
        """The latest RcState regardless of health (for display), or None."""
        with self._lock:
            return self._state

    def is_fresh(self):
        now = time.monotonic()
        with self._lock:
            s = self._state
        return s is not None and now - s.received_at <= self.stale_after

    def not_ok_reason(self):
        """Why get_state() is None, in words; None when it is not."""
        now = time.monotonic()
        with self._lock:
            locked, multi, s, bye = self._locked, list(self._multi), self._state, self._bye
        if locked is None:
            if multi:
                return ("several RCs answered (%s) - pick one with --rc"
                        % ", ".join(multi))
            return "searching for the RC (broadcast on :%d)" % self.port
        if s is None:
            if bye is not None:
                return "the RC app said bye (%s)" % (bye[1] or "no reason")
            return "no state from %s yet" % locked
        age = now - s.received_at
        if age > self.stale_after:
            return "stale: nothing from %s for %.1f s" % (locked, age)
        if s.aircraft_linked:
            return ("AIRCRAFT LINKED - stream blocked (this RC's sticks are "
                    "flying an aircraft)")
        if not s.rc_ok:
            return "RC reports rc_ok=false (MSDK lost the RC, or its liveness probe failed)"
        miss = s.missing_sticks()
        if miss:
            return "stick(s) not served by the RC: %s" % ", ".join(miss)
        return None

    def on_press(self, callback):
        """callback(button, n) for n new presses of `button` (from `presses` deltas)."""
        self._press_cbs.append(callback)

    def locked_rc(self):
        with self._lock:
            return self._locked

    def info(self):
        with self._lock:
            return self._info

    def pop_events(self):
        """Human-readable events (lock-on, bye, version mismatch...) since the last call."""
        with self._lock:
            out = list(self._events)
            self._events.clear()
        return out

    def warnings(self):
        """Standing conditions worth shouting about, from the latest info/state."""
        with self._lock:
            info, s = self._info, self._state
        out = []
        if s is not None and s.aircraft_linked:
            out.append("AIRCRAFT LINKED")
        if info:
            rf = info.get("rf") or {}
            if rf.get("wifi"):
                out.append("RC Wi-Fi is ON")
            if rf.get("wifi_scan"):
                # "Wi-Fi off" with scanning on still transmits probe requests
                out.append("RC Wi-Fi scanning is ON")
            if rf.get("bt"):
                out.append("RC Bluetooth is ON")
            if rf.get("ble_scan"):
                out.append("RC Bluetooth scanning is ON")
            bat = info.get("battery")
            if isinstance(bat, (int, float)) and not isinstance(bat, bool) \
                    and bat < BATTERY_WARN_PCT:
                out.append("RC battery %d%%" % bat)
        if s is not None:
            nd = [k for k in P.DIALS if s.dials.get(k) is None]
            if nd:
                out.append("dial(s) not served: %s" % ", ".join(nd))
        gp = (info or {}).get("gamepad")
        gp = gp if isinstance(gp, dict) else {}
        if s is not None and s.stick_src == "msdk":
            # Still usable, but ~10 Hz instead of the gamepad's ~70 Hz.
            out.append("sticks via MSDK, ~10 Hz (%s)" % (gp.get("why") or "gamepad not in use"))
        inverted = gp.get("inverted")
        if isinstance(inverted, list) and inverted:
            out.append("the RC's MSDK reads %s INVERTED vs its gamepad - withheld on the "
                       "MSDK path" % ", ".join(str(k) for k in inverted))
        return out

    def stats(self):
        now = time.monotonic()
        with self._lock:
            while self._arrivals and now - self._arrivals[0] > RATE_WINDOW_S:
                self._arrivals.popleft()
            while self._gaps and now - self._gaps[0][0] > GAP_WINDOW_S:
                self._gaps.popleft()
            # Divide by the span actually covered, not the full window: a stream
            # that started 1 s ago is not half as fast as one running for 2 s.
            span = now - self._arrivals[0] if self._arrivals else 0.0
            return {
                "rc": self._locked,
                "rx_hz": len(self._arrivals) / span if span >= 0.2 else 0.0,
                "packets": self._packets,
                "lost": self._lost,
                "max_gap_ms": (max(g for _, g in self._gaps) * 1000.0
                               if self._gaps else None),
                "rtt_ms": None if self._rtt is None else self._rtt * 1000.0,
                "foreign": self._foreign,
                "bad": self._bad,
            }

    # --- internals ---------------------------------------------------------------

    def _event(self, text):
        # Callers may or may not hold self._lock; deque.append is atomic.
        self._events.append(text)

    def _rx_loop(self):
        socks = list(self._socks)
        while not self._stop_evt.is_set():
            try:
                ready, _, _ = select.select(socks, [], [], 0.2)
            except (OSError, ValueError):
                if self._stop_evt.is_set():
                    return
                continue
            for s in ready:
                try:
                    data, addr = s.recvfrom(P.RECV_BUFFER)
                except ConnectionResetError:
                    # Windows: an ICMP port-unreachable for an earlier send (the RC
                    # app isn't running). Harmless; keep listening.
                    continue
                except OSError:
                    if self._stop_evt.is_set():
                        return
                    continue
                self._handle(s, data, addr, time.monotonic())

    def _handle(self, sock, data, addr, now):
        ip, port = addr[0], addr[1]
        try:
            msg = P.decode(data)
        except P.VersionMismatch as e:
            key = (ip, str(e))
            if key not in self._warned:
                self._warned.add(key)
                self._event("REJECTED %s:%d - %s" % (ip, port, e))
            return
        except P.ProtocolError:
            with self._lock:
                self._bad += 1
            return
        typ = msg["type"]
        fire = {}
        with self._lock:
            if self._locked is None:
                if typ == "info" and port == self.port:
                    self._candidates[ip] = (sock, msg, now)
                return
            if ip != self._locked or port != self.port:
                self._foreign += 1          # not the RC we locked onto
                return
            self._last_rx = now
            if typ == "state":
                try:
                    st = P.parse_state(msg, received_at=now)
                except P.ProtocolError as e:
                    self._bad += 1
                    key = ("state", str(e))
                    if key not in self._warned:
                        self._warned.add(key)
                        self._event("bad state from %s: %s" % (ip, e))
                    return
                prev = self._state
                gap = None if prev is None else now - prev.received_at
                if self._last_seq is not None:
                    if st.seq <= self._last_seq:
                        # A restarted app starts again at 0, always after a gap;
                        # anything else going backwards is a duplicate or a
                        # reordered datagram and must never regress the state.
                        restarted = (self._last_seq - st.seq >= 100
                                     or gap is None or gap > self.stale_after)
                        if not restarted:
                            return
                        self._press.reset()
                    elif st.seq > self._last_seq + 1:
                        self._lost += st.seq - self._last_seq - 1
                self._last_seq = st.seq
                if gap is not None:
                    self._gaps.append((now, gap))
                    if gap > self.stale_after:
                        self._press.reset()  # never apply a press late
                self._arrivals.append(now)
                self._packets += 1
                self._state = st
                self._bye = None
                fire = self._press.update(st.presses)
            elif typ == "info":
                self._info = msg
                et = msg.get("echo_t")
                if isinstance(et, (int, float)) and not isinstance(et, bool):
                    rtt = now - et
                    if 0.0 <= rtt < 5.0:
                        self._rtt = rtt
            elif typ == "bye":
                self._bye = (now, str(msg.get("reason") or ""))
                self._state = None
                self._last_seq = None
                self._press.reset()
                self._event("the RC at %s said bye (%s)"
                            % (ip, msg.get("reason") or "no reason"))
        for name, n in fire.items():
            for cb in self._press_cbs:
                try:
                    cb(name, n)
                except Exception:
                    pass

    def _send_subscribe(self):
        with self._lock:
            locked, sock = self._locked, self._lock_sock
        if locked is None or sock is None:
            return
        try:
            sock.sendto(P.encode(P.subscribe(self.client_name, self.rate_hz,
                                             time.monotonic())),
                        (locked, self.port))
        except OSError:
            pass    # interface down; keep trying

    def _discover_round(self):
        with self._lock:
            self._candidates = {}
        payload = P.encode(P.subscribe(self.client_name, self.rate_hz, time.monotonic()))
        targets = self._targets or [("255.255.255.255", self.port)]
        for s in self._socks:
            for tgt in targets:
                try:
                    s.sendto(payload, tgt)
                except OSError:
                    pass
        self._stop_evt.wait(DISCOVERY_WAIT_S)
        with self._lock:
            cands = dict(self._candidates)
            if len(cands) == 1:
                ip, (sock, info, t) = next(iter(cands.items()))
                self._locked, self._lock_sock = ip, sock
                self._info, self._last_rx = info, t
                self._state, self._last_seq, self._bye = None, None, None
                self._press.reset()
                self._multi = []
                self._event("locked onto the RC at %s (sn %s, %s)"
                            % (ip, info.get("sn") or "?", info.get("rc_type") or "?"))
            else:
                self._multi = sorted(cands)
        if len(cands) != 1:
            self._stop_evt.wait(max(0.0, P.SUBSCRIBE_PERIOD_S - DISCOVERY_WAIT_S))

    def _keepalive_loop(self):
        while not self._stop_evt.is_set():
            if self.auto:
                with self._lock:
                    locked, last_rx = self._locked, self._last_rx
                if locked is not None and time.monotonic() - last_rx > RELOCK_AFTER_S:
                    with self._lock:
                        self._locked, self._lock_sock, self._state = None, None, None
                    self._event("the RC at %s has been silent for %.0f s - searching again"
                                % (locked, RELOCK_AFTER_S))
                    locked = None
                if locked is None:
                    self._discover_round()
                    continue
            self._send_subscribe()
            self._stop_evt.wait(P.SUBSCRIBE_PERIOD_S)
