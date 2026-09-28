"""
UDP Joystick Receiver
=====================
Python equivalent of Unity's UDPReceiverManager.cs. Receives JoystickData
JSON messages over UDP — the same format produced by readController.py:

    {
      "linear":  {"x": float, "y": float, "z": float},
      "angular": {"x": float, "y": float, "z": float},
      "switches": {"s1": int, "s2": int}
    }

Listens in a background thread; main thread calls get_state() to read the
most recent packet, or None if nothing arrived inside the staleness window
(so a frozen / disconnected controller is treated as 'no input', not 'hold
last command').

JoystickWatchdog is what an ARMED swarm flies once get_state() starts
returning None: neutral sticks for JOYSTICK_LOST_STOP_S, then a STOP. Pure
policy, checked by `python udp_joystick_receiver.py --selftest`.
"""

import json
import socket
import threading
import time
from dataclasses import dataclass, replace


@dataclass
class JoystickState:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0
    s1: int = 0
    s2: int = 0
    received_at: float = 0.0


# Seconds an ARMED swarm rides out a joystick dropout on neutral sticks before
# auto-STOPping. Counted from the first get_state() that returns None, i.e. on
# top of the receiver's own stale_after window (0.5 s by default).
JOYSTICK_LOST_STOP_S = 3.0

# Neutral sticks. angular_x is the spacing knob's CENTRE (1.0, where
# readController.py rests), NOT the dataclass default 0.0:
# swarm_flocking.d_ref_from_ax clamps 0.0 up to 0.6 = the TIGHTEST spacing
# (4 m), so a "neutral" built from JoystickState() would contract the swarm.
NEUTRAL_JS = JoystickState(angular_x=1.0)


class JoystickWatchdog:
    """What an ARMED controller flies when the joystick feed drops out.

    - fresh feed             -> the feed itself
    - stale, never had one   -> NEUTRAL_JS and no auto-stop (a --dry-run desk
                                test with no controller; the arm gate makes
                                this impossible in a real flight)
    - stale after a feed     -> the last state with every stick zeroed but the
                                spacing knob (angular_x) kept, so d_ref does
                                not jump; after `stop_after` seconds -> STOP

    Riding the dropout out on neutral sticks, rather than skipping the control
    tick, keeps flocking, heading-hold and every failsafe running. Skipping it
    left each drone's send thread repeating its last velocity AND yaw RATE, so
    a swarm that lost the joystick mid-turn kept turning, with no min-separation
    or geofence check, until someone pressed Stop.

    Pure: the caller passes `now` and performs the side effects for the event
    step() returns, so the policy is testable without ds_wrapper.
    """

    def __init__(self, stop_after=JOYSTICK_LOST_STOP_S):
        self.stop_after = stop_after
        self.last_js = None
        self.lost_at = None

    def arm(self, js):
        """Start of a swarming stint. `js` is the state the arm gate passed
        (None only in a --dry-run with no controller)."""
        self.last_js = js
        self.lost_at = None

    def step(self, js, now):
        """Return (state_to_fly, event, outage_s).

        event is None, 'lost' (first stale tick), 'restored' (the feed is back;
        outage_s = how long it was gone) or 'stop' (stale for stop_after
        seconds: the caller must STOP the swarm; state_to_fly is None).
        outage_s is 0.0 while no loss is in progress.
        """
        if js is not None:
            event, outage = None, 0.0
            if self.lost_at is not None:
                event, outage = "restored", now - self.lost_at
                self.lost_at = None
            self.last_js = js
            return js, event, outage
        if self.last_js is None:
            return NEUTRAL_JS, None, 0.0
        event = None
        if self.lost_at is None:
            self.lost_at = now
            event = "lost"
        outage = now - self.lost_at
        if outage >= self.stop_after:
            return None, "stop", outage
        return (replace(self.last_js, linear_x=0.0, linear_y=0.0,
                        linear_z=0.0, angular_z=0.0),
                event, outage)


class JoystickReceiver:
    def __init__(self, host="0.0.0.0", port=5055, stale_after=0.5, logger=None):
        """
        Args:
            host:        bind address (default 0.0.0.0 — all interfaces)
            port:        UDP port (default 5055, matches readController.py)
            stale_after: seconds after which get_state() returns None
            logger:      optional FlightLogger; each received packet is logged
                         as a user command. None = no logging.
        """
        self.host = host
        self.port = port
        self.stale_after = stale_after
        self.logger = logger
        self._sock = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._state = JoystickState()
        self._packet_count = 0

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.2)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="JoystickRx")
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                j = json.loads(data.decode())
                lin = j.get("linear", {}) or {}
                ang = j.get("angular", {}) or {}
                sw = j.get("switches", {}) or {}
                state = JoystickState(
                    linear_x=float(lin.get("x", 0.0)),
                    linear_y=float(lin.get("y", 0.0)),
                    linear_z=float(lin.get("z", 0.0)),
                    angular_x=float(ang.get("x", 0.0)),
                    angular_y=float(ang.get("y", 0.0)),
                    angular_z=float(ang.get("z", 0.0)),
                    s1=int(sw.get("s1", 0)),
                    s2=int(sw.get("s2", 0)),
                    received_at=time.time(),
                )
                with self._lock:
                    self._state = state
                    self._packet_count += 1
                if self.logger:
                    self.logger.log_user_command(state, source="udp")
            except (json.JSONDecodeError, ValueError, TypeError, UnicodeDecodeError):
                continue

    def get_state(self):
        """Return the most recent JoystickState, or None if older than stale_after."""
        with self._lock:
            s = self._state
        if time.time() - s.received_at > self.stale_after:
            return None
        return s

    def packet_count(self):
        with self._lock:
            return self._packet_count

    def stop(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1)


def _selftest():
    """Offline check of JoystickWatchdog: no sockets, no ds_wrapper."""
    live = JoystickState(linear_x=0.8, linear_y=-0.3, linear_z=0.5,
                         angular_x=1.3, angular_z=0.9, s1=1, s2=0)

    # Never had a joystick (dry run): neutral, spacing knob centred, no stop.
    assert NEUTRAL_JS.angular_x == 1.0
    dog = JoystickWatchdog(stop_after=3.0)
    dog.arm(None)
    for t in (0.0, 5.0, 60.0):
        js, ev, _ = dog.step(None, t)
        assert js == NEUTRAL_JS and ev is None, (js, ev)

    # Live -> lost -> neutral sticks with the knob kept -> restored.
    dog = JoystickWatchdog(stop_after=3.0)
    dog.arm(live)
    js, ev, _ = dog.step(live, 0.0)
    assert js is live and ev is None
    js, ev, out = dog.step(None, 1.0)
    assert ev == "lost" and out == 0.0
    assert (js.linear_x, js.linear_y, js.linear_z, js.angular_z) == (0.0,) * 4
    assert js.angular_x == 1.3, "d_ref must not jump on a dropout"
    js, ev, out = dog.step(None, 2.5)
    assert ev is None and abs(out - 1.5) < 1e-9 and js.linear_x == 0.0
    js, ev, out = dog.step(live, 3.0)
    assert js is live and ev == "restored" and abs(out - 2.0) < 1e-9

    # A second loss gets its own full window, then stops.
    js, ev, _ = dog.step(None, 10.0)
    assert ev == "lost"
    js, ev, _ = dog.step(None, 12.9)
    assert ev is None and js is not None
    js, ev, out = dog.step(None, 13.0)
    assert js is None and ev == "stop" and abs(out - 3.0) < 1e-9

    # arm() clears a loss carried over from the previous stint.
    dog.arm(live)
    js, ev, _ = dog.step(None, 20.0)
    assert ev == "lost" and js.angular_x == 1.3

    # The substitution never mutates the state it was built from.
    assert live.linear_x == 0.8 and live.angular_z == 0.9
    print("udp_joystick_receiver selftest: OK")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv[1:]:
        _selftest()
        sys.exit(0)

    # Smoke test: print incoming packets at 5 Hz
    rx = JoystickReceiver()
    rx.start()
    print("Listening on :5055 — Ctrl+C to stop")
    try:
        while True:
            s = rx.get_state()
            if s is None:
                print(f"[{rx.packet_count():>5} pkts]  no fresh data")
            else:
                print(f"[{rx.packet_count():>5} pkts]  "
                      f"lin=({s.linear_x:+.2f},{s.linear_y:+.2f},{s.linear_z:+.2f})  "
                      f"ang=({s.angular_x:+.2f},{s.angular_y:+.2f},{s.angular_z:+.2f})  "
                      f"sw=({s.s1},{s.s2})")
            time.sleep(0.2)
    except KeyboardInterrupt:
        rx.stop()
