"""Bridge: RC Pro state -> the exact JSON readController.py sends (Taranis path).

    python -m rcjoy bridge [--rc IP] [--out HOST:PORT]... [--rate 20]
                           [--stick-mode 1|2|3] [--profile flocking|joystick|sim]
                           [--knob-sweep-s 3] [--ax-range LO,HI]

Without --rc it finds the RC by itself: the last RC this PC used (saved on every
lock-on), and a broadcast on the switch. One --rc IP is enough for every later launch.

Every consumer of readController.py's JSON (AOS server's udp_joystick_receiver.py,
the Unity sim's UDPReceiverManager.cs) runs unchanged:

    {"linear": {"x": fwd, "y": right, "z": climb*|climb|},
     "angular": {"x": knob in --ax-range (default [0.6, 1.4]), "y": 0.0, "z": yaw},
     "switches": {"s1": +1|-1 (JSON int), "s2": knob in [-0.999, 0.999]}}

The sim profile adds one field, "marks": a JSON int counting C2 presses since the
bridge started (see below). JsonUtility ignores it in an older Unity; the AOS
profiles never send it.

The RC's dials spring back to centre, so each one drives an integrated KNOB. The
RC Pro has nothing like a Taranis pot. Full deflection sweeps a knob's whole range
in --knob-sweep-s. C1 toggles s1 (the Unity panorama, like the Taranis click
switch). C2 puts both knobs back to their start values, except in the sim profile,
where it is the experiment's identify button: a reset there would snap the spread
and the gimbal at the very moment the pilot reports a target. It is sent as a
cumulative count rather than a level, as the RC sends it, so a press survives a lost
or unread datagram; Unity acts on increases.

It sends ONLY while the RC is fresh and usable. Silence then looks exactly like an
unplugged Taranis, and the swarm's arm gate and dropout failsafe see the truth.
It never pads a gap with neutral packets.
"""

import json
import shutil
import socket
import sys
import threading

from . import protocol as P
from .chooser import choose_rc
from .client import RcJoystickClient, remember_path_for

# Physical stick -> function, per DJI stick mode. Mode 2 is throttle/yaw on the
# left, as with the Taranis setup readController.py was written for.
STICK_MODES = {
    1: {"forward": "lv", "right": "rh", "climb": "rv", "yaw": "lh"},
    2: {"forward": "rv", "right": "rh", "climb": "lv", "yaw": "lh"},
    3: {"forward": "lv", "right": "lh", "climb": "rv", "yaw": "rh"},
}

# Which dial drives which legacy field. The two consumers disagree about what
# angular.x means: swarm_flocking.py reads it as SPACING (and owns the gimbal in its
# GUI), while joystick_controller.py reads it as the GIMBAL and treats s2 only as a
# LAND edge. The joystick profile therefore puts the RC's gimbal dial (the left one)
# on angular.x. The sim profile is flocking's mapping with the Taranis's spacing range:
# the Unity sim takes angular.x unclamped as its spread (Olfati-Saber d_ref).
# `c2` is what C2 does: "reset" the knobs, or "mark" (count it into the "marks" field).
PROFILES = {
    "flocking": {"ax_dial": "r", "s2_dial": "l", "c2": "reset"},
    "joystick": {"ax_dial": "l", "s2_dial": "r", "c2": "reset"},
    "sim": {"ax_dial": "r", "s2_dial": "l", "c2": "mark", "ax_range": (0.4, 1.6)},
}

# swarm_flocking.py clamps angular.x to [0.6, 1.4], so the knob stops there by default.
# The Unity sim uses it unclamped as its spread (Olfati-Saber d_ref), and readController.py's
# pot spans [0.4, 1.6]: --ax-range 0.4,1.6 gives the sim the Taranis's range.
AX_MIN, AX_MAX, AX_START = 0.6, 1.4, 1.0
# Never +-1.0: JoystickReceiver casts s2 to int, and joystick_controller.py treats
# s2 == 1 as a LAND edge. int(+-0.999) == 0.
S2_LIMIT, S2_START = 0.999, 0.0
MAX_RATE_HZ = 20    # Unity drains one datagram per frame; faster only builds a backlog
DIAL_DEADBAND = 0.05
MAX_DT_S = 0.2      # a slow tick must not turn into a knob jump


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Bridge:
    def __init__(self, client, stick_mode=2, profile="flocking",
                 knob_sweep_s=3.0, dial_deadband=DIAL_DEADBAND, ax_range=None):
        if stick_mode not in STICK_MODES:
            raise ValueError("stick_mode must be 1, 2 or 3")
        if profile not in PROFILES:
            raise ValueError("profile must be one of %s" % ", ".join(PROFILES))
        if knob_sweep_s <= 0:
            raise ValueError("knob_sweep_s must be > 0")
        # An explicit range wins over the profile's; flocking and joystick have none.
        ax_range = ax_range or PROFILES[profile].get("ax_range") or (AX_MIN, AX_MAX)
        self.ax_lo, self.ax_hi = float(ax_range[0]), float(ax_range[1])
        if not self.ax_lo < AX_START < self.ax_hi:
            raise ValueError("ax_range must contain the start value %.1f, e.g. 0.6,1.4" % AX_START)
        self.client = client
        self.sticks = STICK_MODES[stick_mode]
        self.dials = PROFILES[profile]
        self.stick_mode, self.profile = stick_mode, profile
        self.sweep = float(knob_sweep_s)
        self.deadband = float(dial_deadband)
        self.ax, self.s2, self.s1 = AX_START, S2_START, 1
        self.marking = self.dials["c2"] == "mark"
        self.marks = 0
        self._presses = P.PressCounter()
        self._last_tick = None
        self._fresh = False
        self.sent = 0

    def reset_knobs(self):
        self.ax, self.s2 = AX_START, S2_START

    def _dial(self, st, name):
        v = st.dials.get(name)
        if v is None:
            return None             # not served: that knob just stays put
        return 0.0 if abs(v) < self.deadband else v

    def tick(self, now):
        """One output message for time `now`, or None (send nothing) while the
        RC is not fresh+usable. Knobs never move while it isn't."""
        st = self.client.get_state()
        if st is None:
            self._fresh = False
            self._last_tick = None
            return None
        if not self._fresh:
            self._presses.reset()   # a press made while stale never lands late
            self._fresh = True
        dt = 0.0 if self._last_tick is None else _clamp(now - self._last_tick, 0.0, MAX_DT_S)
        self._last_tick = now

        for name, n in self._presses.update(st.presses).items():
            if name == "c1" and n % 2:
                self.s1 = -self.s1
            elif name == "c2":
                if self.marking:
                    self.marks += n
                else:
                    self.reset_knobs()

        d = self._dial(st, self.dials["ax_dial"])
        if d is not None:
            self.ax = _clamp(self.ax + d * (self.ax_hi - self.ax_lo) / self.sweep * dt,
                             self.ax_lo, self.ax_hi)
        d = self._dial(st, self.dials["s2_dial"])
        if d is not None:
            self.s2 = _clamp(self.s2 + d * (2 * S2_LIMIT) / self.sweep * dt,
                             -S2_LIMIT, S2_LIMIT)

        m = self.sticks
        climb = st.sticks[m["climb"]]
        msg = {
            "linear": {"x": round(st.sticks[m["forward"]], 4),
                       "y": round(st.sticks[m["right"]], 4),
                       # quadratic, exactly like readController.py
                       "z": round(climb * abs(climb), 4)},
            "angular": {"x": round(self.ax, 4), "y": 0.0,
                        "z": round(st.sticks[m["yaw"]], 4)},
            # s1 must stay a JSON INT: Unity declares Switches.s1 as int.
            "switches": {"s1": int(self.s1), "s2": round(self.s2, 4)},
        }
        if self.marking:
            msg["marks"] = int(self.marks)   # a JSON int: Unity's JoystickData.marks
        return msg

    def run(self, outs, rate_hz=MAX_RATE_HZ, stop_evt=None, verbose=True):
        """Send tick() to every (host, port) in `outs` at rate_hz until stop_evt."""
        if not 0 < rate_hz <= MAX_RATE_HZ:
            raise ValueError("rate must be in (0, %d] Hz" % MAX_RATE_HZ)
        stop_evt = stop_evt or threading.Event()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        period = 1.0 / rate_hz
        tty = verbose and sys.stdout.isatty()
        sending, last_status, last_warn = None, 0.0, None
        marks_shown = self.marks
        next_t = P.now()
        try:
            while not stop_evt.is_set():
                now = P.now()
                msg = self.tick(now)
                if msg is not None:
                    payload = json.dumps(msg).encode("utf-8")
                    for out in outs:
                        try:
                            sock.sendto(payload, out)
                        except OSError:
                            pass
                    self.sent += 1
                if verbose:
                    for ev in self.client.pop_events():
                        _line("[bridge] " + ev, tty)
                    if (msg is not None) != sending:
                        sending = msg is not None
                        _line("[bridge] SENDING to %s" % _outs_desc(outs) if sending
                              else "[bridge] NOT SENDING - %s"
                              % (self.client.not_ok_reason() or "starting"), tty)
                    if self.marks != marks_shown:
                        marks_shown = self.marks
                        _line("[bridge] C2 mark #%d" % self.marks, tty)
                    warn = self.client.warnings()
                    if warn != last_warn:
                        last_warn = warn
                        if warn:
                            _line("[bridge] WARNING: " + "; ".join(warn), tty)
                    if now - last_status >= 0.5:
                        last_status = now
                        self._status(msg, tty)
                next_t += period
                delay = next_t - P.now()
                if delay > 0:
                    stop_evt.wait(delay)
                else:
                    next_t = P.now()   # fell behind: resync, don't burst
        finally:
            sock.close()

    def _status(self, msg, tty):
        # One self-overwriting line, trimmed to the pane: the launcher's controller
        # pane is only ~60 columns, and a wrapped \r line corrupts itself.
        if not tty:
            return
        st = self.client.stats()
        if msg is None:
            body = "-- no output --"
        else:
            lin, ang, sw = msg["linear"], msg["angular"], msg["switches"]
            body = ("f%+.2f r%+.2f c%+.2f y%+.2f ax%.2f s2%+.2f s1%+d"
                    % (lin["x"], lin["y"], lin["z"], ang["z"], ang["x"],
                       sw["s2"], sw["s1"]))
        link = "%.0fHz L%d %s" % (st["rx_hz"], st["lost"],
                                  "-" if st["rtt_ms"] is None
                                  else "%.1fms" % st["rtt_ms"])
        w = _width()
        sys.stdout.write("\r" + ("%s | %s" % (body, link))[:w].ljust(w))
        sys.stdout.flush()


def _width():
    return max(20, shutil.get_terminal_size((80, 20)).columns - 1)


def _line(text, tty):
    if tty:
        sys.stdout.write("\r" + " " * _width() + "\r")
    print(text, flush=True)


def _outs_desc(outs):
    return ", ".join("%s:%d" % o for o in outs)


def parse_range(s):
    """'LO,HI' -> (lo, hi): the angular.x knob's range."""
    try:
        lo, hi = (float(x) for x in str(s).split(","))
    except ValueError:
        raise ValueError("--ax-range must be LO,HI (got %r)" % s)
    return lo, hi


def parse_out(s):
    host, sep, port = s.rpartition(":")
    if not sep or not host:
        raise ValueError("--out must be HOST:PORT (got %r)" % s)
    return host, int(port)


def main(args):
    outs = [parse_out(o) for o in (args.out or ["127.0.0.1:5055"])]
    if not 0 < args.rate <= MAX_RATE_HZ:
        print("--rate must be in (0, %d] Hz: Unity's UDPReceiverManager reads one "
              "datagram per frame, so anything faster builds a growing backlog"
              % MAX_RATE_HZ, file=sys.stderr)
        return 2
    try:
        ax_range = parse_range(args.ax_range) if args.ax_range else None
        client = RcJoystickClient(args.rc, port=args.port, client_name="rcjoy-bridge",
                                  remember=remember_path_for(args.rc), chooser=choose_rc)
        bridge = Bridge(client, stick_mode=args.stick_mode, profile=args.profile,
                        knob_sweep_s=args.knob_sweep_s, ax_range=ax_range)
    except ValueError as e:
        print("[bridge] %s" % e, file=sys.stderr)
        return 2
    rc = args.rc or ("auto: the last RC used (%s), and a broadcast" % client.remembered
                     if client.remembered else "auto: broadcast")
    print("[bridge] RC %s  port %d -> %s at %g Hz  mode %d  profile %s  knob sweep %gs  "
          "angular.x %g..%g" % (rc, args.port, _outs_desc(outs), args.rate,
                                args.stick_mode, args.profile, args.knob_sweep_s,
                                bridge.ax_lo, bridge.ax_hi))
    print("[bridge] C1 = panorama toggle (s1)   C2 = %s   Ctrl+C = quit"
          % ("identify mark (marks)" if bridge.marking else "reset knobs"))
    client.start()
    stop = threading.Event()
    try:
        bridge.run(outs, rate_hz=args.rate, stop_evt=stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        client.stop()
        print("\n[bridge] stopped")
    return 0
