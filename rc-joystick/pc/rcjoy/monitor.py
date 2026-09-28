"""Live console readout of every RC input plus link health.

    python -m rcjoy monitor [--rc IP] [--port 5070] [--rate 50] [--csv PATH]

This is the bench tool: every control should move exactly one field, with the
documented sign (+ = up / right / clockwise). Also shows the per-key "served"
table from `info`, which answers the hardware spike's "which keys does the
firmware serve with no aircraft?" question.

The `rates/s` line puts three layers side by side, because they are different
numbers: `wire` is the RC's send clock (the --rate asked for, whatever the sticks
do); `fresh` is how many of those states carried a new input, measured here; `app
gamepad` is the app's own count of the reports it received. With a stick moving
and --rate 100, fresh should match the app's figure. --csv keeps the evidence.
"""

import collections
import csv
import os
import sys
import threading
import time

from . import protocol as P
from .chooser import choose_rc
from .client import RcJoystickClient, remember_path_for

BAR = 21


def _bar(x):
    if x is None:
        return "[" + " n/a ".center(BAR) + "]"
    pos = int(round((x + 1.0) / 2.0 * (BAR - 1)))
    cells = ["-"] * BAR
    cells[BAR // 2] = "|"
    cells[pos] = "#"
    return "[" + "".join(cells) + "]"


def _fmt_raw(v):
    return " null" if v is None else "%+5d" % v


def _gamepad(info):
    gp = (info or {}).get("gamepad")
    return gp if isinstance(gp, dict) else {}


def _gp_mode(gap):
    """The virtual joystick's report mode from the app's median report gap, ms."""
    if not isinstance(gap, int) or isinstance(gap, bool):
        return "?"
    return "fast mode" if gap <= 30 else "SLOW mode" if gap >= 70 else "?"


def render(client, t0):
    st, info, s = client.stats(), client.info() or {}, client.latest()
    reason = client.not_ok_reason()
    lines = ["LIS RC Joystick monitor  (Ctrl+C to quit)   up %.0f s" % (P.now() - t0)]
    rtt = "-" if st["rtt_ms"] is None else "%.1f ms" % st["rtt_ms"]
    gap = "-" if st["max_gap_ms"] is None else "%.0f ms" % st["max_gap_ms"]
    lines.append("RC %s:%d   %s" % (st["rc"] or "-", client.port,
                                   "OK" if reason is None else "NOT OK: " + reason))
    lines.append("rx %.1f Hz   packets %d   seq loss %d   max gap (10 s) %s   rtt %s   "
                 "foreign %d   bad %d" % (st["rx_hz"], st["packets"], st["lost"], gap, rtt,
                                        st["foreign"], st["bad"]))
    gp = _gamepad(info)
    gp_gap = gp.get("gap_ms")
    fresh = "%.1f" % st["fresh_hz"]
    if st["rx_hz"] > 0 and st["fresh_hz"] == 0:
        fresh += " (nothing moving)"
    lines.append("rates/s: wire %.1f (RC send clock, --rate %d)   fresh at this PC %s   "
                 "app gamepad %s, median gap %s ms (%s)" % (
                     st["rx_hz"], client.rate_hz, fresh,
                     "-" if gp.get("hz") is None else gp.get("hz"),
                     "-" if gp_gap is None else gp_gap, _gp_mode(gp_gap)))
    if info:
        lines.append("app %s   %s   sn %s   fw %s   battery %s%%   stick mode %s   eth %s   "
                     "subscribers %s" % (info.get("app"), info.get("rc_type"), info.get("sn"),
                                         info.get("fw"), info.get("battery"),
                                         info.get("stick_mode"), info.get("eth_ip"),
                                         info.get("subscribers")))
        rf = info.get("rf") or {}
        lines.append("RF: wifi %s (scan %s)   bt %s (scan %s)   airplane %s   aircraft %s" % (
            "ON" if rf.get("wifi") else "off", "ON" if rf.get("wifi_scan") else "off",
            "ON" if rf.get("bt") else "off", "ON" if rf.get("ble_scan") else "off",
            "on" if rf.get("airplane") else "OFF",
            "LINKED" if rf.get("aircraft_linked") else "not linked"))
        if gp:
            lines.append("gamepad: DJI gate %s   verified vs MSDK %s   inverted %s%s" % (
                             {True: "open", False: "CLOSED"}.get(gp.get("gate"), "?"),
                             ",".join(gp.get("verified") or []) or "-",
                             ",".join(gp.get("inverted") or []) or "none",
                             ("   not in use: " + str(gp["why"])) if gp.get("why") else ""))
    lines.append("")
    if s is None:
        lines.append("(no state)")
    else:
        age = P.now() - s.received_at
        lines.append("seq %d   age %.0f ms   rc_ok %s   aircraft_linked %s   mode switch %s   "
                     "sticks from %s" % (s.seq, age * 1000, s.rc_ok, s.aircraft_linked,
                                         s.mode_switch, (s.stick_src or "?").upper()))
        for k, label in (("lh", "left  H"), ("lv", "left  V"), ("rh", "right H"),
                         ("rv", "right V")):
            lines.append("stick %s %s %s  %s" % (label, _bar(s.sticks[k]),
                                                 _fmt_raw(s.raw_sticks[k]),
                                                 "" if s.sticks[k] is None
                                                 else "%+.3f" % s.sticks[k]))
        for k, label in (("l", "left "), ("r", "right")):
            lines.append("dial  %s   %s %s  %s" % (label, _bar(s.dials[k]),
                                                   _fmt_raw(s.raw_dials[k]),
                                                   "" if s.dials[k] is None
                                                   else "%+.3f" % s.dials[k]))
        btn = []
        for k in P.BUTTONS:
            v = s.buttons.get(k)
            mark = "n/a" if v is None else ("DOWN" if v else "up")
            btn.append("%s:%s/%s" % (k, mark, s.presses.get(k, "-")))
        lines.append("buttons (level/presses):")
        for i in range(0, len(btn), 5):
            lines.append("  " + "   ".join(btn[i:i + 5]))
    keys = info.get("keys") if info else None
    if keys:
        served = ["%s:%s" % (k, "%dx" % v.get("n", 0) if v.get("n") else "NOT SERVED")
                  for k, v in keys.items() if isinstance(v, dict)]
        lines.append("keys served (callbacks since app start):")
        for i in range(0, len(served), 6):
            lines.append("  " + "   ".join(served[i:i + 6]))
    warn = client.warnings()
    lines.append("")
    lines.append("warnings: " + ("; ".join(warn) if warn else "none"))
    return lines


def _cell(v):
    return "" if v is None else v


def _unused_path(path):
    """`path`, or name-2.ext, name-3.ext... if it exists: a bench record is evidence, and
    reusing a name must never destroy the previous run."""
    if not os.path.exists(path):
        return path
    stem, ext = os.path.splitext(path)
    n = 2
    while os.path.exists("%s-%d%s" % (stem, n, ext)):
        n += 1
    return "%s-%d%s" % (stem, n, ext)


class CsvLog:
    """One row per received state, written on the client's rx thread: the record of a
    bench run, since the screen redraws five times a second and cannot be pasted.

    t_s is this PC's receive time (perf_counter, from the start of the log), and wall
    the same moment on the PC's clock, to match against notes taken at the bench; t_ms
    is the RC's own clock at send. fresh = 1 when the state's sticks or dials changed.
    app_gp_* are the app's own figures from the latest `info` (1 Hz).

    An existing file is never overwritten: the log goes to name-2.csv, name-3.csv...
    instead, and `path` says which."""

    COLUMNS = (("t_s", "wall", "seq", "t_ms", "fresh", "stick_src", "rc_ok",
                "aircraft_linked") + P.STICKS + P.DIALS + ("app_gp_hz", "app_gp_gap_ms"))

    def __init__(self, path, client):
        self.path = _unused_path(path)
        self._client = client
        self._f = open(self.path, "x", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLUMNS)
        self._lock = threading.Lock()
        self._t0 = P.now()
        self._wall0 = time.time()       # the wall clock at _t0; rows add perf_counter time
        self.rows = self.fresh = 0
        self._first = self._last = None

    def write(self, st, fresh):
        gp = _gamepad(self._client.info())
        t = st.received_at - self._t0
        wall = self._wall0 + t
        row = (["%.6f" % t, time.strftime("%H:%M:%S", time.localtime(wall))
                + ".%03d" % int((wall % 1) * 1000), st.seq, st.t_ms, int(bool(fresh)),
                _cell(st.stick_src),
                int(st.rc_ok), int(st.aircraft_linked)]
               + [_cell(st.raw_sticks.get(k)) for k in P.STICKS]
               + [_cell(st.raw_dials.get(k)) for k in P.DIALS]
               + [_cell(gp.get("hz")), _cell(gp.get("gap_ms"))])
        with self._lock:
            if self._f.closed:
                return
            self._w.writerow(row)
            self.rows += 1
            self.fresh += int(bool(fresh))
            if self._first is None:
                self._first = t
            self._last = t

    def flush(self):
        with self._lock:
            if not self._f.closed:
                self._f.flush()

    def close(self):
        with self._lock:
            self._f.close()

    def summary(self):
        span = (self._last - self._first) if self.rows > 1 else 0.0
        rates = ("" if span <= 0 else
                 ": %.1f states/s, %.1f fresh/s averaged over the whole %.1f s"
                 % ((self.rows - 1) / span, self.fresh / span, span))
        return "csv: %d states, %d fresh%s -> %s" % (self.rows, self.fresh, rates, self.path)


def main(args):
    os.system("")    # enable ANSI escape processing in a classic Windows console
    client = RcJoystickClient(args.rc, port=args.port, rate_hz=args.rate,
                              client_name="rcjoy-monitor", remember=remember_path_for(args.rc),
                              chooser=choose_rc)
    log = None
    if getattr(args, "csv", None):
        try:
            log = CsvLog(args.csv, client)
        except OSError as e:
            print("cannot write %s: %s" % (args.csv, e), file=sys.stderr)
            return 2
        client.on_state(log.write)
    presses = collections.deque()   # appended on the rx thread
    client.on_press(lambda b, n: presses.append("%s x%d" % (b, n)))
    client.start()
    t0 = P.now()
    events = []
    try:
        while True:
            events.extend(client.pop_events())
            while presses:
                events.append("press: " + presses.popleft())
            events = events[-6:]
            out = render(client, t0)
            if log is not None:
                log.flush()
                out += ["", "recording %d states (%d fresh) to %s"
                        % (log.rows, log.fresh, log.path)]
            out += ["", "recent events:"] + (["  " + e for e in events] or ["  -"])
            print("\x1b[H\x1b[J" + "\n".join(out), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        if log is not None:
            log.close()
            print(log.summary())
    return 0
