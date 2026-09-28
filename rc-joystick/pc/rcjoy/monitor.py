"""Live console readout of every RC input plus link health.

    python -m rcjoy monitor [--rc IP] [--port 5070] [--rate 50]

This is the bench tool: every control should move exactly one field, with the
documented sign (+ = up / right / clockwise). Also shows the per-key "served"
table from `info`, which answers the hardware spike's "which keys does the
firmware serve with no aircraft?" question.
"""

import collections
import os
import time

from . import protocol as P
from .client import RcJoystickClient

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


def render(client, t0):
    st, info, s = client.stats(), client.info() or {}, client.latest()
    reason = client.not_ok_reason()
    lines = ["LIS RC Joystick monitor  (Ctrl+C to quit)   up %.0f s" % (time.monotonic() - t0)]
    rtt = "-" if st["rtt_ms"] is None else "%.1f ms" % st["rtt_ms"]
    gap = "-" if st["max_gap_ms"] is None else "%.0f ms" % st["max_gap_ms"]
    lines.append("RC %s:%d   %s" % (st["rc"] or "-", client.port,
                                   "OK" if reason is None else "NOT OK: " + reason))
    lines.append("rx %.1f Hz   packets %d   seq loss %d   max gap (10 s) %s   rtt %s   "
                 "foreign %d   bad %d" % (st["rx_hz"], st["packets"], st["lost"], gap, rtt,
                                        st["foreign"], st["bad"]))
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
        gp = info.get("gamepad")
        if isinstance(gp, dict):
            gap = gp.get("gap_ms")
            mode = ("?" if not isinstance(gap, int) else
                    "fast mode" if gap <= 30 else "SLOW mode" if gap >= 70 else "?")
            lines.append("gamepad: %s reports/s, median gap %s ms (%s)   DJI gate %s   "
                         "verified vs MSDK %s   inverted %s%s" % (
                             gp.get("hz"), "-" if gap is None else gap, mode,
                             {True: "open", False: "CLOSED"}.get(gp.get("gate"), "?"),
                             ",".join(gp.get("verified") or []) or "-",
                             ",".join(gp.get("inverted") or []) or "none",
                             ("   not in use: " + str(gp["why"])) if gp.get("why") else ""))
    lines.append("")
    if s is None:
        lines.append("(no state)")
    else:
        age = time.monotonic() - s.received_at
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


def main(args):
    os.system("")    # enable ANSI escape processing in a classic Windows console
    client = RcJoystickClient(args.rc, port=args.port, rate_hz=args.rate,
                              client_name="rcjoy-monitor")
    presses = collections.deque()   # appended on the rx thread
    client.on_press(lambda b, n: presses.append("%s x%d" % (b, n)))
    client.start()
    t0 = time.monotonic()
    events = []
    try:
        while True:
            events.extend(client.pop_events())
            while presses:
                events.append("press: " + presses.popleft())
            events = events[-6:]
            out = render(client, t0)
            out += ["", "recent events:"] + (["  " + e for e in events] or ["  -"])
            print("\x1b[H\x1b[J" + "\n".join(out), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
    return 0
