"""python -m rcjoy selftest: loopback checks of protocol, client and bridge. No hardware.

The fake RC stands in for the Android app. Everything runs over 127.0.0.x on
ephemeral ports, so it never touches a real RC or the swarm's :5055.
"""

import csv
import json
import os
import re
import socket
import tempfile
import threading
import time
import traceback

from . import protocol as P
from .bridge import Bridge, parse_range
from .chooser import ChooserUnavailable, label as chooser_label, payload as chooser_payload
from .client import RcJoystickClient, read_remembered, remember_path_for
from .fake_rc import FakeRc
from .monitor import CsvLog


class _Checks:
    def __init__(self):
        self.n = 0
        self.failed = []

    def __call__(self, name, cond):
        self.n += 1
        print("  %s  %s" % ("ok  " if cond else "FAIL", name), flush=True)
        if not cond:
            self.failed.append(name)


def _wait(pred, timeout, step=0.01):
    end = P.now() + timeout
    while P.now() < end:
        if pred():
            return True
        time.sleep(step)
    return bool(pred())


def _reason(cl):
    return cl.not_ok_reason() or ""


# --- protocol ------------------------------------------------------------------------

def t_protocol(ck):
    ck("norm: 660 -> 1.0, -330 -> -0.5, 0 -> 0.0",
       P.norm(660) == 1.0 and P.norm(-330) == -0.5 and P.norm(0) == 0.0)
    ck("norm clamps a calibration overshoot", P.norm(700) == 1.0 and P.norm(-701) == -1.0)
    ck("norm keeps null as None", P.norm(None) is None)

    for bad, what in ((b"\xff\xfe", "non-UTF-8"), (b"[1]", "a non-object"),
                      (b'{"v":1}', "a missing type"), (b'{"type":"state"}', "a missing v")):
        try:
            P.decode(bad)
            ok = False
        except P.VersionMismatch:
            ok = False
        except P.ProtocolError:
            ok = True
        ck("decode rejects %s" % what, ok)
    for v in (2, 0, True, "1"):
        try:
            P.decode(P.encode({"type": "state", "v": v}))
            ok = False
        except P.VersionMismatch:
            ok = True
        except P.ProtocolError:
            ok = False
        ck("decode treats v=%r as a version mismatch" % (v,), ok)

    rc = FakeRc(bind="127.0.0.1", port=0)       # not started: only builds messages
    try:
        base = rc.build_state()
        s = P.parse_state(base, 1.0)
        ck("parse_state reads the fake RC's state",
           s.rc_ok and not s.aircraft_linked and s.missing_sticks() == [])
        mutations = (
            (lambda m: m.update(rc_ok=1), "rc_ok given as a number"),
            (lambda m: m.pop("aircraft_linked"), "a missing aircraft_linked"),
            (lambda m: m["sticks"].update(lv=True), "a bool stick value"),
            (lambda m: m["sticks"].update(lv="0"), "a string stick value"),
            (lambda m: m.update(seq=-1), "a negative seq"),
            (lambda m: m["presses"].update(c1=-1), "a negative press count"),
        )
        for mutate, what in mutations:
            m = json.loads(json.dumps(base))
            mutate(m)
            try:
                P.parse_state(m)
                ok = False
            except P.ProtocolError:
                ok = True
            ck("parse_state rejects %s" % what, ok)
        m = json.loads(json.dumps(base))
        m["sticks"]["lv"] = None
        ck("a null stick parses as MISSING, not as zero",
           P.parse_state(m).missing_sticks() == ["lv"])
        ck("stick_src is read (the fake RC reports the gamepad path)",
           s.stick_src == "gamepad")
        m = json.loads(json.dumps(base))
        m.pop("stick_src")
        ck("a state without stick_src (app < 1.5) still parses, as None",
           P.parse_state(m).stick_src is None)
        m = json.loads(json.dumps(base))
        m["stick_src"] = 1
        try:
            P.parse_state(m)
            ok = False
        except P.ProtocolError:
            ok = True
        ck("parse_state rejects a non-string stick_src", ok)

        info = rc.build_info(123456.789)
        info.update(sn="X" * 16, eth_ip="192.168.100.255", fw="01.02.0300")
        for k in info["keys"]:
            info["keys"][k] = {"n": 12345678, "age_ms": 99999}
        n = len(P.encode(info))
        ck("worst-case info fits one datagram (%d <= %d bytes)" % (n, P.MAX_INFO_BYTES),
           n <= P.MAX_INFO_BYTES)
    finally:
        rc.sock.close()

    pc = P.PressCounter()
    ck("presses: the first update is only a baseline", pc.update({"c1": 5}) == {})
    ck("presses: deltas fire", pc.update({"c1": 7}) == {"c1": 2})
    ck("presses: a newly served button is baselined, not fired",
       pc.update({"c1": 7, "c2": 3}) == {})
    ck("presses: a count going down (app restart) fires nothing",
       pc.update({"c1": 0, "c2": 0}) == {})
    ck("presses: counting resumes after a restart", pc.update({"c1": 1, "c2": 0}) == {"c1": 1})


# --- client <-> fake RC ----------------------------------------------------------------

def t_client(ck):
    rc = FakeRc(bind="127.0.0.1", port=0).start()
    rc.press("c1", 4)                   # made before anyone listened: must never fire
    cl = RcJoystickClient("127.0.0.1", port=rc.port, rate_hz=50, client_name="selftest")
    fired = []
    cl.on_press(lambda b, n: fired.append((b, n)))
    cl.start()
    try:
        ck("usable state within 1 s", _wait(lambda: cl.get_state() is not None, 1.0))
        time.sleep(1.2)
        st = cl.stats()
        ck("state rate >= 35 Hz with 50 requested (%.1f Hz)" % st["rx_hz"], st["rx_hz"] >= 35)
        ck("no seq loss on loopback (%d)" % st["lost"], st["lost"] == 0)
        ck("RTT measured from the info echo (%s ms)" % (
            "-" if st["rtt_ms"] is None else "%.2f" % st["rtt_ms"]),
           st["rtt_ms"] is not None and st["rtt_ms"] < 100)
        # time.monotonic() ticks every 15.625 ms on Windows and read this as 0.0.
        ck("RTT resolves below a millisecond (loopback reads non-zero)",
           st["rtt_ms"] is not None and 0.0 < st["rtt_ms"] < 50)
        ck("a press made before subscribing never fires", fired == [])

        rc.set(lh=660, lv=-330, rh=0, rv=700, l=165, r=-660)
        ok = _wait(lambda: (cl.get_state() is not None
                            and cl.get_state().sticks["lh"] == 1.0), 0.5)
        s = cl.get_state()
        ck("normalization end to end (rv 700 clamps to 1.0)",
           ok and s.sticks == {"lh": 1.0, "lv": -0.5, "rh": 0.0, "rv": 1.0}
           and s.dials == {"l": 0.25, "r": -1.0})

        rc.press("c1")
        rc.press("c1")
        rc.press("c2")
        _wait(lambda: sum(n for _, n in fired) >= 3, 0.5)
        tot = {}
        for b, n in fired:
            tot[b] = tot.get(b, 0) + n
        ck("presses arrive as deltas (%r)" % tot, tot == {"c1": 2, "c2": 1})

        rc.aircraft_linked = True
        ok = _wait(lambda: cl.get_state() is None, 0.3)
        ck("aircraft_linked BLOCKS the stream (%s)" % _reason(cl),
           ok and "AIRCRAFT" in _reason(cl))
        rc.aircraft_linked = False
        ck("... and it recovers once unlinked", _wait(lambda: cl.get_state() is not None, 0.3))

        rc.rc_ok = False
        ok = _wait(lambda: cl.get_state() is None, 0.3)
        ck("rc_ok=false blocks the stream", ok and "rc_ok" in _reason(cl))
        rc.rc_ok = True
        _wait(lambda: cl.get_state() is not None, 0.3)

        rc.null_keys = {"lv"}
        ok = _wait(lambda: cl.get_state() is None, 0.3)
        ck("a null STICK blocks the stream (%s)" % _reason(cl), ok and "lv" in _reason(cl))
        rc.null_keys = {"r"}
        ok = _wait(lambda: cl.get_state() is not None, 0.3)
        ck("a null DIAL does not block (only warns)",
           ok and any("dial" in w for w in cl.warnings()))
        rc.null_keys = set()

        rc.rf["wifi_scan"] = True               # arrives with the next info (1 Hz)
        ck("Wi-Fi SCANNING on the RC is warned about (it transmits with Wi-Fi off)",
           _wait(lambda: any("scanning" in w for w in cl.warnings()), 1.6))
        rc.rf["wifi_scan"] = False

        ck("sticks via the gamepad: usable, and no slow-sticks warning",
           cl.get_state() is not None and cl.get_state().stick_src == "gamepad"
           and not any("MSDK" in w for w in cl.warnings()))
        rc.stick_src, rc.gamepad_why = "msdk", "the app is not in front"
        ok = _wait(lambda: any("via MSDK" in w and "not in front" in w
                               for w in cl.warnings()), 1.6)
        ck("the MSDK fallback stays USABLE but is warned about, with the app's reason",
           ok and cl.get_state() is not None)
        rc.inverted = ["lv"]
        ck("an axis the app found inverted is warned about",
           _wait(lambda: any("INVERTED" in w and "lv" in w for w in cl.warnings()), 1.6))
        rc.stick_src, rc.gamepad_why, rc.inverted = "gamepad", None, []

        try:
            rogue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rogue.bind(("127.0.0.3", rc.port))   # the RC's port, another address
        except OSError:
            rogue = None
            print("  skip  source filter (cannot bind 127.0.0.3 here)")
        if rogue is not None:
            evil = rc.build_state()
            evil["sticks"]["rv"] = -660
            evil["seq"] = 10 ** 6
            port = cl._socks[0].getsockname()[1]
            before = cl.stats()["foreign"]
            for _ in range(5):
                rogue.sendto(P.encode(evil), ("127.0.0.1", port))
            time.sleep(0.15)
            s = cl.get_state()
            ck("packets from any other address are dropped",
               cl.stats()["foreign"] >= before + 5 and s is not None
               and s.sticks["rv"] == 1.0)
            rogue.close()

        rc.muted = True                         # the cable is pulled: no bye
        t0 = P.now()
        ok = _wait(lambda: cl.get_state() is None, 1.0, step=0.005)
        dt = P.now() - t0
        ck("silence -> None within stale_after + 0.15 s (%.2f s)" % dt,
           ok and dt <= cl.stale_after + 0.15)
        ck("... and says why (%s)" % _reason(cl), "stale" in _reason(cl))
        rc.muted = False
        ck("... and is usable again when it resumes",
           _wait(lambda: cl.get_state() is not None, 0.5))

        t0 = P.now()
        rc.stop(bye=True)
        ok = _wait(lambda: cl.get_state() is None, 0.5, step=0.002)
        dt = P.now() - t0
        ck("bye -> None at once (%.3f s)" % dt, ok and dt < 0.1)
        ck("... and says why (%s)" % _reason(cl), "bye" in _reason(cl))
    finally:
        cl.stop()
        rc.stop(bye=False)


def t_rates(ck):
    """The fresh-sample count: what the Ethernet rate check reads against the app."""
    rc = FakeRc(bind="127.0.0.1", port=0)
    rc.sweep_hz = 70                    # the gamepad's fast mode
    rc.start()
    cl = RcJoystickClient("127.0.0.1", port=rc.port, rate_hz=100, client_name="selftest")
    seen = []
    cl.on_state(lambda s, fresh: seen.append(fresh))
    cl.start()
    try:
        _wait(lambda: cl.get_state() is not None, 1.0)
        time.sleep(0.3)
        rc0, cl0 = rc.fresh_sent, cl.stats()["fresh"]
        time.sleep(1.5)
        rc1, st = rc.fresh_sent, cl.stats()
        sent, got = rc1 - rc0, st["fresh"] - cl0
        ck("the wire runs at the rate asked for, not the input's (%.1f Hz, 100 asked)"
           % st["rx_hz"], st["rx_hz"] >= 75)
        ck("every changed state the RC sent is counted fresh (sent %d, counted %d)"
           % (sent, got), sent > 50 and abs(sent - got) <= 2)
        ck("fresh follows the 70/s input, below the wire rate (%.1f/s)" % st["fresh_hz"],
           40 <= st["fresh_hz"] <= 72 and st["fresh_hz"] <= st["rx_hz"])
        ck("on_state sees every accepted state with its fresh flag (%d states, %d fresh)"
           % (len(seen), sum(seen)),
           abs(len(seen) - st["packets"]) <= 2 and abs(sum(seen) - st["fresh"]) <= 2)
        rc.sweep_hz = None              # inputs freeze where they are
        time.sleep(2.3)                 # the rate window (2 s) empties of sweep samples
        st = cl.stats()
        ck("held still: fresh reads 0 while the wire keeps running (fresh %.1f, wire %.1f)"
           % (st["fresh_hz"], st["rx_hz"]), st["fresh_hz"] == 0 and st["rx_hz"] >= 75)
    finally:
        cl.stop()
        rc.stop(bye=False)


class _InfoStub:
    def info(self):
        return {"gamepad": {"hz": 70, "gap_ms": 14}}


def t_csv(ck):
    path = os.path.join(tempfile.mkdtemp(prefix="rcjoy-selftest-"), "run.csv")
    log = CsvLog(path, _InfoStub())
    a = _st({"rv": 330})
    a.received_at, a.seq = log._t0 + 0.010, 7
    b = _st({"rv": 340})
    b.received_at, b.seq = log._t0 + 0.020, 8
    b.dials["r"] = b.raw_dials["r"] = None
    log.write(a, False)
    log.write(b, True)
    log.close()
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    ck("csv: a header, then one row per state",
       len(rows) == 3 and tuple(rows[0]) == CsvLog.COLUMNS)
    r = dict(zip(rows[0], rows[2])) if len(rows) == 3 else {}
    ck("csv: seq, fresh, raw inputs and the app's gamepad figures ride along",
       r.get("seq") == "8" and r.get("fresh") == "1" and r.get("rv") == "340"
       and r.get("app_gp_hz") == "70" and r.get("app_gp_gap_ms") == "14")
    ck("csv: a null input is an empty cell, never a zero", r.get("r") == "")
    ck("csv: each row carries the PC's wall-clock time (%s)" % r.get("wall"),
       re.match(r"^\d\d:\d\d:\d\d\.\d{3}$", r.get("wall") or "") is not None)
    ck("csv: the summary counts states and fresh ones (%s)" % log.summary(),
       log.summary().startswith("csv: 2 states, 1 fresh"))
    again = CsvLog(path, _InfoStub())         # the same name for a second run
    again.close()
    with open(path, newline="") as f:
        kept = len(list(csv.reader(f)))
    ck("csv: reusing a name never overwrites a run (%s)" % os.path.basename(again.path),
       again.path.endswith("run-2.csv") and kept == 3)
    for p in (path, again.path):
        os.remove(p)
    os.rmdir(os.path.dirname(path))


def t_version(ck):
    rc = FakeRc(bind="127.0.0.1", port=0, version=2).start()
    cl = RcJoystickClient("127.0.0.1", port=rc.port).start()
    try:
        time.sleep(0.6)
        ev = cl.pop_events()
        ck("a v2 RC is REJECTED and never read",
           cl.latest() is None and any("REJECTED" in e and "v2" in e for e in ev))
    finally:
        cl.stop()
        rc.stop(bye=False)


def t_discovery(ck):
    try:
        a = FakeRc(bind="127.0.0.2", port=0).start()
        b = FakeRc(bind="127.0.0.3", port=a.port).start()
    except OSError:
        print("  skip  discovery (cannot bind 127.0.0.2/.3 here)")
        return
    targets = [("127.0.0.2", a.port), ("127.0.0.3", a.port)]
    cl = RcJoystickClient(None, port=a.port, local_addrs=["127.0.0.1"],
                          discovery_targets=targets).start()
    try:
        time.sleep(1.3)
        r = _reason(cl)
        ck("two RCs answering -> no lock, both named (%s)" % r,
           cl.locked_rc() is None and "127.0.0.2" in r and "127.0.0.3" in r)
        ck("... and nothing is usable meanwhile", cl.get_state() is None)
        b.stop(bye=True)
        ck("one RC answering -> it locks onto that one",
           _wait(lambda: cl.locked_rc() == "127.0.0.2", 3.0))
        ck("... and streams from it", _wait(lambda: cl.get_state() is not None, 1.0))
    finally:
        cl.stop()
        a.stop(bye=False)
        b.stop(bye=False)


def t_remember(ck):
    """The last RC used is saved, and tried by unicast alongside the broadcast."""
    try:
        a = FakeRc(bind="127.0.0.2", port=0).start()
        b = FakeRc(bind="127.0.0.3", port=a.port).start()
    except OSError:
        print("  skip  remember (cannot bind 127.0.0.2/.3 here)")
        return
    folder = tempfile.mkdtemp(prefix="rcjoy-selftest-")
    path = os.path.join(folder, "last_rc.txt")

    def client(targets):
        return RcJoystickClient(None, port=a.port, local_addrs=["127.0.0.1"],
                                discovery_targets=targets, remember=path).start()
    try:
        cl = client([("127.0.0.2", a.port)])
        try:
            ok = (_wait(lambda: cl.locked_rc() == "127.0.0.2", 3.0)
                  and _wait(lambda: read_remembered(path) == "127.0.0.2", 2.0))
        finally:
            cl.stop()
        ck("the RC it locks onto is remembered (%s)" % read_remembered(path), ok)

        cl = client([("127.0.0.9", a.port)])           # the "broadcast" finds nobody
        try:
            ck("the remembered RC is found by unicast when the broadcast finds none",
               _wait(lambda: cl.locked_rc() == "127.0.0.2", 3.0))
            ck("... and streams", _wait(lambda: cl.get_state() is not None, 1.0))
        finally:
            cl.stop()

        with open(path, "w") as f:
            f.write("127.0.0.3\n")
        cl = client([("127.0.0.2", a.port), ("127.0.0.3", a.port)])
        try:
            ck("two RCs answering: it stays with the remembered one",
               _wait(lambda: cl.locked_rc() == "127.0.0.3", 3.0))
        finally:
            cl.stop()

        with open(path, "w") as f:
            f.write("not an address\n")
        ck("a garbled memory is ignored, not trusted", read_remembered(path) is None)
        ck("--rc on this PC (a fake RC) never replaces the remembered real RC",
           remember_path_for("127.0.0.1") is None and remember_path_for("localhost") is None
           and remember_path_for("192.168.100.50") and remember_path_for(None))
    finally:
        a.stop(bye=False)
        b.stop(bye=False)
        if os.path.exists(path):
            os.remove(path)
        os.rmdir(folder)


def t_chooser(ck):
    """Several RCs answering: the operator is asked (a stand-in chooser here: no window)."""
    try:
        a = FakeRc(bind="127.0.0.2", port=0).start()
        b = FakeRc(bind="127.0.0.3", port=a.port).start()
    except OSError:
        print("  skip  chooser (cannot bind 127.0.0.2/.3 here)")
        return
    both = [("127.0.0.2", a.port), ("127.0.0.3", a.port)]
    folder = tempfile.mkdtemp(prefix="rcjoy-selftest-")
    path = os.path.join(folder, "last_rc.txt")

    def client(chooser):
        return RcJoystickClient(None, port=a.port, local_addrs=["127.0.0.1"],
                                discovery_targets=both, remember=path, chooser=chooser).start()
    calls = []
    try:
        def pick_3(options, remembered, cancelled):
            calls.append((options, remembered))
            return "127.0.0.3"
        cl = client(pick_3)
        try:
            ok = _wait(lambda: cl.locked_rc() == "127.0.0.3", 3.0)
            opts = calls[0][0] if calls else []
            ck("several RCs: the operator's pick is locked onto", ok)
            ck("... the pop-up is offered each RC with its serial (%s)"
               % ", ".join(ip for ip, _ in opts),
               [ip for ip, _ in opts] == ["127.0.0.2", "127.0.0.3"]
               and all(info.get("sn") == "FAKE0001" for _, info in opts))
            ck("... and the pick is remembered", _wait(lambda: read_remembered(path) == "127.0.0.3", 2.0))
        finally:
            cl.stop()

        calls[:] = []
        cl = client(lambda options, remembered, cancelled: calls.append(remembered))
        try:
            time.sleep(2.6)                         # three discovery rounds
            ck("'Not now': nothing is locked, and the same RCs are not asked about again (%d)"
               % len(calls), cl.locked_rc() is None and len(calls) == 1)
            ck("... the status says why (%s)" % _reason(cl), "pop-up" in _reason(cl))
            ck("... and the pop-up was told which RC was used last", calls == ["127.0.0.3"])
        finally:
            cl.stop()

        def unavailable(options, remembered, cancelled):
            raise ChooserUnavailable("no desktop")
        cl = client(unavailable)
        try:
            ck("no pop-up possible: it falls back to the one used last",
               _wait(lambda: cl.locked_rc() == "127.0.0.3", 3.0))
        finally:
            cl.stop()

        # A slow choice must not count as the RC being silent (RELOCK_AFTER_S shrunk to 1 s).
        import rcjoy.client as client_module
        saved, client_module.RELOCK_AFTER_S = client_module.RELOCK_AFTER_S, 1.0
        calls[:] = []

        def slow(options, remembered, cancelled):
            calls.append(1)
            time.sleep(1.5)
            return "127.0.0.2"
        cl = client(slow)
        try:
            ok = _wait(lambda: cl.locked_rc() == "127.0.0.2", 4.0)
            time.sleep(1.5)
            ck("a slow choice is not taken for silence: locked once, streaming (%d asked)"
               % len(calls), ok and len(calls) == 1 and cl.get_state() is not None)
        finally:
            cl.stop()
            client_module.RELOCK_AFTER_S = saved
    finally:
        a.stop(bye=False)
        b.stop(bye=False)
        if os.path.exists(path):
            os.remove(path)
        os.rmdir(folder)

    text = chooser_label("192.168.100.50", {"rc_type": "DJI_RC_PRO", "sn": "4QQZ", "battery": 86,
                                            "app": "1.6"}, last_used=True)
    ck("pop-up labels lead with the IP and carry the serial and battery",
       text.startswith("192.168.100.50") and "last used" in text and "sn 4QQZ" in text
       and "battery 86%" in text)
    blob = chooser_payload([("10.0.0.1", {"sn": "été"})], "10.0.0.1")
    ck("the pop-up's input is plain ASCII, whatever an RC reports", all(ord(c) < 128 for c in blob))


# --- bridge ----------------------------------------------------------------------------

class _Stub:
    """Just enough client for Bridge.tick(): a settable get_state()."""
    state = None

    def get_state(self):
        return self.state


def _st(sticks=None, dials=None, presses=None):
    rs = {k: 0 for k in P.STICKS}
    rs.update(sticks or {})
    rd = {k: 0 for k in P.DIALS}
    rd.update(dials or {})
    return P.RcState(seq=0, t_ms=0, rc_ok=True, aircraft_linked=False,
                     sticks={k: P.norm(v) for k, v in rs.items()},
                     dials={k: P.norm(v) for k, v in rd.items()},
                     buttons={k: False for k in P.BUTTONS},
                     presses=dict(presses or {}), mode_switch=None,
                     raw_sticks=rs, raw_dials=rd)


def t_bridge(ck):
    cl = _Stub()
    br = Bridge(cl)                             # Mode 2, flocking profile
    ck("nothing is sent while the RC is not usable", br.tick(0.0) is None)
    cl.state = _st({"rv": 660, "rh": -330, "lh": 165, "lv": 330})
    m = br.tick(0.0)
    ck("Mode 2: right V -> linear.x, right H -> linear.y, left H -> angular.z",
       m["linear"]["x"] == 1.0 and m["linear"]["y"] == -0.5 and m["angular"]["z"] == 0.25)
    ck("climb is quadratic like readController.py (0.5 -> 0.25)", m["linear"]["z"] == 0.25)
    ck("starts at angular.x 1.0, angular.y 0.0, s1 +1, s2 0.0",
       m["angular"]["x"] == 1.0 and m["angular"]["y"] == 0.0
       and m["switches"]["s1"] == 1 and m["switches"]["s2"] == 0.0)
    payload = json.dumps(m)
    ck("s1 is a JSON integer, as Unity's Switches.s1 (int) needs",
       type(json.loads(payload)["switches"]["s1"]) is int and '"s1": 1,' in payload)
    ck("the message has exactly readController.py's shape",
       set(m) == {"linear", "angular", "switches"}
       and set(m["linear"]) == set(m["angular"]) == {"x", "y", "z"}
       and set(m["switches"]) == {"s1", "s2"})
    cl.state = _st({"lv": -660})
    ck("full-down climb is -1.0", br.tick(0.05)["linear"]["z"] == -1.0)

    cl.state = _st({"lv": 660, "rv": 330})
    m = Bridge(cl, stick_mode=1).tick(0.0)
    ck("Mode 1: left V -> forward, right V -> climb",
       m["linear"]["x"] == 1.0 and m["linear"]["z"] == 0.25)
    cl.state = _st({"lh": 660, "rh": -660})
    m = Bridge(cl, stick_mode=3).tick(0.0)
    ck("Mode 3: left H -> right, right H -> yaw",
       m["linear"]["y"] == 1.0 and m["angular"]["z"] == -1.0)

    # knobs
    br, t = Bridge(cl, knob_sweep_s=3.0), 0.0
    cl.state = _st(dials={"r": 660})
    br.tick(t)
    for _ in range(15):
        t += 0.05
        m = br.tick(t)
    ck("a full dial sweeps the knob's range in knob_sweep_s (0.75 s -> 1.2)",
       abs(m["angular"]["x"] - 1.2) < 1e-6)
    for _ in range(400):
        t += 0.05
        m = br.tick(t)
    ck("angular.x clamps at 1.4", m["angular"]["x"] == 1.4)
    cl.state = _st(dials={"l": -660})
    ints, peak = set(), 0.0
    for _ in range(400):
        t += 0.05
        m = br.tick(t)
        ints.add(int(m["switches"]["s2"]))
        peak = max(peak, abs(m["switches"]["s2"]))
    ck("s2 clamps at -0.999 and never reaches +-1.0",
       m["switches"]["s2"] == -0.999 and peak < 1.0)
    ck("int(s2) stays 0, so JoystickReceiver can never see a LAND edge", ints == {0})
    cl.state = _st(dials={"r": int(0.04 * 660)})
    ax0 = br.ax
    for _ in range(100):
        t += 0.05
        br.tick(t)
    ck("a dial inside the 0.05 deadband leaves its knob alone", br.ax == ax0)
    cl.state = _st(dials={"r": -660})
    br.tick(t)
    ax1 = br.ax
    t += 5.0
    br.tick(t)
    ck("a long gap between ticks is capped: no knob jump",
       abs(br.ax - ax1) <= 0.8 / 3.0 * 0.2 + 1e-9)

    # presses, C1/C2, stale behaviour
    br, t = Bridge(cl), 0.0
    cl.state = _st(presses={"c1": 3, "c2": 0})
    ck("presses already counted at start do nothing", br.tick(t)["switches"]["s1"] == 1)
    cl.state = _st(presses={"c1": 4, "c2": 0})
    t += 0.05
    ck("C1 toggles s1 to -1", br.tick(t)["switches"]["s1"] == -1)
    cl.state = _st(presses={"c1": 6, "c2": 0})
    t += 0.05
    ck("two C1 presses in one tick cancel out", br.tick(t)["switches"]["s1"] == -1)
    cl.state = _st(dials={"r": 660, "l": 660}, presses={"c1": 6, "c2": 0})
    for _ in range(10):
        t += 0.05
        br.tick(t)
    cl.state = _st(presses={"c1": 6, "c2": 1})
    t += 0.05
    m = br.tick(t)
    ck("C2 resets both knobs", m["angular"]["x"] == 1.0 and m["switches"]["s2"] == 0.0)
    cl.state = None
    t += 0.05
    ck("stale -> no output", br.tick(t) is None)
    ax_before = br.ax
    cl.state = _st(dials={"r": 660}, presses={"c1": 9, "c2": 1})
    t += 2.0
    m = br.tick(t)
    ck("after a stale spell: no knob jump, and a press made meanwhile never lands late",
       m["angular"]["x"] == ax_before and m["switches"]["s1"] == -1)

    st = _st(dials={"r": 660})
    st.dials["r"] = None
    cl.state = st
    br = Bridge(cl)
    br.tick(0.0)
    ck("a null dial leaves its knob where it is", br.tick(0.1)["angular"]["x"] == 1.0)

    cl.state = _st(dials={"l": 660, "r": -660})
    bj = Bridge(cl, profile="joystick")
    bj.tick(0.0)
    m = bj.tick(0.1)
    ck("joystick profile: the LEFT (gimbal) dial drives angular.x",
       m["angular"]["x"] > 1.0 and m["switches"]["s2"] < 0.0)

    # --ax-range: the Unity sim takes angular.x unclamped as its spread (readController 0.4..1.6)
    ck("--ax-range parses LO,HI", parse_range("0.4,1.6") == (0.4, 1.6))
    try:
        parse_range("0.4")
        ok = False
    except ValueError:
        ok = True
    ck("--ax-range rejects anything but LO,HI", ok)
    try:
        Bridge(cl, ax_range=(1.2, 1.6))
        ok = False
    except ValueError:
        ok = True
    ck("an --ax-range without the 1.0 start is refused", ok)
    cl.state = _st(dials={"r": 660})
    bw, t = Bridge(cl, ax_range=(0.4, 1.6)), 0.0
    bw.tick(t)
    for _ in range(15):
        t += 0.05
        m = bw.tick(t)
    ck("a wider range sweeps in the same knob_sweep_s (0.75 s -> 1.3)",
       abs(m["angular"]["x"] - 1.3) < 1e-6)
    for _ in range(400):
        t += 0.05
        top = bw.tick(t)["angular"]["x"]
    cl.state = _st(dials={"r": -660})
    for _ in range(400):
        t += 0.05
        bottom = bw.tick(t)["angular"]["x"]
    ck("--ax-range 0.4,1.6 reaches both ends (%.2f, %.2f)" % (bottom, top),
       top == 1.6 and bottom == 0.4)
    bs = Bridge(cl, profile="sim")
    ck("sim profile: flocking's dials with readController.py's 0.4..1.6 spacing",
       bs.dials["ax_dial"] == "r" and bs.dials["s2_dial"] == "l"
       and (bs.ax_lo, bs.ax_hi) == (0.4, 1.6))
    ck("an explicit --ax-range overrides the profile's",
       Bridge(cl, profile="sim", ax_range=(0.6, 1.4)).ax_lo == 0.6)
    ck("flocking keeps swarm_flocking.py's 0.6..1.4",
       (Bridge(cl).ax_lo, Bridge(cl).ax_hi) == (0.6, 1.4))


def t_bridge_e2e(ck):
    rc = FakeRc(bind="127.0.0.1", port=0).start()
    rc.set(rv=330, lv=660)
    cl = RcJoystickClient("127.0.0.1", port=rc.port).start()
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(("127.0.0.1", 0))
    sink.settimeout(0.05)
    br, stop = Bridge(cl), threading.Event()
    th = threading.Thread(target=br.run, args=([sink.getsockname()],),
                          kwargs=dict(rate_hz=20, stop_evt=stop, verbose=False),
                          daemon=True)
    try:
        _wait(lambda: cl.get_state() is not None, 1.0)
        th.start()
        got, end = [], P.now() + 1.0
        while P.now() < end:
            try:
                got.append(sink.recvfrom(4096)[0])
            except socket.timeout:
                pass
        ck("the bridge sends at ~20 Hz (%d in 1 s)" % len(got), 14 <= len(got) <= 22)
        ok = bool(got)
        for d in got:
            # read it the way AOS server's JoystickReceiver does
            j = json.loads(d.decode())
            lin, ang, sw = j["linear"], j["angular"], j["switches"]
            ok = ok and (float(lin["x"]) == 0.5 and float(lin["z"]) == 1.0
                         and float(ang["x"]) == 1.0 and int(sw["s1"]) == 1
                         and int(sw["s2"]) == 0 and type(sw["s1"]) is int)
        ck("every datagram reads like readController.py's JSON", ok)
        rc.muted = True
        t0, last = P.now(), None
        end = t0 + 1.0
        while P.now() < end:
            try:
                sink.recvfrom(4096)
                last = P.now()
            except socket.timeout:
                pass
        quiet = (last or t0) - t0
        ck("output stops within stale_after + one period of the RC going quiet (%.2f s)"
           % quiet, quiet <= cl.stale_after + 0.1)
    finally:
        stop.set()
        if th.is_alive():
            th.join(1.0)
        cl.stop()
        rc.stop(bye=False)
        sink.close()


def main(_args=None):
    ck = _Checks()
    t_start = P.now()
    for title, fn in (("protocol", t_protocol), ("client <-> fake RC", t_client),
                      ("rates: wire vs fresh", t_rates), ("monitor csv", t_csv),
                      ("version mismatch", t_version), ("discovery", t_discovery),
                      ("remembered RC", t_remember), ("choosing among RCs", t_chooser),
                      ("bridge mapping", t_bridge), ("bridge end to end", t_bridge_e2e)):
        print("[%s]" % title, flush=True)
        try:
            fn(ck)
        except Exception as e:
            traceback.print_exc()
            ck("%s ran without an exception (%r)" % (title, e), False)
    took = P.now() - t_start
    if ck.failed:
        print("\nselftest: %d of %d checks FAILED (%.1f s):" % (len(ck.failed), ck.n, took))
        for name in ck.failed:
            print("  - " + name)
        return 1
    print("\nselftest: all %d checks passed (%.1f s)" % (ck.n, took))
    return 0
