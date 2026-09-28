"""rc-joystick wire protocol v1: constants, encode/decode, validation, normalization.

The contract is rc-joystick/PROTOCOL.md; keep this module and that document in step.
Parsing is deliberately strict. A malformed field from the app raises ProtocolError
rather than being defaulted, because a silently defaulted stick is a stick at zero.
"""

import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

VERSION = 1
DEFAULT_PORT = 5070             # the RC listens here (free in this repo)
RAW_FULL_SCALE = 660            # MSDK stick/dial range is +-660
SUBSCRIBE_PERIOD_S = 1.0        # PC -> RC keepalive
SUBSCRIPTION_TTL_S = 3.0        # the RC drops a subscriber this long after its last subscribe
RATE_MIN_HZ = 10
RATE_MAX_HZ = 100
RATE_DEFAULT_HZ = 50
MAX_INFO_BYTES = 1200           # `info` must fit one unfragmented datagram
RECV_BUFFER = 8192

# The one clock every rcjoy module times with. Not time.monotonic(): on Windows that is
# GetTickCount64, 15.625 ms per tick, which reads a 2 ms RTT as 0 or 15.6 and cannot
# resolve the gamepad's ~14 ms report spacing. perf_counter is QueryPerformanceCounter
# there (sub-microsecond) and is monotonic everywhere.
now = time.perf_counter

STICKS = ("lh", "lv", "rh", "rv")
DIALS = ("l", "r")
BUTTONS = ("c1", "c2", "c3", "shutter", "record", "rth", "pause", "rc_switch",
           "5d_up", "5d_down", "5d_left", "5d_right", "5d_press")


class ProtocolError(ValueError):
    """A datagram that is not a valid v1 message."""


class VersionMismatch(ProtocolError):
    """A well-formed message from a peer speaking another protocol version."""


def encode(msg):
    return json.dumps(msg, separators=(",", ":")).encode("utf-8")


def decode(data):
    """bytes -> message dict. Raises ProtocolError / VersionMismatch."""
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as e:
        raise ProtocolError("not UTF-8 JSON: %s" % e)
    if not isinstance(msg, dict):
        raise ProtocolError("not a JSON object")
    if not isinstance(msg.get("type"), str):
        raise ProtocolError("no 'type'")
    if "v" not in msg:
        raise ProtocolError("no 'v'")
    v = msg["v"]
    if isinstance(v, bool) or v != VERSION:
        raise VersionMismatch("peer speaks protocol v%s, this side speaks v%d"
                              % (v, VERSION))
    return msg


# --- message builders (PC side; fake_rc also builds the RC side) ---------------

def subscribe(client, rate_hz, t):
    return {"type": "subscribe", "v": VERSION, "client": str(client),
            "rate_hz": int(rate_hz), "t": float(t)}


def unsubscribe():
    return {"type": "unsubscribe", "v": VERSION}


def bye(reason=""):
    return {"type": "bye", "v": VERSION, "reason": str(reason)}


def clamp_rate(rate_hz):
    try:
        r = int(rate_hz)
    except (TypeError, ValueError):
        return RATE_DEFAULT_HZ
    return max(RATE_MIN_HZ, min(RATE_MAX_HZ, r))


# --- normalization --------------------------------------------------------------

def norm(raw):
    """Raw +-660 -> [-1, 1], clamped (calibration can overshoot). None stays None."""
    if raw is None:
        return None
    x = raw / float(RAW_FULL_SCALE)
    return -1.0 if x < -1.0 else 1.0 if x > 1.0 else x


# --- parsing --------------------------------------------------------------------

def _num_or_none(v, what):
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ProtocolError("%s: expected a number or null, got %r" % (what, v))
    return v


def _bool_or_none(v, what):
    if v is None or isinstance(v, bool):
        return v
    raise ProtocolError("%s: expected true/false/null, got %r" % (what, v))


def _group(msg, name, keys, conv):
    g = msg.get(name)
    if g is None:
        g = {}
    if not isinstance(g, dict):
        raise ProtocolError("%s: expected an object, got %r" % (name, g))
    return {k: conv(g.get(k), "%s.%s" % (name, k)) for k in keys}


def _require(msg, key, types):
    """msg[key], which must be of `types`. A bool is never accepted as a number."""
    v = msg.get(key)
    if types is bool:
        ok = isinstance(v, bool)
    else:
        ok = isinstance(v, types) and not isinstance(v, bool)
    if not ok:
        raise ProtocolError("%s: bad or missing (%r)" % (key, v))
    return v


@dataclass
class RcState:
    """One decoded `state`. Sticks/dials are normalized to [-1, 1]; None = not served."""
    seq: int
    t_ms: float
    rc_ok: bool
    aircraft_linked: bool
    sticks: Dict[str, Optional[float]]
    dials: Dict[str, Optional[float]]
    buttons: Dict[str, Optional[bool]]
    presses: Dict[str, int]
    mode_switch: Optional[str]
    raw_sticks: Dict[str, Optional[float]]
    raw_dials: Dict[str, Optional[float]]
    received_at: float = 0.0        # PC now() (perf_counter) when it arrived
    # Where the sticks/dials came from: "gamepad" (the RC's built-in HID, ~70 Hz) or
    # "msdk" (~10 Hz, the fallback). None from an app older than 1.5.
    stick_src: Optional[str] = None

    def missing_sticks(self) -> List[str]:
        return [k for k in STICKS if self.sticks.get(k) is None]


def parse_state(msg, received_at=0.0):
    """A decoded `state` message -> RcState. Raises ProtocolError on any bad field."""
    seq = _require(msg, "seq", int)
    if seq < 0:
        raise ProtocolError("seq: negative")
    t_ms = _require(msg, "t_ms", (int, float))
    rc_ok = _require(msg, "rc_ok", bool)
    linked = _require(msg, "aircraft_linked", bool)
    raw_sticks = _group(msg, "sticks", STICKS, _num_or_none)
    raw_dials = _group(msg, "dials", DIALS, _num_or_none)
    buttons = _group(msg, "buttons", BUTTONS, _bool_or_none)
    pr = msg.get("presses") or {}
    if not isinstance(pr, dict):
        raise ProtocolError("presses: expected an object")
    presses = {}
    for k in BUTTONS:
        n = pr.get(k)
        if n is None:
            continue
        if isinstance(n, bool) or not isinstance(n, int) or n < 0:
            raise ProtocolError("presses.%s: expected a count, got %r" % (k, n))
        presses[k] = n
    ms = msg.get("mode_switch")
    if ms is not None and not isinstance(ms, str):
        raise ProtocolError("mode_switch: expected a string or null")
    src = msg.get("stick_src")      # optional: added in app 1.5, within v1
    if src is not None and not isinstance(src, str):
        raise ProtocolError("stick_src: expected a string or null")
    return RcState(
        seq=seq, t_ms=t_ms, rc_ok=rc_ok, aircraft_linked=linked,
        sticks={k: norm(v) for k, v in raw_sticks.items()},
        dials={k: norm(v) for k, v in raw_dials.items()},
        buttons=buttons, presses=presses, mode_switch=ms,
        raw_sticks=raw_sticks, raw_dials=raw_dials, received_at=received_at,
        stick_src=src)


class PressCounter:
    """Turns the RC's cumulative `presses` counts into press events.

    The first update after construction or reset() only takes a baseline, so a
    press made before we were listening (or while the feed was stale) never fires
    late. A count that goes DOWN means the RC app restarted: re-baseline and fire
    nothing. A button seen for the first time is baselined, not fired.
    """

    def __init__(self):
        self._base = None

    def reset(self):
        self._base = None

    def update(self, presses):
        """Return {button: new presses since the last update}."""
        if self._base is None:
            self._base = dict(presses)
            return {}
        if any(n < self._base.get(k, 0) for k, n in presses.items()):
            self._base = dict(presses)
            return {}
        out = {}
        for k, n in presses.items():
            b = self._base.get(k)
            if b is not None and n > b:
                out[k] = n - b
        self._base = dict(presses)
        return out
