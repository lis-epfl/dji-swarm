"""
Per-drone command->response rotation monitor
============================================
Continuously estimates, per drone, the ROTATION and GAIN between the velocity
commands actually sent (world N/E m/s, post --slow scaling) and the velocity
the aircraft actually flew (derived from fresh GPS fixes). A healthy drone
sits near rot 0 deg / gain 1. The 2026-07-05 six-drone flight failed because
several aircraft executed their commands rotated 90-180 deg (suspected FC
yaw/compass-estimate error) — this monitor makes that visible live and in the
flight log instead of requiring offline analysis.

Method (same fit used in the post-flight analysis of that flight): treat 2D
vectors as complex numbers z = north + 1j*east. Over a sliding window, build
GPS velocity segments from consecutive FRESH fixes, pair each segment with the
command that was in effect `lag_s` earlier, and least-squares fit a single
complex factor a with  response ~= a * command:

    a = sum(resp * conj(cmd)) / sum(|cmd|^2)

angle(a) is the rotation (deg, positive = response rotated clockwise/toward-
east from the command, compass sense), |a| is the gain. Segments with a
near-zero command carry no rotation information and are excluded, so the
estimate simply goes stale (None) while the swarm hovers.

Threading: NOT thread-safe by design — all calls must come from the one
control-loop thread (the same rule as the rest of swarm_flocking.run()).

Pure Python (stdlib only), no ds_wrapper import; Python 3.7-compatible.
"""

import math
from collections import deque

from olfati_saber import gps_to_local


# Sliding window the fit runs over. Long enough for ~15-20 fresh GPS samples
# at the app's ~5 Hz fresh-telemetry rate, short enough to catch a drone going
# bad within a few seconds.
DEFAULT_WINDOW_S = 4.0
# Actuation lag: commanded velocity takes roughly this long to show up in the
# GPS track (app relay + FC ramp + GPS filtering; matches the lag that gave
# the cleanest fits in the 2026-07-05 offline analysis).
DEFAULT_LAG_S = 0.6
# Commands smaller than this (m/s) carry no usable direction signal — with
# --slow 0.2 a typical cohesion correction is still >= 0.1 m/s.
DEFAULT_MIN_CMD_MPS = 0.05
# Minimum velocity segments for a fit to be reported at all.
DEFAULT_MIN_SEGMENTS = 6
# Recompute at most this often per drone (the fit is cheap, but there is no
# point re-fitting on every 20 Hz tick when fresh fixes arrive at ~5 Hz).
DEFAULT_REFIT_S = 0.25


class ResponseMonitor:
    """Sliding-window rotation/gain estimator for every drone in the swarm."""

    def __init__(self, window_s=DEFAULT_WINDOW_S, lag_s=DEFAULT_LAG_S,
                 min_cmd_mps=DEFAULT_MIN_CMD_MPS,
                 min_segments=DEFAULT_MIN_SEGMENTS, refit_s=DEFAULT_REFIT_S):
        self.window_s = window_s
        self.lag_s = lag_s
        self.min_cmd_mps = min_cmd_mps
        self.min_segments = min_segments
        self.refit_s = refit_s
        # Local-metres reference shared by all drones, fixed at the first fix
        # (the fit differentiates positions, so any fixed ref works; the
        # per-tick drifting centroid ref used by the flocking loop would NOT —
        # its motion would alias into every velocity segment).
        self._ref = None
        self._fixes = {}      # did -> deque[(t, n, e)]
        self._last_latlon = {}  # did -> (lat, lon) for freshness dedupe
        self._cmds = {}       # did -> deque[(t, vn, ve)]
        self._cache = {}      # did -> (t_fitted, result-or-None)

    # ---- feeding (control-loop thread only) ----

    def note_command(self, did, t, v_n, v_e):
        """Record the velocity command SENT to drone `did` (world N/E m/s,
        after every scale/clamp — i.e. exactly what set_velocity got)."""
        dq = self._cmds.get(did)
        if dq is None:
            dq = self._cmds[did] = deque()
        dq.append((t, v_n, v_e))
        self._trim(dq, t)

    def note_fix(self, did, t, lat, lon):
        """Record a GPS fix for drone `did`. Call with every telemetry sample;
        stale repeats (identical lat/lon — ~75% of fetches at the app's ~5 Hz
        fresh rate) are dropped here so callers don't need to dedupe."""
        if self._last_latlon.get(did) == (lat, lon):
            return
        self._last_latlon[did] = (lat, lon)
        if self._ref is None:
            self._ref = (lat, lon)
        n, e = gps_to_local(lat, lon, self._ref[0], self._ref[1])
        dq = self._fixes.get(did)
        if dq is None:
            dq = self._fixes[did] = deque()
        if dq and t - dq[-1][0] < 0.05:
            return  # duplicate-timestamp guard
        dq.append((t, n, e))
        self._trim(dq, t)

    def _trim(self, dq, now):
        # Keep one extra lag's worth so segments at the window edge can still
        # find their matching command.
        horizon = now - (self.window_s + self.lag_s + 1.0)
        while dq and dq[0][0] < horizon:
            dq.popleft()

    # ---- fitting ----

    def fit(self, did, now):
        """Return (rot_deg, gain, n_segments) for drone `did`, or None when
        there is not enough recent commanded motion to estimate. Cached and
        recomputed at most every `refit_s` seconds."""
        cached = self._cache.get(did)
        if cached is not None and now - cached[0] < self.refit_s:
            return cached[1]
        result = self._fit_now(did, now)
        self._cache[did] = (now, result)
        return result

    def _fit_now(self, did, now):
        fixes = self._fixes.get(did)
        cmds = self._cmds.get(did)
        if not fixes or not cmds or len(fixes) < 2:
            return None
        num_re = num_im = den = 0.0
        n_seg = 0
        prev = None
        for cur in fixes:
            if prev is not None and cur[0] >= now - self.window_s:
                dt = cur[0] - prev[0]
                if 0.05 <= dt <= 1.0:
                    t_mid = 0.5 * (cur[0] + prev[0])
                    cmd = self._cmd_at(cmds, t_mid - self.lag_s)
                    if cmd is not None:
                        cn, ce = cmd
                        if math.hypot(cn, ce) >= self.min_cmd_mps:
                            vn = (cur[1] - prev[1]) / dt
                            ve = (cur[2] - prev[2]) / dt
                            # resp * conj(cmd), complex n + 1j*e
                            num_re += vn * cn + ve * ce
                            num_im += ve * cn - vn * ce
                            den += cn * cn + ce * ce
                            n_seg += 1
            prev = cur
        if n_seg < self.min_segments or den <= 0.0:
            return None
        rot_deg = math.degrees(math.atan2(num_im, num_re))
        gain = math.hypot(num_re, num_im) / den
        return rot_deg, gain, n_seg

    @staticmethod
    def _cmd_at(dq, t):
        """Latest command at or before time t (None if t predates them all)."""
        best = None
        for c in reversed(dq):
            if c[0] <= t:
                best = c
                break
        if best is None:
            return None
        if t - best[0] > 1.0:
            return None  # command stream had a gap; don't pair across it
        return best[1], best[2]
