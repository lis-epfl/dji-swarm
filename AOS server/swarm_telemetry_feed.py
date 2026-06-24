"""
LIS_Swarm telemetry feed publisher
==================================
Pushes a JSON snapshot of every drone's parsed telemetry over local UDP to the
browser GUI server (swarm_gui.py).

This is the ONLY way the GUI receives data. The GUI deliberately never touches
ds_wrapper itself, so it can run unprivileged in its own terminal without
contending with this (the flight controller) process for the DroneSwarmServer
shared-memory protocol, which busy-waits on a status byte and does not tolerate
two processes poking it at once.

The flight controllers (joystick_controller.py, swarm_flocking.py) embed one of
these and call .start() once their telemetry threads are running. Publishing is
best-effort and fire-and-forget: a send failure (e.g. nothing listening yet) is
swallowed so it can never disturb the 20 Hz control loop.

Wire format — one UDP datagram per tick, JSON:
    {
      "t": 1719230000.123,                 # publisher send time (epoch seconds)
      "drones": {
        "1": { ...telemetry fields... },   # exactly what parse_telemetry produced
        "2": {},                           # empty dict == no telemetry yet
        ...
      }
    }
Telemetry field names/units match joystick_controller.parse_telemetry
(lat, lon, alt, heading, sat_count, vx, vy, vz, vs_enabled, gimbal_*, ...).
"""

import json
import socket
import threading
import time


DEFAULT_GUI_HOST = "127.0.0.1"
DEFAULT_GUI_PORT = 5099


class TelemetryFeedPublisher:
    """Background thread that periodically UDP-sends a telemetry snapshot."""

    def __init__(self, source, host=DEFAULT_GUI_HOST, port=DEFAULT_GUI_PORT,
                 rate_hz=5.0):
        """
        Args:
            source:  zero-arg callable returning {drone_id: telemetry_dict}.
                     Called every tick; a telemetry_dict may be empty for a
                     drone that has not produced a fix yet.
            host/port: UDP destination — the swarm_gui.py listener.
            rate_hz: publish rate (5 Hz is plenty for a map).
        """
        self._source = source
        self._addr = (host, int(port))
        self._interval = 1.0 / rate_hz if rate_hz > 0 else 0.2
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = False
        self._thread = None

    def _loop(self):
        while self._running:
            try:
                snap = self._source() or {}
                payload = {
                    "t": time.time(),
                    "drones": {str(did): (t or {}) for did, t in snap.items()},
                }
                self._sock.sendto(json.dumps(payload).encode("utf-8"), self._addr)
            except Exception:
                # The GUI is non-critical to flight; never propagate.
                pass
            time.sleep(self._interval)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                         name="GuiTelemetryFeed")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._sock.close()
        except Exception:
            pass
