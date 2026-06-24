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
"""

import json
import socket
import threading
import time
from dataclasses import dataclass


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


if __name__ == "__main__":
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
