"""
LIS_Swarm Flight Data Logger
============================
Records, to disk, everything that matters during a flight so it can be reviewed
afterwards (debugging, flocking-algorithm tuning, incident review). It captures
three streams plus an optional swarm breakdown:

  - user_commands.csv   the operator's joystick input (one row per UDP packet)
  - drone_commands.csv  every command actually sent to each drone (VS + events)
  - telemetry.csv       the parsed telemetry coming back from each drone
  - swarm_debug.csv      per-drone flocking decomposition (swarm_flocking only)

All four files live under a single timestamped session directory:

    flight_logs/flight_YYYYMMDD_HHMMSS/

Design notes
------------
* Stdlib only (csv, json, os, time, datetime, threading, queue) and **no
  ds_wrapper dependency**, so this module imports and can be unit-tested without
  the drone hardware or DroneSwarmServer running.
* The log* methods are called from the 20 Hz command / 10 Hz telemetry control
  threads, so they must never block. They only stamp a timestamp and do a
  non-blocking put onto a bounded queue; a single daemon writer thread does all
  the disk I/O. If the queue ever fills (writer can't keep up), rows are dropped
  and counted rather than stalling a control loop.
* Python 3.7 compatible.

Usage (from a launch script):

    from flight_logger import FlightLogger
    logger = FlightLogger(meta={"script": "joystick_controller", ...})
    drone.logger = logger                 # DroneController logs VS/EVENT/telemetry
    receiver = JoystickReceiver(logger=logger)   # logs user commands
    ...
    logger.close()                        # flush + close in a finally block
"""

import csv
import json
import os
import queue
import threading
import time
from datetime import datetime


# Telemetry field order, matching joystick_controller.parse_telemetry().
TELEMETRY_FIELDS = [
    'lat', 'lon', 'alt', 'heading',
    'gimbal_pitch', 'gimbal_roll', 'gimbal_yaw', 'sat_count',
    'drone_pitch', 'drone_roll', 'drone_yaw',
    'vx', 'vy', 'vz',
    'waypoint_done', 'value_check', 'vs_enabled',
]

# CSV column layout for each stream.
_USER_COLS = [
    't_epoch', 't_iso', 'source',
    'linear_x', 'linear_y', 'linear_z',
    'angular_x', 'angular_y', 'angular_z',
    's1', 's2', 'raw',
]
_DRONE_COLS = [
    't_epoch', 't_iso', 'drone_id', 'type',
    'pitch', 'roll', 'yaw', 'throttle',
    'gimbal_pitch', 'gimbal_yaw', 'cmd',
]
_TELEM_COLS = ['t_epoch', 't_iso', 'drone_id'] + TELEMETRY_FIELDS
_SWARM_COLS = [
    't_epoch', 't_iso', 'drone_id',
    'v_n_des', 'v_e_des', 'v_n_corr', 'v_e_corr',
    'v_n_total', 'v_e_total', 'd_ref', 'n_neighbours',
]

# Stream name -> (filename, column list).
_STREAMS = {
    'user': ('user_commands.csv', _USER_COLS),
    'drone': ('drone_commands.csv', _DRONE_COLS),
    'telemetry': ('telemetry.csv', _TELEM_COLS),
    'swarm': ('swarm_debug.csv', _SWARM_COLS),
}

# Max rows buffered before the writer thread; ~20k rows is several seconds of
# headroom at the combined ~150 row/s rate for a small swarm.
_QUEUE_MAXSIZE = 20000

# How often the writer flushes open files to disk (seconds). A crash loses at
# most this much data.
_FLUSH_INTERVAL = 1.0

# Sentinel pushed by close() to tell the writer thread to drain and exit.
_SENTINEL = object()


class FlightLogger:
    """Buffered, thread-safe flight-data logger writing CSV streams to disk."""

    def __init__(self, base_dir="flight_logs", meta=None):
        """
        Args:
            base_dir: parent directory for session folders (created if absent).
            meta:     JSON-serializable dict describing the run (script name,
                      args, ...). Stored in session.json alongside a column
                      legend and the start time.
        """
        self.session_name = "flight_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(base_dir, self.session_name)
        os.makedirs(self.session_dir, exist_ok=True)

        self._queue = queue.Queue(maxsize=_QUEUE_MAXSIZE)
        self.dropped = 0          # rows dropped because the queue was full
        self._written = 0

        # Open one file + DictWriter per stream and write headers immediately.
        self._files = {}
        self._writers = {}
        for stream, (fname, cols) in _STREAMS.items():
            f = open(os.path.join(self.session_dir, fname), 'w', newline='')
            wr = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            wr.writeheader()
            f.flush()
            self._files[stream] = f
            self._writers[stream] = wr

        self._write_session_json(meta)

        self._running = True
        self._thread = threading.Thread(
            target=self._writer_loop, daemon=True, name="FlightLogger")
        self._thread.start()

    # ------------------------------------------------------------------ #
    # Producer API (called from control threads; must not block)
    # ------------------------------------------------------------------ #

    def log_user_command(self, state, source="udp", raw=""):
        """Log one operator input. `state` is a JoystickState (or None for CLI,
        where `raw` holds the typed command line)."""
        row = {'source': source, 'raw': raw}
        if state is not None:
            row.update({
                'linear_x': state.linear_x,
                'linear_y': state.linear_y,
                'linear_z': state.linear_z,
                'angular_x': state.angular_x,
                'angular_y': state.angular_y,
                'angular_z': state.angular_z,
                's1': state.s1,
                's2': state.s2,
            })
        self._enqueue('user', row)

    def log_drone_command(self, drone_id, type_, pitch=None, roll=None, yaw=None,
                          throttle=None, gimbal_pitch=None, gimbal_yaw=None, cmd=None):
        """Log a command sent to a drone. type_ is "VS" (virtual-stick state) or
        "EVENT" (ENABLE_VS/DISABLE_VS/TAKEOFF/LAND); cmd is the raw string sent."""
        self._enqueue('drone', {
            'drone_id': drone_id,
            'type': type_,
            'pitch': pitch,
            'roll': roll,
            'yaw': yaw,
            'throttle': throttle,
            'gimbal_pitch': gimbal_pitch,
            'gimbal_yaw': gimbal_yaw,
            'cmd': cmd,
        })

    def log_telemetry(self, drone_id, telem):
        """Log one parsed telemetry dict for a drone."""
        row = {'drone_id': drone_id}
        for field in TELEMETRY_FIELDS:
            row[field] = telem.get(field)
        self._enqueue('telemetry', row)

    def log_swarm_debug(self, drone_id, v_n_des, v_e_des, v_n_corr, v_e_corr,
                        v_n_total, v_e_total, d_ref, n_neighbours):
        """Log the Olfati-Saber decomposition for one drone (swarm mode only)."""
        self._enqueue('swarm', {
            'drone_id': drone_id,
            'v_n_des': v_n_des,
            'v_e_des': v_e_des,
            'v_n_corr': v_n_corr,
            'v_e_corr': v_e_corr,
            'v_n_total': v_n_total,
            'v_e_total': v_e_total,
            'd_ref': d_ref,
            'n_neighbours': n_neighbours,
        })

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _enqueue(self, stream, row):
        """Stamp the row with the current time and hand it to the writer thread.
        Never blocks: if the queue is full the row is dropped and counted."""
        now = time.time()
        row['t_epoch'] = now
        row['t_iso'] = datetime.now().isoformat(timespec='milliseconds')
        try:
            self._queue.put_nowait((stream, row))
        except queue.Full:
            self.dropped += 1

    def _writer_loop(self):
        """Drain the queue, writing each row through its stream's DictWriter and
        flushing all files at most once per _FLUSH_INTERVAL."""
        last_flush = time.time()
        while True:
            try:
                item = self._queue.get(timeout=_FLUSH_INTERVAL)
            except queue.Empty:
                item = None

            if item is _SENTINEL:
                break
            if item is not None:
                stream, row = item
                writer = self._writers.get(stream)
                if writer is not None:
                    try:
                        writer.writerow(row)
                        self._written += 1
                    except Exception as e:
                        # Never let a bad row kill the writer thread.
                        print("[FlightLogger] write error: {}".format(e), flush=True)

            now = time.time()
            if now - last_flush >= _FLUSH_INTERVAL:
                self._flush_all()
                last_flush = now

    def _flush_all(self):
        for f in self._files.values():
            try:
                f.flush()
            except (OSError, ValueError):
                pass

    def _write_session_json(self, meta):
        legend = {stream: cols for stream, (_fname, cols) in _STREAMS.items()}
        info = {
            'session': self.session_name,
            'started_iso': datetime.now().isoformat(timespec='seconds'),
            'started_epoch': time.time(),
            'meta': meta or {},
            'columns': legend,
        }
        path = os.path.join(self.session_dir, 'session.json')
        with open(path, 'w') as f:
            json.dump(info, f, indent=2, default=str)

    def close(self):
        """Flush remaining rows, stop the writer thread, and close all files."""
        if not self._running:
            return
        self._running = False
        # Drain anything queued, then signal the writer to exit.
        try:
            self._queue.put(_SENTINEL, timeout=2)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._flush_all()
        for f in self._files.values():
            try:
                f.close()
            except (OSError, ValueError):
                pass
        msg = "[FlightLogger] saved {} rows to {}".format(self._written, self.session_dir)
        if self.dropped:
            msg += " ({} rows dropped — writer fell behind)".format(self.dropped)
        print(msg, flush=True)


if __name__ == "__main__":
    # Standalone smoke test: write a few rows of each stream and close.
    from dataclasses import dataclass

    @dataclass
    class _FakeState:
        linear_x: float = 0.1
        linear_y: float = -0.2
        linear_z: float = 0.0
        angular_x: float = 1.0
        angular_y: float = 0.0
        angular_z: float = 0.3
        s1: int = 0
        s2: int = 0

    log = FlightLogger(meta={"script": "flight_logger", "test": True})
    for i in range(5):
        log.log_user_command(_FakeState(), source="udp")
        log.log_drone_command(1, "VS", 0.1, -0.2, 90.0, 4.0, -10.0, 0.0,
                              cmd="VS:0.10:-0.20:90.00:4.00:-10.00:0.00")
        log.log_telemetry(1, {f: float(i) for f in TELEMETRY_FIELDS})
        log.log_swarm_debug(1, 0.1, -0.2, 0.01, 0.02, 0.11, -0.18, 0.5, 2)
        time.sleep(0.05)
    log.log_drone_command(1, "EVENT", cmd="ENABLE_VS")
    log.close()
    print("Smoke test complete — inspect:", log.session_dir)
