"""
LIS_Swarm browser GUI server
============================
Serves a satellite map (default view: EPFL Lausanne) showing every drone in the
swarm with its live position + heading, a complete graph of inter-drone lines
with the distance between each pair labelled in metres, and a per-drone status
panel. Up to 10 drones; works fine with 1, 2, or any number.

DATA SOURCE
-----------
This server NEVER imports or touches ds_wrapper. It only LISTENS on a local UDP
port for telemetry pushed by the running flight controller
(joystick_controller.py / swarm_flocking.py), which embed
swarm_telemetry_feed.TelemetryFeedPublisher and publish by default.

That separation means you run this in its own terminal, with no elevation and
on any Python >= 3.7 (it does not need the cp37 wrapper):

    python swarm_gui.py
    # then open http://127.0.0.1:8000 in a browser

To view from a tablet/phone on the same network, bind all interfaces:
    python swarm_gui.py --http-host 0.0.0.0
    # then browse to http://<this-PC-ip>:8000

The map tiles (Esri World Imagery) are fetched from the internet, so the GUI PC
needs connectivity; no API key is required.
"""

import argparse
import json
import os
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Shared shapes validation + shapes.json persistence (olfati_saber.py is pure
# stdlib, no ds_wrapper — safe to import here). The GUI keeps its own store so
# obstacles/geofence can be drawn, saved, and shown with NO controller running.
from olfati_saber import (
    DEFAULT_SHAPES_FILE,
    MAX_OBSTACLES,
    normalize_obstacle,
    validate_fence,
    load_shapes,
    save_shapes,
)

# Demostitch offset bounds shared with the controller (heading_demostitch.py
# is pure stdlib, no ds_wrapper — safe to import here too).
from heading_demostitch import OFFSET_MIN_DEG, OFFSET_MAX_DEG

# Vertical-plane gain bounds, shared with the controller the same way (
# swarm_plane.py is pure stdlib too) so the GUI and the CLI cannot drift apart.
from swarm_plane import PLANE_GAIN_MIN, PLANE_GAIN_MAX


STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui")

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


class SwarmState:
    """Thread-safe store of the latest telemetry received over UDP."""

    def __init__(self):
        self._lock = threading.Lock()
        self._drones = {}        # id(str) -> {"telem", "send_hz", "recv_hz", "rx"}
        self._meta = {}          # swarm-level fields, e.g. {"d_ref_m": 5.0}
        self._last_packet = 0.0

    def update(self, payload):
        now = time.time()
        with self._lock:
            self._last_packet = now
            if "meta" in payload:
                self._meta = payload.get("meta") or {}
            for did, d in payload.get("drones", {}).items():
                # Accept both the rich {"telem":..., "send_hz":...} form and a
                # bare telemetry dict (older/simpler publishers).
                if isinstance(d, dict) and "telem" in d:
                    telem, send_hz, recv_hz = d.get("telem") or {}, d.get("send_hz"), d.get("recv_hz")
                else:
                    telem, send_hz, recv_hz = (d or {}), None, None
                self._drones[str(did)] = {
                    "telem": telem, "send_hz": send_hz, "recv_hz": recv_hz, "rx": now}

    def snapshot(self):
        now = time.time()
        with self._lock:
            drones = {
                did: {"telem": rec["telem"], "send_hz": rec["send_hz"],
                      "recv_hz": rec["recv_hz"], "age": round(now - rec["rx"], 2)}
                for did, rec in self._drones.items()
            }
            feed_age = round(now - self._last_packet, 2) if self._last_packet else None
            return {"server_time": now, "feed_age": feed_age,
                    "meta": dict(self._meta), "drones": drones}


class ShapesStore:
    """Thread-safe obstacles/geofence store persisted to shapes.json.

    Exists so the operator can draw (and keep) shapes with NO controller
    running: every edit is applied here, saved to disk, and ALSO forwarded to
    the controller's command port. While a controller is alive its meta echo
    is authoritative — sync_from_feed() adopts it (without re-saving; the
    controller persisted that edit itself), so ids and content converge on the
    controller's view. Both processes default to the same file in this folder,
    and duplicate saves of the same edit write identical content atomically.
    """

    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        self._obstacles, self._geofence = load_shapes(path)

    def get(self):
        with self._lock:
            return {"obstacles": list(self._obstacles),
                    "geofence": self._geofence}

    def sync_from_feed(self, meta):
        """Adopt the controller's live shapes from a telemetry datagram."""
        if not isinstance(meta, dict):
            return
        with self._lock:
            if "obstacles" in meta:
                self._obstacles = list(meta.get("obstacles") or [])
            if "geofence" in meta:
                self._geofence = meta.get("geofence")

    def _save(self):
        save_shapes(self.path, self._obstacles, self._geofence)

    def add_obstacle(self, lat1, lon1, lat2, lon2):
        ob = normalize_obstacle(lat1, lon1, lat2, lon2)
        if ob is None:
            return False
        with self._lock:
            if len(self._obstacles) >= MAX_OBSTACLES:
                return False
            ob["id"] = max([o["id"] for o in self._obstacles] or [0]) + 1
            self._obstacles = self._obstacles + [ob]
            self._save()
        return True

    def delete_obstacle(self, ob_id):
        with self._lock:
            self._obstacles = [o for o in self._obstacles if o["id"] != ob_id]
            self._save()

    def clear_obstacles(self):
        with self._lock:
            self._obstacles = []
            self._save()

    def set_geofence(self, vertices):
        fence = validate_fence(vertices)
        if fence is None:
            return False
        with self._lock:
            self._geofence = fence
            self._save()
        return True

    def clear_geofence(self):
        with self._lock:
            self._geofence = None
            self._save()


def udp_listener(state, host, port, shapes=None):
    """Receive telemetry datagrams from the flight controller forever."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"  UDP telemetry listener on {host}:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            payload = json.loads(data.decode("utf-8"))
            state.update(payload)
            if shapes is not None:
                shapes.sync_from_feed(payload.get("meta"))
        except Exception:
            continue  # ignore malformed packets, keep listening


def make_handler(state, cmd_sock=None, cmd_addr=None, shapes=None):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionAbortedError):
                pass

        def do_GET(self):
            path = self.path.split("?", 1)[0]

            if path == "/telemetry":
                snap = state.snapshot()
                # Shapes come from the GUI's own store (seeded from
                # shapes.json, synced from the controller's echo while one is
                # running) so they stay visible with no controller at all.
                if shapes is not None:
                    snap["shapes"] = shapes.get()
                body = json.dumps(snap).encode("utf-8")
                self._send(200, body, "application/json")
                return

            if path in ("/", ""):
                path = "/index.html"

            # Resolve + sandbox to STATIC_DIR (no path traversal).
            rel = os.path.normpath(path.lstrip("/\\"))
            full = os.path.join(STATIC_DIR, rel)
            if not os.path.abspath(full).startswith(os.path.abspath(STATIC_DIR)):
                self._send(403, b"forbidden", "text/plain")
                return
            if not os.path.isfile(full):
                self._send(404, b"not found", "text/plain")
                return

            ctype = _CONTENT_TYPES.get(os.path.splitext(full)[1].lower(),
                                       "application/octet-stream")
            with open(full, "rb") as f:
                self._send(200, f.read(), ctype)

        def do_POST(self):
            # The only POST route: swarm controls (Start/Stop buttons, gimbal
            # pitch slider, heading mode + point-inwards toggles, obstacle/
            # geofence edits drawn on the map). We forward
            # the action to the flight controller
            # (swarm_flocking.py) as a local UDP datagram on the command port.
            # This server never touches ds_wrapper.
            path = self.path.split("?", 1)[0]
            if path != "/command":
                self._send(404, b"not found", "text/plain")
                return
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = self.rfile.read(length) if length else b""
                msg = json.loads(body.decode("utf-8"))
                action = (msg.get("action") or "").lower()
            except Exception:
                self._send(400, b'{"ok":false,"error":"bad json"}', "application/json")
                return
            if action in ("start", "stop", "toggle"):
                out = {"action": action}
            elif action == "gimbal":
                # Gimbal pitch slider: forward the numeric target (deg). The
                # controller clamps it to the drone's tilt range.
                try:
                    out = {"action": "gimbal", "value": float(msg.get("value"))}
                except (TypeError, ValueError):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
            elif action == "heading":
                # Heading-mode selector (manual | convexhull | demostitch).
                value = (str(msg.get("value") or "")).lower()
                if value not in ("manual", "convexhull", "demostitch"):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                out = {"action": "heading", "value": value}
            elif action == "point_inwards":
                # Convex-hull facing toggle: boundary drones face the centroid.
                out = {"action": "point_inwards", "value": bool(msg.get("value"))}
            elif action == "stitch_offset":
                # Demostitch per-rank fan offset (deg), from the GUI's number
                # input; bounds shared with the controller's CLI validation.
                try:
                    v = float(msg.get("value"))
                except (TypeError, ValueError):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                if not (OFFSET_MIN_DEG <= v <= OFFSET_MAX_DEG):
                    self._send(400, b'{"ok":false,"error":"out of range"}', "application/json")
                    return
                out = {"action": "stitch_offset", "value": v}
            elif action == "plane":
                # Vertical-plane ("wall") toggle. Forwarded as a request only —
                # the controller owns the transition and refuses (clearing the
                # flag back) if the drones' altitudes disagree too much.
                out = {"action": "plane", "value": bool(msg.get("value"))}
            elif action == "plane_gain":
                # Restoring pull onto the plane (m/s per m of offset); bounds
                # shared with the controller's CLI validation.
                try:
                    v = float(msg.get("value"))
                except (TypeError, ValueError):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                if not (PLANE_GAIN_MIN <= v <= PLANE_GAIN_MAX):
                    self._send(400, b'{"ok":false,"error":"out of range"}', "application/json")
                    return
                out = {"action": "plane_gain", "value": v}
            elif action == "rotation_check":
                # Open-loop actuation probe: forwarded as a plain request; the
                # controller runs it only while swarming is held and reports
                # progress/results back via meta.rotation_check.
                out = {"action": "rotation_check"}
            elif action == "add_obstacle":
                # Rectangular virtual obstacle: two opposite corners drawn on
                # the map. Applied to the GUI's own store (validated + saved
                # to shapes.json — works with no controller) and forwarded so
                # a running controller applies the identical edit.
                corners = {k: msg.get(k) for k in ("lat1", "lon1", "lat2", "lon2")}
                if shapes is None or not shapes.add_obstacle(**corners):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                out = {k: float(v) for k, v in corners.items()}
                out["action"] = "add_obstacle"
            elif action == "delete_obstacle":
                try:
                    ob_id = int(msg.get("id"))
                except (TypeError, ValueError):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                if shapes is not None:
                    shapes.delete_obstacle(ob_id)
                out = {"action": "delete_obstacle", "id": ob_id}
            elif action == "clear_obstacles":
                if shapes is not None:
                    shapes.clear_obstacles()
                out = {"action": action}
            elif action == "clear_geofence":
                if shapes is not None:
                    shapes.clear_geofence()
                out = {"action": action}
            elif action == "set_geofence":
                # Geofence polygon: ordered [lat, lon] vertex list.
                fence = validate_fence(msg.get("vertices"))
                if fence is None or shapes is None:
                    self._send(400, b'{"ok":false,"error":"bad vertices"}', "application/json")
                    return
                shapes.set_geofence(fence)
                out = {"action": "set_geofence", "vertices": fence}
            else:
                self._send(400, b'{"ok":false,"error":"bad action"}', "application/json")
                return
            if cmd_sock is not None:
                try:
                    cmd_sock.sendto(json.dumps(out).encode("utf-8"), cmd_addr)
                except Exception:
                    pass  # controller not up yet; button is best-effort
            self._send(200, b'{"ok":true}', "application/json")

        def log_message(self, *args):
            pass  # quiet; telemetry polling would otherwise spam the console

    return Handler


def main():
    ap = argparse.ArgumentParser(description="LIS_Swarm browser GUI server")
    ap.add_argument("--http-host", default="127.0.0.1",
                    help="HTTP bind address (use 0.0.0.0 to expose on the LAN)")
    ap.add_argument("--http-port", type=int, default=8000, help="HTTP port")
    ap.add_argument("--udp-host", default="127.0.0.1",
                    help="UDP telemetry bind address (match the controller's --gui-host)")
    ap.add_argument("--udp-port", type=int, default=5099,
                    help="UDP telemetry port (match the controller's --gui-port)")
    ap.add_argument("--cmd-host", default="127.0.0.1",
                    help="Host to send Start/Stop commands to (the controller; "
                         "default 127.0.0.1 — it runs on this same PC)")
    ap.add_argument("--cmd-port", type=int, default=5098,
                    help="UDP port the controller listens on for Start/Stop "
                         "commands (match swarm_flocking.py --cmd-port)")
    ap.add_argument("--shapes-file", default=DEFAULT_SHAPES_FILE,
                    metavar="PATH",
                    help="JSON file the drawn obstacles/geofence persist to "
                         f"(default {DEFAULT_SHAPES_FILE}; relative paths "
                         "resolve against this script's directory — keep it "
                         "matching swarm_flocking.py --shapes-file)")
    ap.add_argument("--open", action="store_true",
                    help="Open the GUI in the default browser on startup")
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(STATIC_DIR, "index.html")):
        ap.error(f"missing GUI assets: {os.path.join(STATIC_DIR, 'index.html')}")

    state = SwarmState()

    shapes_path = args.shapes_file
    if not os.path.isabs(shapes_path):
        shapes_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), shapes_path)
    shapes = ShapesStore(shapes_path)
    loaded = shapes.get()
    print(f"  Shapes: {len(loaded['obstacles'])} obstacle(s), "
          f"geofence {'set' if loaded['geofence'] else 'none'} ({shapes_path})")

    t = threading.Thread(target=udp_listener,
                         args=(state, args.udp_host, args.udp_port, shapes),
                         daemon=True, name="UdpTelemetryListener")
    t.start()

    # Outbound UDP socket for forwarding Start/Stop button presses to the
    # controller (swarm_flocking.py). Co-located on this PC, so the default
    # target is 127.0.0.1:5098.
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_addr = (args.cmd_host, args.cmd_port)

    httpd = ThreadingHTTPServer((args.http_host, args.http_port),
                                make_handler(state, cmd_sock, cmd_addr, shapes))
    url = f"http://{'127.0.0.1' if args.http_host in ('0.0.0.0', '') else args.http_host}:{args.http_port}"
    print("LIS_Swarm GUI server")
    print(f"  Open: {url}")
    print(f"  Waiting for telemetry from a running controller "
          f"(joystick_controller.py / swarm_flocking.py).")
    print(f"  Start/Stop buttons -> {args.cmd_host}:{args.cmd_port} "
          f"(swarm_flocking.py --cmd-port).")
    print("  Ctrl+C to stop.")

    if args.open:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
