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

The map tiles are fetched from the internet, so the GUI PC needs connectivity;
no API key is required. The layer control offers Esri World Imagery (the
default) plus two swisstopo basemaps — SWISSIMAGE and the cadastral webmap,
whose ground-true building and parcel outlines are the visual cross-check for a
picked footprint. The swisstopo layers cover Switzerland/Liechtenstein only.

The map's "Pick building" tool resolves a click to a real building outline via
/footprint (see building_footprint.py) and stores it as a polygon obstacle; one
edge of that outline can then be kept as a PLANAR inspection wall, which is what
gives clip_replay.py --set-plane-from-facade a wall azimuth that is not snapped
to a compass axis.
"""

import argparse
import json
import os
import socket
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Shared shapes validation + shapes.json persistence (olfati_saber.py is pure
# stdlib, no ds_wrapper — safe to import here). The GUI keeps its own store so
# obstacles/geofence can be drawn, saved, and shown with NO controller running.
from olfati_saber import (
    DEFAULT_SHAPES_FILE,
    MAX_FACADES,
    MAX_OBSTACLES,
    normalize_obstacle,
    normalize_polygon_obstacle,
    validate_facade,
    validate_fence,
    load_facades,
    load_shapes,
    save_shapes,
)

# Click-a-building footprint lookup (OSM Overpass + an on-disk cache). Pure
# stdlib like the rest of this server's imports — see building_footprint.py for
# why OSM rather than a swisstopo layer.
from building_footprint import (
    DEFAULT_CACHE_FILE,
    DEFAULT_RADIUS_M,
    FootprintCache,
    lookup as lookup_footprint,
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
    """Thread-safe obstacles/facades/geofence store persisted to shapes.json.

    Exists so the operator can draw (and keep) shapes with NO controller
    running: every edit is applied here, saved to disk, and ALSO forwarded to
    the controller's command port. While a controller is alive its meta echo
    is authoritative — sync_from_feed() adopts it (without re-saving; the
    controller persisted that edit itself), so ids and content converge on the
    controller's view. Both processes default to the same file in this folder,
    and duplicate saves of the same edit write identical content atomically.

    An obstacle is either the hand-drawn axis-aligned rectangle or a real
    building footprint (`kind: "poly"`); a facade is one edge of a footprint,
    stored so clip_replay.py --set-plane-from-facade can turn it into a PLANAR
    scene plane with the wall's true bearing. Facades are inert for flight.
    """

    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        self._obstacles, self._geofence = load_shapes(path)
        self._facades = load_facades(path)

    def get(self):
        with self._lock:
            return {"obstacles": list(self._obstacles),
                    "facades": list(self._facades),
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
            # Absent key => leave alone, so a controller too old to echo
            # facades cannot silently wipe the ones drawn here.
            if "facades" in meta:
                self._facades = list(meta.get("facades") or [])

    def _save(self):
        save_shapes(self.path, self._obstacles, self._geofence, self._facades)

    def _add_locked(self, ob):
        """Assign the next id, append, persist. Caller holds the lock.
        Returns the new id, or None when the cap is reached."""
        if len(self._obstacles) >= MAX_OBSTACLES:
            return None
        ob["id"] = max([o["id"] for o in self._obstacles] or [0]) + 1
        self._obstacles = self._obstacles + [ob]
        self._save()
        return ob["id"]

    def add_obstacle(self, lat1, lon1, lat2, lon2):
        ob = normalize_obstacle(lat1, lon1, lat2, lon2)
        if ob is None:
            return None
        with self._lock:
            return self._add_locked(ob)

    def add_building(self, vertices, label=None, source=None):
        """Add a building footprint as a polygon obstacle. Returns its id."""
        ob = normalize_polygon_obstacle(vertices, label, source)
        if ob is None:
            return None
        with self._lock:
            return self._add_locked(ob)

    def add_facade(self, p1, p2, obstacle_id=None, label=None, source=None):
        pair = validate_facade(p1, p2)
        if pair is None:
            return None
        with self._lock:
            if len(self._facades) >= MAX_FACADES:
                return None
            rec = {"id": max([f["id"] for f in self._facades] or [0]) + 1,
                   "p1": pair[0], "p2": pair[1]}
            if obstacle_id is not None:
                rec["obstacle_id"] = int(obstacle_id)
            if label:
                rec["label"] = str(label)[:80]
            if source:
                rec["source"] = str(source)[:120]
            self._facades = self._facades + [rec]
            self._save()
            return rec["id"]

    def delete_facade(self, fa_id):
        with self._lock:
            self._facades = [f for f in self._facades if f["id"] != fa_id]
            self._save()

    def clear_facades(self):
        with self._lock:
            self._facades = []
            self._save()

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


def _by_id(records, want_id):
    """The record with `want_id`, or None. Used to echo back what the store
    actually saved rather than what was asked for."""
    for rec in records or []:
        if rec.get("id") == want_id:
            return rec
    return None


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


def make_handler(state, cmd_sock=None, cmd_addr=None, shapes=None,
                 footprints=None):
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

            if path == "/footprint":
                # Building lookup for the map's "Pick building" tool. Proxied
                # here rather than fetched by the browser so the result can be
                # cached to disk — a site surveyed at the office then answers
                # in the field with no connectivity. Blocking network I/O is
                # safe: this is a ThreadingHTTPServer, so /telemetry keeps
                # being served while Overpass is slow or down.
                if footprints is None:      # --no-footprint-lookup
                    self._send(200, b'{"ok":false,"error":"building lookup is '
                                    b'disabled (--no-footprint-lookup)"}',
                               "application/json")
                    return
                q = urllib.parse.parse_qs(
                    self.path.split("?", 1)[1] if "?" in self.path else "")
                res = lookup_footprint(
                    q.get("lat", [None])[0], q.get("lon", [None])[0],
                    radius_m=q.get("radius", [DEFAULT_RADIUS_M])[0],
                    cache=footprints,
                    refresh=q.get("refresh", ["0"])[0] not in ("0", "", "false"))
                # Failures ride on a 200 with ok:false so the GUI can show the
                # reason instead of a bare network error.
                self._send(200, json.dumps(res).encode("utf-8"),
                           "application/json")
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
            # pitch slider, heading mode + point-inwards toggles, clip
            # recording, obstacle/geofence edits drawn on the map). We forward
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
            elif action in ("record_start", "record_stop"):
                # Clip recording (video + the flight data for the window).
                # Zero-arg on purpose: the duration cap lives on the
                # controller's CLI, so there is nothing to validate and this
                # server stays free of a clip_recorder import — that module
                # needs cv2, and the GUI must keep running unprivileged on any
                # plain Python >= 3.7.
                out = {"action": action}
            elif action == "add_obstacle":
                # Rectangular virtual obstacle: two opposite corners drawn on
                # the map. Applied to the GUI's own store (validated + saved
                # to shapes.json — works with no controller) and forwarded so
                # a running controller applies the identical edit.
                corners = {k: msg.get(k) for k in ("lat1", "lon1", "lat2", "lon2")}
                new_id = shapes.add_obstacle(**corners) if shapes else None
                if new_id is None:
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                out = {k: float(v) for k, v in corners.items()}
                out["action"] = "add_obstacle"
                out["id"] = new_id
            elif action == "add_building":
                # A real building footprint picked off the map (see
                # building_footprint.py). Stored as a polygon obstacle that
                # ALSO carries its bounding box, so a process on an older
                # shapes.json revision still reads a valid — and conservative
                # — rectangle. See olfati_saber.py's compatibility contract.
                verts = msg.get("vertices")
                label = msg.get("label")
                source = msg.get("source")
                new_id = (shapes.add_building(verts, label, source)
                          if shapes else None)
                if new_id is None:
                    self._send(400, b'{"ok":false,"error":"bad footprint"}',
                               "application/json")
                    return
                # Echo the stored (validated, bbox-stamped) ring, not the raw
                # request, so the controller applies exactly what was saved.
                # `or {}` because a controller's meta echo can replace the list
                # between the add and this read (sync_from_feed); the id is
                # already assigned either way, so fall back to the request.
                stored = _by_id(shapes.get()["obstacles"], new_id) or {}
                out = {"action": "add_building", "id": new_id,
                       "vertices": stored.get("vertices") or verts}
                for k in ("label", "source"):
                    if stored.get(k) or msg.get(k):
                        out[k] = stored.get(k) or msg.get(k)
            elif action == "add_facade":
                # One edge of a footprint, kept as the PLANAR inspection wall.
                # Inert for flight — it exists so clip_replay.py
                # --set-plane-from-facade can route it through
                # clip_scene_plane.facade_from_line, whose azimuth is the
                # wall's TRUE bearing rather than the nearest compass axis.
                ob_id = msg.get("obstacle_id")
                try:
                    ob_id = int(ob_id) if ob_id is not None else None
                except (TypeError, ValueError):
                    ob_id = None
                new_id = (shapes.add_facade(msg.get("p1"), msg.get("p2"),
                                            ob_id, msg.get("label"),
                                            msg.get("source"))
                          if shapes else None)
                if new_id is None:
                    self._send(400, b'{"ok":false,"error":"bad facade"}',
                               "application/json")
                    return
                stored = _by_id(shapes.get()["facades"], new_id)
                out = dict(stored) if stored else {"id": new_id,
                                                   "p1": msg.get("p1"),
                                                   "p2": msg.get("p2")}
                out["action"] = "add_facade"
            elif action == "delete_facade":
                try:
                    fa_id = int(msg.get("id"))
                except (TypeError, ValueError):
                    self._send(400, b'{"ok":false,"error":"bad value"}', "application/json")
                    return
                if shapes is not None:
                    shapes.delete_facade(fa_id)
                out = {"action": "delete_facade", "id": fa_id}
            elif action == "clear_facades":
                if shapes is not None:
                    shapes.clear_facades()
                out = {"action": action}
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
            # Echo the id the store assigned. The map needs it to link a facade
            # to the footprint it was picked from, and it is the id the
            # delete button and clip_replay.py --set-plane-from-facade use.
            reply = {"ok": True}
            if "id" in out:
                reply["id"] = out["id"]
            self._send(200, json.dumps(reply).encode("utf-8"),
                       "application/json")

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
    ap.add_argument("--footprint-cache", default=DEFAULT_CACHE_FILE,
                    metavar="PATH",
                    help="JSON file the picked building footprints are cached "
                         f"to (default {DEFAULT_CACHE_FILE}; relative paths "
                         "resolve against this script's directory). Lets a "
                         "building picked once be re-picked offline.")
    ap.add_argument("--no-footprint-lookup", action="store_true",
                    help="Disable the 'Pick building' tool's outbound OSM "
                         "Overpass queries (the map's Pick building button "
                         "then reports the lookup as unavailable). Hand-drawn "
                         "obstacles and geofences are unaffected.")
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
          f"{len(loaded['facades'])} facade(s), "
          f"geofence {'set' if loaded['geofence'] else 'none'} ({shapes_path})")

    # Footprints picked off the map are cached beside shapes.json so a site
    # surveyed with connectivity still answers in the field without it.
    cache_path = args.footprint_cache
    if not os.path.isabs(cache_path):
        cache_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), cache_path)
    footprints = None if args.no_footprint_lookup else FootprintCache(cache_path)

    t = threading.Thread(target=udp_listener,
                         args=(state, args.udp_host, args.udp_port, shapes),
                         daemon=True, name="UdpTelemetryListener")
    t.start()

    # Outbound UDP socket for forwarding Start/Stop button presses to the
    # controller (swarm_flocking.py). Co-located on this PC, so the default
    # target is 127.0.0.1:5098.
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_addr = (args.cmd_host, args.cmd_port)

    httpd = ThreadingHTTPServer(
        (args.http_host, args.http_port),
        make_handler(state, cmd_sock, cmd_addr, shapes, footprints))
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
