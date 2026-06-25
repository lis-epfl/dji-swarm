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


def udp_listener(state, host, port):
    """Receive telemetry datagrams from the flight controller forever."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"  UDP telemetry listener on {host}:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            state.update(json.loads(data.decode("utf-8")))
        except Exception:
            continue  # ignore malformed packets, keep listening


def make_handler(state):
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
                body = json.dumps(state.snapshot()).encode("utf-8")
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
    ap.add_argument("--open", action="store_true",
                    help="Open the GUI in the default browser on startup")
    args = ap.parse_args()

    if not os.path.isfile(os.path.join(STATIC_DIR, "index.html")):
        ap.error(f"missing GUI assets: {os.path.join(STATIC_DIR, 'index.html')}")

    state = SwarmState()

    t = threading.Thread(target=udp_listener,
                         args=(state, args.udp_host, args.udp_port),
                         daemon=True, name="UdpTelemetryListener")
    t.start()

    httpd = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(state))
    url = f"http://{'127.0.0.1' if args.http_host in ('0.0.0.0', '') else args.http_host}:{args.http_port}"
    print("LIS_Swarm GUI server")
    print(f"  Open: {url}")
    print(f"  Waiting for telemetry from a running controller "
          f"(joystick_controller.py / swarm_flocking.py).")
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
