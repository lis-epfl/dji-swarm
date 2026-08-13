"""
Building-footprint lookup for the GUI's "Pick building" tool
============================================================
Turns one map click into a real building outline, so the operator no longer has
to hand-drag an axis-aligned rectangle around a structure — and, more
importantly, so an edge of that outline can become a PLANAR inspection wall
with its TRUE bearing (see clip_scene_plane.facade_from_line, versus
facade_from_obstacle whose azimuth can only be 0 or 90).

SOURCE: OpenStreetMap, via the Overpass API. Chosen after checking the Swiss
federal alternatives, which do NOT work for this:

  - `ch.swisstopo.vec25-gebaeude` (api3 identify) is the national map at
    1:25 000. At EPFL it answers with ONE MultiPolygon of area 117 425 m² —
    the whole built-up block generalized and merged. Useless for one facade.
  - `ch.kantone.cadastralwebmap-farbe` IS identify-queryable and IS ground-true,
    but it returns the cadastral PARCEL (egris_egrid / number / ak), not a
    building.
  - swissTLM3D / swissBUILDINGS3D have the accuracy but are download-only
    (STAC tiles), not a click endpoint.

So swisstopo enters this feature as a *basemap* — the ground-true visual
cross-check you switch to in the GUI's layer control — and OSM supplies the
geometry.

ACCURACY, stated plainly: OSM is crowd-sourced, not a survey product. Swiss
coverage is good and often cadastral-derived, but no per-building guarantee
exists. What makes it usable is that CLAUDE.md's plane-error budget is
explicit — the seam cost is f·B·δZ/Z² px, so the 2026-08-11 MED clips
(Z ≈ 34 m, B ≈ 10.8 m, f = 525 px) tolerate ≈ 1 m — and
clip_scene_plane.analyse() prints `tolerance_m` for each clip's own geometry.
Pick the wall here, then let analyse() say whether it was good enough.
A footprint is at least a GROUND outline, so unlike tracing a roofline off
satellite imagery it carries no off-nadir lean (that gotcha is 5-7 m).

Pure stdlib (urllib), no ds_wrapper, no numpy — importable by swarm_gui.py,
which must keep running unprivileged on any Python >= 3.7.

    python building_footprint.py            # offline self-check
    python building_footprint.py 46.5197 6.5665   # live probe (needs internet)
"""

import json
import math
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from olfati_saber import (
    MAX_POLY_VERTICES,
    closest_point_on_polygon,
    closest_point_on_segment,
    gps_to_local,
    point_in_polygon,
    ring_area_m2,
)

__all__ = ["lookup", "FootprintCache", "simplify_ring",
           "DEFAULT_CACHE_FILE", "DEFAULT_RADIUS_M"]


DEFAULT_CACHE_FILE = "footprint_cache.json"
DEFAULT_RADIUS_M = 40
MAX_RADIUS_M = 250
DEFAULT_TIMEOUT_S = 8.0
# Entries kept on disk. Each is a small ring; 200 is a few hundred kB and more
# buildings than any one site needs.
MAX_CACHE_ENTRIES = 200
# A click this far outside a ring still counts as picking it — the operator
# aiming at a wall naturally clicks on or just past the line.
EDGE_GRACE_M = 3.0
# Vertex-thinning target when a traced building exceeds MAX_POLY_VERTICES.
SIMPLIFY_TOL_M = 0.25

# overpass-api.de first, kumi.systems as the fallback: both are public
# instances of the same API and either can be rate-limited at any moment.
OVERPASS_ENDPOINTS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
)

_USER_AGENT = "LIS_Swarm-GUI/1.0 (DJI swarm facade inspection)"


# ---------- ring helpers ----------

def simplify_ring(ring, max_vertices=MAX_POLY_VERTICES, tol_m=SIMPLIFY_TOL_M):
    """Thin a closed lat/lon ring down to `max_vertices`.

    Greedy least-significant-vertex removal: repeatedly drop the vertex whose
    perpendicular offset from the segment joining its two neighbours is
    smallest. Works directly on a closed ring (unlike plain Douglas-Peucker,
    which needs an open polyline) and is bounded by construction.

    Returns (ring, worst_offset_m). The offset is reported rather than enforced
    — the cap has to be met either way, and a caller that gets back a large
    number should say so rather than silently pretend the outline is exact."""
    out = [list(v) for v in ring]
    if len(out) <= max_vertices:
        return out, 0.0
    lat0, lon0 = out[0]
    pts = [list(gps_to_local(v[0], v[1], lat0, lon0)) for v in out]
    keep = list(range(len(out)))
    worst = 0.0
    while len(keep) > max_vertices and len(keep) > 3:
        best_k, best_d = None, float("inf")
        for k in range(len(keep)):
            a = pts[keep[k - 1]]
            b = pts[keep[k]]
            c = pts[keep[(k + 1) % len(keep)]]
            cn, ce = closest_point_on_segment(b[0], b[1], a, c)
            d = math.hypot(cn - b[0], ce - b[1])
            if d < best_d:
                best_k, best_d = k, d
        if best_k is None:
            break
        worst = max(worst, best_d)
        keep.pop(best_k)
    return [out[i] for i in keep], worst


def _open_ring(coords):
    """[[lat, lon], ...] with OSM's repeated closing node removed."""
    ring = [[float(c[0]), float(c[1])] for c in coords]
    if len(ring) > 3 and ring[0] == ring[-1]:
        ring.pop()
    return ring


def _ring_hit(ring, lat, lon):
    """Distance from (lat, lon) to `ring`: 0.0 inside, else metres to the
    boundary. None when the ring is degenerate."""
    if len(ring) < 3:
        return None
    poly = [gps_to_local(v[0], v[1], lat, lon) for v in ring]
    if point_in_polygon(0.0, 0.0, poly):
        return 0.0
    _, _, dist = closest_point_on_polygon(0.0, 0.0, poly)
    return dist


def _label_of(tags):
    for key in ("name", "addr:housename", "ref", "building"):
        val = (tags or {}).get(key)
        if val and val != "yes":
            return str(val)
    return "building"


# ---------- Overpass ----------

def _overpass(lat, lon, radius_m, timeout_s, endpoints=OVERPASS_ENDPOINTS):
    """POST one Overpass query; return the decoded JSON. Raises IOError with a
    human-readable message when every endpoint fails."""
    query = ("[out:json][timeout:{t}];("
             "way(around:{r},{lat},{lon})[\"building\"];"
             "relation(around:{r},{lat},{lon})[\"building\"];"
             ");out geom;").format(t=int(timeout_s), r=int(radius_m),
                                   lat=float(lat), lon=float(lon))
    body = urllib.parse.urlencode({"data": query}).encode("utf-8")
    errors = []
    for url in endpoints:
        req = urllib.request.Request(
            url, data=body,
            headers={"User-Agent": _USER_AGENT,
                     "Content-Type": "application/x-www-form-urlencoded"})
        try:
            with urllib.request.urlopen(req, timeout=timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            # 429/504 are Overpass telling you to back off — worth naming.
            errors.append("%s: HTTP %s" % (url.split("/")[2], e.code))
        except Exception as e:                       # timeout, DNS, TLS, ...
            errors.append("%s: %s" % (url.split("/")[2], e))
    raise IOError("Overpass unreachable (" + "; ".join(errors) + ")")


def _candidates(data):
    """Extract [{vertices, label, source, tags, area_m2}] from an Overpass
    response. Ways are used directly; a multipolygon relation contributes its
    largest CLOSED outer member (full multipolygon assembly is not worth it
    here — a building whose outline needs stitching is not a facade target)."""
    out = []
    for el in (data or {}).get("elements") or []:
        tags = el.get("tags") or {}
        if el.get("type") == "way":
            geom = el.get("geometry") or []
            ring = _open_ring([[g["lat"], g["lon"]] for g in geom
                               if "lat" in g and "lon" in g])
            rings = [ring] if len(ring) >= 3 else []
        elif el.get("type") == "relation":
            rings = []
            for mem in el.get("members") or []:
                if mem.get("role") != "outer" or mem.get("type") != "way":
                    continue
                geom = mem.get("geometry") or []
                pts = [[g["lat"], g["lon"]] for g in geom
                       if "lat" in g and "lon" in g]
                if len(pts) >= 4 and pts[0] == pts[-1]:      # closed on its own
                    rings.append(_open_ring(pts))
            rings = [max(rings, key=ring_area_m2)] if rings else []
        else:
            continue
        for ring in rings:
            out.append({
                "vertices": ring,
                "label": _label_of(tags),
                "source": "osm:%s/%s" % (el.get("type"), el.get("id")),
                "tags": tags,
                "area_m2": round(ring_area_m2(ring), 1),
            })
    return out


def _pick(candidates, lat, lon):
    """The building the operator meant: the smallest ring CONTAINING the click
    (smallest, so an inner courtyard block wins over the campus outline it sits
    in), else the nearest one within EDGE_GRACE_M of its boundary."""
    inside = []
    near = []
    for c in candidates:
        d = _ring_hit(c["vertices"], lat, lon)
        if d is None:
            continue
        if d == 0.0:
            inside.append(c)
        elif d <= EDGE_GRACE_M:
            near.append((d, c))
    if inside:
        return min(inside, key=lambda c: c["area_m2"])
    if near:
        return min(near, key=lambda dc: dc[0])[1]
    return None


# ---------- cache ----------

class FootprintCache:
    """Footprints kept on disk so a building picked once stays available.

    Keyed by GEOMETRY, not by click coordinates: a lookup hits when the click
    falls inside (or within EDGE_GRACE_M of) a ring already on file. That is
    what makes it useful in the field — the flight site was surveyed from the
    office, and out there the same building answers wherever you click it, with
    no connectivity at all."""

    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        self._entries = self._load()

    def _load(self):
        if not self.path or not os.path.isfile(self.path):
            return []
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
            out = []
            for e in data.get("footprints") or []:
                ring = [[float(v[0]), float(v[1])] for v in e["vertices"]]
                if len(ring) >= 3:
                    out.append({"vertices": ring,
                                "label": str(e.get("label") or "building"),
                                "source": str(e.get("source") or ""),
                                "area_m2": float(e.get("area_m2") or 0.0),
                                "ts": float(e.get("ts") or 0.0)})
            return out
        except (OSError, ValueError, KeyError, TypeError, IndexError) as e:
            print("WARNING: could not load %s: %s — starting with an empty "
                  "footprint cache" % (self.path, e))
            return []

    def _save_locked(self):
        if not self.path:
            return
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"version": 1, "footprints": self._entries}, f,
                          indent=1)
            os.replace(tmp, self.path)
        except OSError as e:
            print("WARNING: could not save %s: %s" % (self.path, e))

    def find(self, lat, lon):
        with self._lock:
            entries = list(self._entries)
        hit = _pick(entries, lat, lon)
        return dict(hit) if hit else None

    def put(self, entry):
        with self._lock:
            src = entry.get("source")
            keep = [e for e in self._entries
                    if not (src and e.get("source") == src)]
            rec = {k: entry[k] for k in ("vertices", "label", "source",
                                         "area_m2") if k in entry}
            rec["ts"] = time.time()
            keep.append(rec)
            # Oldest out first once the cap is hit.
            keep.sort(key=lambda e: e.get("ts", 0.0))
            self._entries = keep[-MAX_CACHE_ENTRIES:]
            self._save_locked()


# ---------- public entry point ----------

def lookup(lat, lon, radius_m=DEFAULT_RADIUS_M, cache=None, refresh=False,
           timeout_s=DEFAULT_TIMEOUT_S):
    """Find the building at (lat, lon).

    Returns a dict that is always JSON-safe and never raises:
        {"ok": True, "vertices": [[lat, lon], ...], "label": str,
         "source": "osm:way/123", "area_m2": float, "cached": bool,
         "note": str}                       # note present only when relevant
        {"ok": False, "error": str, "cached": False}

    `cache` is an optional FootprintCache. `refresh=True` bypasses a cache HIT
    but still writes the fresh result back."""
    try:
        lat = float(lat)
        lon = float(lon)
        radius_m = max(5, min(int(radius_m), MAX_RADIUS_M))
    except (TypeError, ValueError):
        return {"ok": False, "error": "bad coordinates", "cached": False}
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return {"ok": False, "error": "bad coordinates", "cached": False}

    if cache is not None and not refresh:
        hit = cache.find(lat, lon)
        if hit:
            hit.pop("ts", None)
            hit["ok"] = True
            hit["cached"] = True
            return hit

    try:
        data = _overpass(lat, lon, radius_m, timeout_s)
    except IOError as e:
        # Last resort: an offline click still works if the building was picked
        # before, even when `refresh` asked for a fresh copy.
        if cache is not None:
            hit = cache.find(lat, lon)
            if hit:
                hit.pop("ts", None)
                hit["ok"] = True
                hit["cached"] = True
                hit["note"] = "offline — served from cache (%s)" % (e,)
                return hit
        return {"ok": False, "error": str(e), "cached": False}

    best = _pick(_candidates(data), lat, lon)
    if best is None:
        return {"ok": False, "cached": False,
                "error": "no OSM building outline here — try clicking inside "
                         "the footprint, or draw a rectangle by hand"}

    ring, worst = simplify_ring(best["vertices"])
    out = {"ok": True, "cached": False, "vertices": ring,
           "label": best["label"], "source": best["source"],
           "area_m2": best["area_m2"]}
    if len(ring) != len(best["vertices"]):
        out["note"] = ("outline thinned from %d to %d vertices (moved a corner "
                       "by up to %.2f m)" % (len(best["vertices"]), len(ring),
                                             worst))
    if cache is not None:
        cache.put(out)
    return out


# ---------- self-check ----------

def _selftest():
    """Offline checks against a synthetic Overpass response — no network."""
    import shutil
    import tempfile

    ok = [True]

    def check(name, cond, detail=""):
        print("  {:<50} {}{}".format(name, "PASS" if cond else "FAIL",
                                     "" if cond else "  <- " + str(detail)))
        if not cond:
            ok[0] = False

    lat0, lon0 = 46.5197, 6.5665
    dlat = lambda m: m / 111320.0
    dlon = lambda m: m / (111320.0 * math.cos(math.radians(lat0)))

    def box(n0, e0, n1, e1):
        return [{"lat": lat0 + dlat(n0), "lon": lon0 + dlon(e0)},
                {"lat": lat0 + dlat(n0), "lon": lon0 + dlon(e1)},
                {"lat": lat0 + dlat(n1), "lon": lon0 + dlon(e1)},
                {"lat": lat0 + dlat(n1), "lon": lon0 + dlon(e0)},
                {"lat": lat0 + dlat(n0), "lon": lon0 + dlon(e0)}]  # closed

    resp = {"elements": [
        {"type": "way", "id": 1, "tags": {"building": "university",
                                          "name": "ME D"},
         "geometry": box(0, 0, 40, 60)},
        {"type": "way", "id": 2, "tags": {"building": "yes"},
         "geometry": box(5, 5, 15, 20)},          # small block inside ME D
    ]}

    cands = _candidates(resp)
    check("ways parsed", len(cands) == 2, cands)
    check("closing node dropped", all(len(c["vertices"]) == 4 for c in cands))
    check("name used as label", cands[0]["label"] == "ME D")
    check("source tagged", cands[0]["source"] == "osm:way/1")

    hit = _pick(cands, lat0 + dlat(10), lon0 + dlon(12))
    check("smallest containing ring wins", hit and hit["source"] == "osm:way/2",
          hit and hit["source"])
    hit = _pick(cands, lat0 + dlat(30), lon0 + dlon(40))
    check("outer ring when only it contains", hit and hit["source"] == "osm:way/1")
    hit = _pick(cands, lat0 + dlat(20), lon0 - dlon(1.5))
    check("click just outside an edge still picks it",
          hit and hit["source"] == "osm:way/1", hit and hit["source"])
    check("click far away picks nothing",
          _pick(cands, lat0 + dlat(400), lon0) is None)

    rel = {"elements": [{"type": "relation", "id": 9,
                         "tags": {"building": "yes", "name": "Courtyard"},
                         "members": [
                             {"type": "way", "role": "outer",
                              "geometry": box(0, 0, 30, 30)},
                             {"type": "way", "role": "inner",
                              "geometry": box(10, 10, 20, 20)}]}]}
    rc = _candidates(rel)
    check("relation: largest closed outer member used",
          len(rc) == 1 and rc[0]["source"] == "osm:relation/9", rc)

    big = [[lat0 + dlat(20 * math.cos(2 * math.pi * i / 120)),
            lon0 + dlon(20 * math.sin(2 * math.pi * i / 120))]
           for i in range(120)]
    thin, worst = simplify_ring(big)
    check("over-long ring thinned to the cap",
          len(thin) == MAX_POLY_VERTICES, len(thin))
    check("thinning reports its worst offset", worst > 0.0)
    check("short ring untouched",
          simplify_ring([[0, 0], [0, 1], [1, 1]])[0] == [[0, 0], [0, 1], [1, 1]])

    tmpdir = tempfile.mkdtemp(prefix="footprint_selftest_")
    try:
        cache = FootprintCache(os.path.join(tmpdir, DEFAULT_CACHE_FILE))
        cache.put({"vertices": cands[0]["vertices"], "label": "ME D",
                   "source": "osm:way/1", "area_m2": cands[0]["area_m2"]})
        check("cache hit anywhere inside the ring",
              cache.find(lat0 + dlat(35), lon0 + dlon(50)) is not None)
        check("cache miss outside", cache.find(lat0 + dlat(400), lon0) is None)
        reloaded = FootprintCache(cache.path)
        check("cache survives a restart",
              reloaded.find(lat0 + dlat(35), lon0 + dlon(50)) is not None)
        cache.put({"vertices": cands[0]["vertices"], "label": "ME D v2",
                   "source": "osm:way/1", "area_m2": 1.0})
        check("re-put replaces rather than duplicates",
              len(FootprintCache(cache.path)._entries) == 1)

        # Offline lookup must be served by the cache, not error out.
        res = lookup(lat0 + dlat(35), lon0 + dlon(50), cache=cache,
                     timeout_s=0.001,
                     )
        check("cached lookup needs no network", res.get("ok") is True
              and res.get("cached") is True, res)

        bad = lookup("nope", lon0, cache=cache)
        check("bad coordinates rejected cleanly", bad.get("ok") is False, bad)
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    print("\n%s" % ("ALL PASS" if ok[0] else "FAILURES ABOVE"))
    return 0 if ok[0] else 1


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 3:
        print("live Overpass probe at %s, %s" % (sys.argv[1], sys.argv[2]))
        result = lookup(sys.argv[1], sys.argv[2])
        if result.get("ok"):
            print("  %s (%s)  %s m^2, %d vertices"
                  % (result["label"], result["source"], result["area_m2"],
                     len(result["vertices"])))
            if result.get("note"):
                print("  note: %s" % result["note"])
        else:
            print("  FAILED: %s" % result.get("error"))
        sys.exit(0 if result.get("ok") else 1)
    print("building_footprint self-check")
    sys.exit(_selftest())
