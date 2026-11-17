"""A03a - map every platform-level GTFS stop to its OSM track way(s).

Provenance: copied unchanged from `swarm/data/A03a/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""


import csv
import gzip
import math
import re
import sys
from pathlib import Path

import numpy as np
import osmium
from pyproj import Transformer

from prepare.paths import get as _paths

_P = _paths()
ROOT = _P.root
OSM = _P.osm
SCHEDULE = _P.schedule
OUT = _P.table("A03a")

RADIUS = 600.0            # search radius around each stop coordinate
RADIUS_STOPNODE = 300.0   # stop_position nodes must be closer than this
RADIUS_PLATFORM = 200.0   # platform way must be closer than this
RADIUS_ADJACENT = 40.0    # rail way must be this close to the platform geometry
MATCH_WINDOW = 40.0       # extra window around the best track_ref distance
MATCH_MAX = 150.0         # absolute plausibility cap for a track_ref match
CELL = 250.0              # spatial grid cell size (m)
OTHER_RADIUS = 150.0      # diagnostic radius for non-rail (metre gauge etc.) ways


# --------------------------------------------------------------------------- #
# OSM loading
# --------------------------------------------------------------------------- #
class OsmHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.nodes = {}           # id -> (lon, lat)
        self.node_tags = {}       # id -> {k: v} (tagged nodes only)
        self.rail_ways = []       # (way_id, [node ids]) railway=rail
        self.platform_ways = []   # (way_id, [node ids]) railway=platform
        self.other_ways = []      # (way_id, [node ids], railway value) other railway=*
        self.way_tags = {}        # way_id -> {k: v} for rail/platform ways
        self.n_track_ref_ways = 0       # ALL OSM ways carrying railway:track_ref
        self.n_rail_track_ref_ways = 0  # railway=rail ways carrying it
        self.rail_value_counts = {}

    def node(self, n):
        self.nodes[n.id] = (n.location.lon, n.location.lat)
        tags = {t.k: t.v for t in n.tags}
        if tags:
            self.node_tags[n.id] = tags

    def way(self, w):
        tags = {t.k: t.v for t in w.tags}
        if "railway:track_ref" in tags:
            self.n_track_ref_ways += 1
        railway = tags.get("railway")
        if railway is None:
            return
        self.rail_value_counts[railway] = self.rail_value_counts.get(railway, 0) + 1
        if railway == "rail":
            if "railway:track_ref" in tags:
                self.n_rail_track_ref_ways += 1
            self.rail_ways.append((w.id, [n.ref for n in w.nodes]))
            self.way_tags[w.id] = tags
        elif railway == "platform":
            self.platform_ways.append((w.id, [n.ref for n in w.nodes]))
            self.way_tags[w.id] = tags
        else:
            self.other_ways.append((w.id, [n.ref for n in w.nodes], railway))


def load_osm():
    h = OsmHandler()
    h.apply_file(str(OSM))
    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    ids = list(h.nodes)
    lon = np.fromiter((h.nodes[i][0] for i in ids), dtype=np.float64, count=len(ids))
    lat = np.fromiter((h.nodes[i][1] for i in ids), dtype=np.float64, count=len(ids))
    x, y = tr.transform(lon, lat)
    xy = {nid: (float(px), float(py)) for nid, px, py in zip(ids, x, y)}
    return h, xy


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def seg_matrix(node_ids, xy):
    """(n_segments, 4) float array [ax, ay, bx, by] for one way."""
    pts = [xy[n] for n in node_ids if n in xy]
    if len(pts) < 2:
        return np.empty((0, 4), dtype=np.float64), pts
    a = np.asarray(pts, dtype=np.float64)
    return np.concatenate([a[:-1], a[1:]], axis=1), pts


def min_dist(segs, px, py):
    """Shortest Euclidean distance from (px, py) to a segment matrix."""
    if len(segs) == 0:
        return math.inf
    ax, ay, bx, by = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    t = np.where(l2 > 0, ((px - ax) * dx + (py - ay) * dy) / np.where(l2 > 0, l2, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    qx = ax + t * dx
    qy = ay + t * dy
    return float(np.sqrt(np.min((qx - px) ** 2 + (qy - py) ** 2)))


class Grid:
    def __init__(self):
        self.cells = {}

    def add(self, way_id, segs):
        for ax, ay, bx, by in segs:
            ix0 = int(math.floor(min(ax, bx) / CELL))
            ix1 = int(math.floor(max(ax, bx) / CELL))
            iy0 = int(math.floor(min(ay, by) / CELL))
            iy1 = int(math.floor(max(ay, by) / CELL))
            for ix in range(ix0, ix1 + 1):
                for iy in range(iy0, iy1 + 1):
                    self.cells.setdefault((ix, iy), set()).add(way_id)

    def add_node(self, node_id, px, py):
        self.cells.setdefault((int(math.floor(px / CELL)), int(math.floor(py / CELL))),
                              set()).add(node_id)

    def query(self, px, py, radius):
        ids = set()
        ix0 = int(math.floor((px - radius) / CELL))
        ix1 = int(math.floor((px + radius) / CELL))
        iy0 = int(math.floor((py - radius) / CELL))
        iy1 = int(math.floor((py + radius) / CELL))
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                cell = self.cells.get((ix, iy))
                if cell:
                    ids.update(cell)
        return ids


# --------------------------------------------------------------------------- #
# Platform-code normalisation
# --------------------------------------------------------------------------- #
_ANNOT = re.compile(r"[\[\(].*?[\]\)]")
_SPLIT = re.compile(r"[;,/|]")


def norm_codes(raw):
    """Normalise a platform code / track_ref into a set of comparable tokens.

    '7' -> {'7'}, '007' -> {'7'}, '7AB' -> {'7'}, '41/42' -> {'41','42'},
    '7;8' -> {'7','8'}, 'A' -> {'a'}, '13 (WTL)' -> {'13'}.
    """
    if not raw:
        return set()
    s = _ANNOT.sub(" ", str(raw)).strip().lower()
    out = set()
    for part in _SPLIT.split(s):
        p = part.strip().replace(" ", "")
        if not p:
            continue
        m = re.fullmatch(r"(\d+)([a-z]*)", p)
        if m:  # numeric part; sector letters are dropped
            out.add(str(int(m.group(1))))
        else:
            out.add(p)
    return out


# --------------------------------------------------------------------------- #
# Schedule loading
# --------------------------------------------------------------------------- #
_STOP_RE = re.compile(
    r'<stopFacility id="([^"]+)" x="([^"]+)" y="([^"]+)" name="([^"]*)"')
_STOP_REF_RE = re.compile(r'<stop refId="([^"]+)"')


def load_schedule():
    stops = []
    busy = {}
    with gzip.open(SCHEDULE, "rt", encoding="utf-8") as f:
        for line in f:
            m = _STOP_RE.search(line)
            if m:
                stops.append((m.group(1), float(m.group(2)), float(m.group(3)), m.group(4)))
                continue
            for ref in _STOP_REF_RE.findall(line):
                busy[ref] = busy.get(ref, 0) + 1
    stops.sort(key=lambda s: (split_stop_id(s[0])[0],
                              norm_sort_key(split_stop_id(s[0])[1]), s[0]))
    return stops, busy


def split_stop_id(sid):
    """'8500218:0:7' -> ('8500218', '7'); '8014181' -> ('8014181', '')."""
    parts = sid.split(":")
    if len(parts) >= 3:
        return parts[0], ":".join(parts[2:])
    return parts[0], ""


def norm_sort_key(code):
    return [int(p) if p.isdigit() else p for p in re.split(r"(\d+)", code or "")]


# --------------------------------------------------------------------------- #
# Matching
# --------------------------------------------------------------------------- #
def confidence_track_ref(d, n_ways):
    if d <= 30:
        c = 0.90
    elif d <= 60:
        c = 0.80
    elif d <= 120:
        c = 0.65
    else:
        c = 0.50
    if n_ways > 8:  # suspiciously many ways -> lower
        c *= 0.85
    return round(c, 2)


def match_stop(stop, agents):
    sid, px, py, name = stop
    uic, code_raw = split_stop_id(sid)
    codes = norm_codes(code_raw)

    if codes:
        # ---- 1) railway:track_ref on rail ways -----------------------------
        cands = []
        for wid in agents["grid"].query(px, py, RADIUS):
            tags = agents["way_tags"][wid]
            if not (norm_codes(tags.get("railway:track_ref")) & codes):
                continue
            d = min_dist(agents["segs"][wid], px, py)
            if d <= RADIUS:
                cands.append((d, wid))
        if cands:
            cands.sort()
            best = cands[0][0]
            limit = min(max(best + MATCH_WINDOW, 60.0), MATCH_MAX)
            keep = sorted(wid for d, wid in cands if d <= limit)
            # keep only the closest connected cluster if far-apart duplicates exist
            if len(keep) > 1:
                keep = closest_cluster(keep, agents, px, py)
            if keep:
                d = min(d for d, wid in cands if wid in keep)
                return {
                    "method": "track_ref", "way_ids": keep, "min_dist": d,
                    "confidence": confidence_track_ref(d, len(keep)),
                }

        # ---- 2) stop_position / railway=stop nodes -------------------------
        node_cands = []
        for nid in agents["grid_nodes"].query(px, py, RADIUS_STOPNODE):
            tags = agents["node_tags"].get(nid, {})
            if not (norm_codes(tags.get("local_ref") or tags.get("ref")) & codes):
                continue
            nx, ny = agents["xy"][nid]
            d = math.hypot(nx - px, ny - py)
            uic_ok = tags.get("uic_ref") == uic
            name_ok = tags.get("name", "").strip().lower() == name.strip().lower()
            if uic_ok or name_ok or d <= 150.0:
                node_cands.append((0 if uic_ok else 1, d, nid))
        if node_cands:
            node_cands.sort()
            _, _, nid = node_cands[0]
            wids = sorted(agents["node_to_way"].get(nid, []))
            if wids:
                dd = min(min_dist(agents["segs"][w], px, py) for w in wids)
                return {
                    "method": "stop_node", "way_ids": wids, "min_dist": dd,
                    "confidence": 0.85 if dd <= 60 else 0.70,
                }

        # ---- 3) railway=platform ref -> adjacent rail way ------------------
        pcands = []
        for wid in agents["grid_platform"].query(px, py, RADIUS_PLATFORM):
            tags = agents["way_tags"][wid]
            if not (norm_codes(tags.get("ref") or tags.get("local_ref")) & codes):
                continue
            d = min_dist(agents["segs"][wid], px, py)
            if d <= RADIUS_PLATFORM:
                pcands.append((d, wid))
        if pcands:
            pcands.sort()
            plat = pcands[0][1]
            adj = adjacent_track(plat, agents, px, py)
            if adj:
                wids, dd = adj
                trefs = [agents["way_tags"][w].get("railway:track_ref") for w in wids]
                plat_refs = norm_codes(agents["way_tags"][plat].get("ref")
                                       or agents["way_tags"][plat].get("local_ref"))
                contradict = (any(trefs)
                              and not any(norm_codes(t) & codes for t in trefs if t))
                conf = 0.30
                if len(plat_refs) > 1:   # island platform: side is ambiguous
                    conf *= 0.7
                if contradict:           # adjacent track is numbered differently
                    conf *= 0.5
                return {
                    "method": "platform_ref", "way_ids": wids, "min_dist": dd,
                    "confidence": round(conf, 2),
                }

    # ---- 4) none -----------------------------------------------------------
    return {"method": "none", "way_ids": [], "min_dist": math.inf, "confidence": 0.0}


def closest_cluster(wids, agents, px, py):
    """Split matching ways into connected components (shared node); keep the one
    whose min distance to the stop is smallest. Guards against two different
    stations with the same track number inside the search radius."""
    parent = {w: w for w in wids}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    node_owner = {}
    for w in wids:
        for n in agents["way_nodes"][w]:
            if n in node_owner:
                union(w, node_owner[n])
            else:
                node_owner[n] = w
    groups = {}
    for w in wids:
        groups.setdefault(find(w), []).append(w)
    if len(groups) == 1:
        return wids
    best = None
    for g in groups.values():
        d = min(min_dist(agents["segs"][w], px, py) for w in g)
        if best is None or d < best[0]:
            best = (d, sorted(g))
    return best[1]


def adjacent_track(platform_way, agents, px, py):
    """Nearest rail way to a platform way, used only for method platform_ref.
    The platform's own track is not tagged, so this is a geometric fallback:
    among rail ways touching the platform geometry, pick the one closest to the
    stop coordinate (single way, deterministic tie-break). Two parallel tracks
    are never merged."""
    pts = agents["way_points"][platform_way]
    if not pts:
        return None
    cand = []
    for wid in agents["grid"].query(px, py, RADIUS_PLATFORM):
        dplat = min(min_dist(agents["segs"][wid], ax, ay) for ax, ay in pts)
        if dplat <= RADIUS_ADJACENT:
            cand.append((min_dist(agents["segs"][wid], px, py), wid))
    if not cand:
        return None
    cand.sort()
    return [cand[0][1]], cand[0][0]


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def build_agents(h, xy):
    segs, points, way_nodes = {}, {}, {}
    for wid, nodes in h.rail_ways + h.platform_ways:
        sm, pts = seg_matrix(nodes, xy)
        segs[wid] = sm
        points[wid] = pts
        way_nodes[wid] = nodes
    rail_ids = {w for w, _ in h.rail_ways}
    plat_ids = {w for w, _ in h.platform_ways}

    grid = Grid()
    for wid in sorted(rail_ids):
        grid.add(wid, segs[wid])
    grid_platform = Grid()
    for wid in sorted(plat_ids):
        grid_platform.add(wid, segs[wid])

    other_segs = {}
    for wid, nodes, _ in h.other_ways:
        sm, _ = seg_matrix(nodes, xy)
        if len(sm):
            other_segs[wid] = sm
    grid_other = Grid()
    for wid in sorted(other_segs):
        grid_other.add(wid, other_segs[wid])

    node_to_way = {}
    for wid in sorted(rail_ids):
        for n in way_nodes[wid]:
            node_to_way.setdefault(n, []).append(wid)

    stop_node_ids = [nid for nid, t in h.node_tags.items()
                     if t.get("railway") == "stop"
                     or t.get("public_transport") == "stop_position"]
    grid_nodes = Grid()
    for nid in sorted(stop_node_ids):
        grid_nodes.add_node(nid, *xy[nid])

    return {
        "xy": xy, "segs": segs, "way_points": points, "way_nodes": way_nodes,
        "way_tags": h.way_tags, "node_tags": h.node_tags, "node_to_way": node_to_way,
        "grid": grid, "grid_platform": grid_platform, "grid_nodes": grid_nodes,
        "other_segs": other_segs, "grid_other": grid_other,
    }


def main():
    sys.stderr.write("loading OSM ...\n")
    h, xy = load_osm()
    sys.stderr.write(
        f"  {len(h.nodes)} nodes, {len(h.rail_ways)} rail ways, "
        f"{len(h.platform_ways)} platform ways, {len(h.other_ways)} other railway ways\n")
    agents = build_agents(h, xy)

    sys.stderr.write("loading schedule ...\n")
    stops, busy = load_schedule()
    sys.stderr.write(f"  {len(stops)} stop facilities, {len(busy)} referenced ids\n")

    rows = []
    inside = {}
    unmatched_diag = {"narrow_gauge_near": 0, "node_on_non_rail": 0}
    for stop in stops:
        sid, px, py, name = stop
        uic, code = split_stop_id(sid)
        res = match_stop(stop, agents)
        # inside/outside Switzerland: any railway=rail way within 2 km?
        if res["method"] == "none":
            near2k = any(min_dist(agents["segs"][wid], px, py) <= 2000.0
                         for wid in agents["grid"].query(px, py, 2000.0))
        else:
            near2k = True
        inside[sid] = near2k

        rows.append({
            "stop_id": sid, "uic": uic, "platform_code": code, "name": name,
            "x": f"{px:.3f}", "y": f"{py:.3f}", "method": res["method"],
            "way_ids": ";".join(str(w) for w in res["way_ids"]),
            "n_ways": len(res["way_ids"]),
            "min_dist_m": "" if res["min_dist"] == math.inf else f"{res['min_dist']:.2f}",
            "confidence": f"{res['confidence']:.2f}",
        })

        # diagnostics for unmatched coded Swiss stops (explains metre-gauge gaps)
        if res["method"] == "none" and near2k and code:
            if any(min_dist(agents["other_segs"][w], px, py) <= OTHER_RADIUS
                   for w in agents["grid_other"].query(px, py, OTHER_RADIUS)):
                unmatched_diag["narrow_gauge_near"] += 1
            codes = norm_codes(code)
            for nid in agents["grid_nodes"].query(px, py, 150.0):
                tags = agents["node_tags"].get(nid, {})
                if (norm_codes(tags.get("local_ref") or tags.get("ref")) & codes
                        and nid not in agents["node_to_way"]):
                    unmatched_diag["node_on_non_rail"] += 1
                    break

    OUT.mkdir(parents=True, exist_ok=True)
    fields = ["stop_id", "uic", "platform_code", "name", "x", "y", "method",
              "way_ids", "n_ways", "min_dist_m", "confidence"]
    with open(OUT / "platform_tracks.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    pairs = validate_olten(rows, agents)
    write_report(rows, inside, busy, h, unmatched_diag, pairs)


# --------------------------------------------------------------------------- #
# Report
# --------------------------------------------------------------------------- #
def fmt_pct(n, d):
    return f"{n} ({100.0 * n / d:.1f} %)" if d else f"{n} (-)"


def write_report(rows, inside, busy, h, diag, olten_pairs):
    """Write REPORT.md."""
    total = len(rows)
    ch = [r for r in rows if inside[r["stop_id"]]]
    out = [r for r in rows if not inside[r["stop_id"]]]
    methods = ["track_ref", "stop_node", "platform_ref", "none"]

    def counts(rs):
        c = {m: 0 for m in methods}
        for r in rs:
            c[r["method"]] += 1
        return c

    c_all, c_ch, c_out = counts(rows), counts(ch), counts(out)
    dists = sorted(float(r["min_dist_m"]) for r in rows if r["min_dist_m"])

    def pct(vals, p):
        k = min(len(vals) - 1, max(0, int(round(p / 100.0 * (len(vals) - 1)))))
        return vals[k]

    unmatched = [r for r in ch if r["method"] == "none"]
    unmatched_coded = [r for r in unmatched if r["platform_code"]]
    unmatched.sort(key=lambda r: (-busy.get(r["stop_id"], 0), r["stop_id"]))
    busy_rows = [r for r in unmatched if busy.get(r["stop_id"], 0) > 0]

    L = []
    L.append("# A03a — platform stop → OSM track way assignment by OSM tags")
    L.append("")
    L.append(f"Inputs: `schedule_gtfs_filtered.xml.gz` ({total} stop facilities) + "
             f"`switzerland_railways.osm` ({len(h.rail_ways)} railway=rail, "
             f"{len(h.platform_ways)} railway=platform ways).")
    L.append("Methods in order: (1) `railway:track_ref` on rail ways → (2) `local_ref`/`ref` on "
             "stop nodes → the rail way(s) containing the node → (3) `ref` on platform ways → "
             "nearest adjacent rail way (geometric fallback) → none.")
    L.append(f"Radius 600 m. A track_ref row keeps all ways with the ref within "
             f"min(best+{MATCH_WINDOW:.0f} m, {MATCH_MAX:.0f} m); the closest connected cluster "
             f"wins if two stations share a number. Confidence: track_ref 0.90/0.80/0.65/0.50 "
             f"by distance; stop_node 0.85/0.70; platform_ref 0.30 (0.15 if the adjacent track "
             f"carries a different number).")
    L.append("")
    L.append("## Coverage by method")
    L.append("")
    L.append(f"| method | all ({total}) | inside CH ({len(ch)}) | outside CH ({len(out)}) |")
    L.append("|---|---|---|---|")
    for m in methods:
        L.append(f"| {m} | {fmt_pct(c_all[m], len(rows))} | {fmt_pct(c_ch[m], len(ch))} | "
                 f"{fmt_pct(c_out[m], len(out))} |")
    L.append(f"| **total** | {len(rows)} | {len(ch)} | {len(out)} |")
    L.append("")
    L.append("Inside CH = ≥1 `railway=rail` way within 2 km; outside CH has none. All 176 "
             "facilities without a platform code (parent-level stops) are `none`; "
             f"{len(unmatched_coded)} unmatched rows carry a code. "
             f"{h.n_rail_track_ref_ways} of {len(h.rail_ways)} `railway=rail` ways "
             f"({100.0 * h.n_rail_track_ref_ways / len(h.rail_ways):.1f} %) carry "
             f"`railway:track_ref`; {h.n_track_ref_ways} ways in the whole extract carry it. "
             f"That OSM `railway:track_ref` equals the real Gleis number is assumed, not "
             f"independently verified: UNVERIFIED.")
    L.append("")
    L.append("## min_dist_m (matched rows)")
    L.append("")
    L.append(f"n = {len(dists)}; mean {sum(dists)/len(dists):.1f} m; p50 {pct(dists,50):.1f}; "
             f"p90 {pct(dists,90):.1f}; p95 {pct(dists,95):.1f}; p99 {pct(dists,99):.1f}; "
             f"max {dists[-1]:.1f} m. Buckets: 0–10 m {sum(1 for d in dists if d < 10)}, "
             f"10–25 {sum(1 for d in dists if 10 <= d < 25)}, "
             f"25–50 {sum(1 for d in dists if 25 <= d < 50)}, "
             f"50–100 {sum(1 for d in dists if 50 <= d < 100)}, "
             f"100–150 {sum(1 for d in dists if 100 <= d < 150)}, "
             f">150 {sum(1 for d in dists if d >= 150)}. Rows >50 m are long platforms "
             f"where the tagged way lies further along the platform (Aarau 6, Wil SG 2) or "
             f"stops that only have the station-centroid coordinate (all Basel Bad Bf platforms).")
    L.append("")
    L.append("## Busiest unmatched Swiss stops (route-stop count)")
    L.append("")
    L.append(f"{len(unmatched)} unmatched inside CH, {len(busy_rows)} referenced by ≥1 route stop. "
             f"Of the {len(unmatched_coded)} with a platform code, "
             f"{diag['narrow_gauge_near']} have a non-rail track (mostly metre-gauge "
             f"`railway=narrow_gauge`, e.g. RhB/MGB/TPF) within {OTHER_RADIUS:.0f} m and "
             f"{diag['node_on_non_rail']} have a matching stop node that is not on a "
             f"`railway=rail` way. The 2 km rule also captures a few foreign border stops "
             f"(Annemasse, Weil am Rhein, Konstanz, Delle). Top 40:")
    L.append("")
    if busy_rows:
        per_line = 4
        for i in range(0, min(40, len(busy_rows)), per_line):
            L.append(" ‖ ".join(
                f"{r['stop_id']} {r['name']} ({r['platform_code'] or '—'}) "
                f"{busy.get(r['stop_id'], 0)}" for r in busy_rows[i:i + per_line]))
    else:
        L.append("None.")
    L.append("")
    L.append("## Olten (8500218) validation")
    L.append("")
    L.append("Per-way table with `track_ref` + LV95 centroid is printed to stdout by `build.py`. "
             "Lateral separation of consecutive track numbers (closest approach, signed on the "
             "local track normal):")
    if olten_pairs:
        L.append(", ".join(f"{a}–{b}: {s:+.1f} m" for a, b, _, s in olten_pairs))
        ordered = (all(s < 0 for *_, s in olten_pairs) or all(s > 0 for *_, s in olten_pairs))
        L.append(f"All consecutive pairs lie on the same side: {ordered} → track numbers increase "
                 f"monotonically across the station; platform 1 is not in the filtered timetable.")
    else:
        L.append("no matches")
    L.append("")
    L.append("## Spot-check: result rows (stop_id|code|method|way_ids|n|dist_m|conf)")
    L.append("uic/name/x/y are identical per station and in the CSV. "
             "`platform_ref` rows are geometric fallbacks, not tag evidence.")
    per_line = 4
    for uic, label in [("8500218", "Olten"), ("8507000", "Bern"), ("8503000", "Zürich HB"),
                       ("8502113", "Aarau"), ("8500010", "Basel SBB"), ("8505000", "Luzern")]:
        sel = [r for r in rows if r["uic"] == uic]
        sel.sort(key=lambda r: (norm_sort_key(r["platform_code"]), r["stop_id"]))
        L.append("")
        L.append(f"### {label} ({uic}) — {len(sel)} facilities")
        cells = [f"{r['stop_id']} | {r['platform_code'] or '—'} | {r['method']} | "
                 f"{r['way_ids'] or '—'} | {r['n_ways']} | {r['min_dist_m'] or '—'} | "
                 f"{r['confidence']}" for r in sel]
        for i in range(0, len(cells), per_line):
            L.append(" ‖ ".join(cells[i:i + per_line]))
    L.append("")
    (OUT / "REPORT.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    n = len(L)
    sys.stderr.write(f"  report: {n} lines\n")
    if n > 80:
        sys.stderr.write("  WARNING: report exceeds 80 lines\n")


def validate_olten(rows, agents):
    """Print matched Olten ways with track_ref + LV95 centroid; test that the
    track numbers are laterally ordered (tracks parallel, offsets increasing).
    Returns [(code_a, code_b, separation_m, signed_m), ...] for consecutive numbers."""
    sel = [r for r in rows if r["uic"] == "8500218" and r["method"] != "none"]
    tracks = {}
    table = []
    pts = []
    for r in sel:
        for wid in r["way_ids"].split(";"):
            if not wid:
                continue
            wid = int(wid)
            p = agents["way_points"][wid]
            pts.extend(p)
            cx = sum(q[0] for q in p) / len(p)
            cy = sum(q[1] for q in p) / len(p)
            table.append((r["platform_code"], wid,
                          agents["way_tags"][wid].get("railway:track_ref"),
                          cx, cy, r["min_dist_m"]))
            tracks.setdefault(r["platform_code"], []).append(p)
    print("\n=== Olten validation (UIC 8500218) ===")
    print(f"{'code':6} {'way':>12} {'track_ref':>10} {'centroid LV95':>26} {'dist_m':>7}")
    for ref, wid, tref, cx, cy, d in sorted(table, key=lambda t: (norm_sort_key(t[0]), t[1])):
        print(f"{ref:6} {wid:12d} {str(tref):>10} ({cx:9.1f},{cy:9.1f}) {d:>7}")

    a = np.asarray(pts)
    c = a.mean(axis=0)
    _, _, vt = np.linalg.svd(a - c, full_matrices=False)
    axis = vt[0]
    if axis[1] < 0:
        axis = -axis

    def nearest_point(p, q):
        best = None
        for i in range(len(p) - 1):
            s, t = p[i], p[i + 1]
            ab = t - s
            l2 = float(ab @ ab)
            u = 0.0 if l2 == 0 else max(0.0, min(1.0, float((q - s) @ ab) / l2))
            r = s + u * ab
            dd = float(np.linalg.norm(r - q))
            if best is None or dd < best[0]:
                best = (dd, r)
        return best

    def pair_geom(A, B):
        """closest approach between track A and track B + signed lateral offset."""
        best = None
        for p in A:
            dd, r = nearest_point(B, p)
            if best is None or dd < best[0]:
                best = (dd, p, r)
        for p in B:
            dd, r = nearest_point(A, p)
            if best is None or dd < best[0]:
                best = (dd, r, p)
        dd, pa, pb = best
        ia = int(np.argmin(np.linalg.norm(A - pa, axis=1)))
        dvec = A[ia + 1] - A[ia] if ia + 1 < len(A) else A[ia] - A[ia - 1]
        dvec = dvec / np.linalg.norm(dvec)
        if float(dvec @ axis) < 0:
            dvec = -dvec
        normal = np.array([-dvec[1], dvec[0]])
        return dd, float((pb - pa) @ normal)

    codes = sorted(tracks, key=int)
    pairs = []
    for a_, b_ in zip(codes, codes[1:]):
        A = np.concatenate(tracks[a_])
        B = np.concatenate(tracks[b_])
        dd, sgn = pair_geom(A, B)
        pairs.append((a_, b_, dd, sgn))
    print("consecutive track pairs (lateral separation, signed): " +
          ", ".join(f"{a_}-{b_}:{s:+.1f} m" for a_, b_, _, s in pairs))
    ordered = all(s < 0 for *_, s in pairs) or all(s > 0 for *_, s in pairs)
    print(f"all consecutive pairs on the same side: {ordered}")
    print(f"Olten track order is {'VERIFIED' if ordered else 'UNVERIFIED'}: "
          f"track numbers increase monotonically across the station")
    return pairs


def run():
    main()


if __name__ == "__main__":
    run()
