#!/usr/bin/env python3
"""OFFMODEL/legs -- real distance of every consecutive stop pair of the timetable, measured on the
OSM railway graph of the extract (standard gauge, narrow gauge and dual gauge together).

Ported unchanged from round 2 agent F1 (`leg_distance.py`); only the inputs/outputs now come from
`prepare.paths` instead of hard-coded `swarm/` paths.  Depends on the OFFS table.

This is what the router uses to give off-model (unlimited-capacity) links a realistic length:
"length = real distance if known, otherwise straight line x detour factor".

Run (through the pipeline): uv run ... prepare/build_tables.py --only OFFL

Output: <out>/OFFMODEL/leg_distance.csv
  from_stop,to_stop,dist_m,beeline_m,n_nodes,connected,detour
  connected=1  -> shortest path found in the OSM railway graph, dist_m = its length
  connected=0  -> no path (line not in the extract, e.g. abroad); dist_m = beeline * DETOUR

Only pairs that need it are measured: a pair of stops where at least one stop is off-model,
plus every pair of the timetable (the table is the same for real pairs, where the router just
does not use it). A* with the straight-line heuristic, bounded to 4 x beeline + 2 km.
"""
import collections
import csv
import gzip
import heapq
import math
import sys
import xml.etree.ElementTree as ET

import osmium
from pyproj import Transformer

from prepare.paths import get as _paths

DETOUR = 1.25          # detour factor straight line -> track, used only when no path exists
CELL = 200.0

DISUSED = {"disused", "abandoned", "razed", "construction", "proposed",
           "dismantled", "demolished", "unused"}


class NodeCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.lon = {}
        self.lat = {}

    def node(self, n):
        self.lon[n.id] = n.location.lon
        self.lat[n.id] = n.location.lat


class WayCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = []

    def way(self, w):
        t = dict(w.tags)
        rw = t.get("railway", "")
        if rw in ("rail", "narrow_gauge") and rw not in DISUSED:
            self.ways.append((w.id, t, [n.ref for n in w.nodes]))


def run():
    paths = _paths()
    osm = str(paths.osm)
    sched = str(paths.schedule)
    cls_path = str(paths.out / "OFFMODEL/stop_track_gauge.csv")
    out = str(paths.table("OFFMODEL") / "leg_distance.csv")
    print("[1/5] nodes ...", file=sys.stderr, flush=True)
    nc = NodeCollector()
    nc.apply_file(osm, locations=False)
    ids = list(nc.lon.keys())
    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    px, py = tr.transform([nc.lon[i] for i in ids], [nc.lat[i] for i in ids])
    xy = dict(zip(ids, zip(px, py)))
    del nc, ids, px, py

    print("[2/5] ways ...", file=sys.stderr, flush=True)
    wc = WayCollector()
    wc.apply_file(osm, locations=False)
    adj = collections.defaultdict(list)
    seen = set()
    nseg = 0
    for wid, t, refs in wc.ways:
        pts = [r for r in refs if r in xy]
        for a, b in zip(pts, pts[1:]):
            if a == b:
                continue
            key = (a, b) if a < b else (b, a)
            if key in seen:
                continue
            seen.add(key)
            d = math.hypot(xy[a][0] - xy[b][0], xy[a][1] - xy[b][1])
            adj[a].append((b, d))
            adj[b].append((a, d))
            nseg += 1
    print("      %d ways, %d undirected segments, %d nodes" % (len(wc.ways), nseg, len(adj)),
          file=sys.stderr, flush=True)

    # grid over graph nodes for nearest-node lookup
    grid = collections.defaultdict(list)
    for nid, (x, y) in xy.items():
        if nid in adj:
            grid[(int(x // CELL), int(y // CELL))].append(nid)

    def nearest_nodes(x, y, maxd=200.0, k=4):
        cx, cy = int(x // CELL), int(y // CELL)
        found = []
        for ring in range(0, 4):
            for gx in range(cx - ring, cx + ring + 1):
                for gy in range(cy - ring, cy + ring + 1):
                    if max(abs(gx - cx), abs(gy - cy)) != ring:
                        continue
                    for nid in grid.get((gx, gy), ()):
                        nx, ny = xy[nid]
                        found.append((math.hypot(nx - x, ny - y), nid))
            if found and min(found)[0] <= ring * CELL:
                break
        found.sort()
        return [(nid, d) for d, nid in found[:k] if d <= maxd]

    # connected components (union-find) so that a stop node in a stub component is not used
    parent = {n: n for n in adj}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a in adj:
        for b, _ in adj[a]:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
    comp_size = collections.Counter(find(n) for n in adj)

    print("[3/5] stops and route pairs ...", file=sys.stderr, flush=True)
    cls = {r["stop_id"]: r for r in csv.DictReader(open(cls_path))}
    # coordinates from the classification table (same 994 facilities)
    stopxy = {r["stop_id"]: (float(r["x"]), float(r["y"])) for r in cls.values()}
    node_of = {}
    for sid, (x, y) in stopxy.items():
        node_of[sid] = [(nid, d) for nid, d in nearest_nodes(x, y)
                        if comp_size[find(nid)] >= 200] or nearest_nodes(x, y, k=1)

    pairs = set()
    routes = 0
    for _, el in ET.iterparse(gzip.open(sched, "rb"), events=("end",)):
        if el.tag == "transitRoute":
            routes += 1
            refs = [s.get("refId") for s in el.iter("stop")]
            for a, b in zip(refs, refs[1:]):
                if a != b:
                    pairs.add((a, b) if a < b else (b, a))
            el.clear()
    print("      %d routes, %d distinct consecutive stop pairs" % (routes, len(pairs)),
          file=sys.stderr, flush=True)

    def astar(src, dst, limit):
        # straight line heuristic scaled to a lower bound of the graph distance
        tx, ty = xy[dst]
        h = lambda n: math.hypot(xy[n][0] - tx, xy[n][1] - ty)
        best = {src: 0.0}
        pq = [(h(src), 0.0, src)]
        settled = set()
        while pq:
            _, g, n = heapq.heappop(pq)
            if n in settled:
                continue
            settled.add(n)
            if n == dst:
                return g, len(settled)
            if g > limit:
                continue
            for m, d in adj[n]:
                if m in settled:
                    continue
                ng = g + d
                if ng > limit or (m in best and best[m] <= ng):
                    continue
                best[m] = ng
                heapq.heappush(pq, (ng + h(m), ng, m))
        return None, len(settled)

    print("[4/5] distances ...", file=sys.stderr, flush=True)
    rows = []
    nconn = 0
    for i, (a, b) in enumerate(sorted(pairs)):
        xa, ya = stopxy[a]
        xb, yb = stopxy[b]
        bl = math.hypot(xa - xb, ya - yb)
        dist, nn, conn = bl * DETOUR, 0, 0
        best = None
        for na, da in node_of.get(a, []):
            for nb, db in node_of.get(b, []):
                if na == nb or find(na) != find(nb):
                    continue
                if best is not None and da + db >= best[0]:
                    continue
                d, k = astar(na, nb, 4.0 * bl + 2000.0)
                nn = max(nn, k)
                if d is not None and d > 0:
                    tot = d + da + db
                    if best is None or tot < best[0]:
                        best = (tot, nn)
        if best is not None:
            dist, conn = best[0], 1
            nconn += 1
        rows.append({"from_stop": a, "to_stop": b, "dist_m": "%.1f" % dist,
                     "beeline_m": "%.1f" % bl, "n_nodes": nn, "connected": conn,
                     "detour": "%.3f" % (dist / bl if bl > 1 else 0)})
        if (i + 1) % 1000 == 0:
            print("      %d/%d (%d connected)" % (i + 1, len(pairs), nconn),
                  file=sys.stderr, flush=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("[5/5] %d pairs, %d connected on the OSM graph (%.1f %%) -> %s"
          % (len(rows), nconn, 100.0 * nconn / max(1, len(rows)), out))

