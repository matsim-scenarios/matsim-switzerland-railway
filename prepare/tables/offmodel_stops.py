#!/usr/bin/env python3
"""OFFMODEL/stops -- decide, from the OSM data, whether a timetable stop has real Swiss
standard-gauge track in our routable network, or whether the track it serves is not in our
infrastructure data (metre gauge, dual-gauge-only, foreign station).

Ported unchanged from round 2 agent F1 (`stop_gauge.py`); only the inputs/outputs now come from
`prepare.paths` instead of hard-coded `swarm/` paths.

Run (through the pipeline): uv run ... prepare/build_tables.py --only OFFS
Inputs (read-only):
  original_data/osm/switzerland_railways.osm       all railway ways of the extract
  <out>/A03a/platform_tracks.csv                   tagged platform -> OSM way match (method, min_dist_m)
  <out>/A01/stops_distance.csv                     stop <-> routable component distance / bbox flag
  matsim_input/tmp/schedule_gtfs_filtered.xml.gz   the 994 stop facilities

Output: <out>/OFFMODEL/stop_track_gauge.csv

Way classes (the same candidate set as A03b for "std"):
  std_run    railway=rail, gauge empty or contains 1435, service empty or main/branch-like
  std_other  same but service in {siding,yard,spur}  (never a platform track, used for context)
  narrow     railway=narrow_gauge, or railway=rail/light_rail with a gauge but no 1435
  mixed      gauge contains both 1000 and 1435 (RhB/TPF/RBS corridors) -- NOT standard gauge
  platform   railway=platform / platform_edge

Decision (first rule that fires; every rule is a data test, no station list):
  1 tag       A03a method track_ref/stop_node AND min_dist_m <= 25          -> REAL
  2 narrowcode a narrow way within 60 m carries this platform's code as
              railway:track_ref and no std way within 60 m does             -> OFF-MODEL
  3 platform  a platform way within 15 m AND a std running track within 15 m
              AND the nearest narrow way is not closer than the std one      -> REAL
  4 close     a std running track within 8 m, nearest narrow not closer      -> REAL
  5 station   no platform code (station-level facility) and a std track
              within 60 m, nearest narrow not closer                        -> REAL
  6 else                                                                     -> OFF-MODEL
"""
import collections
import csv
import gzip
import math
import sys
import xml.etree.ElementTree as ET

import osmium
from pyproj import Transformer

from prepare.paths import get as _paths

DISUSED = {"disused", "abandoned", "razed", "construction", "proposed",
           "dismantled", "demolished", "unused"}
SIDING = {"siding", "yard", "spur"}
CELL = 250.0
NEAR = 60.0


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
        if not rw or rw in DISUSED:
            return
        gauge = t.get("gauge", "")
        service = t.get("service", "")
        if rw in ("platform", "platform_edge"):
            klass = "platform"
        elif rw == "rail" and service != "crossover":
            if gauge and "1435" not in gauge and "1000" in gauge:
                klass = "mixed"
            elif gauge == "" or "1435" in gauge:
                if ";" in gauge and gauge.replace(";", "").replace("1435", ""):
                    klass = "mixed"
                else:
                    klass = "std_other" if service in SIDING else "std_run"
            else:
                klass = "narrow"
        elif rw in ("narrow_gauge", "light_rail") and "1435" not in gauge:
            klass = "narrow"
        elif rw in ("tram", "subway", "funicular", "rail", "light_rail"):
            klass = "other"
        else:
            return
        self.ways.append({"id": w.id, "klass": klass, "tags": t,
                          "refs": [n.ref for n in w.nodes]})


def load_stops(sched):
    stops = []
    for _, el in ET.iterparse(gzip.open(sched, "rb"), events=("end",)):
        if el.tag == "stopFacility":
            stops.append((el.get("id"), el.get("name"), float(el.get("x")), float(el.get("y"))))
            el.clear()
    return stops


def route_stop_usage(sched):
    """how many route-stop entries reference each facility (streaming pass over the schedule)"""
    use = collections.Counter()
    for _, el in ET.iterparse(gzip.open(sched, "rb"), events=("end",)):
        if el.tag == "stop":
            use[el.get("refId")] += 1
            el.clear()
    return use


def run():
    paths = _paths()
    osm = str(paths.osm)
    sched = str(paths.schedule)
    a03a = str(paths.out / "A03a/platform_tracks.csv")
    stopd = str(paths.out / "A01/stops_distance.csv")
    out = str(paths.table("OFFMODEL") / "stop_track_gauge.csv")
    print("[1/4] nodes ...", file=sys.stderr, flush=True)
    nc = NodeCollector()
    nc.apply_file(osm, locations=False)
    ids = list(nc.lon.keys())
    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    px, py = tr.transform([nc.lon[i] for i in ids], [nc.lat[i] for i in ids])
    coords = dict(zip(ids, zip(px, py)))
    del nc, ids, px, py

    print("[2/4] ways ...", file=sys.stderr, flush=True)
    wc = WayCollector()
    wc.apply_file(osm, locations=False)
    ways = wc.ways
    print("      ways by class:", dict(collections.Counter(w["klass"] for w in ways)),
          file=sys.stderr, flush=True)

    print("[3/4] segment grid ...", file=sys.stderr, flush=True)
    grid = {}
    segs = {}
    for w in ways:
        pts = [coords[r] for r in w["refs"] if r in coords]
        segs[w["id"]] = [(pts[i - 1][0], pts[i - 1][1], pts[i][0], pts[i][1])
                         for i in range(1, len(pts))]
        for i, (x1, y1, x2, y2) in enumerate(segs[w["id"]]):
            for cx in range(int(math.floor(min(x1, x2) / CELL)), int(math.floor(max(x1, x2) / CELL)) + 1):
                for cy in range(int(math.floor(min(y1, y2) / CELL)), int(math.floor(max(y1, y2) / CELL)) + 1):
                    grid.setdefault((cx, cy), []).append((w["id"], i))
    wklass = {w["id"]: w["klass"] for w in ways}
    wtags = {w["id"]: w["tags"] for w in ways}
    wayklass = {str(w["id"]): w["klass"] for w in ways}

    def seg_dist(px_, py_, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        if L == 0:
            return math.hypot(px_ - x1, py_ - y1)
        t = max(0.0, min(1.0, ((px_ - x1) * dx + (py_ - y1) * dy) / L))
        return math.hypot(px_ - (x1 + t * dx), py_ - (y1 + t * dy))

    print("[4/4] per stop ...", file=sys.stderr, flush=True)
    a03 = {r["stop_id"]: r for r in csv.DictReader(open(a03a))}
    sd = {r["stop_id"]: r for r in csv.DictReader(open(stopd))}
    use = route_stop_usage(sched)
    rows = []
    for sid, name, x, y in load_stops(sched):
        cls = collections.defaultdict(lambda: (1e9, None))
        cx, cy = int(math.floor(x / CELL)), int(math.floor(y / CELL))
        for ring in range(0, 30):
            for gx in range(cx - ring, cx + ring + 1):
                for gy in range(cy - ring, cy + ring + 1):
                    if max(abs(gx - cx), abs(gy - cy)) != ring:
                        continue
                    for wid, i in grid.get((gx, gy), ()):
                        x1, y1, x2, y2 = segs[wid][i]
                        d = seg_dist(x, y, x1, y1, x2, y2)
                        k = wklass[wid]
                        if d < cls[k][0]:
                            cls[k] = (d, wid)
            outer = ring * CELL
            if outer > NEAR + CELL:
                break
            if all(cls[k][0] <= outer for k in ("std_run", "narrow", "platform")):
                break
        d_std, w_std = cls["std_run"]
        d_nar, w_nar = cls["narrow"]
        d_mix, w_mix = cls["mixed"]
        d_oth, w_oth = cls["std_other"]
        d_plat, w_plat = cls["platform"]
        code = (a03[sid]["platform_code"].strip() if sid in a03 else "")
        method = a03[sid]["method"] if sid in a03 else "none"
        # the gauge of the ways A03a matched by tag: tag evidence only counts on standard gauge
        tag_std = False
        for w in (a03[sid]["way_ids"] if sid in a03 else "").split(";"):
            if wayklass.get(w.strip()) in ("std_run", "std_other"):
                tag_std = True
                break
        try:
            a03d = float(a03[sid]["min_dist_m"]) if sid in a03 and a03[sid]["min_dist_m"] else 1e9
        except ValueError:
            a03d = 1e9
        # track_ref codes of std / narrow ways within NEAR
        std_codes, nar_codes = set(), set()
        op_off, n_off = "", 0
        for ring in range(0, 2):
            for gx in range(cx - ring, cx + ring + 1):
                for gy in range(cy - ring, cy + ring + 1):
                    for wid, i in grid.get((gx, gy), ()):
                        x1, y1, x2, y2 = segs[wid][i]
                        if seg_dist(x, y, x1, y1, x2, y2) > NEAR:
                            continue
                        tref = wtags[wid].get("railway:track_ref", "").strip()
                        if not tref:
                            continue
                        if wklass[wid] in ("std_run", "std_other"):
                            std_codes.add(tref)
                        elif wklass[wid] == "narrow":
                            nar_codes.add(tref)

        # operator of the off-model track at this stop (nearest narrow or mixed way within 60 m)
        best_off = (1e9, "")
        for ring in range(0, 2):
            for gx in range(cx - ring, cx + ring + 1):
                for gy in range(cy - ring, cy + ring + 1):
                    for wid, i in grid.get((gx, gy), ()):
                        if wklass[wid] not in ("narrow", "mixed"):
                            continue
                        x1, y1, x2, y2 = segs[wid][i]
                        d = seg_dist(x, y, x1, y1, x2, y2)
                        if d <= NEAR and d < best_off[0]:
                            best_off = (d, wtags[wid].get("operator", "") or wtags[wid].get("name", ""))
        narrow_closer = w_nar is not None and d_nar < d_std
        if method in ("track_ref", "stop_node") and a03d <= 150 and tag_std and not narrow_closer:
            real, reason = 1, "tag:" + method
        elif code and code in nar_codes and code not in std_codes:
            real, reason = 0, "off:narrow track carries code %s" % code
        elif d_plat <= 15 and d_std <= 15 and not narrow_closer:
            real, reason = 1, "geometric:platform+track"
        elif d_std <= 8 and not narrow_closer:
            real, reason = 1, "geometric:track<=8m"
        elif not code and d_std <= NEAR and not narrow_closer:
            real, reason = 1, "geometric:station-level"
        elif d_std > NEAR:
            real, reason = 0, "off:no std track within %.0f m" % NEAR
        elif narrow_closer:
            real, reason = 0, "off:narrow track closer"
        else:
            real, reason = 0, "off:platform code not on std track"
        near = sorted([(d_std, "std"), (d_nar, "narrow"), (d_mix, "mixed"),
                       (d_oth, "std_other"), (d_plat, "platform")])
        rows.append({
            "stop_id": sid, "name": name, "x": "%.2f" % x, "y": "%.2f" % y,
            "route_stops": use.get(sid, 0),
            "inside_bbox": sd[sid]["inside_osm_bbox"] if sid in sd else "",
            "a03a_method": method, "platform_code": code,
            "dist_std_m": "" if w_std is None else "%.2f" % d_std,
            "dist_narrow_m": "" if w_nar is None else "%.2f" % d_nar,
            "dist_mixed_m": "" if w_mix is None else "%.2f" % d_mix,
            "dist_platform_m": "" if w_plat is None else "%.2f" % d_plat,
            "nearest_class": near[0][1],
            "off_track_operator": best_off[1],
            "std_track_refs": ";".join(sorted(std_codes)),
            "narrow_track_refs": ";".join(sorted(nar_codes)),
            "real": real, "reason": reason,
        })
    rows.sort(key=lambda r: r["stop_id"])
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    nreal = sum(r["real"] for r in rows)
    print(f"{len(rows)} stops, real {nreal}, off-model {len(rows)-nreal} -> {out}")
    print("reasons:", dict(collections.Counter(r["reason"] for r in rows)))
    print("route-stop entries: real %d, off-model %d of %d" % (
        sum(int(r["route_stops"]) for r in rows if r["real"]),
        sum(int(r["route_stops"]) for r in rows if not r["real"]),
        sum(int(r["route_stops"]) for r in rows)))

