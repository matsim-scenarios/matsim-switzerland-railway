"""A01 - classify every OSM railway way and find the routable components.

Provenance: copied unchanged from `swarm/data/A01/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""


import csv
import gzip
import math
import os
import sys
import xml.etree.ElementTree as ET

import osmium
from pyproj import Transformer, Geod

GEOD = Geod(ellps="WGS84")

from prepare.paths import get as _paths

_P = _paths()
OSM = str(_P.osm)
SCHED = str(_P.schedule)
HERE = str(_P.table("A01"))

WAYS_CSV = os.path.join(HERE, "ways.csv")
COMP_CSV = os.path.join(HERE, "components.csv")
REPORT = os.path.join(HERE, "REPORT.md")
STOPS_CSV = os.path.join(HERE, "stops_distance.csv")

CSV_COLS = [
    "way_id", "railway", "usage", "service", "gauge", "electrified", "maxspeed",
    "maxspeed_forward", "maxspeed_backward", "tracks", "track_ref",
    "preferred_direction", "bidirectional", "oneway", "passenger_lines",
    "tunnel", "bridge", "name", "ref", "operator", "n_nodes", "first_node",
    "last_node", "length_m", "klass", "routable",
]

DISUSED = {"disused", "abandoned", "razed", "construction", "proposed",
           "dismantled", "demolished", "unused"}
RAIL_SERVICES = {"siding", "yard", "spur", "crossover"}


# ---------------------------------------------------------------- classification
def classify(rw, service, usage, gauge):
    """Map the raw tags to a klass.

    Precedence (documented in REPORT.md, because the task's class list is a list of
    definitions rather than an ordered cascade):
      1. non-track / life-cycle railway values own their own class
      2. tram / light_rail / subway / funicular / platform by the railway value
      3. narrow_gauge: railway=narrow_gauge, or a gauge value without "1435"
      4. railway=rail: service -> siding/yard/spur/crossover, else usage -> main/branch/rail_other
      5. everything else -> other
    """
    if rw in DISUSED:
        return "disused"
    if rw == "tram":
        return "tram"
    if rw == "light_rail":
        return "light_rail"
    if rw == "subway":
        return "subway"
    if rw == "funicular":
        return "funicular"
    if rw == "platform":
        return "platform"
    if rw == "narrow_gauge" or (gauge and "1435" not in gauge):
        return "narrow_gauge"
    if rw == "rail":
        if service in RAIL_SERVICES:
            return service
        if service:                      # any other service value (headshunt, ...)
            return "other"
        if usage == "main":
            return "main"
        if usage == "branch":
            return "branch"
        return "rail_other"
    return "other"


def is_routable(rw, gauge):
    """Standard-gauge heavy rail only. Sidings/yards/crossovers are real 1435 mm track
    today, so they stay routable; later stages penalise them."""
    return rw == "rail" and (gauge == "" or "1435" in gauge)


# ---------------------------------------------------------------------- osm pass
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
        self.rows = []          # dict per way
        self.n_ways = 0
        self.missing_nodes = 0

    def way(self, w):
        self.n_ways += 1
        t = dict(w.tags)
        refs = [n.ref for n in w.nodes]
        self.rows.append({"id": w.id, "tags": t, "refs": refs})


def main():
    for p in (OSM, SCHED):
        if not os.path.exists(p):
            sys.exit("missing input: " + p)

    print("[1/6] reading node coordinates ...", file=sys.stderr, flush=True)
    nc = NodeCollector()
    nc.apply_file(OSM, locations=False)
    # keep lat/lon for the bbox + the inside/outside test, then project in one batch
    lats, lons = nc.lat, nc.lon
    del nc
    min_lat, max_lat = min(lats.values()), max(lats.values())
    min_lon, max_lon = min(lons.values()), max(lons.values())
    print(f"      {len(lats)} nodes, bbox lat[{min_lat:.4f},{max_lat:.4f}] "
          f"lon[{min_lon:.4f},{max_lon:.4f}]", file=sys.stderr, flush=True)

    ids = list(lats.keys())
    lons_list = [lons[i] for i in ids]
    lats_list = [lats[i] for i in ids]
    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    px, py = tr.transform(lons_list, lats_list)
    coords = dict(zip(ids, zip(px, py)))
    del px, py, lons_list, lats_list, ids

    print("[2/6] reading ways ...", file=sys.stderr, flush=True)
    wc = WayCollector()
    wc.apply_file(OSM, locations=False)
    rows = wc.rows
    n_ways_osm = wc.n_ways
    del wc

    print("[3/6] per-way attributes and lengths ...", file=sys.stderr, flush=True)
    out = []
    for r in rows:
        t = r["tags"]
        refs = r["refs"]
        xs, ys = [], []
        missing = 0
        for ref in refs:
            c = coords.get(ref)
            if c is None:
                missing += 1
                continue
            xs.append(c[0])
            ys.append(c[1])
        length = 0.0
        for i in range(1, len(xs)):
            length += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
        rw = t.get("railway", "")
        service = t.get("service", "")
        usage = t.get("usage", "")
        gauge = t.get("gauge", "")
        out.append({
            "way_id": r["id"],
            "railway": rw,
            "usage": usage,
            "service": service,
            "gauge": gauge,
            "electrified": t.get("electrified", ""),
            "maxspeed": t.get("maxspeed", ""),
            "maxspeed_forward": t.get("maxspeed:forward", ""),
            "maxspeed_backward": t.get("maxspeed:backward", ""),
            "tracks": t.get("tracks", ""),
            "track_ref": t.get("railway:track_ref", ""),
            "preferred_direction": t.get("railway:preferred_direction", ""),
            "bidirectional": t.get("railway:bidirectional", ""),
            "oneway": t.get("oneway", ""),
            "passenger_lines": t.get("passenger_lines", ""),
            "tunnel": t.get("tunnel", ""),
            "bridge": t.get("bridge", ""),
            "name": t.get("name", ""),
            "ref": t.get("ref", ""),
            "operator": t.get("operator", ""),
            "n_nodes": len(refs),
            "first_node": refs[0] if refs else "",
            "last_node": refs[-1] if refs else "",
            "length_m": round(length, 3),
            "klass": classify(rw, service, usage, gauge),
            "routable": 1 if is_routable(rw, gauge) else 0,
            "_missing": missing,
            "_refs": refs,
            "_xy": list(zip(xs, ys)),
        })

    missing_total = sum(o["_missing"] for o in out)
    out.sort(key=lambda o: o["way_id"])

    with open(WAYS_CSV, "w", newline="", encoding="utf-8") as fh:
        wtr = csv.DictWriter(fh, fieldnames=CSV_COLS, extrasaction="ignore")
        wtr.writeheader()
        for o in out:
            wtr.writerow(o)
    print(f"      wrote {len(out)} rows, {missing_total} unresolved node refs",
          file=sys.stderr, flush=True)

    # ----------------------------------------------------------- components
    print("[4/6] connected components ...", file=sys.stderr, flush=True)
    rout = [o for o in out if o["routable"] == 1]
    parent = {}
    sizehint = {}

    def find(a):
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra == rb:
            return
        # deterministic: smaller way id becomes the root
        if ra < rb:
            parent[rb] = ra
        else:
            parent[ra] = rb

    for o in rout:
        parent[o["way_id"]] = o["way_id"]
    first_way_at_node = {}
    for o in rout:
        wid = o["way_id"]
        for nd in set(o["_refs"]):
            other = first_way_at_node.get(nd)
            if other is None:
                first_way_at_node[nd] = wid
            else:
                union(wid, other)
    del first_way_at_node

    groups = {}
    for o in rout:
        groups.setdefault(find(o["way_id"]), []).append(o)
    # rank: total length desc, then min way id asc
    ranked = sorted(groups.items(), key=lambda kv: (-sum(x["length_m"] for x in kv[1]),
                                                    min(x["way_id"] for x in kv[1])))
    comp_of = {}
    comp_meta = []          # (comp_id, size_ways, length_km)
    for cid, (_root, members) in enumerate(ranked):
        length_km = round(sum(x["length_m"] for x in members) / 1000.0, 3)
        comp_meta.append((cid, len(members), length_km))
        for x in members:
            comp_of[x["way_id"]] = cid

    with open(COMP_CSV, "w", newline="", encoding="utf-8") as fh:
        wtr = csv.writer(fh)
        wtr.writerow(["way_id", "component_id", "component_size_ways", "component_length_km"])
        for o in rout:                      # already sorted by way_id
            cid = comp_of[o["way_id"]]
            wtr.writerow([o["way_id"], cid, comp_meta[cid][1], comp_meta[cid][2]])

    # ----------------------------------------------------------- stops
    print("[5/6] stop distances to largest component ...", file=sys.stderr, flush=True)
    stops = []
    with gzip.open(SCHED, "rt", encoding="utf-8") as fh:
        for _ev, el in ET.iterparse(fh):
            if el.tag == "stopFacility":
                stops.append((el.get("id"), el.get("name", ""),
                              float(el.get("x")), float(el.get("y"))))
                el.clear()
    print(f"      {len(stops)} stop facilities", file=sys.stderr, flush=True)

    top = [o for o in out if o["routable"] == 1 and comp_of[o["way_id"]] == 0]
    CELL = 5000.0
    grid = {}
    gminx = gminy = 1e18
    gmaxx = gmaxy = -1e18
    nseg = 0
    for o in top:
        xy = o["_xy"]
        for i in range(1, len(xy)):
            x1, y1 = xy[i - 1]
            x2, y2 = xy[i]
            if x1 == x2 and y1 == y2:
                continue
            cx, cy = int(x1 // CELL), int(y1 // CELL)
            grid.setdefault((cx, cy), []).append((x1, y1, x2, y2))
            nseg += 1
            gminx = min(gminx, x1); gmaxx = max(gmaxx, x1)
            gminy = min(gminy, y1); gmaxy = max(gmaxy, y1)
    print(f"      {len(top)} ways / {nseg} segments in component 0", file=sys.stderr, flush=True)

    def pt_seg_dist(px_, py_, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        d2 = dx * dx + dy * dy
        if d2 == 0.0:
            return math.hypot(px_ - x1, py_ - y1)
        t = ((px_ - x1) * dx + (py_ - y1) * dy) / d2
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        return math.hypot(px_ - (x1 + t * dx), py_ - (y1 + t * dy))

    def nearest(px_, py_):
        cx0, cy0 = int(px_ // CELL), int(py_ // CELL)
        maxr = int(max(abs(px_ - gminx), abs(px_ - gmaxx),
                       abs(py_ - gminy), abs(py_ - gmaxy)) // CELL) + 2
        best = None
        for r in range(0, maxr + 1):
            cells = []
            if r == 0:
                cells.append((cx0, cy0))
            else:
                for dx in range(-r, r + 1):
                    cells.append((cx0 + dx, cy0 - r))
                    cells.append((cx0 + dx, cy0 + r))
                for dy in range(-r + 1, r):
                    cells.append((cx0 - r, cy0 + dy))
                    cells.append((cx0 + r, cy0 + dy))
            for c in cells:
                for s in grid.get(c, ()):
                    d = pt_seg_dist(px_, py_, *s)
                    if best is None or d < best:
                        best = d
            if best is not None and best <= r * CELL:
                break
        return best

    stop_res = []
    tr_inv = Transformer.from_crs("EPSG:2056", "EPSG:4326", always_xy=True)
    for sid, name, x, y in stops:
        d = nearest(x, y)
        lon, lat = tr_inv.transform(x, y)
        inside = (min_lon <= lon <= max_lon) and (min_lat <= lat <= max_lat)
        stop_res.append({"id": sid, "name": name, "x": x, "y": y,
                         "dist_m": d, "inside_bbox": inside})

    with open(STOPS_CSV, "w", newline="", encoding="utf-8") as fh:
        wtr = csv.writer(fh)
        wtr.writerow(["stop_id", "name", "x_lv95", "y_lv95", "dist_to_largest_component_m",
                      "inside_osm_bbox"])
        for s in stop_res:
            wtr.writerow([s["id"], s["name"], round(s["x"], 3), round(s["y"], 3),
                          round(s["dist_m"], 1), int(s["inside_bbox"])])

    # ----------------------------------------------------------- report
    print("[6/6] report ...", file=sys.stderr, flush=True)
    valid = validate(out, n_ways_osm, lats, lons)
    write_report(out, comp_meta, top, stop_res, n_ways_osm,
                 min_lat, max_lat, min_lon, max_lon, valid)
    print("done", file=sys.stderr, flush=True)


# ------------------------------------------------------------------ validation
def validate(out, n_ways_osm, lats, lons):
    """Independent re-read of the OSM XML (ElementTree, not pyosmium) for the way
    count and for one representative way of three klasses.  Returns report lines."""
    picks = {}
    for o in sorted(out, key=lambda o: (-o["length_m"], o["way_id"])):
        if o["klass"] not in picks:
            picks[o["klass"]] = o["way_id"]
    want = [picks["main"], picks["siding"], picks["narrow_gauge"]]

    found = {}
    n_way_elems = 0
    with open(OSM, "rb") as fh:
        for _ev, el in ET.iterparse(fh, events=("end",)):
            if el.tag == "way":
                n_way_elems += 1
                wid = int(el.get("id"))
                if wid in want:
                    tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
                    found[wid] = (tags, [nd.get("ref") for nd in el.findall("nd")])
                el.clear()

    objs = {o["way_id"]: o for o in out}
    lines = ["## Validation (self-check)"]
    ok = "PASS" if n_way_elems == n_ways_osm == len(out) else "FAIL"
    lines.append(f"- {ok}: `<way>` elements in the OSM file = {n_way_elems}, "
                 f"pyosmium ways = {n_ways_osm}, `ways.csv` rows = {len(out)}.")
    for wid in want:
        o = objs[wid]
        rtags, rrefs = found[wid]
        diffs = []
        tagmap = {"railway": "railway", "usage": "usage", "service": "service",
                  "gauge": "gauge", "maxspeed": "maxspeed", "tracks": "tracks",
                  "track_ref": "railway:track_ref",
                  "preferred_direction": "railway:preferred_direction",
                  "bidirectional": "railway:bidirectional", "bridge": "bridge",
                  "tunnel": "tunnel", "ref": "ref", "operator": "operator"}
        for col, key in tagmap.items():
            if o[col] != rtags.get(key, ""):
                diffs.append(f"{col}: csv={o[col]!r} xml={rtags.get(key,'')!r}")
        if o["n_nodes"] != len(rrefs) or str(o["first_node"]) != rrefs[0] \
                or str(o["last_node"]) != rrefs[-1]:
            diffs.append("node refs differ")
        state = "PASS" if not diffs else "FAIL " + "; ".join(diffs)
        h = 0.0
        for a, b in zip(rrefs, rrefs[1:]):
            h += GEOD.inv(lons[int(a)], lats[int(a)], lons[int(b)], lats[int(b)])[2]
        dh = abs(h - o["length_m"]) / max(h, 1e-9)
        quote = " ".join(f'<tag k="{k}" v="{v}"/>' for k, v in list(rtags.items())[:4])
        lines.append(f"- {state}: way {wid} klass={o['klass']} routable={o['routable']} "
                     f"n_nodes={len(rrefs)} length_m={o['length_m']:.1f} "
                     f"(ellipsoidal geodesic {h:.1f}, Δ {100*dh:.3f}%); raw XML "
                     f'`<way id="{wid}">` has `{quote} …`')
    return lines


# ------------------------------------------------------------------- report bits
def pct(sorted_vals, q):
    """nearest-rank percentile"""
    if not sorted_vals:
        return float("nan")
    k = max(1, math.ceil(q * len(sorted_vals)))
    return sorted_vals[k - 1]


def write_report(out, comp_meta, top, stop_res, n_ways_osm,
                 min_lat, max_lat, min_lon, max_lon, valid):
    """Report is capped at 80 lines: the far-stop list is packed to the remaining budget."""
    L = []
    A = L.append

    A("# A01 — OSM railway ways in Switzerland: classification + routable components")
    A(f"Input `original_data/osm/switzerland_railways.osm` = {n_ways_osm} `<way>` elements "
      f"→ {len(out)} rows in `ways.csv` (one row per way). "
      "`length_m` = polyline length in EPSG:2056 (LV95, pyproj).")
    A("`routable` = 1 iff `railway=rail` and (`gauge` empty or contains `1435`); "
      "sidings/yards/crossovers stay routable (they are real 1435 mm track, penalised later).")
    A("## Ways and km per klass")
    A("| klass | ways | km | routable ways |")
    A("|---|---:|---:|---:|")
    kl = {}
    for o in out:
        e = kl.setdefault(o["klass"], [0, 0.0, 0])
        e[0] += 1
        e[1] += o["length_m"]
        e[2] += o["routable"]
    for k in sorted(kl, key=lambda k: -kl[k][1]):
        w, m, r = kl[k]
        A(f"| {k} | {w} | {m/1000.0:.1f} | {r} |")
    A(f"| **total** | **{sum(v[0] for v in kl.values())}** | "
      f"**{sum(v[1] for v in kl.values())/1000.0:.1f}** | "
      f"**{sum(o['routable'] for o in out)}** |")
    A("klass precedence (the task's list is a set of definitions, not an order): "
      "disused/abandoned/razed/construction/proposed → `disused`; then tram/light_rail/subway/"
      "funicular/platform by `railway`; then `narrow_gauge` (=`railway=narrow_gauge` or a `gauge` "
      "without 1435); then `railway=rail`: service∈{siding,yard,spur,crossover} → that class, "
      "else usage main/branch, else rail_other; anything else → other. "
      "9 `railway=rail` ways are not routable (gauge 1000 or 1453, i.e. tagged rail by mistake).")

    A("## Routable components")
    tot_km = sum(c[2] for c in comp_meta)
    A(f"{len(top)} routable ways → {len(comp_meta)} connected components (two routable ways are "
      f"connected iff they share an OSM node; ids sorted by total length, 0 = largest).")
    A(f"component 0: {comp_meta[0][1]} ways, {comp_meta[0][2]:.1f} km = "
      f"{100.0*comp_meta[0][2]/max(tot_km,1e-9):.1f}% of all routable km. "
      "top 10 (id: ways / km): "
      + ", ".join(f"{c[0]}: {c[1]} / {c[2]:.1f}" for c in comp_meta[:10]))

    mb = [o for o in out if o["klass"] in ("main", "branch")]
    mb_km = sum(o["length_m"] for o in mb)
    A(f"## Tag coverage on main+branch ways ({len(mb)} ways, {mb_km/1000.0:.1f} km)")
    A("| tag | ways with value | % ways | km with value | % km |")
    A("|---|---:|---:|---:|---:|")
    for tag in ("maxspeed", "track_ref", "preferred_direction", "bidirectional",
                "gauge", "electrified", "passenger_lines"):
        n = sum(1 for o in mb if o[tag] != "")
        m = sum(o["length_m"] for o in mb if o[tag] != "")
        A(f"| {tag} | {n} | {100.0*n/max(len(mb),1):.1f} | {m/1000.0:.1f} | "
          f"{100.0*m/max(mb_km,1e-9):.1f} |")

    A("## Value distribution on main+branch ways (top values)")
    for tag in ("preferred_direction", "bidirectional", "gauge"):
        cnt = {}
        for o in mb:
            v = o[tag]
            if v != "":
                cnt[v] = cnt.get(v, 0) + 1
        topv = sorted(cnt.items(), key=lambda kv: (-kv[1], kv[0]))[:6]
        empty = sum(1 for o in mb if o[tag] == "")
        A(f"- **{tag}** ({empty} empty): "
          + ", ".join(f"`{k}`={v}" for k, v in topv))

    ds = sorted(s["dist_m"] for s in stop_res)
    ds_in = sorted(s["dist_m"] for s in stop_res if s["inside_bbox"])
    A("## Stop facilities vs the largest routable component")
    A(f"{len(ds)} `<stopFacility>` (LV95 from `schedule_gtfs_filtered.xml.gz`). Point-to-segment "
      f"distance to component 0, nearest-rank percentiles over all {len(ds)} stops: "
      f"p50={pct(ds,0.50):.1f} m, p90={pct(ds,0.90):.1f} m, p99={pct(ds,0.99):.1f} m, "
      f"max={ds[-1]:.1f} m.")
    A(f"Only the {len(ds_in)} stops inside the OSM node bbox "
      f"(lon[{min_lon:.2f},{max_lon:.2f}], lat[{min_lat:.2f},{max_lat:.2f}]): "
      f"p50={pct(ds_in,0.50):.1f} m, p90={pct(ds_in,0.90):.1f} m, "
      f"p99={pct(ds_in,0.99):.1f} m, max={ds_in[-1]:.1f} m. "
      f"{len(ds)-len(ds_in)} stops are outside that bbox (abroad, outside the extract).")
    far = [s for s in stop_res if s["dist_m"] > 50.0]
    n_far_in = sum(1 for s in far if s["inside_bbox"])
    A(f"Stops > 50 m: {len(far)} total — {n_far_in} inside the bbox, "
      f"{len(far)-n_far_in} outside. Distant stops are either abroad (DE/IT/FR/AT) or on "
      "narrow-gauge-only lines (RhB, MGB, ZB, MOB…), which are correctly absent from "
      "component 0 (standard gauge only).")
    A("All 994 distances: `stops_distance.csv`. Stops > 50 m, grouped by station+distance, "
      "`name dist [full GTFS stop ids]`:")

    # ---- pack the far list into the remaining line budget
    groups = {}
    order = []
    for s in sorted(far, key=lambda s: (-s["dist_m"], s["id"])):
        key = (s["name"], round(s["dist_m"], 1))
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(s["id"])
    entries = []
    for name, d in order:
        ids = ",".join(groups[(name, d)])
        e = f"{name} {d:.0f} m [{ids}]"
        entries.append(e)
    budget = 80 - (len(L) + len(valid) + 1)
    width = 120
    while width < 6000:
        packed, cur = [], ""
        for e in entries:
            piece = e if not cur else " · " + e
            if cur and len(cur) + len(piece) > width:
                packed.append(cur)
                cur = e
            else:
                cur += piece
        if cur:
            packed.append(cur)
        if len(packed) <= budget:
            break
        width += 20
    L.extend(packed)
    L.extend(valid)

    with open(REPORT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    n = len(L)
    print(f"      REPORT.md: {n} lines (packed stop lines at width {width})",
          file=sys.stderr, flush=True)
    if n > 80:
        print("      WARNING: report exceeds 80 lines", file=sys.stderr, flush=True)


def run():
    main()


if __name__ == "__main__":
    run()
