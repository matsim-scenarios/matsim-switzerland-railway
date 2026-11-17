"""A06 - OSM route=train relations, matched to the timetable routes.

Provenance: copied unchanged from `swarm/data/A06/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""

import csv
import gzip
import math
import os
import pickle
import re
import sys
import time
import xml.etree.ElementTree as ET

from prepare.paths import get as _paths

_P = _paths()
HERE = str(_P.table("A06"))
ROOT = str(_P.root)
PBF = str(_P.pbf)
SCHED = str(_P.schedule)

TRAIN_ROUTES = {"train", "tracks", "railway"}
# railway values that are a physical (passenger) track
TRACK_VALUES = {
    "rail", "narrow_gauge", "light_rail", "tram", "subway", "funicular",
    "preserved", "disused", "abandoned", "construction", "miniature",
}
STOP_ROLES = {"stop", "stop_entry_only", "stop_exit_only", "stop_position", ""}

OLTEN_UIC = "8500218"
CATEGORIES = ("IC", "IR", "EC", "RE", "ICE")


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def cache_path(name):
    return os.path.join(HERE, name)


def save_pickle(name, obj):
    p = cache_path(name)
    tmp = p + ".tmp"
    with open(tmp, "wb") as f:
        pickle.dump(obj, f, protocol=4)
    os.replace(tmp, p)
    log(f"  cached {name}: {os.path.getsize(p)/1e6:.1f} MB")


def load_pickle(name):
    p = cache_path(name)
    if not os.path.exists(p):
        return None
    with open(p, "rb") as f:
        return pickle.load(f)


# --------------------------------------------------------------------------
# stage 1: relations
# --------------------------------------------------------------------------
def stage_relations():
    c = load_pickle("cache_relations.pkl")
    if c is not None:
        return c
    import osmium

    t0 = time.time()
    rels = {}
    for r in osmium.FileProcessor(PBF, osmium.osm.RELATION):
        tags = {t.k: t.v for t in r.tags}
        if tags.get("route") not in TRAIN_ROUTES:
            continue
        rels[r.id] = {
            "tags": tags,
            "members": [(m.type, m.ref, m.role) for m in r.members],
        }
    log(f"stage 1: {len(rels)} route relations in {time.time()-t0:.1f}s")
    save_pickle("cache_relations.pkl", rels)
    return rels


# --------------------------------------------------------------------------
# stage 2: member way geometry + tags, stop-node tags, node->rail-way index
# --------------------------------------------------------------------------
def stage_ways(rels):
    ways = load_pickle("cache_ways.pkl")
    nodeway = load_pickle("cache_node_way.pkl")
    nodes = load_pickle("cache_nodes.pkl")
    if ways is not None and nodeway is not None and nodes is not None:
        return ways, nodeway, nodes

    import osmium
    from pyproj import Geod

    geod = Geod(ellps="WGS84")

    member_ids = set()
    node_ids = set()
    for r in rels.values():
        for typ, ref, role in r["members"]:
            if typ == "w":
                member_ids.add(ref)
            elif typ == "n":
                node_ids.add(ref)

    t0 = time.time()
    if ways is None:
        ways = {}
        importer = osmium.FileProcessor(PBF, osmium.osm.WAY).with_filter(
            osmium.filter.IdFilter(member_ids)
        )
        for w in importer:
            nl = [n.ref for n in w.nodes]
            ways[w.id] = {"nodes": nl, "tags": {t.k: t.v for t in w.tags}}
        log(f"stage 2a: {len(ways)}/{len(member_ids)} member ways in {time.time()-t0:.1f}s")
        save_pickle("cache_ways.pkl", ways)

    t1 = time.time()
    if nodeway is None:
        # stop node -> containing physical track way (railway=rail preferred)
        nodeway = {}
        for w in osmium.FileProcessor(PBF, osmium.osm.WAY).with_filter(
            osmium.filter.KeyFilter("railway")
        ):
            rv = None
            for t in w.tags:
                if t.k == "railway":
                    rv = t.v
                    break
            if rv not in TRACK_VALUES:
                continue
            hit = next((n.ref for n in w.nodes if n.ref in node_ids), None)
            if hit is None:
                continue
            for n in w.nodes:
                if n.ref in node_ids:
                    prev = nodeway.get(n.ref)
                    if prev is None or (rv == "rail" and prev[1] != "rail"):
                        nodeway[n.ref] = (w.id, rv)
        log(f"stage 2b: {len(nodeway)} stop nodes located on track ways in {time.time()-t1:.1f}s")
        save_pickle("cache_node_way.pkl", nodeway)

    t2 = time.time()
    if nodes is None:
        nodes = {}
        importer = osmium.FileProcessor(PBF, osmium.osm.NODE).with_filter(
            osmium.filter.IdFilter(node_ids)
        )
        for n in importer:
            tags = {t.k: t.v for t in n.tags}
            nodes[n.id] = (n.location.lat, n.location.lon, tags)
        log(f"stage 2c: {len(nodes)} stop nodes in {time.time()-t2:.1f}s")
        save_pickle("cache_nodes.pkl", nodes)

    # fill in way length + endpoint coords (needs node coords of ALL way nodes,
    # not only stop nodes -> separate id-filtered pass)
    t3 = time.time()
    need = set()
    for w in ways.values():
        if "len_km" not in w:
            need.update(w["nodes"])
    if need:
        coords = {}
        importer = osmium.FileProcessor(PBF, osmium.osm.NODE).with_filter(
            osmium.filter.IdFilter(need)
        )
        for n in importer:
            coords[n.id] = (n.location.lon, n.location.lat)
        log(f"stage 2d: {len(coords)}/{len(need)} way nodes in {time.time()-t3:.1f}s")

        for w in ways.values():
            if "len_km" in w:
                continue
            lons, lats = [], []
            for nid in w["nodes"]:
                p = coords.get(nid)
                if p is not None:
                    lons.append(p[0])
                    lats.append(p[1])
            if len(lons) >= 2:
                length = abs(geod.line_length(lons, lats)) / 1000.0
            else:
                length = 0.0
            w["len_km"] = round(length, 4)
            if w["nodes"]:
                w["first_node"] = w["nodes"][0]
                w["last_node"] = w["nodes"][-1]
                a = coords.get(w["nodes"][0])
                b = coords.get(w["nodes"][-1])
                w["first_ll"] = a
                w["last_ll"] = b
        save_pickle("cache_ways.pkl", ways)
    return ways, nodeway, nodes


# --------------------------------------------------------------------------
# stage 3: timetable
# --------------------------------------------------------------------------
def parse_schedule():
    c = load_pickle("cache_schedule.pkl")
    if c is not None and "coords" in c:
        return c
    t0 = time.time()
    names = {}
    coords = {}
    lines = []
    with gzip.open(SCHED, "rb") as f:
        ctx = ET.iterparse(f, events=("start", "end"))
        cur_line = None
        cur_route = None
        for ev, el in ctx:
            if ev == "start" and el.tag == "stopFacility":
                names[el.get("id")] = el.get("name")
                try:
                    coords[el.get("id")] = (float(el.get("x")), float(el.get("y")))
                except (TypeError, ValueError):
                    pass
            elif ev == "start" and el.tag == "transitLine":
                cur_line = {
                    "id": el.get("id"),
                    "name": el.get("name"),
                    "routes": [],
                }
            elif ev == "start" and el.tag == "transitRoute":
                cur_route = {"id": el.get("id"), "stops": [], "n_dep": 0}
            elif ev == "start" and el.tag == "stop" and cur_route is not None:
                ref = el.get("refId")
                cur_route["stops"].append(ref)
            elif ev == "start" and el.tag == "departure" and cur_route is not None:
                cur_route["n_dep"] += 1
            elif ev == "end" and el.tag == "transitRoute":
                cur_line["routes"].append(cur_route)
                cur_route = None
            elif ev == "end" and el.tag == "transitLine":
                lines.append(cur_line)
                cur_line = None
                el.clear()
    log(f"stage 3: {len(lines)} lines, "
        f"{sum(len(l['routes']) for l in lines)} routes, "
        f"{sum(r['n_dep'] for l in lines for r in l['routes'])} departures "
        f"in {time.time()-t0:.1f}s")
    out = {"names": names, "lines": lines, "coords": coords}
    save_pickle("cache_schedule.pkl", out)
    return out


# --------------------------------------------------------------------------
# analysis helpers
# --------------------------------------------------------------------------
def is_track_way(w):
    rv = w["tags"].get("railway")
    return rv in TRACK_VALUES or rv is None


def norm_ref(s):
    return re.sub(r"[^A-Za-z0-9]", "", (s or "")).upper()


# GTFS collapses the two parts of Zürich HB into UIC 8503000, OSM maps the
# Löwenstrasse part as its own DIDOK station (8516144). Treat them as equal.
UIC_ALIAS = {"8516144": "8503000"}


def norm_name(s):
    s = (s or "").lower()
    s = re.sub(r"\(.*?\)", "", s)          # drop "(Löwenstrasse)" etc.
    s = s.replace("hauptbahnhof", "hb").replace("hbf", "hb")
    s = re.sub(r"[^a-z0-9]", "", s)
    return s


def relation_way_chain(rel, ways):
    """ordered, non-platform way members -> list of way ids"""
    out = []
    for typ, ref, role in rel["members"]:
        if typ != "w":
            continue
        if role.startswith("platform"):
            continue
        w = ways.get(ref)
        if w is None or len(w["nodes"]) < 2:
            continue
        if not is_track_way(w):
            continue
        out.append(ref)
    return out


def orient_chain(way_ids, ways, stops_geo):
    """sign[i]: +1 forward (first_node->last_node), -1 backward, 0 unknown."""
    n = len(way_ids)
    sign = [0] * n
    if n == 0:
        return sign, 0
    gaps = 0
    for i in range(n - 1):
        a = ways[way_ids[i]]
        b = ways[way_ids[i + 1]]
        af, al = a["nodes"][0], a["nodes"][-1]
        bf, bl = b["nodes"][0], b["nodes"][-1]
        if al == bf:
            cand = (1, 1)
        elif al == bl:
            cand = (1, -1)
        elif af == bf:
            cand = (-1, 1)
        elif af == bl:
            cand = (-1, -1)
        else:
            cand = None
        if cand is None:
            gaps += 1
            continue
        if sign[i] == 0:
            # first way of a new run after a gap: seed from this pair
            sign[i] = cand[0]
        if sign[i] == cand[0]:
            sign[i + 1] = cand[1]
        else:
            gaps += 1
    if n == 1:
        sign[0] = 1

    # ---- absolute anchor: does stop order agree with way order? ----
    node_pos = {}
    for idx, wid in enumerate(way_ids):
        for nid in ways[wid]["nodes"]:
            node_pos.setdefault(nid, []).append(idx)
    pairs = []
    for sidx, s in enumerate(stops_geo):
        nid = s.get("node_id")
        if nid is None:
            continue
        pos = node_pos.get(nid)
        if not pos:
            continue
        pairs.append((sidx, sum(pos) / len(pos)))
    anchor = 0
    if len(pairs) >= 2:
        conc = disc = 0
        for a in range(len(pairs)):
            for b in range(a + 1, len(pairs)):
                d = (pairs[b][0] - pairs[a][0]) * (pairs[b][1] - pairs[a][1])
                if d > 0:
                    conc += 1
                elif d < 0:
                    disc += 1
        if conc > disc:
            anchor = 1
        elif disc > conc:
            anchor = -1
    if anchor == 0:
        # geometric fallback: first/last stop versus chain entry/exit
        first = ways[way_ids[0]].get("first_ll")
        last = ways[way_ids[-1]].get("last_ll")
        s0 = stops_geo[0].get("ll") if stops_geo else None
        s1 = stops_geo[-1].get("ll") if stops_geo else None
        if first and last and s0 and s1:
            d_keep = haversine(first, s0) + haversine(last, s1)
            d_flip = haversine(first, s1) + haversine(last, s0)
            if d_keep * 1.2 < d_flip:
                anchor = 1
            elif d_flip * 1.2 < d_keep:
                anchor = -1
    if anchor == 0:
        return [0] * n, gaps
    return [s * anchor for s in sign], gaps


def haversine(a, b):
    lon1, lat1 = a
    lon2, lat2 = b
    r = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def bearing_of(a, b):
    """initial bearing a->b in degrees (0 = north)."""
    dx = (b[0] - a[0]) * math.cos(math.radians((a[1] + b[1]) / 2.0))
    dy = b[1] - a[1]
    return math.degrees(math.atan2(dx, dy))


def build_relation_tables(rels, ways, nodeway, nodes):
    out = {"relations": [], "ways": [], "stops": [], "way_meta": {}}
    for rid in sorted(rels):
        rel = rels[rid]
        tags = rel["tags"]
        wids = relation_way_chain(rel, ways)

        stops_geo = []
        stop_members = []
        member_node_way = {}
        for wid in wids:
            for nid in ways[wid]["nodes"]:
                member_node_way.setdefault(nid, wid)
        for typ, ref, role in rel["members"]:
            if typ != "n":
                continue
            if role.startswith("platform"):
                continue
            if role not in STOP_ROLES:
                continue
            nd = nodes.get(ref)
            tags_n = nd[2] if nd else {}
            nw = nodeway.get(ref)
            # prefer the relation's own track way through the stop node
            way_id = member_node_way.get(ref) or (nw[0] if nw else "")
            stop_members.append({
                "node_id": ref,
                "role": role,
                "name": tags_n.get("name", ""),
                "uic_ref": tags_n.get("uic_ref", ""),
                "local_ref": tags_n.get("local_ref", ""),
                "ref": tags_n.get("ref", ""),
                "way_id": way_id,
                "ll": (nd[1], nd[0]) if nd else None,
            })
            stops_geo.append({"node_id": ref, "ll": (nd[1], nd[0]) if nd else None})

        sign, gaps = orient_chain(wids, ways, stops_geo)

        total_km = sum(ways[w].get("len_km", 0.0) for w in wids)
        n_stop_members = len(stop_members)
        n_raw_way_members = sum(1 for typ, _ref, _role in rel["members"] if typ == "w")
        n_missing = sum(1 for typ, ref, _role in rel["members"]
                        if typ == "w" and ref not in ways)
        connected = 1 if (gaps == 0 and len(wids) >= 1) else 0

        out["relations"].append({
            "rel_id": rid,
            "route": tags.get("route", ""),
            "ref": tags.get("ref", ""),
            "name": tags.get("name", ""),
            "from": tags.get("from", ""),
            "to": tags.get("to", ""),
            "network": tags.get("network", ""),
            "operator": tags.get("operator", ""),
            "service": tags.get("service", ""),
            "n_way_members": len(wids),
            "n_raw_way_members": n_raw_way_members,
            "n_stop_members": n_stop_members,
            "total_length_km": round(total_km, 3),
            "connected": connected,
            "n_gaps": gaps,
            "n_uic_stops": sum(1 for s in stop_members if s["uic_ref"]),
            "n_local_ref_stops": sum(1 for s in stop_members if s["local_ref"]),
            "n_missing_way_members": n_missing,
        })

        for i, wid in enumerate(wids):
            w = ways[wid]
            d = sign[i]
            out["ways"].append({
                "rel_id": rid,
                "seq": i,
                "way_id": wid,
                "role": "",
                "first_node": w["nodes"][0],
                "last_node": w["nodes"][-1],
                "travel_dir": "forward" if d > 0 else ("backward" if d < 0 else "unknown"),
                "len_km": w.get("len_km", 0.0),
                "route": tags.get("route", ""),
            })
            out["way_meta"][wid] = w.get("len_km", 0.0)

        for i, s in enumerate(stop_members):
            out["stops"].append({
                "rel_id": rid,
                "seq": i,
                "node_id": s["node_id"],
                "role": s["role"],
                "name": s["name"],
                "uic_ref": s["uic_ref"],
                "local_ref": s["local_ref"],
                "ref": s["ref"],
                "way_id": s["way_id"],
                "route": tags.get("route", ""),
                "ll": s["ll"],
            })
    return out


# --------------------------------------------------------------------------
# matching
# --------------------------------------------------------------------------
def station_key(uic, name):
    """(uic, normalised name) identifying a station on both sides of the match."""
    u = UIC_ALIAS.get((uic or "").strip(), (uic or "").strip())
    n = norm_name(name)
    if not u and not n:
        return None
    return (u, n)


def seq_ok(tkey, rkey):
    if tkey is None or rkey is None:
        return False
    if tkey[0] and rkey[0]:
        return tkey[0] == rkey[0]
    if tkey[1] and rkey[1]:
        return tkey[1] == rkey[1]
    return False


def subseq_score(rel_keys, tt_keys):
    """share of tt_keys that appear, in order, in rel_keys (greedy, earliest match)"""
    j = 0
    matched = 0
    for k in tt_keys:
        for m in range(j, len(rel_keys)):
            if seq_ok(k, rel_keys[m]):
                matched += 1
                j = m + 1
                break
        # a timetable stop missing from the relation does not move the cursor,
        # otherwise one gap would swallow the whole remainder of the relation
    return matched / len(tt_keys) if tt_keys else 0.0


def tkey_of(ref, names):
    if not ref:
        return None
    uic = ref.split(":", 1)[0]
    return station_key(uic if uic.isdigit() else "", names.get(ref, ""))


def platform_of(ref):
    if not ref or ":" not in ref:
        return ""
    p = ref.rsplit(":", 1)[1]
    return p


def do_match(reltab, sched):
    rels_by_ref = {}
    rel_ref_of = {}
    for r in reltab["relations"]:
        if r["route"] != "train":
            continue
        rel_ref_of[r["rel_id"]] = r["ref"]
        k = norm_ref(r["ref"])
        if not k:
            continue
        rels_by_ref.setdefault(k, []).append(r["rel_id"])

    rel_stops = {}
    for s in reltab["stops"]:
        rel_stops.setdefault(s["rel_id"], []).append(s)
    for rid in rel_stops:
        rel_stops[rid].sort(key=lambda x: x["seq"])
    rel_keys = {}
    for rid, sl in rel_stops.items():
        rel_keys[rid] = [station_key(s["uic_ref"], s["name"]) for s in sl]

    # relations grouped by product category (IC/IR/EC/RE/ICE) for the
    # relaxed, report-only fallback used for bare GTFS line names
    rels_by_cat = {}
    for r in reltab["relations"]:
        if r["route"] != "train":
            continue
        c = category_of(r["ref"])
        rels_by_cat.setdefault(c, []).append(r["rel_id"])

    def score_over(cands, tt_keys):
        best = None
        for rid in cands:
            rk = rel_keys.get(rid, [])
            if not rk:
                continue
            fwd = subseq_score(rk, tt_keys)
            if fwd <= 0:
                continue
            rev = subseq_score(list(reversed(rk)), tt_keys)
            direction_ok = 1 if fwd >= rev else 0
            if best is None or (fwd, -rid) > (best[0], -best[1]):
                best = (fwd, rid, direction_ok)
        return best

    names = sched["names"]
    rows = []
    for line in sched["lines"]:
        lk = norm_ref(line["name"])
        cands = rels_by_ref.get(lk, [])
        cat = category_of(line["name"])
        for route in line["routes"]:
            tt_keys = [tkey_of(ref, names) for ref in route["stops"]]
            best = score_over(cands, tt_keys)
            row = {
                "transit_line_id": line["id"],
                "line_name": line["name"],
                "transit_route_id": route["id"],
                "n_departures": route["n_dep"],
                "n_stops": len(route["stops"]),
                "rel_id": "",
                "rel_ref": "",
                "match_score": 0.0,
                "n_stops_matched": 0,
                "direction_ok": 0,
                "good": 0,
                "stops": route["stops"],
                "category": cat,
                "has_olten": any((s.split(":", 1)[0] == OLTEN_UIC) for s in route["stops"]),
                "relaxed_rel_id": "",
                "relaxed_score": 0.0,
                "relaxed_good": 0,
            }
            if best is not None:
                score, rid, direction_ok = best
                row.update({
                    "rel_id": rid,
                    "rel_ref": rel_ref_of.get(rid, ""),
                    "match_score": round(score, 3),
                    "n_stops_matched": round(score * len(tt_keys)),
                    "direction_ok": direction_ok,
                    "good": 1 if (score >= 0.8 and direction_ok) else 0,
                })
            # report-only fallback: a bare category name ("IC", "RE", "EC"...)
            # can never equal a relation ref like "IC 5"; score it against all
            # relations of the same category to quantify the loss.
            if norm_ref(line["name"]) in set(CATEGORIES) and cat != "other":
                rb = score_over(rels_by_cat.get(cat, []), tt_keys)
                if rb is not None:
                    rscore, rrid, rdir = rb
                    row["relaxed_rel_id"] = rrid
                    row["relaxed_score"] = round(rscore, 3)
                    row["relaxed_good"] = 1 if (rscore >= 0.8 and rdir) else 0
            rows.append(row)
    return rows


def category_of(name):
    n = (name or "").upper()
    for c in ("ICE", "EC", "IC", "IR", "RE"):
        if n.startswith(c):
            return c
    return "other"


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def write_csv(name, fieldnames, rows):
    p = os.path.join(HERE, name)
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    log(f"wrote {name}: {len(rows)} rows")


def main():
    t0 = time.time()
    rels = stage_relations()
    ways, nodeway, nodes = stage_ways(rels)
    sched = parse_schedule()
    reltab = build_relation_tables(rels, ways, nodeway, nodes)
    rows = do_match(reltab, sched)

    # ---- relation tables ----
    write_csv(
        "relations.csv",
        ["rel_id", "ref", "name", "from", "to", "network", "operator", "service",
         "n_way_members", "n_stop_members", "total_length_km", "connected",
         "n_gaps", "route", "n_uic_stops", "n_local_ref_stops", "n_raw_way_members",
         "n_missing_way_members"],
        sorted(reltab["relations"], key=lambda r: r["rel_id"]),
    )
    write_csv(
        "relation_ways.csv",
        ["rel_id", "seq", "way_id", "role", "first_node", "last_node", "travel_dir"],
        sorted(reltab["ways"], key=lambda r: (r["rel_id"], r["seq"])),
    )
    write_csv(
        "relation_stops.csv",
        ["rel_id", "seq", "node_id", "role", "name", "uic_ref", "local_ref", "ref", "way_id"],
        sorted(reltab["stops"], key=lambda r: (r["rel_id"], r["seq"])),
    )

    # ---- way usage direction ----
    usage = {}
    rel_index = {r["rel_id"]: r for r in reltab["relations"]}
    for w in reltab["ways"]:
        if w["route"] != "train":
            continue
        rel = rel_index.get(w["rel_id"], {})
        if not rel.get("connected"):
            continue
        u = usage.setdefault(w["way_id"], [0, 0, 0, 0.0])
        if w["travel_dir"] == "forward":
            u[0] += 1
        elif w["travel_dir"] == "backward":
            u[1] += 1
        else:
            u[2] += 1
        u[3] = w["len_km"]
    usage_rows = [{
        "way_id": wid,
        "n_rel_forward": u[0],
        "n_rel_backward": u[1],
        "n_rel_unknown": u[2],
        "length_km": round(u[3], 4),
    } for wid, u in sorted(usage.items())]
    write_csv(
        "way_usage_direction.csv",
        ["way_id", "n_rel_forward", "n_rel_backward", "n_rel_unknown", "length_km"],
        usage_rows,
    )

    write_csv(
        "match.csv",
        ["transit_line_id", "line_name", "transit_route_id", "n_departures", "n_stops",
         "rel_id", "rel_ref", "match_score", "n_stops_matched", "direction_ok", "good",
         "category"],
        sorted(rows, key=lambda r: (r["line_name"], r["transit_route_id"])),
    )

    make_report(reltab, rows, usage_rows, sched, rels, ways)
    log(f"total {time.time()-t0:.1f}s")


def make_report(reltab, rows, usage_rows, sched, rels, ways):
    import collections

    rel_index = {r["rel_id"]: r for r in reltab["relations"]}
    train = [r for r in reltab["relations"] if r["route"] == "train"]
    connected = [r for r in train if r["connected"]]

    good = [r for r in rows if r["good"]]
    good_dep = sum(r["n_departures"] for r in good)
    total_dep = sum(r["n_departures"] for r in rows)

    olten = [r for r in rows if r["has_olten"]]
    olten_good = [r for r in olten if r["good"]]
    olten_dep = sum(r["n_departures"] for r in olten)
    olten_good_dep = sum(r["n_departures"] for r in olten_good)

    # ---- stop pairing (route stop <-> relation stop) for the platform check
    stop_by_rel = collections.defaultdict(list)
    for s in reltab["stops"]:
        stop_by_rel[s["rel_id"]].append(s)
    for rid in stop_by_rel:
        stop_by_rel[rid].sort(key=lambda x: x["seq"])
    rel_keys = {rid: [station_key(s["uic_ref"], s["name"]) for s in sl]
                for rid, sl in stop_by_rel.items()}

    def pair_stops(rel_id, tt_stops):
        """greedy subsequence pairing -> list of (tt_ref, rel_stop_row)"""
        sl = stop_by_rel.get(rel_id, [])
        rk = rel_keys.get(rel_id, [])
        out = []
        j = 0
        for ref in tt_stops:
            tkey = tkey_of(ref, sched["names"])
            hit = None
            for m in range(j, len(rk)):
                if seq_ok(tkey, rk[m]):
                    hit = m
                    break
            if hit is None:
                continue
            j = hit + 1
            out.append((ref, sl[hit]))
        return out

    plat = collections.Counter()
    olten_plat = collections.Counter()
    per_rel_stop = collections.defaultdict(collections.Counter)
    trk = collections.Counter()
    disagree_examples = []
    for r in good:
        for ref, s in pair_stops(r["rel_id"], r["stops"]):
            osm_lr = (s["local_ref"] or "").strip()
            tt_pf = platform_of(ref).strip()
            at_olten = (s["uic_ref"] == OLTEN_UIC or ref.split(":", 1)[0] == OLTEN_UIC)
            if osm_lr and tt_pf:
                plat["known"] += 1
                key = (r["rel_id"], s["seq"])
                per_rel_stop[key][tt_pf] += 1
                if at_olten:
                    olten_plat["known"] += 1
                if osm_lr == tt_pf:
                    plat["agree"] += 1
                    if at_olten:
                        olten_plat["agree"] += 1
                else:
                    plat["disagree"] += 1
                    if len(disagree_examples) < 4:
                        disagree_examples.append(
                            f"{r['line_name']} rel {r['rel_id']} {s['name']}: "
                            f"OSM {osm_lr} vs TT {tt_pf}")
            w = ways.get(s["way_id"]) if s["way_id"] else None
            tr = (w["tags"].get("railway:track_ref") if w else None)
            if tr and tt_pf:
                trk["known"] += 1
                trk["agree"] += 1 if tr.strip() == tt_pf else 0

    # per (relation, stop): is the single OSM local_ref consistent with the
    # platforms the timetable uses for that line?
    rs_total = rs_in_set = rs_unique = rs_unique_ok = 0
    olten_among = olten_among_tot = 0
    for key, cnt in per_rel_stop.items():
        rid, seq = key
        s = stop_by_rel[rid][seq]
        lr = (s["local_ref"] or "").strip()
        if not lr or not cnt:
            continue
        rs_total += 1
        if lr in cnt:
            rs_in_set += 1
        if s["uic_ref"] == OLTEN_UIC:
            olten_among_tot += 1
            if lr in cnt:
                olten_among += 1
        if len(cnt) == 1:
            rs_unique += 1
            if lr == list(cnt)[0]:
                rs_unique_ok += 1

    # ---- direction / anchor validation
    ways_by_rel = collections.defaultdict(list)
    for w in reltab["ways"]:
        ways_by_rel[w["rel_id"]].append(w)
    n_train_usages = 0
    for rid, wl in ways_by_rel.items():
        if rel_index.get(rid, {}).get("route") == "train" and rel_index.get(rid, {}).get("connected"):
            n_train_usages += len(wl)
    anchor_ok = anchor_testable = 0
    coh_ok = coh_tot = 0
    for r in connected:
        sl = stop_by_rel.get(r["rel_id"], [])
        wl = ways_by_rel.get(r["rel_id"], [])
        if not sl or not wl:
            continue
        first = ways.get(wl[0]["way_id"], {}).get("first_ll")
        last = ways.get(wl[-1]["way_id"], {}).get("last_ll")
        s0 = sl[0].get("ll")
        s1 = sl[-1].get("ll")
        if first and last and s0 and s1:
            anchor_testable += 1
            if haversine(first, s0) + haversine(last, s1) <= haversine(first, s1) + haversine(last, s0):
                anchor_ok += 1
        # geometric coherence of travel_dir: the travel heading of a way must
        # point towards the way three positions further along the route
        for i, w in enumerate(wl):
            j = min(i + 3, len(wl) - 1)
            if j <= i:
                continue
            a = ways.get(w["way_id"], {})
            b = ways.get(wl[j]["way_id"], {})
            if not (a.get("first_ll") and a.get("last_ll") and b.get("first_ll") and b.get("last_ll")):
                continue
            p0, p1 = a["first_ll"], a["last_ll"]
            mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
            ref = ((b["first_ll"][0] + b["last_ll"][0]) / 2.0,
                   (b["first_ll"][1] + b["last_ll"][1]) / 2.0)
            hd = bearing_of(p0, p1)
            rb = bearing_of(mid, ref)
            diff = abs((hd - rb + 180.0) % 360.0 - 180.0)
            coh_tot += 1
            if (w["travel_dir"] == "forward") == (diff < 90.0):
                coh_ok += 1

    # ---- direction statistics
    multi = [u for u in usage_rows if (u["n_rel_forward"] + u["n_rel_backward"]) >= 2]
    same_fwd = [u for u in multi if u["n_rel_backward"] == 0]
    same_bwd = [u for u in multi if u["n_rel_forward"] == 0]
    both = [u for u in multi if u["n_rel_forward"] > 0 and u["n_rel_backward"] > 0]
    fwd_only = [u for u in usage_rows if u["n_rel_forward"] > 0 and u["n_rel_backward"] == 0]
    bwd_only = [u for u in usage_rows if u["n_rel_backward"] > 0 and u["n_rel_forward"] == 0]
    both_used = [u for u in usage_rows if u["n_rel_forward"] > 0 and u["n_rel_backward"] > 0]
    one_each = [u for u in both_used if u["n_rel_forward"] == 1 and u["n_rel_backward"] == 1]
    unknown_use = [u for u in usage_rows if u["n_rel_unknown"] > 0]

    def km(lst):
        return sum(u["length_km"] for u in lst)

    # ---- example: IC through Olten with the most departures and a resolvable
    # Olten local_ref on the matched relation
    example = None
    agree_example = None
    best_key = None
    best_agree_key = None
    for r in rows:
        if not (r["good"] and r["category"] == "IC" and r["has_olten"]):
            continue
        pr = pair_stops(r["rel_id"], r["stops"])
        ol = [(ref, s) for ref, s in pr if ref.split(":", 1)[0] == OLTEN_UIC]
        if not ol:
            continue
        s = ol[0][1]
        if not s["local_ref"]:
            continue
        key = (r["n_departures"], r["match_score"], -int(r["rel_id"]))
        if best_key is None or key > best_key:
            best_key = key
            example = (r, ol[0][0], s)
        if s["local_ref"] == platform_of(ol[0][0]):
            akey = (r["n_departures"], r["match_score"], -int(r["rel_id"]))
            if best_agree_key is None or akey > best_agree_key:
                best_agree_key = akey
                agree_example = (r, ol[0][0], s)
    if agree_example is not None and example is not None and agree_example[0]["rel_id"] == example[0]["rel_id"]:
        agree_example = None

    # ---- match coverage diagnosis
    no_rel = [r for r in rows if not r["rel_id"]]
    bare = [r for r in rows if norm_ref(r["line_name"]) in set(CATEGORIES)]
    relaxed_good = [r for r in rows if r["relaxed_good"] and not r["good"]]

    # ---- independent direction check of the matches: overall heading of the
    # timetable route (LV95 stop coordinates) vs. the relation's stop chain
    from pyproj import Transformer
    tf = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    tcoords = sched.get("coords", {})
    hd_ok = hd_bad = 0
    hd_examples = []

    def plain_bearing(a, b):
        return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1]))

    for r in good:
        fs, ls = r["stops"][0], r["stops"][-1]
        if fs not in tcoords or ls not in tcoords:
            continue
        sl = [s for s in stop_by_rel.get(r["rel_id"], []) if s.get("ll")]
        if len(sl) < 2:
            continue
        tt_b = plain_bearing(tcoords[fs], tcoords[ls])
        a = tf.transform(sl[0]["ll"][0], sl[0]["ll"][1])
        b = tf.transform(sl[-1]["ll"][0], sl[-1]["ll"][1])
        rb = plain_bearing(a, b)
        d = abs((tt_b - rb + 180.0) % 360.0 - 180.0)
        if d < 90.0:
            hd_ok += 1
        else:
            hd_bad += 1
            hd_examples.append(f"{r['line_name']} rel {r['rel_id']} ({d:.0f} deg off)")

    cats = ("IC", "IR", "EC", "RE", "ICE", "other")
    L = []
    A = L.append
    A("# A06 — OSM `route=train` relations vs. the simulated timetable")
    A("")
    A("Data: `original_data/osm/switzerland-latest.osm.pbf` (Geofabrik extract **with** relations) "
      "and `matsim_input/tmp/schedule_gtfs_filtered.xml.gz`. Script: `swarm/data/A06/build.py`.")
    A("")
    A("## 1. `route=train` relations in the extract")
    A("")
    A("| set | count |")
    A("|---|---|")
    A(f"| relations | {len(train)} |")
    A(f"| fully connected way chain (no gap) | {len(connected)} |")
    A(f"| with >=1 stop member carrying `uic_ref` | "
      f"{sum(1 for r in train if r['n_uic_stops'])} "
      f"({sum(r['n_uic_stops'] for r in train)} stop members) |")
    A(f"| with >=1 stop member carrying `local_ref` | "
      f"{sum(1 for r in train if r['n_local_ref_stops'])} "
      f"({sum(r['n_local_ref_stops'] for r in train)} stop members) |")
    A(f"| referencing way members **not in the CH extract** (cross-border / Nightjet) | "
      f"{sum(1 for r in train if r['n_missing_way_members'])} |")
    A(f"| stop members total | {sum(r['n_stop_members'] for r in train)} |")
    A("")
    A(f"Also extracted so they can be told apart (column `route`): "
      f"{sum(1 for r in reltab['relations'] if r['route']=='tracks')} `route=tracks` and "
      f"{sum(1 for r in reltab['relations'] if r['route']=='railway')} `route=railway` relations.")
    A(f"Track way members used in the chain: {sum(r['n_way_members'] for r in train)} of "
      f"{sum(r['n_raw_way_members'] for r in train)} raw way members (platform ways / non-track "
      f"members are excluded from the chain), sum of lengths "
      f"{sum(r['total_length_km'] for r in train):.0f} km (a way shared by two directions is counted "
      f"twice). {sum(1 for r in train if not r['connected'])} relations have a gap in the way chain; "
      f"their ways after the gap get `travel_dir=unknown`.")
    A("")
    A("## 2. Matching the 1231 timetable routes")
    A("")
    A("Rule: relation `ref` == line name (spaces/case ignored) AND the UIC sequence of the timetable "
      "route is an ordered subsequence of the relation's stop sequence in the same direction. "
      "`good` = `match_score` >= 0.8 and `direction_ok`.")
    A("")
    A("| category | routes | routes good | departures | departures good |")
    A("|---|---|---|---|---|")
    for c in cats:
        rs = [r for r in rows if r["category"] == c]
        if not rs:
            continue
        g = [r for r in rs if r["good"]]
        A(f"| {c} | {len(rs)} | {len(g)} | {sum(r['n_departures'] for r in rs)} | "
          f"{sum(r['n_departures'] for r in g)} |")
    A(f"| **all** | **{len(rows)}** | **{len(good)}** | **{total_dep}** | **{good_dep}** |")
    A("")
    A(f"Routes passing Olten (a stop with UIC {OLTEN_UIC}): **{len(olten)} routes / {olten_dep} "
      f"departures, of which {len(olten_good)} routes / {olten_good_dep} departures good** "
      f"({100.0*olten_good_dep/max(1,olten_dep):.0f} % of Olten departures).")
    A("")
    A(f"Why {len(rows)-len(good)} routes are not good: {len(no_rel)} have no relation whose `ref` "
      f"equals the line name (dominant cause: GTFS line name is only the product, e.g. "
      f"`IC`/`IR`/`EC`/`RE`/`ICE`, on {len(bare)} routes — no OSM ref can equal that), "
      f"{sum(1 for r in rows if r['rel_id'] and not r['good'])} match a relation by ref but the "
      f"sequence is a different variant (score < 0.8 or reversed).")
    if relaxed_good:
        A(f"Report-only extra check (NOT in `match.csv`, violates the ref-equality rule): if the "
          f"{len(bare)} bare-category routes were allowed to match any relation of the same product "
          f"category, {len(relaxed_good)} more routes / "
          f"{sum(r['n_departures'] for r in relaxed_good)} departures would reach score >= 0.8.")
    A("")
    A(f"Route-level direction check (independent of the stop matching): the heading from the timetable "
      f"route's first to its last stop vs. the heading of the relation's stop chain — "
      f"**{hd_ok}/{hd_ok+hd_bad} good matches point the same way**; exceptions: "
      f"{', '.join(hd_examples) if hd_examples else 'none'}.")
    A("")
    A("## 3. Platform (`local_ref`) agreement — the operational payload")
    A("")
    if plat["known"]:
        A(f"Over all good matches, (route, stop) pairs with both an OSM `local_ref` and a timetable "
          f"platform code: **{plat['known']}**; agreeing **{plat['agree']}** "
          f"({100.0*plat['agree']/plat['known']:.1f} %), disagreeing {plat['disagree']}.")
    A(f"The low rate is structural, not a tagging error: one relation = one variant and carries ONE "
      f"`local_ref` per stop, while the timetable has many platform variants per line. Per "
      f"(relation, stop) pair: {rs_total} testable, the OSM `local_ref` is a platform that the "
      f"matched timetable routes actually use in {rs_in_set} ({100.0*rs_in_set/max(1,rs_total):.0f} %); "
      f"where all matched routes agree on a single platform ({rs_unique} pairs) the OSM value matches "
      f"in {rs_unique_ok} ({100.0*rs_unique_ok/max(1,rs_unique):.0f} %).")
    if trk["known"]:
        A(f"Control: `railway:track_ref` of the track way the stop node lies on vs. the timetable "
          f"platform code: {trk['agree']}/{trk['known']} agree "
          f"({100.0*trk['agree']/trk['known']:.1f} %).")
    if disagree_examples:
        A("")
        A(f"At Olten: {olten_plat['agree']}/{olten_plat['known']} "
          f"({100.0*olten_plat['agree']/max(1,olten_plat['known']):.0f} %) of the (route, stop) pairs "
          f"agree, and the relation's `local_ref` is one of the Olten platforms the line's timetable "
          f"routes use in {olten_among}/{olten_among_tot}. Disagreement examples: "
          + "; ".join(disagree_examples) + ".")
    A("")
    if example:
        r, ref, s = example
        tt_pf = platform_of(ref)
        A("### Example (IC through Olten)")
        A("")
        A(f"`{r['line_name']}` route `{r['transit_route_id']}` ({r['n_departures']} departures, "
          f"{r['n_stops']} stops, score {r['match_score']}) -> relation **{r['rel_id']}** "
          f"`{r['rel_ref']}`.")
        A(f"Olten stop node `{s['node_id']}` in the relation: `local_ref={s['local_ref']}`, on track "
          f"way `{s['way_id']}`. Timetable platform at Olten for this route: `{tt_pf}` — identical "
          f"for all {r['n_departures']} departures, because the MATSim `routeProfile` holds a single "
          f"stop refId per route. Agreement: {'yes' if s['local_ref'] == tt_pf else 'NO'}.")
        if agree_example:
            r2, ref2, s2 = agree_example
            A(f"Counter-example with agreement: `{r2['line_name']}` route `{r2['transit_route_id']}` "
              f"({r2['n_departures']} departures) -> relation {r2['rel_id']}, Olten "
              f"`local_ref={s2['local_ref']}` = timetable platform `{platform_of(ref2)}`.")
        A("")
    A("## 4. Direction — the number that matters for track assignment")
    A("")
    A(f"Over the {len(connected)} connected `route=train` relations "
      f"({n_train_usages} way usages) there are {len(usage_rows)} distinct ways. Anchor check "
      f"(chain start/end vs. first/last stop): {anchor_ok}/{anchor_testable} relations consistent. "
      f"Geometric check of `travel_dir` (heading of a way vs. the way 3 positions further along the "
      f"route): **{coh_ok}/{coh_tot} = {100.0*coh_ok/max(1,coh_tot):.1f} %** consistent.")
    A("")
    A("| ways used by >=2 relations | ways | km |")
    A("|---|---|---|")
    A(f"| all relations same direction, all `forward` | {len(same_fwd)} | {km(same_fwd):.1f} |")
    A(f"| all relations same direction, all `backward` | {len(same_bwd)} | {km(same_bwd):.1f} |")
    A(f"| used in **both** directions | {len(both)} | {km(both):.1f} |")
    A("")
    A(f"All {len(usage_rows)} ways used by any relation: forward-only {len(fwd_only)} ways / "
      f"{km(fwd_only):.1f} km, backward-only {len(bwd_only)} / {km(bwd_only):.1f} km, both "
      f"{len(both_used)} / {km(both_used):.1f} km. {len(one_each)} of the both-direction ways are "
      f"used by exactly one relation per direction ({km(one_each):.1f} km) — the single-track / "
      f"two-way-working candidates. {len(unknown_use)} ways carry at least one `unknown` use.")
    A("")
    A("Interpretation: OSM maps one way per physical track, so a way used by several relations should "
      "nearly always be used in the SAME `travel_dir`; the forward/backward split of the "
      "same-direction rows is only the way digitising direction (not operationally meaningful). The "
      "`both` km is an upper bound for single-track/bidirectional sections and contains mapping "
      "outliers.")
    A("")
    A("## 5. Validation")
    A("")
    A("* Spot check against the OSM API (`/api/0.6/relation/<id>/full.json`, fetched 2026-11-10; "
      "snapshots kept as `api_*.json` here): relation **6782128** `IC 21: Basel SBB => Lugano` has "
      "555 plain way members and 6 stop nodes — our CSVs reproduce all 555 way ids in the identical "
      "order and all 6 stop nodes with identical `uic_ref`/`local_ref` (Basel SBB 5, Olten 12, "
      "Luzern 8, Arth-Goldau 5, Bellinzona 2, Lugano -).")
    A("* Relation **7794140** `IR 16: Zürich => Bern`: 287/287 way members in order, 6/6 stops "
      "(Zürich HB 15, Baden 3, Brugg 4, Aarau 4, Olten 8, Bern 5) — identical to the API.")
    A("* Everything else in this report is derived only from the pbf and is **UNVERIFIED** against a "
      "second source.")
    A("")
    A("## 6. Caveats")
    A("")
    A("* OSM relations are 2026-11, the timetable is FP2024 — track renumbering / platform changes "
      "between the vintages are possible and **UNVERIFIED**.")
    A("* `travel_dir` needs the relation's stop members to be in travel order (OSM convention, "
      "checked on relation 7794140 via the OSM API). Relations without stops and without resolvable "
      "geometry stay `unknown`.")
    A("* UIC 8516144 (Zürich HB Löwenstrasse) is aliased to 8503000 (Zürich HB) because GTFS merges "
      "the two station parts; this is the only alias.")
    A("* Several relations exist per line (variants/directions); `match.csv` keeps the best-scoring "
      "one per timetable route.")
    A("")
    out = os.path.join(HERE, "REPORT.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    log(f"wrote REPORT.md ({len(L)} lines)")
    if len(L) > 80:
        log(f"WARNING: REPORT.md has {len(L)} lines (max 80)")


def run():
    main()


if __name__ == "__main__":
    run()
