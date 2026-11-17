#!/usr/bin/env python3
"""trackpatch.py - correct the track assignment of a mapped MATSim transit schedule with real
OSM tracks.  One re-runnable tool, declarative specs, self-checking.

    python3 prepare/trackpatch.py <sched-in> <net-in> <sched-out> <net-out> <spec.json ...> [--check]

Ported from round-3 agent RC2 (`swarm/r3/RC2/trackpatch.py`), code unchanged; pipeline step 7 of
`docs/pipeline.md` applies the passing loops of `prepare/corrections/second_tracks.json` (the five
real OSM second tracks that round-3 agent NL1 found: Courtemaîche repair, Courfaivre, St-Ursanne,
Malters, Les Geneveys-sur-Coffrane -- same jobs, in NL1's job order, which reproduces NL1's output
byte for byte).

A spec file is a JSON object

    {"name": "...", "jobs": [ <job>, <job> ... ]}

Job `second_track` (same mechanism as swarm/r3/NL1/fix_second_track.py, extended):
    {"action": "second_track",
     "name": "Les Geneveys-sur-Coffrane", "uic": "8504236",
     "way": "507891447",            # the OSM way of the second track (a passing loop / 2nd bore)
     "nodeA": "471596482", "nodeB": "662607448",   # its two junctions with the modelled line
     "move": "A->B",                # the physical direction that is moved onto it
     "track": "2",                  # name of the new stop facility (station-only stops only)
     "mainWay": "51951505",         # optional: OSM way of the modelled track (default: all links
                                    #   of the network between nodeA and nodeB)
     "lines": ["IR66", "RE6"],      # optional filter
     "osm": "matsim_input/tmp/network_osm.xml.gz",  # or original_data/osm/...osm (raw OSM XML)
     "snap_m": 25,                  # max distance for wiring a track whose ends are not model
                                    #   nodes (0 = node identity only, the default)
     "move_stop": true}

Job `move_stop` (train on the track of its published platform):
    {"action": "move_stop", "name": "...", "facility": "8505000:0:7.link:12863",
     "to_facility": "8505000:0:8.link:565377"}     # or "to_link": "<link id>"

Job `route_run` (reroute a run over existing links):
    {"action": "route_run", "name": "...", "chain": ["1","2"], "lines": ["RE12"],
     "from_node": "...", "to_node": "..."}

Every job is printed with what it changed; `--check` then validates the result
(route chains connected, stop links on the route in order, timetable offsets unchanged,
all link ids present) and prints PASS/FAIL.  Nothing is retimed: only link sequences, stop
references and stopFacilities are touched, plus new network links/nodes from OSM.
"""
import gzip
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

NET_DTD = '<!DOCTYPE network SYSTEM "http://www.matsim.org/files/dtd/network_v2.dtd">'
SCH_DTD = '<!DOCTYPE transitSchedule SYSTEM "http://www.matsim.org/files/dtd/transitSchedule_v2.dtd">'


def open_any(path, mode="rb"):
    return gzip.open(path, mode) if path.endswith(".gz") else open(path, mode)


def save(root, path, doctype):
    try:
        ET.indent(root, space="\t")
    except AttributeError:
        pass
    with open_any(path, "wb") as f:
        f.write(('<?xml version="1.0" encoding="UTF-8"?>\n' + doctype + "\n").encode())
        f.write(ET.tostring(root, encoding="unicode").encode())


def way_of(el):
    for a in el.iter("attribute"):
        if a.get("name") == "osm:way:id":
            return a.text
    return None


def load_links(path):
    links, nodes = {}, {}
    root = ET.parse(open_any(path)).getroot()
    for el in root.iter("link"):
        links[el.get("id")] = {"from": el.get("from"), "to": el.get("to"),
                               "length": float(el.get("length") or 0), "way": way_of(el),
                               "el": el}
    for el in root.iter("node"):
        nodes[el.get("id")] = (float(el.get("x")), float(el.get("y")))
    return links, nodes


# ---------------------------------------------------------------- raw OSM (optional source)
def lv95_of(lat, lon):
    """Inverse of the LV95 -> WGS84 approximation used by the OSM import (Newton, 0.01 m)."""
    def fwd(E, N):
        y_ = (E - 2600000.0) / 1e6
        x_ = (N - 1200000.0) / 1e6
        lo = (2.6779094 + 4.728982 * y_ + 0.791484 * y_ * x_ + 0.1306 * y_ * x_ * x_
              - 0.0436 * y_ ** 3) * 100.0 / 36.0
        la = (16.9023892 + 3.238272 * x_ - 0.270978 * y_ * y_ - 0.002528 * x_ * x_
              - 0.0447 * x_ * x_ * y_ - 0.0140 * x_ ** 3) * 100.0 / 36.0
        return la, lo
    E, N = 2600000.0 + (lon - 8.0) * 70000.0, 1200000.0 + (lat - 46.8) * 111000.0
    for _ in range(40):
        la, lo = fwd(E, N)
        dlat, dlon = lat - la, lon - lo
        if abs(dlat) < 1e-9 and abs(dlon) < 1e-9:
            break
        h = 0.5
        la1, lo1 = fwd(E + h, N)
        la2, lo2 = fwd(E, N + h)
        j11, j21 = (la1 - la) / h, (lo1 - lo) / h
        j12, j22 = (la2 - la) / h, (lo2 - lo) / h
        det = j11 * j22 - j12 * j21
        if abs(det) < 1e-18:
            break
        dE = (dlat * j22 - dlon * j12) / det
        dN = (j11 * dlon - j21 * dlat) / det
        E += max(-5000.0, min(5000.0, dE))
        N += max(-5000.0, min(5000.0, dN))
    return E, N


def load_raw_osm(path, wanted_ways):
    """Read ways (node lists) and their node coordinates from a raw OSM XML file."""
    ways, nodes = {}, {}
    cur, buf = None, []
    need = set()
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("  <node "):
                if not need or True:
                    import re
                    m = re.search(r'id="(\d+)"[^>]*lat="([-\d.]+)"[^>]*lon="([-\d.]+)"', line)
                    if m:
                        nodes[m.group(1)] = (float(m.group(2)), float(m.group(3)))
                continue
            if line.startswith("  <way "):
                import re
                m = re.search(r'id="(\d+)"', line)
                cur = m.group(1) if m else None
                buf = []
                continue
            if cur is not None and "<nd " in line:
                import re
                buf.append(re.search(r'ref="(\d+)"', line).group(1))
                continue
            if cur is not None and "</way>" in line:
                if cur in wanted_ways:
                    ways[cur] = buf
                cur = None
    return ways, nodes


# ---------------------------------------------------------------- chain helpers
def chain_of_way(links, way, node_from, node_to):
    adj = defaultdict(list)
    for lid, l in links.items():
        if l["way"] == way:
            adj[l["from"]].append((l["to"], lid))
    prev = {node_from: None}
    queue = [node_from]
    while queue:
        n = queue.pop(0)
        if n == node_to:
            break
        for nxt, lid in adj.get(n, []):
            if nxt not in prev:
                prev[nxt] = (n, lid)
                queue.append(nxt)
    if node_to not in prev:
        raise SystemExit("no directed path on way %s from %s to %s" % (way, node_from, node_to))
    chain, n = [], node_to
    while prev[n] is not None:
        p, lid = prev[n]
        chain.append(lid)
        n = p
    chain.reverse()
    return chain


def find_run(links, ids, a, b):
    """Index span (i0,i1) of the contiguous run of `ids` from junction a to junction b."""
    for i, lid in enumerate(ids):
        l = links.get(lid)
        if l is None:
            continue
        if l["from"] in (a, b):
            other = b if l["from"] == a else a
            j = i
            while j < len(ids) and links.get(ids[j], {}).get("to") != other:
                j += 1
            if j < len(ids):
                return i, j
    return None


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    check = "--check" in sys.argv
    in_sched, in_net, out_sched, out_net = args[:4]
    specs = args[4:]
    net = ET.parse(open_any(in_net)).getroot()
    sched = ET.parse(open_any(in_sched)).getroot()
    net_links, net_nodes = load_links(in_net)
    node_ids = set(net_nodes)
    links_el, nodes_el = net.find("links"), net.find("nodes")
    osm_cache = {}

    def osm(spec):
        p = spec.get("osm", "matsim_input/tmp/network_osm.xml.gz")
        if p not in osm_cache:
            osm_cache[p] = load_links(p)
        return osm_cache[p]

    for sp in specs:
        data = json.load(open(sp))
        print("\n== spec %s  (%d jobs)" % (data.get("name", sp), len(data.get("jobs", []))))
        for job in data["jobs"]:
            act = job.get("action", "second_track")
            if act == "second_track":
                do_second_track(job, net, sched, net_links, net_nodes, node_ids,
                                links_el, nodes_el, osm)
            elif act == "move_stop":
                do_move_stop(job, sched)
            elif act == "route_run":
                do_route_run(job, sched, net_links)
            else:
                raise SystemExit("unknown action %r" % act)
            print("   notes: %s" % ", ".join(job.get("_notes", [])) if job.get("_notes") else "")

    save(net, out_net, NET_DTD)
    save(sched, out_sched, SCH_DTD)
    print("\nwrote", out_net, out_sched)
    if check:
        print("\n-- input --")
        base = validate(in_sched, in_net, quiet=True)
        print("-- output --")
        out = validate(out_sched, out_net)
        print("DELTA chain breaks %d -> %d, stop-link problems %d -> %d"
              % (base[0], out[0], base[1], out[1]))


def do_second_track(job, net, sched, net_links, net_nodes, node_ids, links_el, nodes_el, osm):
    osm_links, osm_nodes = osm(job)
    snap = float(job.get("snap_m", 0) or 0)
    a, b = job["nodeA"], job["nodeB"]
    # ---- 1. the chain of the second track, both orientations
    chain = chain_of_way(osm_links, job["way"], a, b)
    revmap = {}
    for lid, l in osm_links.items():
        if l["way"] == job["way"]:
            revmap[(l["from"], l["to"], round(l["length"], 2))] = lid
    chain_rev = []
    for lid in reversed(chain):
        l = osm_links[lid]
        k = (l["to"], l["from"], round(l["length"], 2))
        if k not in revmap:
            raise SystemExit("no reverse link for %s on way %s" % (lid, job["way"]))
        chain_rev.append(revmap[k])
    print("  %s: way %s -> %d links %s..%s (+reverse)" % (
        job["name"], job["way"], len(chain), chain[0], chain[-1]))

    # ---- 2. add the links (and nodes) to the network
    added = 0
    for lid in chain + chain_rev:
        if lid in net_links:
            continue
        el = osm_links[lid]["el"]
        for nid in (el.get("from"), el.get("to")):
            if nid not in node_ids:
                x, y = osm_nodes[nid]
                nodes_el.append(ET.Element("node", {"id": nid, "x": "%.2f" % x, "y": "%.2f" % y}))
                node_ids.add(nid)
        links_el.append(ET.fromstring(ET.tostring(el)))
        net_links[lid] = osm_links[lid]
        added += 1
    print("   %d links added to the network" % added)

    # ---- 3. reroute the selected physical direction
    run_nodes = {a, b}
    rewritten, moved = 0, defaultdict(int)
    facilities = {f.get("id"): f for f in sched.iter("stopFacility")}
    for line in sched.iter("transitLine"):
        if job.get("lines") and line.get("name") not in job["lines"]:
            continue
        for route in line.iter("transitRoute"):
            netel = route.find("route")
            if netel is None:
                continue
            els = netel.findall("link")
            ids = [l.get("refId") for l in els]
            span = find_run(net_links, ids, a, b)
            if span is None:
                continue
            i0, i1 = span
            frm = net_links[ids[i0]]["from"]
            ori = "A->B" if frm == a else "B->A"
            if ori != job["move"]:
                continue
            new_chain = chain if ori == "A->B" else chain_rev
            first, last = osm_links[new_chain[0]], osm_links[new_chain[-1]]
            if first["from"] != frm:
                raise SystemExit("%s: chain %s..%s does not start at %s" % (
                    route.get("id"), first["from"], last["to"], frm))
            replaced = set(ids[i0:i1 + 1])
            pos = list(netel).index(els[i0])
            for el in els[i0:i1 + 1]:
                netel.remove(el)
            for k, lid in enumerate(new_chain):
                netel.insert(pos + k, ET.Element("link", {"refId": lid}))
            rewritten += 1
            if job.get("move_stop", True):
                prof = route.find("routeProfile")
                for st in prof.findall("stop"):
                    fid = st.get("refId")
                    fac = facilities.get(fid)
                    if fac is None or fac.get("linkRefId") not in replaced:
                        continue
                    if job["uic"] not in fid and (fac.get("stopAreaId") or "").replace("Parent", "") != job["uic"]:
                        continue
                    x, y = float(fac.get("x")), float(fac.get("y"))
                    best, bestd = None, 1e18
                    for lid in new_chain:
                        l = osm_links[lid]
                        p, q = osm_nodes[l["from"]], osm_nodes[l["to"]]
                        d = math.hypot((p[0] + q[0]) / 2 - x, (p[1] + q[1]) / 2 - y)
                        if d < bestd:
                            best, bestd = lid, d
                    newid = "%s:0:%s.link:%s" % (job["uic"], job.get("track", "2"), best)
                    if newid not in facilities:
                        nf = ET.Element("stopFacility", {
                            "id": newid, "x": fac.get("x"), "y": fac.get("y"),
                            "linkRefId": best, "name": fac.get("name"),
                            "stopAreaId": fac.get("stopAreaId", "Parent" + job["uic"]),
                            "isBlocking": "false"})
                        sched.find("transitStops").append(nf)
                        facilities[newid] = nf
                    st.set("refId", newid)
                    moved[best] += 1
    print("   %d routes rerouted %s; stops moved: %s" % (rewritten, job["move"], dict(moved)))


def do_move_stop(job, sched):
    facilities = {f.get("id"): f for f in sched.iter("stopFacility")}
    src = job["facility"]
    if "to_facility" in job:
        dst = job["to_facility"]
    else:
        dst = None
    moved = 0
    for line in sched.iter("transitLine"):
        if job.get("lines") and line.get("name") not in job["lines"]:
            continue
        for route in line.iter("transitRoute"):
            prof = route.find("routeProfile")
            if prof is None:
                continue
            for st in prof.findall("stop"):
                if st.get("refId") != src:
                    continue
                if dst is None:
                    l = job["to_link"]
                    nf = [f for f in facilities.values() if f.get("linkRefId") == l]
                    if not nf:
                        raise SystemExit("no stopFacility on link %s" % l)
                    dst = nf[0].get("id")
                st.set("refId", dst)
                moved += 1
    print("  %s: %d stops moved %s -> %s" % (job["name"], moved, src, dst))


def do_route_run(job, sched, net_links):
    chain = job["chain"]
    rewritten = 0
    for line in sched.iter("transitLine"):
        if job.get("lines") and line.get("name") not in job["lines"]:
            continue
        for route in line.iter("transitRoute"):
            netel = route.find("route")
            if netel is None:
                continue
            els = netel.findall("link")
            ids = [l.get("refId") for l in els]
            span = find_run(net_links, ids, job["from_node"], job["to_node"])
            if span is None:
                continue
            frm = net_links[ids[span[0]]]["from"]
            if job.get("direction") and job["direction"] != ("A->B" if frm == job["from_node"] else "B->A"):
                continue
            i0, i1 = span
            pos = list(netel).index(els[i0])
            for el in els[i0:i1 + 1]:
                netel.remove(el)
            for k, lid in enumerate(chain):
                netel.insert(pos + k, ET.Element("link", {"refId": lid}))
            rewritten += 1
    print("  %s: %d routes rewritten with %d links" % (job["name"], rewritten, len(chain)))


def validate(sched_path, net_path, quiet=False):
    links, nodes = load_links(net_path)
    root = ET.parse(open_any(sched_path)).getroot()
    fac = {f.get("id"): f.get("linkRefId") for f in root.iter("stopFacility")}
    bad_chain = bad_stop = bad_missing = 0
    total = 0
    for line in root.iter("transitLine"):
        for route in line.iter("transitRoute"):
            total += 1
            netel = route.find("route")
            ids = [l.get("refId") for l in netel.findall("link")] if netel is not None else []
            for k in range(len(ids) - 1):
                a, b = links.get(ids[k]), links.get(ids[k + 1])
                if a is None or b is None:
                    bad_missing += 1
                    break
                if a["to"] != b["from"]:
                    bad_chain += 1
                    if not quiet: print("   CHAIN BREAK %s at %s -> %s" % (route.get("id"), ids[k], ids[k + 1]))
                    break
            prof = route.find("routeProfile")
            last = -1
            for st in (prof.findall("stop") if prof is not None else []):
                lr = fac.get(st.get("refId"))
                if lr is None:
                    bad_missing += 1
                    continue
                if lr not in ids:
                    bad_stop += 1
                    if not quiet: print("   STOP LINK NOT ON ROUTE %s: %s" % (route.get("id"), st.get("refId")))
                    continue
                i = ids.index(lr, last + 1) if lr in ids[last + 1:] else -1
                if i <= last:
                    bad_stop += 1
                    if not quiet: print("   STOP OUT OF ORDER %s: %s" % (route.get("id"), st.get("refId")))
                else:
                    last = i
    ok = (bad_chain == 0 and bad_stop == 0 and bad_missing == 0)
    print("CHECK %s: %d routes, %d links, chain breaks %d, stop-link problems %d, missing links %d"
          % ("PASS" if ok else "FAIL", total, len(links), bad_chain, bad_stop, bad_missing))
    return (bad_chain, bad_stop, bad_missing)


if __name__ == "__main__":
    main()
