"""A04 - legal movements through every track junction node.

Provenance: copied unchanged from `swarm/data/A04/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""


import collections
import csv
import math

import numpy as np
import osmium
from pyproj import Transformer

from prepare.paths import get as _paths

_P = _paths()
OSM_PATH = str(_P.osm)
OUT_DIR = str(_P.table("A04"))

TARGET_M = 20.0          # chord length used for the robust leg direction
THRESHOLD_DEG = 30.0     # calibrated: observed legitimate switch deflections stop at 23.0 deg
SLIP_TYPES = ("double_slip", "single_slip")


# ----------------------------------------------------------------------------- load + project
class Loader(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.node_ids = []
        self.lat = []
        self.lon = []
        self.node_railway = {}   # id -> railway tag value
        self.node_switch = {}    # id -> railway:switch tag value
        self.ways = []           # (way_id, [node ids])

    def node(self, n):
        self.node_ids.append(n.id)
        self.lat.append(n.location.lat)
        self.lon.append(n.location.lon)
        rt = n.tags.get("railway")
        if rt is not None:
            self.node_railway[n.id] = rt
        st = n.tags.get("railway:switch")
        if st is not None:
            self.node_switch[n.id] = st

    def way(self, w):
        if w.tags.get("railway") != "rail":
            return
        gauge = w.tags.get("gauge")
        if gauge and "1435" not in gauge:
            return
        refs = [nd.ref for nd in w.nodes]
        if len(refs) >= 2:
            self.ways.append((w.id, refs))


def load():
    h = Loader()
    h.apply_file(OSM_PATH, locations=True)
    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)
    x, y = tr.transform(np.asarray(h.lon), np.asarray(h.lat))
    xy = {nid: (float(x[i]), float(y[i])) for i, nid in enumerate(h.node_ids)}
    wgs = {nid: (float(h.lat[i]), float(h.lon[i])) for i, nid in enumerate(h.node_ids)}
    return xy, wgs, h.node_railway, h.node_switch, h.ways


# ----------------------------------------------------------------------------- graph
def build_graph(ways):
    """inc[node] = [(way_id, seg_index, other_node), ...]; two segments per interior way node."""
    inc = collections.defaultdict(list)
    for wid, refs in ways:
        for i in range(len(refs) - 1):
            a, b = refs[i], refs[i + 1]
            if a == b:
                continue
            inc[a].append((wid, i, b))
            inc[b].append((wid, i, a))
    return inc


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def leg_vector(node, seg, inc, xy, deg):
    """Unit vector pointing away from `node` along `seg`, measured at TARGET_M (or as far as the
    track allows), following the way through degree-2 continuation nodes."""
    wid, si, other = seg
    pts = [xy[node], xy[other]]
    cur_seg = (wid, si)
    cur = other
    total = dist(xy[node], xy[other])
    visited = {node, other}
    while total < TARGET_M and deg.get(cur, 0) == 2:
        others = [s for s in inc[cur] if (s[0], s[1]) != cur_seg]
        if len(others) != 1:
            break
        nxt = others[0]
        if nxt[2] in visited:
            break
        pts.append(xy[nxt[2]])
        total += dist(xy[cur], xy[nxt[2]])
        visited.add(nxt[2])
        cur_seg = (nxt[0], nxt[1])
        cur = nxt[2]

    acc = 0.0
    tip = pts[-1]
    for i in range(1, len(pts)):
        d = dist(pts[i - 1], pts[i])
        if d > 0 and acc + d >= TARGET_M:
            t = (TARGET_M - acc) / d
            tip = (pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
                   pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1]))
            break
        acc += d
    vx, vy = tip[0] - xy[node][0], tip[1] - xy[node][1]
    n = math.hypot(vx, vy)
    if n == 0:
        return None, 0.0, tip
    return (vx / n, vy / n), min(total, TARGET_M), tip


def deflection(u, v):
    d = max(-1.0, min(1.0, u[0] * v[0] + u[1] * v[1]))
    return abs(180.0 - math.degrees(math.acos(d)))


# ----------------------------------------------------------------------------- rule
def legal_flags(node_railway, switch_type, pairs, threshold=THRESHOLD_DEG):
    """pairs: list of dicts with 'defl'.  Returns 0/1 flags aligned with pairs."""
    if (node_railway == "railway_crossing" and switch_type not in SLIP_TYPES
            and len(pairs) == 6):
        return _matching_rule(pairs)
    return [1 if p["defl"] < threshold else 0 for p in pairs]


def _matching_rule(pairs):
    """4 legs -> 6 pairs.  Pick the perfect matching of the 4 legs with the smallest summed
    deflection: those two pairs are the straight-through movements of a plain crossing."""
    best = _matched_pairs(pairs)
    if best is None:
        return [1 if p["defl"] < THRESHOLD_DEG else 0 for p in pairs]
    return [1 if tuple(sorted((p["from_node"], p["to_node"]))) in best else 0 for p in pairs]


def _matched_pairs(pairs):
    legs = sorted({p["from_node"] for p in pairs} | {p["to_node"] for p in pairs})
    if len(legs) != 4:
        return None
    defl = {(p["from_node"], p["to_node"]): p["defl"] for p in pairs}
    a, b, c, d = legs

    def D(u, v):
        return defl[(min(u, v), max(u, v))]

    matchings = [
        (D(a, b) + D(c, d), [(a, b), (c, d)]),
        (D(a, c) + D(b, d), [(a, c), (b, d)]),
        (D(a, d) + D(b, c), [(a, d), (b, c)]),
    ]
    matchings.sort(key=lambda m: (m[0], m[1]))
    return {tuple(sorted(p)) for p in matchings[0][1]}


# ----------------------------------------------------------------------------- main
def main():
    xy, wgs, node_railway, node_switch, ways = load()
    inc = build_graph(ways)
    deg = {n: len(v) for n, v in inc.items()}

    # ---- all pairs of incident segments at nodes with degree >= 3
    nodes = {}
    degenerate = {}
    for node in sorted(n for n in inc if deg[n] >= 3):
        segs = inc[node]
        legs = [leg_vector(node, s, inc, xy, deg) for s in segs]
        pairs = []
        dropped = 0
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                if legs[i][0] is None or legs[j][0] is None:
                    continue
                if segs[i][2] == segs[j][2]:
                    # both legs end at the same neighbour node (two OSM ways duplicating the
                    # same track segment).  Both legs leave in the same direction, so the pair
                    # is a 180 deg reversal and illegal; it carries no information and would
                    # break "from_node < to_node", so it is dropped and only counted.
                    dropped += 1
                    continue
                f, t = sorted((segs[i][2], segs[j][2]))
                fw, tw = ((segs[i][0], segs[j][0]) if segs[i][2] <= segs[j][2]
                          else (segs[j][0], segs[i][0]))
                pairs.append({"node": node, "from_node": f, "to_node": t,
                              "from_way": fw, "to_way": tw,
                              "defl": deflection(legs[i][0], legs[j][0])})
        pairs.sort(key=lambda p: (p["from_node"], p["to_node"], p["from_way"], p["to_way"]))
        for p, ok in zip(pairs, legal_flags(node_railway.get(node), node_switch.get(node), pairs)):
            p["legal"] = ok
        nodes[node] = pairs
        if dropped:
            degenerate[node] = dropped

    # ---- write turns.csv
    with open(f"{OUT_DIR}/turns.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["node_id", "node_railway_tag", "switch_type", "degree", "from_node", "to_node",
                    "from_way", "to_way", "deflection_deg", "legal"])
        for node in sorted(nodes):
            for p in nodes[node]:
                w.writerow([node, node_railway.get(node, ""), node_switch.get(node, ""), deg[node],
                            p["from_node"], p["to_node"], p["from_way"], p["to_way"],
                            f"{p['defl']:.3f}", p["legal"]])

    # ---- write node_degree.csv
    with open(f"{OUT_DIR}/node_degree.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["node_id", "degree", "railway_tag"])
        for node in sorted(deg):
            if deg[node] != 2:
                w.writerow([node, deg[node], node_railway.get(node, "")])

    # ---- stats ------------------------------------------------------------------------------
    out = []
    P = out.append
    dhist = collections.Counter(deg.values())
    P(f"rail standard-gauge ways                 : {len(ways)}")
    P(f"nodes on those ways                      : {len(inc)}")
    P("degree histogram                         : "
      + ", ".join(f"{k}:{dhist[k]}" for k in sorted(dhist)))
    P(f"turn pairs written (nodes with degree>=3): {sum(len(v) for v in nodes.values())}"
      f" at {len(nodes)} nodes")
    P(f"dropped pairs (both legs to the same neighbour node, duplicated OSM ways / 180 deg "
      f"reversal, illegal by construction): {sum(degenerate.values())} at {len(degenerate)} nodes")
    P(f"TARGET_M (leg chord)                     : {TARGET_M:.0f} m")
    P(f"THRESHOLD_DEG                            : {THRESHOLD_DEG:.0f} deg")

    def legal_count(node, threshold=THRESHOLD_DEG):
        return sum(legal_flags(node_railway.get(node), node_switch.get(node), nodes[node], threshold))

    def hist(vals, label):
        bins = [0] * 37
        for v in vals:
            bins[min(int(v // 5), 36)] += 1
        P(f"\n{label}   (n={len(vals)})")
        top = max(bins) if bins else 1
        for i, c in enumerate(bins):
            if c:
                lab = "180.0 " if i == 36 else f"{i*5:3d}-{i*5+5:3d}"
                P(f"  {lab} : {c:6d} {'#' * max(1, min(76, round(76 * c / top)))}")

    def is_simple_switch(n):
        return (node_railway.get(n) == "switch" and deg[n] == 3
                and node_switch.get(n) in (None, "default"))

    def is_any_switch3(n):
        return node_railway.get(n) == "switch" and deg[n] == 3

    simple3 = [n for n in nodes if is_simple_switch(n)]
    hist([p["defl"] for n in simple3 for p in nodes[n]],
         "deflection histogram - railway=switch, degree 3, plain switches (type absent/default)")
    hist([p["defl"] for n in nodes if is_any_switch3(n) for p in nodes[n]],
         "same, ALL degree-3 railway=switch nodes (incl. slip / three_way / wye tags)")

    vals = sorted(p["defl"] for n in simple3 for p in nodes[n])
    near = [v for v in vals if abs(v - THRESHOLD_DEG) <= 10]
    P(f"\npairs within +-10 deg of {THRESHOLD_DEG:.0f} deg (ambiguous): {len(near)}  -> "
      + ", ".join(f"{v:.1f}" for v in near))
    gaps = [(vals[i], vals[i + 1]) for i in range(len(vals) - 1) if vals[i + 1] - vals[i] > 8]
    P("largest empty intervals in the sorted pair list: "
      + "; ".join(f"({a:.1f}, {b:.1f})" for a, b in sorted(gaps, key=lambda g: g[0] - g[1])[:4]))

    # threshold sensitivity
    P("\nthreshold sensitivity (same pairs, different threshold; the tag rule for crossings does "
      "not use the threshold):")
    P("  thr | legal pairs | d3 switch ==2 | crossing d4 ==2 | dslip d4 ==4")
    n_cross = sum(1 for n in nodes if node_railway.get(n) == "railway_crossing" and deg[n] == 4)
    n_ds4 = sum(1 for n in nodes if node_switch.get(n) == "double_slip" and deg[n] == 4)
    for t in (20., 25., 30., 35., 45., 60., 90., 120.):
        nlp = sum(legal_count(n, t) for n in nodes)
        s = sum(1 for n in simple3 if legal_count(n, t) == 2)
        x = sum(1 for n in nodes if node_railway.get(n) == "railway_crossing" and deg[n] == 4
                and legal_count(n, t) == 2)
        ds = sum(1 for n in nodes if node_switch.get(n) == "double_slip" and deg[n] == 4
                 and legal_count(n, t) == 4)
        P(f"  {t:3.0f} | {nlp:11d} | {s:5d}/{len(simple3):<5d} | {x:4d}/{n_cross:<4d}| "
          f"{ds:4d}/{n_ds4}")

    # consistency
    P("\nconsistency at the final rule:")
    x4 = [n for n in nodes if node_railway.get(n) == "railway_crossing" and deg[n] == 4
          and node_switch.get(n) not in SLIP_TYPES]
    x4s = [n for n in nodes if node_railway.get(n) == "railway_crossing" and deg[n] == 4
           and node_switch.get(n) in SLIP_TYPES]
    ds4 = [n for n in nodes if node_switch.get(n) == "double_slip" and deg[n] == 4]
    ds3 = [n for n in nodes if node_switch.get(n) == "double_slip" and deg[n] == 3]
    tw4 = [n for n in nodes if node_switch.get(n) == "three_way" and deg[n] == 4]
    for label, grp, want in (("degree-3 plain switch", simple3, 2),
                             ("degree-3 ANY switch", [n for n in nodes if is_any_switch3(n)], 2),
                             ("degree-4 railway_crossing, no slip tag", x4, 2),
                             ("degree-4 railway_crossing + double_slip", x4s, 4),
                             ("degree-4 double_slip", ds4, 4),
                             ("degree-4 three_way", tw4, 3)):
        ok = sum(1 for n in grp if legal_count(n) == want)
        P(f"  {label:38s}: {ok:5d}/{len(grp):<5d} = "
          f"{100.0*ok/max(1,len(grp)):6.2f}% have exactly {want} legal pairs")
    dist3 = collections.Counter(legal_count(n) for n in ds3)
    P(f"  degree-3 double_slip (only 3 legs mapped, {len(ds3)} nodes): legal-pair count "
      f"distribution {dict(sorted(dist3.items()))}")

    # what the pure angle rule would do at crossings (justifies the tag rule)
    cross_angle = [n for n in x4 if any(
        p["defl"] < THRESHOLD_DEG for p in nodes[n]
        if tuple(sorted((p["from_node"], p["to_node"]))) not in (_matched_pairs(nodes[n]) or set()))]
    if x4:
        small = sorted(p["defl"] for n in x4 for p in nodes[n]
                       if tuple(sorted((p["from_node"], p["to_node"]))) not in _matched_pairs(nodes[n]))
        P(f"  of the {len(x4)} plain crossings, at least one non-straight pair is below "
          f"{THRESHOLD_DEG:.0f} deg at {len(cross_angle)} nodes -> the pure angle rule would "
          f"over-permit there")
        if small:
            P(f"  deflection of the forbidden (crossing) movement at those nodes: n={len(small)}, "
              f"median={np.median(small):.1f} deg, min={min(small):.1f} deg")
        else:
            P("  no crossing pair below the threshold")

    bad = [n for n in simple3 if legal_count(n) != 2]
    P(f"  degree-3 plain switches with != 2 legal pairs: {len(bad)} " + ", ".join(map(str, bad)))
    for n in bad:
        P(f"    node {n} deg={deg[n]} tag={node_railway.get(n)}/{node_switch.get(n)} "
          f"defls=" + ", ".join(f"{p['defl']:.1f}" for p in nodes[n])
          + f"  wgs={wgs[n][0]:.5f},{wgs[n][1]:.5f}")

    # duplicate (node, from, to) keys: same movement twice because two OSM ways duplicate a track
    dup = collections.Counter((p["node"], p["from_node"], p["to_node"])
                              for n in nodes for p in nodes[n])
    P("\nduplicate (node_id, from_node, to_node) keys (same movement on two OSM ways that map the "
      f"same track): {sum(1 for v in dup.values() if v > 1)} keys / "
      f"{sum(v for v in dup.values() if v > 1)} rows; full key is "
      f"(node_id, from_way, to_way, from_node, to_node), which is unique")

    # switch coverage
    all_sw = sum(1 for n, v in node_railway.items() if v == "switch")
    on_ways = [n for n in node_railway if node_railway[n] == "switch" and n in deg]
    dh = collections.Counter(deg[n] for n in on_ways)
    P(f"\nrailway=switch nodes in the file: {all_sw}; on kept rail standard-gauge ways: "
      f"{len(on_ways)}; degree distribution of those: "
      + ", ".join(f"{k}:{v}" for k, v in sorted(dh.items())))
    P(f"  switch nodes with degree < 3 (no branch mapped as a separate way segment): "
      f"{sum(v for k, v in dh.items() if k < 3)}")

    # untagged junctions
    unt = [n for n in nodes if node_railway.get(n) not in ("switch", "railway_crossing")]
    P(f"\nnodes with degree>=3 and NO railway=switch / railway=railway_crossing tag: {len(unt)}")
    P("  degree distribution: " + ", ".join(
        f"{k}:{v}" for k, v in sorted(collections.Counter(deg[n] for n in unt).items())))
    P("  node tag distribution: " + str(collections.Counter(
        node_railway.get(n, "<none>") for n in unt).most_common()))
    dist = collections.Counter((legal_count(n), len(nodes[n])) for n in unt)
    P("  (legal pairs, possible pairs) -> nodes:")
    for (lg, poss), v in sorted(dist.items()):
        P(f"    {lg} of {poss}: {v}")

    # degree-2 kinks
    kinks = []
    for node in sorted(n for n in inc if deg[n] == 2):
        segs = inc[node]
        legs = [leg_vector(node, s, inc, xy, deg) for s in segs]
        if legs[0][0] is None or legs[1][0] is None:
            continue
        d = deflection(legs[0][0], legs[1][0])
        if d > THRESHOLD_DEG:
            kinks.append((node, d))
    kinks.sort(key=lambda k: (-k[1], k[0]))
    n2 = sum(1 for n in deg if deg[n] == 2)
    P(f"\ndegree-2 nodes with deflection > {THRESHOLD_DEG:.0f} deg: {len(kinks)} of {n2}")
    P("  worst 20: node, defl_deg, lat, lon, LV95 E, N")
    for node, d in kinks[:20]:
        P(f"    {node:>12} {d:6.1f}  {wgs[node][0]:.6f} {wgs[node][1]:.6f}  "
          f"{xy[node][0]:9.1f} {xy[node][1]:9.1f}")

    # ---- Olten validation
    P("\n--- Olten check (station 47.3519, 7.9077) ---")
    cx, cy = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True).transform(7.9077, 47.3519)
    cand = sorted((n for n in nodes if node_railway.get(n) == "switch"),
                  key=lambda n: math.hypot(xy[n][0] - cx, xy[n][1] - cy))
    P("3 nearest railway=switch nodes:")
    for n in cand[:3]:
        P(f"node {n}  deg={deg[n]}  railway:switch={node_switch.get(n, '<none>')}  "
          f"dist={(math.hypot(xy[n][0]-cx, xy[n][1]-cy)):.0f} m  "
          f"wgs={wgs[n][0]:.6f},{wgs[n][1]:.6f}")
        for s in inc[n]:
            v, ln, tip = leg_vector(n, s, inc, xy, deg)
            brg = math.degrees(math.atan2(v[0], v[1]))
            P(f"    leg -> {s[2]:>12} way {s[0]:>10}  bearing {brg:7.1f} deg  chord {ln:5.1f} m"
              f"  tip {tip[0]:.1f},{tip[1]:.1f}")
        for p in nodes[n]:
            P(f"    pair {p['from_node']:>12} <-> {p['to_node']:>12}: "
              f"deflection {p['defl']:6.1f} deg  legal={p['legal']}")

    text = "\n".join(out)
    with open(f"{OUT_DIR}/stats.txt", "w") as fh:
        fh.write(text + "\n")
    print(text)


def run():
    main()


if __name__ == "__main__":
    run()
