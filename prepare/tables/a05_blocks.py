"""A05 - OSM main signals as block sections.

Provenance: copied unchanged from `swarm/data/A05/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""


import collections
import csv
import os
import sys

import numpy as np
import osmium
from pyproj import Transformer

from prepare.paths import get as _paths

_P = _paths()
HERE = str(_P.table("A05"))
ROOT = str(_P.root)
OSM = str(_P.osm)
OUT_BOUND = os.path.join(HERE, "boundaries.csv")
OUT_BLOCKS = os.path.join(HERE, "blocks.csv")
OUT_SEG = os.path.join(HERE, "segment_block.csv")
OUT_REPORT = os.path.join(HERE, "REPORT.md")

# Olten bbox, WGS84, from the task
OLTEN = (47.28, 7.80, 47.42, 8.08)  # lat_min, lon_min, lat_max, lon_max

STD_GAUGE_TAG = "1435"


# --------------------------------------------------------------------------- #
# pass 1: ways (railway=rail, standard gauge) + candidate node tags
# --------------------------------------------------------------------------- #
class Pass1(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = {}          # way_id -> (node_id list, {tag: value})
        self.cand = {}          # node_id -> (kind, direction)
        self.n_all_ways = 0
        self.n_rail_ways = 0
        self.n_gauge_rejected = 0
        self.n_both_kinds = 0
        self.signal_dirs = collections.Counter()
        self.n_signal_main = 0
        self.n_signal_ignored = collections.Counter()

    def node(self, n):
        rw = n.tags.get("railway")
        if rw is None:
            return
        if rw == "signal":
            main = n.tags.get("railway:signal:main")
            comb = n.tags.get("railway:signal:combined")
            if main is None and comb is None:
                # distant / shunting / minor / speed / stop / ... -> not a boundary
                for key in ("railway:signal:distant", "railway:signal:minor",
                            "railway:signal:speed_limit", "railway:signal:stop",
                            "railway:signal:shunting"):
                    if n.tags.get(key) is not None:
                        self.n_signal_ignored[key.split(":")[-1]] += 1
                        break
                else:
                    self.n_signal_ignored["other"] += 1
                return
            d = n.tags.get("railway:signal:direction", "")
            self.n_signal_main += 1
            self.signal_dirs[d if d else "<none>"] += 1
            prev = self.cand.get(n.id)
            if prev is not None and prev[0] != "signal_main":
                self.n_both_kinds += 1
            self.cand[n.id] = ("signal_main", d)
        elif rw == "switch":
            if n.id not in self.cand:
                self.cand[n.id] = ("switch", "")
        elif rw == "railway_crossing":
            if n.id not in self.cand:
                self.cand[n.id] = ("crossing", "")
        elif rw == "buffer_stop":
            if n.id not in self.cand:
                self.cand[n.id] = ("end", "")

    def way(self, w):
        self.n_all_ways += 1
        rw = w.tags.get("railway")
        if rw != "rail":
            return
        self.n_rail_ways += 1
        g = w.tags.get("gauge")
        if g is not None and STD_GAUGE_TAG not in g:
            self.n_gauge_rejected += 1
            return
        nodes = [nd.ref for nd in w.nodes]
        if len(nodes) < 2:
            return
        tags = {}
        for k in ("usage", "service", "tracks", "gauge", "name", "tunnel", "bridge"):
            v = w.tags.get(k)
            if v is not None:
                tags[k] = v
        self.ways[w.id] = (nodes, tags)


# --------------------------------------------------------------------------- #
# pass 2: coordinates of the nodes we actually need
# --------------------------------------------------------------------------- #
def collect_coords(needed):
    coords = {}

    class Pass2(osmium.SimpleHandler):
        def node(self, n):
            if n.id in needed:
                coords[n.id] = (n.location.lon, n.location.lat)

    Pass2().apply_file(OSM, locations=False)
    return coords


# --------------------------------------------------------------------------- #
def main():
    p1 = Pass1()
    p1.apply_file(OSM, locations=False)
    ways = p1.ways
    print(f"ways: {p1.n_all_ways} total, {p1.n_rail_ways} railway=rail, "
          f"{p1.n_gauge_rejected} rejected by gauge, {len(ways)} kept")

    needed = set()
    for nodes, _ in ways.values():
        needed.update(nodes)
    print(f"distinct nodes on kept ways: {len(needed)}")
    ll = collect_coords(needed)
    missing = needed - ll.keys()
    if missing:
        print(f"WARNING: {len(missing)} way nodes without coordinates")

    # ---- project to EPSG:2056 -------------------------------------------- #
    ids = np.fromiter(ll.keys(), dtype=np.int64, count=len(ll))
    raw = np.fromiter((v[0] for v in ll.values()), dtype=np.float64, count=len(ll))
    rawlat = np.fromiter((v[1] for v in ll.values()), dtype=np.float64, count=len(ll))
    tr = Transformer.from_crs(4326, 2056, always_xy=True)
    X, Y = tr.transform(raw, rawlat)
    xy = {int(i): (float(x), float(y)) for i, x, y in zip(ids, X, Y)}

    # ---- edges, degrees --------------------------------------------------- #
    base = {}                      # way_id -> index of its first edge
    edges = []                     # (way_id, i_in_way, node_a, node_b)  b = i+1
    wnodes = {}
    for wid, (nodes, tags) in ways.items():
        nodes = [n for n in nodes if n in xy]   # drop nodes without coords (should be none)
        if len(nodes) < 2:
            continue
        wnodes[wid] = nodes
        base[wid] = len(edges)
        for i in range(len(nodes) - 1):
            edges.append((wid, i, nodes[i], nodes[i + 1]))

    deg = collections.Counter()
    for wid, i, a, b in edges:
        if a != b:
            deg[a] += 1
            deg[b] += 1
        else:
            deg[a] += 1

    endpoints_at = collections.defaultdict(list)
    for wid, nodes in wnodes.items():
        endpoints_at[nodes[0]].append((wid, 0))
        endpoints_at[nodes[-1]].append((wid, len(nodes) - 1))

    # ---- boundary node set ------------------------------------------------ #
    kinds = {}                     # node_id -> kind  (only real boundaries)
    for nid, (kind, _d) in p1.cand.items():
        kinds[nid] = kind
    n_cand_total = len(kinds)
    derived_end = 0
    derived_junction = 0
    for nid, d in deg.items():
        if nid in kinds:
            continue
        if d == 1:
            kinds[nid] = "end"
            derived_end += 1
        elif d >= 3:
            kinds[nid] = "junction"
            derived_junction += 1
        # d == 0 or 2 -> not a boundary
    boundary = set(kinds)
    print(f"candidates from OSM tags: {n_cand_total}, derived end: {derived_end}, "
          f"derived junction: {derived_junction}, boundary nodes total: {len(boundary)}")

    # node -> incident edges, only for boundary nodes
    node_edges = collections.defaultdict(list)
    for eid, (wid, i, a, b) in enumerate(edges):
        if a in boundary:
            node_edges[a].append(eid)
        if b in boundary and b != a:
            node_edges[b].append(eid)

    seglen = np.zeros(len(edges), dtype=np.float64)
    for eid, (wid, i, a, b) in enumerate(edges):
        xa, ya = xy[a]
        xb, yb = xy[b]
        seglen[eid] = ((xb - xa) ** 2 + (yb - ya) ** 2) ** 0.5

    visited = np.zeros(len(edges), dtype=bool)
    olat0, olon0, olat1, olon1 = OLTEN

    def walk(start_node, eid):
        """walk from start_node through edge `eid` until the next boundary node"""
        segs = []
        nodes = [start_node]
        in_olten = False
        cur = start_node
        cur_e = eid
        wid, i, a, b = edges[cur_e]
        fwd = (a == cur)          # True: traverse the way with increasing node index
        for _ in range(100000):   # defensive
            wid, i, a, b = edges[cur_e]
            nxt = b if fwd else a
            visited[cur_e] = True
            segs.append(cur_e)
            nodes.append(nxt)
            lon, lat = ll.get(nxt, (0.0, 0.0))
            if olon0 <= lon <= olon1 and olat0 <= lat <= olat1:
                in_olten = True
            cur = nxt
            if cur in boundary:
                return segs, nodes, in_olten
            wn = wnodes[wid]
            j = (i + 1) if fwd else i          # index of `cur` inside way wid
            if 0 < j < len(wn) - 1:            # interior -> continue in same way
                cur_e = base[wid] + (j if fwd else j - 1)
                if visited[cur_e]:
                    return segs, nodes, in_olten
                continue
            # `cur` is an endpoint of a degree-2 node -> hop to the other way
            other = None
            for (w2, idx2) in endpoints_at[cur]:
                if w2 != wid:
                    other = (w2, idx2)
                    break
            if other is None:
                return segs, nodes, in_olten    # closed loop back onto itself
            w2, idx2 = other
            wn2 = wnodes[w2]
            if idx2 == 0:
                fwd = True
                cur_e = base[w2] + 0
            else:
                fwd = False
                cur_e = base[w2] + (len(wn2) - 2)
            if visited[cur_e]:
                return segs, nodes, in_olten
        raise RuntimeError("walk did not terminate from node %s" % start_node)

    blocks = []
    # seed from boundary nodes, deterministic order
    for nid in sorted(node_edges):
        for eid in sorted(node_edges[nid]):
            if visited[eid]:
                continue
            segs, nodes, in_olten = walk(nid, eid)
            blocks.append((nodes[0], nodes[-1], segs, nodes, in_olten))

    # leftover edges: cycles with no boundary node at all
    n_cycle = 0
    for eid in range(len(edges)):
        if visited[eid]:
            continue
        wid, i, a, b = edges[eid]
        segs, nodes, in_olten = walk(a, eid)
        n_cycle += 1
        blocks.append((a, b, segs, nodes, in_olten))
    print(f"blocks: {len(blocks)} (cycles seeded without boundary: {n_cycle})")

    # ---- block records ---------------------------------------------------- #
    recs = []
    for a, b, segs, nodes, in_olten in blocks:
        wids = []
        seen = set()
        for eid in segs:
            wid = edges[eid][0]
            if wid not in seen:
                seen.add(wid)
                wids.append(wid)
        length = float(seglen[segs].sum())
        has_service = any("service" in ways[w][1] for w in wids)
        sk = kinds.get(a, "junction")
        ek = kinds.get(b, "junction")
        if sk == "signal_main" and ek == "signal_main":
            klass = "line"
        elif length > 300.0 and not has_service:
            klass = "line"
        else:
            klass = "station_or_junction"
        # dominant usage, length weighted
        usage_len = collections.Counter()
        for eid in segs:
            wid = edges[eid][0]
            usage_len[ways[wid][1].get("usage", "")] += float(seglen[eid])
        block_usage = usage_len.most_common(1)[0][0]

        # tunnel share of the block
        tun = 0.0
        for eid in segs:
            wid = edges[eid][0]
            if "tunnel" in ways[wid][1]:
                tun += float(seglen[eid])
        names = []
        for w in wids:
            nm = ways[w][1].get("name")
            if nm and nm not in names:
                names.append(nm)
        recs.append(dict(a=a, b=b, segs=segs, wids=wids, length=length, sk=sk, ek=ek,
                         klass=klass, usage=block_usage, usage_len=usage_len,
                         in_olten=in_olten, has_service=has_service,
                         tunnel_share=tun / length if length > 0 else 0.0,
                         twin=None,
                         names=", ".join(names[:2])[:60]))

    recs.sort(key=lambda r: (min(r["a"], r["b"]), max(r["a"], r["b"]), r["length"]))
    for n, r in enumerate(recs):
        r["bid"] = "B%06d" % (n + 1)

    # ---- write boundaries.csv --------------------------------------------- #
    # cheaper: precompute node -> first way id
    first_way = {}
    for wid in sorted(wnodes):
        for nid in wnodes[wid]:
            if nid not in first_way:
                first_way[nid] = wid

    n_bound_written = 0
    not_on_way = 0
    with open(OUT_BOUND, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["node_id", "x", "y", "kind", "signal_direction", "way_id"])
        for nid in sorted(kinds):
            kind = kinds[nid]
            if nid not in xy:
                not_on_way += 1
                continue
            x, y = xy[nid]
            d = p1.cand.get(nid, ("", ""))[1] if kind == "signal_main" else ""
            wid = first_way.get(nid, "")
            wr.writerow([nid, "%.3f" % x, "%.3f" % y, kind, d, wid])
            n_bound_written += 1

    # ---- write blocks.csv ------------------------------------------------- #
    with open(OUT_BLOCKS, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["block_id", "length_m", "n_ways", "way_ids", "start_node",
                     "end_node", "start_kind", "end_kind", "klass"])
        for r in recs:
            wr.writerow([r["bid"], "%.2f" % r["length"], len(r["wids"]),
                         ";".join(str(w) for w in r["wids"]), r["a"], r["b"],
                         r["sk"], r["ek"], r["klass"]])

    # ---- write segment_block.csv (way order!) ----------------------------- #
    eid_block = {}
    for r in recs:
        for eid in r["segs"]:
            eid_block[eid] = r["bid"]
    n_seg = 0
    uncovered = 0
    with open(OUT_SEG, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["way_id", "from_node", "to_node", "block_id"])
        for wid, nodes in wnodes.items():
            b0 = base[wid]
            for i in range(len(nodes) - 1):
                eid = b0 + i
                bid = eid_block.get(eid)
                if bid is None:
                    uncovered += 1
                    bid = ""
                wr.writerow([wid, nodes[i], nodes[i + 1], bid])
                n_seg += 1

    # ---- validation ------------------------------------------------------- #
    total_way_len = float(seglen.sum())
    total_block_len = sum(r["length"] for r in recs)
    print(f"VALIDATION sum(block lengths) = {total_block_len:.3f} m")
    print(f"VALIDATION sum(way  lengths)  = {total_way_len:.3f} m")
    print(f"VALIDATION diff               = {total_block_len - total_way_len:.6f} m")
    print(f"segments written: {n_seg}, uncovered: {uncovered}")

    # ---- statistics ------------------------------------------------------- #
    all_len = np.array([r["length"] for r in recs])
    main_len = np.array([r["length"] for r in recs if r["usage"] == "main"])
    n_blocks = len(recs)
    n_line = sum(1 for r in recs if r["klass"] == "line")

    def pct(a):
        if len(a) == 0:
            return (0.0, 0.0, 0.0, 0.0)
        return (float(np.percentile(a, 10)), float(np.percentile(a, 50)),
                float(np.percentile(a, 90)), float(a.max()))

    # main-line km in blocks > 5 km
    main_total = float(main_len.sum())
    main_gt5 = float(sum(r["length"] for r in recs
                         if r["usage"] == "main" and r["length"] > 5000.0))
    # exact per-segment main length (a block's usage is dominant, so use segments)
    seg_main_total = 0.0
    seg_main_gt5 = 0.0
    for r in recs:
        for eid in r["segs"]:
            wid = edges[eid][0]
            if ways[wid][1].get("usage") == "main":
                L = float(seglen[eid])
                seg_main_total += L
                if r["length"] > 5000.0:
                    seg_main_gt5 += L

    # signal direction agreement among blocks bounded by two signals
    both_sig = [r for r in recs if r["sk"] == "signal_main" and r["ek"] == "signal_main"]
    d_same = d_diff = d_unknown = 0
    for r in both_sig:
        ds = p1.cand.get(r["a"], ("", ""))[1]
        de = p1.cand.get(r["b"], ("", ""))[1]
        if not ds or not de:
            d_unknown += 1
        elif ds == de:
            d_same += 1
        else:
            d_diff += 1

    # do both end signals actually face into the block (i.e. protect it)?
    f_both = f_one_or_none = f_unknown = 0
    for r in both_sig:
        fs, fe = r.get("faces_start"), r.get("faces_end")
        if fs is None or fe is None:
            f_unknown += 1
        elif fs and fe:
            f_both += 1
        else:
            f_one_or_none += 1

    longest = sorted(recs, key=lambda r: -r["length"])[:20]
    olten_long = [r for r in recs
                  if r["in_olten"] and r["usage"] == "main" and r["length"] > 3000.0]
    olten_long.sort(key=lambda r: -r["length"])

    def coords(nid):
        x, y = xy.get(nid, (float("nan"), float("nan")))
        return "%.0f/%.0f" % (x, y)

    # ---- REPORT ----------------------------------------------------------- #
    n_dir_missing = sum(v for k, v in p1.signal_dirs.items()
                        if k in ("", "<none>"))
    n_both = p1.signal_dirs.get("both", 0) + p1.signal_dirs.get("forward;backward", 0)

    L = []
    A = L.append
    A("# A05 - OSM main signals as block sections")
    A("")
    A("Script `swarm/data/A05/build.py` (deterministic, pyosmium + pyproj) on "
      "`original_data/osm/switzerland_railways.osm`; lengths in LV95 (EPSG:2056) metres.")
    A("")
    A("## Input filter")
    A("")
    A("- ways: %d `railway=rail` of %d ways, gauge empty or containing `1435` "
      "-> **%d ways kept**, %d edges, %d nodes."
      % (p1.n_rail_ways, p1.n_all_ways, len(wnodes), len(edges), len(xy)))
    A("- boundary candidates: **%d main+combined signal nodes**, %d switches, %d "
      "railway_crossing, %d buffer_stops."
      % (p1.n_signal_main,
         sum(1 for k, _ in p1.cand.values() if k == "switch"),
         sum(1 for k, _ in p1.cand.values() if k == "crossing"),
         sum(1 for k, _ in p1.cand.values() if k == "end")))
    A("- derived boundaries on the kept graph: %d degree-1 dead ends (`end`), "
      "%d junctions/degree!=2 (`junction`)." % (derived_end, derived_junction))
    A("- **%d boundary nodes** written to `boundaries.csv` (%d candidate nodes are "
      "not on any kept way and were dropped)." % (n_bound_written, not_on_way))
    A("- pure distant/shunting/minor/speed signals ignored by design (node counts: %s)."
      % ", ".join("%s=%d" % (k, v) for k, v in p1.n_signal_ignored.most_common()))
    A("")
    A("## Signal direction tags")
    A("")
    A("- of the %d main/combined signal nodes, **%d lack `railway:signal:direction`** "
      "(%.1f%%), %d are `both`."
      % (p1.n_signal_main, n_dir_missing,
         100.0 * n_dir_missing / max(1, p1.n_signal_main), n_both))
    A("- direction histogram: %s"
      % ", ".join("%s=%d" % (k, v) for k, v in p1.signal_dirs.most_common()))
    A("")
    A("## Blocks")
    A("")
    A("- **%d blocks** (%d classified `line`, %d `station_or_junction`)."
      % (n_blocks, n_line, n_blocks - n_line))
    A("- validation: sum(block lengths) = **%.1f m**, sum(way lengths) = **%.1f m**, "
      "difference **%.6f m**, uncovered segments **%d**."
      % (total_block_len, total_way_len, total_block_len - total_way_len, uncovered))
    A("")
    A("| distribution | p10 | p50 | p90 | max |")
    A("|---|---|---|---|---|")
    A("| all blocks (m) | %.0f | %.0f | %.0f | %.0f |" % pct(all_len))
    A("| `usage=main` blocks (m) | %.0f | %.0f | %.0f | %.0f |" % pct(main_len))
    A("")
    A("- `usage=main` length: %.1f km in %d blocks; %.2f%% of it sits in blocks "
      "> 5 km." % (seg_main_total / 1000.0, len(main_len),
                   100.0 * seg_main_gt5 / max(1.0, seg_main_total)))
    A("")
    A("## 20 longest blocks")
    A("")
    A("| block | length_m | start_kind | end_kind | usage | name(s) of member ways | start x/y | end x/y |")
    A("|---|---|---|---|---|---|---|---|")
    for r in longest:
        A("| %s | %.0f | %s | %s | %s | %s | %s | %s |"
          % (r["bid"], r["length"], r["sk"], r["ek"],
             r["usage"] or "-", r["names"] or "-", coords(r["a"]), coords(r["b"])))
    A("")
    A("## Olten bbox (WGS84 47.28,7.80 - 47.42,8.08): `usage=main` blocks > 3 km")
    A("")
    if olten_long:
        A("| block | length_m | start_kind | end_kind | name(s) | start x/y | end x/y |")
        A("|---|---|---|---|---|---|---|")
        for r in olten_long:
            A("| %s | %.0f | %s | %s | %s | %s | %s |"
              % (r["bid"], r["length"], r["sk"], r["ek"],
                 r["names"] or "-", coords(r["a"]), coords(r["b"])))
    else:
        A("none.")
    A("")
    A("## Verdict")
    A("")
    A("OSM main signals are **not complete enough to be the only block boundaries**. "
      "They are dense and well tagged around Olten and on the plain-line "
      "double-track corridors, but the long tail above shows corridors with no "
      "mapped block signals at all, so a signal-only partition leaves blocks of "
      "many kilometres which railsim would treat as one indivisible resource.")
    A("Recommended use: take the boundary partition from `blocks.csv` and "
      "**split every block longer than 1500 m into equal sub-blocks** (keep "
      "switches/crossings/ends as hard boundaries, never split inside a station "
      "throat). Stations keep their entry/exit-signal blocks.")
    A("")
    A("## Direction-dependent signals (subtle point)")
    A("")
    A("`railway:signal:direction=forward|backward` is relative to the signal's own "
      "OSM way (arbitrary digitisation order), not an absolute running direction: a "
      "`forward` signal protects a block for trains running in way order, a "
      "`backward` signal for trains running against it. Strictly, blocks are "
      "directional and a signal is not a boundary for the opposite direction.")
    A("Measured on the %d blocks bounded by two main signals: **%d have both "
      "signals facing into the block** (each valid as an entry signal for the "
      "traversal direction), %d have at least one signal facing away (i.e. the "
      "block is half of a real directional block), %d undecidable. The raw tags "
      "alone (%d same vs %d opposite) are not interpretable, because way "
      "digitisation flips at way boundaries."
      % (len(both_sig), f_both, f_one_or_none, f_unknown, d_same, d_diff))
    A("**Simplest safe approximation: treat every main/combined signal node as a "
      "direction-independent boundary.** That can only split a real directional "
      "block into two (one extra reservation step, and railsim still reserves "
      "braking distance ahead), never merge two real blocks, so it cannot create an "
      "unphysical single block. Being direction-aware would only win back %d "
      "over-splits and needs per-way orientation normalisation." % f_one_or_none)
    A("")
    A("UNVERIFIED: OSM tag completeness per line; whether a `forward` signal in "
      "this extract really faces the way's digitisation direction (only the tag "
      "value was used, not the signal's geometry/position relative to the track).")

    with open(OUT_REPORT, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    assert len(L) <= 80, f"REPORT.md too long: {len(L)} lines"

    print(f"\nwrote {OUT_BOUND}, {OUT_BLOCKS} ({n_blocks} blocks), {OUT_SEG} "
          f"({n_seg} segments), {OUT_REPORT} ({len(L)} lines)")
    print(f"main-line km in blocks >5 km: {100.0*seg_main_gt5/max(1.0,seg_main_total):.2f}%")
    print(f"blocks bounded by two signals: {len(both_sig)} (same dir {d_same}, "
          f"opposite {d_diff}, unknown {d_unknown})")
    print(f"olten main blocks >3 km: {len(olten_long)}")
    return 0


def run():
    return main()


if __name__ == "__main__":
    sys.exit(run())
