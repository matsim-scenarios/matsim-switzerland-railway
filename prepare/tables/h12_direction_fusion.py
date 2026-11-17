"""H12 - fuse OSM tags, route relations and geometry into one running-direction table.

Provenance: copied unchanged from `swarm/hotspots/H12-direction-fusion/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""

from __future__ import annotations

import csv
import gzip
import hashlib
import math
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

from prepare.paths import get as _paths

_P = _paths()
BASE = _P.root
HERE = _P.table("H12")
DATA = _P.out

FORWARD, BACKWARD, BOTH, UNKNOWN = "forward", "backward", "both", "unknown"

# ----------------------------------------------------------------------------- helpers
def read_csv(path):
    with open(path, newline="") as fh:
        yield from csv.DictReader(fh)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for r in rows:
            w.writerow(r)


def fnum(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def km(length_m):
    return length_m / 1000.0


def h2(*parts):
    return hashlib.md5("|".join(str(p) for p in parts).encode()).digest()


def is_dev(way_id):
    """Deterministic 50/50 split of the tagged reference set."""
    return h2("H12-split", way_id)[0] & 1 == 0


# ----------------------------------------------------------------------------- load ways
def run():
    """Compute the fused table from the A01/A02a/A04/A05/A06 tables (module body, tab-indented)."""
    WAYS = {}
    for r in read_csv(DATA / "A01" / "ways.csv"):
        WAYS[r["way_id"]] = r
    ROUT = sorted((w for w in WAYS.values() if w["routable"] == "1"), key=lambda w: int(w["way_id"]))
    ROUT_IDS = {w["way_id"] for w in ROUT}
    print(f"  routable ways: {len(ROUT)}  ({sum(km(fnum(w['length_m'])) for w in ROUT):.1f} km)")

    # ----------------------------------------------------------------------------- geometry
    print("loading A02a direction ...")
    GEOM = {}
    for r in read_csv(DATA / "A02a" / "direction.csv"):
        if r["way_id"] in ROUT_IDS:
            GEOM[r["way_id"]] = r

    # ----------------------------------------------------------------------------- relations
    print("loading A06 relation data ...")
    REL_FWD = defaultdict(set)                 # way -> set(rel_id) traversing in way order
    REL_BWD = defaultdict(set)
    for r in read_csv(DATA / "A06" / "relation_ways.csv"):
        wid = r["way_id"]
        if wid not in ROUT_IDS:
            continue
        d = r["travel_dir"]
        if d == FORWARD:
            REL_FWD[wid].add(r["rel_id"])
        elif d == BACKWARD:
            REL_BWD[wid].add(r["rel_id"])

    # cross-check against A06 pre-aggregated file (usage counts may include duplicates)
    REL_USAGE = {}
    for r in read_csv(DATA / "A06" / "way_usage_direction.csv"):
        if r["way_id"] in ROUT_IDS:
            REL_USAGE[r["way_id"]] = (int(r["n_rel_forward"]), int(r["n_rel_backward"]))

    # ----------------------------------------------------------------------------- node order
    print("loading A05 segment_block (node order) ...")
    NODES_OF = defaultdict(list)
    for r in read_csv(DATA / "A05" / "segment_block.csv"):
        wid = r["way_id"]
        if wid not in ROUT_IDS:
            continue
        segs = NODES_OF[wid]
        if not segs:
            segs.append(r["from_node"])
        segs.append(r["to_node"])

    # ----------------------------------------------------------------------------- node coordinates
    print("reading node coordinates from network_osm.xml.gz ...")
    COORD = {}
    node_re = re.compile(r'<node id="([^"]+)" x="([^"]+)" y="([^"]+)"')
    with gzip.open(BASE / "matsim_input" / "tmp" / "network_osm.xml.gz", "rt") as fh:
        for line in fh:
            if 'x="' in line and line.lstrip().startswith("<node"):
                m = node_re.search(line)
                if m:
                    COORD[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    print(f"  nodes with coordinates: {len(COORD)}")

    WAY_XY = {}
    for w in ROUT:
        nodes = NODES_OF.get(w["way_id"], [])
        pts = [COORD[n] for n in nodes if n in COORD]
        if not pts:
            WAY_XY[w["way_id"]] = (math.nan, math.nan)
            continue
        # length-weighted midpoint is close enough to the centroid of the vertices
        WAY_XY[w["way_id"]] = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

    # ----------------------------------------------------------------------------- foreign infrastructure
    # Right-hand running operators next to Switzerland. Empirically the A02a left-hand
    # geometry is exactly inverted on these ways (see REPORT.md, section 4).
    FOREIGN_KEYS = ("DB Netz", "DB InfraGO", "SNCF", "ÖBB", "Oebb", "Rete Ferroviaria", "RFI")


    def is_foreign(w):
        op = w["operator"] or ""
        if op.startswith("SBB"):
            return False
        return any(k in op for k in FOREIGN_KEYS)


    def rel_dir(wid):
        f, b = len(REL_FWD.get(wid, ())), len(REL_BWD.get(wid, ()))
        if f and b:
            return BOTH
        if f:
            return FORWARD
        if b:
            return BACKWARD
        return None


    def geom_raw(wid):
        g = GEOM.get(wid)
        if not g:
            return None, 0.0
        d = g["direction"]
        return (d if d in (FORWARD, BACKWARD) else None), fnum(g["confidence"])


    def geom_dir(w):
        """A02a direction corrected for right-hand-running foreign infrastructure."""
        d, c = geom_raw(w["way_id"])
        if d and is_foreign(w):
            d = BACKWARD if d == FORWARD else FORWARD
        return d, c


    def tag_overruled(w):
        """A preferred_direction tag is overruled only on overwhelming, independent evidence:
        >=5 unanimous relations AND the (right-hand corrected) geometry agrees with them.
        Empirically this affects the St. Margrethen ref 880 platform-tag artefact only."""
        if w["preferred_direction"] not in (FORWARD, BACKWARD):
            return False
        rd = rel_dir(w["way_id"])
        gd, gc = geom_dir(w)
        nrel = len(REL_FWD.get(w["way_id"], ())) + len(REL_BWD.get(w["way_id"], ()))
        return (rd in (FORWARD, BACKWARD) and rd != w["preferred_direction"]
                and nrel >= 5 and gd == rd and gc >= 0.5)


    def geom_class(w):
        """A02a class including `both` (used only for the 3-class matrices)."""
        d, _ = geom_dir(w)
        if d:
            return d
        g = GEOM.get(w["way_id"])
        return g["direction"] if g and g["direction"] == BOTH else None


    def partner_class(wid):
        """0 / 1 / >=2 parallel partners from A02a sampled shares."""
        g = GEOM.get(wid)
        if not g:
            return "n/a"
        no = fnum(g["share_no_partner"])
        both = fnum(g["share_both_sides"])
        if no >= 0.5:
            return "0"
        if both >= 0.2 and both > no:
            return "2+"
        return "1"


    # ----------------------------------------------------------------------------- chains
    print("building chains through degree-2 nodes ...")
    node_ways = defaultdict(set)
    endpoint_uses = defaultdict(list)
    for wid in sorted(ROUT_IDS, key=int):
        nodes = NODES_OF.get(wid)
        if not nodes:
            continue
        for n in nodes:
            node_ways[n].add(wid)
        endpoint_uses[nodes[0]].append((wid, "f"))
        endpoint_uses[nodes[-1]].append((wid, "l"))

    ADJ = defaultdict(list)                     # way -> list[(neighbour, this_end, other_end)]
    for n in sorted(endpoint_uses, key=int):
        uses = endpoint_uses[n]
        if len(uses) != 2 or len(node_ways[n]) != 2:
            continue
        (a, ea), (b, eb) = sorted(uses)
        if a == b:                              # closed loop digitised as one way
            continue
        ADJ[a].append((b, ea, eb))
        ADJ[b].append((a, eb, ea))

    CHAINS = []                                 # list of {"ways": [...], "orient": {way: +-1}}
    seen = set()
    for start in sorted(ADJ, key=int):
        if len(ADJ[start]) > 1 or start in seen:
            continue
        chain = [start]
        orient = {start: 1}
        seen.add(start)
        prev, cur = None, start
        while True:
            nxt = [(nb, ce, ne) for (nb, ce, ne) in ADJ[cur] if nb != prev]
            if not nxt:
                break
            nb, ce, ne = sorted(nxt)[0]
            if nb in seen:
                break
            orient[nb] = orient[cur] if (ce, ne) in (("l", "f"), ("f", "l")) else -orient[cur]
            chain.append(nb)
            seen.add(nb)
            prev, cur = cur, nb
        CHAINS.append({"ways": chain, "orient": orient})
    # isolated ways and closed rings that were not reached
    for wid in sorted(ROUT_IDS, key=int):
        if wid in seen:
            continue
        CHAINS.append({"ways": [wid], "orient": {wid: 1}})
        seen.add(wid)
    # ring components: every way has 2 neighbours and none was a start
    for wid in sorted(ROUT_IDS, key=int):
        if wid in seen:
            continue
        chain, orient = [wid], {wid: 1}
        seen.add(wid)
        prev, cur = None, wid
        while True:
            nxt = [(nb, ce, ne) for (nb, ce, ne) in ADJ[cur] if nb != prev]
            if not nxt:
                break
            nb, ce, ne = sorted(nxt)[0]
            if nb in seen:
                break
            orient[nb] = orient[cur] if (ce, ne) in (("l", "f"), ("f", "l")) else -orient[cur]
            chain.append(nb)
            seen.add(nb)
            prev, cur = cur, nb
        CHAINS.append({"ways": chain, "orient": orient})
    CHAINS.sort(key=lambda c: (-sum(fnum(WAYS[w]["length_m"]) for w in c["ways"]), int(c["ways"][0])))
    print(f"  chains: {len(CHAINS)}  (single-way {sum(1 for c in CHAINS if len(c['ways']) == 1)})")

    way_chain = {}
    for i, c in enumerate(CHAINS):
        for w in c["ways"]:
            way_chain[w] = i

    # ----------------------------------------------------------------------------- fusion core
    def initial_direction(w, use_tags=True):
        """Return (direction, confidence, source) for the per-way evidence (no chain yet).

        use_tags=False hides every OSM direction tag (used for the hold-out evaluation, where the
        preferred_direction tag is the reference and must not leak into the prediction).
        """
        wid = w["way_id"]
        tag = w["preferred_direction"] if use_tags else ""
        pd_tag = tag if tag in (FORWARD, BACKWARD, BOTH) else None
        bi = w["bidirectional"] if use_tags else ""
        ow = w["oneway"] if use_tags else ""
        rd = rel_dir(wid)
        gd, gc = geom_dir(w)
        nf, nb = len(REL_FWD.get(wid, ())), len(REL_BWD.get(wid, ()))
        nrel = nf + nb

        # 1. explicit preferred_direction, unless overwhelmingly contradicted (see REPORT §4b:
        #    the tag follows platform use, not the running side, in a few station throats)
        if pd_tag:
            if tag_overruled(w):
                return rd, 0.90, "relations"
            return pd_tag, 0.95, "tag"
        # 2. explicit oneway / bidirectional tags (only when no preferred_direction)
        if ow == "yes":
            return FORWARD, 0.85, "tag"
        if bi in ("regular", "possible") and rd == BOTH:
            return BOTH, 0.85, "tag"
        # 3. route relations, when unanimous
        if rd in (FORWARD, BACKWARD):
            conf = 0.85 if nrel == 1 else (0.90 if nrel == 2 else 0.95)
            return rd, conf, "relations"
        # 4. relations say both: single track => genuinely both, else trust geometry
        if rd == BOTH:
            pc = partner_class(wid)
            if pc in ("0", "n/a"):
                return BOTH, 0.90, "relations"
            if gd and gc >= GEOM_TAU:
                return gd, round(min(0.95, gc * 0.9), 3), "geometry"
            return BOTH, 0.55, "relations"
        # 5. geometry
        if gd and gc >= GEOM_TAU:
            return gd, round(min(0.95, gc), 3), "geometry"
        if gd and gc >= GEOM_WEAK_TAU:
            return gd, round(min(0.5, gc * 0.7), 3), "geometry"
        # 6. geometry saw no parallel partner at all => real single track, bidirectional
        raw = GEOM.get(wid)
        if (raw and raw["direction"] == BOTH and fnum(raw["share_no_partner"]) >= 0.5):
            return BOTH, round(min(0.90, gc), 3), "geometry"
        return UNKNOWN, 0.0, "none"


    # thresholds (tuned on the dev half of the tagged ways, see tune())
    GEOM_TAU = 0.75
    GEOM_WEAK_TAU = 0.50

    print("first pass: per-way evidence ...")
    DIR = {}      # wid -> (direction, confidence, source)
    for w in ROUT:
        DIR[w["way_id"]] = initial_direction(w)


    def apply_chain():
        """Propagate majority direction along chains; only fills ways that are still unknown."""
        resolved = 0
        for c in CHAINS:
            ways = c["ways"]
            votes = 0.0
            supporters = 0
            both_km = 0.0
            total_km = 0.0
            for wid in ways:
                L = km(fnum(WAYS[wid]["length_m"]))
                d, conf, src = DIR[wid]
                total_km += L
                if d == FORWARD:
                    votes += L * c["orient"][wid]
                    supporters += 1
                elif d == BACKWARD:
                    votes -= L * c["orient"][wid]
                    supporters += 1
                elif d == BOTH:
                    both_km += L
            if supporters == 0:
                if both_km > 0.5 * total_km:
                    for wid in ways:
                        if DIR[wid][0] == UNKNOWN:
                            DIR[wid] = (BOTH, 0.65, "chain")
                            resolved += 1
                continue
            if abs(votes) < 1e-9:
                continue                        # tie: do not guess
            chain_dir = FORWARD if votes > 0 else BACKWARD
            conf = 0.75 if supporters >= 2 else 0.65
            for wid in ways:
                if DIR[wid][0] != UNKNOWN:
                    continue
                d = chain_dir if c["orient"][wid] > 0 else (
                    BACKWARD if chain_dir == FORWARD else FORWARD)
                DIR[wid] = (d, conf, "chain")
                resolved += 1
        return resolved


    # ---- threshold tuning on the dev half of the tag reference (tag hidden)
    TAGGED = [w for w in ROUT if w["preferred_direction"] in (FORWARD, BACKWARD)]
    DEV = [w for w in TAGGED if is_dev(w["way_id"])]
    TEST = [w for w in TAGGED if not is_dev(w["way_id"])]


    def predict_no_tag(w, tau, weak_tau, min_rel):
        """The fusion rule with the preferred_direction tag hidden (for hold-out scoring)."""
        wid = w["way_id"]
        rd = rel_dir(wid)
        gd, gc = geom_dir(w)
        nf, nb = len(REL_FWD.get(wid, ())), len(REL_BWD.get(wid, ()))
        nrel = nf + nb
        if rd in (FORWARD, BACKWARD) and nrel >= min_rel:
            return rd
        if rd == BOTH:
            pc = partner_class(wid)
            if pc in ("0", "n/a"):
                return BOTH
            if gd and gc >= tau:
                return gd
            return BOTH
        if gd and gc >= tau:
            return gd
        if gd and gc >= weak_tau:
            return gd
        return UNKNOWN


    def accuracy(pairs):
        """pairs of (predicted, reference) restricted to committed predicted f/b."""
        n = sum(1 for p, r in pairs if p in (FORWARD, BACKWARD))
        ok = sum(1 for p, r in pairs if p in (FORWARD, BACKWARD) and p == r)
        return n, ok, (ok / n if n else 0.0)


    print("tuning thresholds on dev half (tag hidden) ...")
    TUNE = []
    BEST = None
    for tau in (0.60, 0.75, 0.85, 0.90):
        for weak in (0.0, 0.50):
            for min_rel in (1, 2):
                pairs = [(predict_no_tag(w, tau, weak, min_rel), w["preferred_direction"]) for w in DEV]
                n, ok, acc = accuracy(pairs)
                TUNE.append([tau, weak, min_rel, len(DEV), n, ok, round(acc, 4),
                             round(n / len(DEV), 4)])
                key = (acc, n)
                if BEST is None or key > BEST[0]:
                    BEST = (key, tau, weak, min_rel)
    _, GEOM_TAU, GEOM_WEAK_TAU, MIN_REL = BEST
    print(f"  chosen: geom_tau={GEOM_TAU} weak_tau={GEOM_WEAK_TAU} min_rel={MIN_REL}")

    # recompute pass 1 with the tuned thresholds
    DIR = {}
    for w in ROUT:
        DIR[w["way_id"]] = initial_direction(w)

    print("second pass: chain smoothing ...")
    RESOLVED_CHAIN = apply_chain()
    print(f"  unknown ways resolved by chain: {RESOLVED_CHAIN}")

    write_csv(HERE / "rule_tuning.csv",
              ["geom_tau", "weak_tau", "min_rel", "dev_ways", "predicted", "correct",
               "accuracy", "coverage"],
              TUNE)

    # ----------------------------------------------------------------------------- final table
    with open(HERE / "direction_fused.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["way_id", "direction", "confidence", "source"])
        for r in ROUT:
            d, conf, src = DIR[r["way_id"]]
            w.writerow([r["way_id"], d, f"{conf:.3f}", src])

    # ways where the preferred_direction tag was overruled (documented exception)
    OVERRULED = []
    for r in ROUT:
        if tag_overruled(r):
            gd, gc = geom_dir(r)
            x, y = WAY_XY[r["way_id"]]
            OVERRULED.append([r["way_id"], r["preferred_direction"], rel_dir(r["way_id"]),
                              len(REL_FWD.get(r["way_id"], ())), len(REL_BWD.get(r["way_id"], ())),
                              gd, gc, round(km(fnum(r["length_m"])), 3), r["klass"], r["ref"],
                              r["operator"], f"{x:.0f}", f"{y:.0f}"])
    OVERRULED.sort(key=lambda z: -z[7])
    write_csv(HERE / "tag_overruled.csv",
              ["way_id", "tag", "relations", "n_rel_forward", "n_rel_backward", "geom_dir",
               "geom_conf", "length_km", "klass", "ref", "operator", "mid_x", "mid_y"],
              OVERRULED)

    # audit index
    rows = []
    for r in ROUT:
        wid = r["way_id"]
        gd, gc = geom_dir(r)
        raw_g = GEOM.get(wid)
        x, y = WAY_XY[wid]
        rows.append([
            wid, r["klass"], f"{km(fnum(r['length_m'])):.4f}", r["operator"], r["ref"],
            r["tracks"], r["track_ref"], r["preferred_direction"], r["bidirectional"], r["oneway"],
            f"{x:.1f}" if x == x else "", f"{y:.1f}" if y == y else "",
            (raw_g or {}).get("direction", ""), (raw_g or {}).get("confidence", ""),
            (raw_g or {}).get("share_no_partner", ""), (raw_g or {}).get("share_both_sides", ""),
            partner_class(wid), 1 if is_foreign(r) else 0,
            len(REL_FWD.get(wid, ())), len(REL_BWD.get(wid, ())), rel_dir(wid) or "",
            way_chain[wid],
            DIR[wid][0], f"{DIR[wid][1]:.3f}", DIR[wid][2],
        ])
    write_csv(HERE / "way_index.csv",
              ["way_id", "klass", "length_km", "operator", "ref", "tracks", "track_ref",
               "preferred_direction", "bidirectional", "oneway", "x", "y",
               "geom_dir", "geom_conf", "share_no_partner", "share_both_sides",
               "partner_class", "foreign", "n_rel_forward", "n_rel_backward", "rel_dir",
               "chain_id", "fused_dir", "fused_conf", "fused_source"],
              rows)

    # ----------------------------------------------------------------------------- agreement matrices
    def commit(d):
        return d if d in (FORWARD, BACKWARD) else None


    AGREE_ROWS = []
    SRC = [
        ("tag", lambda w: commit(w["preferred_direction"])),
        ("relations", lambda w: commit(rel_dir(w["way_id"]))),
        ("geometry", lambda w: commit(geom_dir(w)[0])),
    ]

    # by klass
    for klass in ("main", "branch", "yard", "siding", "spur", "crossover", "rail_other", "other"):
        sub = [w for w in ROUT if w["klass"] == klass]
        for i in range(len(SRC)):
            for j in range(i + 1, len(SRC)):
                a_name, a_fn = SRC[i]
                b_name, b_fn = SRC[j]
                cells = defaultdict(int)
                for w in sub:
                    a, b = a_fn(w), b_fn(w)
                    if a and b:
                        cells[(a, b)] += 1
                n = sum(cells.values())
                if n == 0:
                    continue
                ok = cells[(FORWARD, FORWARD)] + cells[(BACKWARD, BACKWARD)]
                AGREE_ROWS.append([klass, a_name, b_name, n, ok, round(ok / n, 4),
                                   cells[(FORWARD, FORWARD)], cells[(FORWARD, BACKWARD)],
                                   cells[(BACKWARD, FORWARD)], cells[(BACKWARD, BACKWARD)]])
    # by partner class
    for pc in ("0", "1", "2+", "n/a"):
        sub = [w for w in ROUT if partner_class(w["way_id"]) == pc]
        for i in range(len(SRC)):
            for j in range(i + 1, len(SRC)):
                a_name, a_fn = SRC[i]
                b_name, b_fn = SRC[j]
                cells = defaultdict(int)
                for w in sub:
                    a, b = a_fn(w), b_fn(w)
                    if a and b:
                        cells[(a, b)] += 1
                n = sum(cells.values())
                if n == 0:
                    continue
                ok = cells[(FORWARD, FORWARD)] + cells[(BACKWARD, BACKWARD)]
                AGREE_ROWS.append([f"partner={pc}", a_name, b_name, n, ok, round(ok / n, 4),
                                   cells[(FORWARD, FORWARD)], cells[(FORWARD, BACKWARD)],
                                   cells[(BACKWARD, FORWARD)], cells[(BACKWARD, BACKWARD)]])
    write_csv(HERE / "agreement_by_klass.csv",
              ["group", "source_a", "source_b", "n_both_commit", "n_agree", "agreement",
               "a_fwd_b_fwd", "a_fwd_b_bwd", "a_bwd_b_fwd", "a_bwd_b_bwd"],
              AGREE_ROWS)

    # 3-class matrix including `both` (tag vs relations, tag vs geometry, rel vs geometry)
    class_rows = []
    for label, a_name, a_fn, b_name, b_fn in (
            ("tag", "tag", lambda w: w["preferred_direction"] or None,
             "relations", lambda w: rel_dir(w["way_id"])),
            ("tag", "tag", lambda w: w["preferred_direction"] or None,
             "geometry", lambda w: geom_class(w)),
            ("all", "relations", lambda w: rel_dir(w["way_id"]),
             "geometry", lambda w: geom_class(w))):
        cells = defaultdict(int)
        for w in ROUT:
            a, b = a_fn(w), b_fn(w)
            if a in (FORWARD, BACKWARD, BOTH) and b in (FORWARD, BACKWARD, BOTH):
                cells[(a, b)] += 1
        for a in (FORWARD, BACKWARD, BOTH):
            for b in (FORWARD, BACKWARD, BOTH):
                class_rows.append([label, a_name, a, b_name, b, cells[(a, b)]])
    write_csv(HERE / "agreement_3class.csv",
              ["subset", "source_a", "a_class", "source_b", "b_class", "ways"], class_rows)

    # ----------------------------------------------------------------------------- per-source accuracy on the reference
    ACC_ROWS = []
    groups = [("all", lambda w: True),
              ("main", lambda w: w["klass"] == "main"),
              ("branch", lambda w: w["klass"] == "branch"),
              ("partner=0", lambda w: partner_class(w["way_id"]) == "0"),
              ("partner=1", lambda w: partner_class(w["way_id"]) == "1"),
              ("partner=2+", lambda w: partner_class(w["way_id"]) == "2+")]
    for label, sel in groups:
        sub = [w for w in TAGGED if sel(w)]
        rel_sub = [w for w in sub if rel_dir(w["way_id"]) in (FORWARD, BACKWARD)]
        geo_sub = [w for w in sub if geom_dir(w)[0] in (FORWARD, BACKWARD)]
        rel_ok = sum(1 for w in rel_sub if rel_dir(w["way_id"]) == w["preferred_direction"])
        geo_ok = sum(1 for w in geo_sub if geom_dir(w)[0] == w["preferred_direction"])
        ACC_ROWS.append([label, len(sub), len(rel_sub), rel_ok,
                         round(rel_ok / len(rel_sub), 4) if rel_sub else "",
                         len(geo_sub), geo_ok,
                         round(geo_ok / len(geo_sub), 4) if geo_sub else ""])
    write_csv(HERE / "source_accuracy.csv",
              ["group", "tagged_ways", "rel_commit", "rel_correct", "rel_accuracy",
               "geom_commit", "geom_correct", "geom_accuracy"], ACC_ROWS)

    # ----------------------------------------------------------------------------- hold-out evaluation
    print("hold-out evaluation ...")
    pairs_test = [(predict_no_tag(w, GEOM_TAU, GEOM_WEAK_TAU, MIN_REL), w["preferred_direction"])
                  for w in TEST]
    n_test, ok_test, acc_test = accuracy(pairs_test)
    # also with the final fused (including chain) but tag hidden: recompute a shadow copy
    SHADOW = {}
    for w in ROUT:
        SHADOW[w["way_id"]] = initial_direction(w, use_tags=False)
    # chain on shadow
    shadow_resolved = 0
    for c in CHAINS:
        votes, supporters = 0.0, 0
        for wid in c["ways"]:
            d = SHADOW[wid][0]
            if d == FORWARD:
                votes += km(fnum(WAYS[wid]["length_m"])) * c["orient"][wid]
                supporters += 1
            elif d == BACKWARD:
                votes -= km(fnum(WAYS[wid]["length_m"])) * c["orient"][wid]
                supporters += 1
        if supporters and abs(votes) > 1e-9:
            cd = FORWARD if votes > 0 else BACKWARD
            for wid in c["ways"]:
                if SHADOW[wid][0] == UNKNOWN:
                    SHADOW[wid] = (cd if c["orient"][wid] > 0 else
                                   (BACKWARD if cd == FORWARD else FORWARD), 0.7, "chain")
                    shadow_resolved += 1
    pairs_test2 = []
    for w in TEST:
        d = SHADOW.get(w["way_id"], (UNKNOWN, 0, "none"))[0]
        pairs_test2.append((d, w["preferred_direction"]))
    n_t2, ok_t2, acc_t2 = accuracy(pairs_test2)

    cm = defaultdict(int)
    for p, r in pairs_test2:
        if p in (FORWARD, BACKWARD):
            cm[(p, r)] += 1

    # ----------------------------------------------------------------------------- chain contradictions
    print("chain contradictions ...")
    CONTRA = []
    for ci, c in enumerate(CHAINS):
        ways = c["ways"]
        pos = neg = 0.0
        npos = nneg = 0
        for wid in ways:
            d = DIR[wid][0]
            L = km(fnum(WAYS[wid]["length_m"]))
            if d == FORWARD:
                v = c["orient"][wid]
                if v > 0:
                    pos += L; npos += 1
                else:
                    neg += L; nneg += 1
            elif d == BACKWARD:
                v = -c["orient"][wid]
                if v > 0:
                    pos += L; npos += 1
                else:
                    neg += L; nneg += 1
        if npos and nneg:
            xs = [WAY_XY[wid][0] for wid in ways if WAY_XY[wid][0] == WAY_XY[wid][0]]
            ys = [WAY_XY[wid][1] for wid in ways if WAY_XY[wid][1] == WAY_XY[wid][1]]
            CONTRA.append([
                ci, len(ways), round(sum(km(fnum(WAYS[w]["length_m"])) for w in ways), 3),
                npos, round(pos, 3), nneg, round(neg, 3),
                f"{sum(xs)/len(xs):.0f}" if xs else "", f"{sum(ys)/len(ys):.0f}" if ys else "",
                " ".join(ways[:6]),
            ])
    CONTRA.sort(key=lambda r: (-r[2], r[0]))
    write_csv(HERE / "chain_contradictions.csv",
              ["chain_id", "n_ways", "length_km", "n_ways_pos", "km_pos", "n_ways_neg", "km_neg",
               "mid_x", "mid_y", "sample_ways"],
              CONTRA)

    # ----------------------------------------------------------------------------- coverage + changes
    def coverage_map(direction_getter, length_getter):
        cov = defaultdict(float)
        for r in ROUT:
            cov[direction_getter(r["way_id"])] += length_getter(r)
        return cov


    def a02a_dir(wid):
        g = GEOM.get(wid)
        if not g:
            return "no entry"
        return g["direction"]


    cov_before = coverage_map(a02a_dir, lambda r: km(fnum(r["length_m"])))
    cov_after = coverage_map(lambda wid: DIR[wid][0], lambda r: km(fnum(r["length_m"])))
    coverage_rows = []
    for cls in (FORWARD, BACKWARD, BOTH, UNKNOWN, "no entry"):
        coverage_rows.append(["A02a", cls, round(cov_before.get(cls, 0.0), 1)])
    for cls in (FORWARD, BACKWARD, BOTH, UNKNOWN):
        coverage_rows.append(["fused", cls, round(cov_after.get(cls, 0.0), 1)])
    # by klass
    for klass in ("main", "branch", "yard", "siding", "spur", "crossover", "rail_other", "other"):
        sub = [w for w in ROUT if w["klass"] == klass]
        for label, getter in (("A02a", lambda wid: a02a_dir(wid)),
                              ("fused", lambda wid: DIR[wid][0])):
            tot = defaultdict(float)
            for w in sub:
                tot[getter(w["way_id"])] += km(fnum(w["length_m"]))
            for cls in (FORWARD, BACKWARD, BOTH, UNKNOWN, "no entry"):
                if tot.get(cls, 0) > 0:
                    coverage_rows.append([f"{label}:{klass}", cls, round(tot[cls], 1)])
    write_csv(HERE / "coverage.csv", ["table", "direction", "km"], coverage_rows)

    # direction changes vs A02a
    CHANGED = []
    for r in ROUT:
        wid = r["way_id"]
        g = GEOM.get(wid)
        if not g:
            continue
        old, new = g["direction"], DIR[wid][0]
        if old in (FORWARD, BACKWARD, BOTH) and new in (FORWARD, BACKWARD, BOTH) and old != new:
            x, y = WAY_XY[wid]
            CHANGED.append([wid, r["klass"], old, new, round(km(fnum(r["length_m"])), 3),
                            g["confidence"], DIR[wid][2],
                            f"{x:.0f}" if x == x else "", f"{y:.0f}" if y == y else ""])
    CHANGED.sort(key=lambda r: (-r[4], int(r[0])))
    write_csv(HERE / "changed_stretches.csv",
              ["way_id", "klass", "a02a_dir", "fused_dir", "length_km", "a02a_conf", "source",
               "mid_x", "mid_y"], CHANGED)
    _trans = defaultdict(lambda: [0, 0.0])
    for r in CHANGED:
        key = f"{r[2]}->{r[3]}"
        _trans[key][0] += 1
        _trans[key][1] += r[4]
    write_csv(HERE / "direction_changes.csv", ["transition", "ways", "km"],
              [[k, v[0], round(v[1], 1)] for k, v in sorted(_trans.items())])

    # ----------------------------------------------------------------------------- DB / foreign summary
    foreign = [w for w in ROUT if is_foreign(w)]
    db_agree = db_flip = db_n = 0
    for w in foreign:
        gd, _ = geom_raw(w["way_id"])
        rd = rel_dir(w["way_id"])
        if gd in (FORWARD, BACKWARD) and rd in (FORWARD, BACKWARD):
            db_n += 1
            if gd == rd:
                db_agree += 1
            else:
                db_flip += 1

    # ----------------------------------------------------------------------------- stats.md
    print("writing stats.md ...")
    tagged_before = defaultdict(float)
    for r in TAGGED:
        g = GEOM.get(r["way_id"])
        tagged_before[(g or {}).get("direction", "no entry")] += km(fnum(r["length_m"]))

    lines = []
    A = lines.append
    A("# H12 — fusion statistics (generated by build.py)")
    A("")
    A(f"routable ways {len(ROUT)}, {sum(km(fnum(w['length_m'])) for w in ROUT):.1f} km")
    A("")
    A("## Coverage km by direction")
    A("")
    A("| table | forward | backward | both | unknown | no entry |")
    A("|---|---:|---:|---:|---:|---:|")
    for label in ("A02a", "fused"):
        src = cov_before if label == "A02a" else cov_after
        A(f"| {label} | {src.get(FORWARD,0):.1f} | {src.get(BACKWARD,0):.1f} | "
          f"{src.get(BOTH,0):.1f} | {src.get(UNKNOWN,0):.1f} | {src.get('no entry',0):.1f} |")
    A("")
    A("## Agreement (ways where both sources commit forward/backward)")
    A("")
    A("| group | pair | n | agree | % |")
    A("|---|---|---:|---:|---:|")
    for r in AGREE_ROWS:
        if r[0] in ("main", "partner=0", "partner=1", "partner=2+"):
            A(f"| {r[0]} | {r[1]}/{r[2]} | {r[3]} | {r[4]} | {100*r[5]:.1f} |")
    A("")
    A("## Threshold tuning (tag hidden, dev half)")
    A("")
    A("| geom_tau | weak_tau | min_rel | predicted | correct | accuracy | coverage |")
    A("|---|---|---|---:|---:|---:|---:|")
    for t in TUNE:
        A(f"| {t[0]} | {t[1]} | {t[2]} | {t[4]} | {t[5]} | {100*t[6]:.1f} | {100*t[7]:.1f} |")
    A("")
    A(f"chosen geom_tau={GEOM_TAU} weak_tau={GEOM_WEAK_TAU} min_rel={MIN_REL}")
    A("")
    A("## Hold-out (other half of tagged ways, tag hidden)")
    A("")
    A(f"test ways {len(TEST)}; predicted f/b {n_t2}; correct {ok_t2}; accuracy {100*acc_t2:.1f} %; "
      f"coverage {100*n_t2/len(TEST):.1f} %")
    A("")
    A("predicted \\ tagged | forward | backward")
    A("|---|---:|---:|")
    for p in (FORWARD, BACKWARD):
        A(f"| {p} | {cm[(p, FORWARD)]} | {cm[(p, BACKWARD)]} |")
    A("")
    A(f"chain-resolved within hold-out: {shadow_resolved}")
    A("")
    A("## Chains")
    A("")
    A(f"chains {len(CHAINS)}, multi-way {sum(1 for c in CHAINS if len(c['ways'])>1)}, "
      f"unknown ways resolved {RESOLVED_CHAIN}, contradictory chains {len(CONTRA)}")
    A("")
    A("## Source accuracy on the tag reference")
    A("")
    A("| group | tagged | relations n | rel acc | geometry n | geom acc |")
    A("|---|---:|---:|---:|---:|---:|")
    for r in ACC_ROWS:
        A(f"| {r[0]} | {r[1]} | {r[2]} | {100*r[4]:.1f} | {r[5]} | "
          f"{100*r[7] if r[7] != '' else 0:.1f} |")
    A("")
    A("## Direction changes vs A02a")
    A("")
    ch_km = sum(r[4] for r in CHANGED)
    A(f"{len(CHANGED)} ways, {ch_km:.1f} km changed (f/b/both classes only)")
    A("")
    A("| transition | ways | km |")
    A("|---|---:|---:|")
    for k, v in sorted(_trans.items()):
        A(f"| {k} | {v[0]} | {v[1]:.1f} |")
    A("")
    A("## Foreign (right-hand) infrastructure")
    A("")
    A(f"{len(foreign)} routable foreign-operated ways, "
      f"{sum(km(fnum(w['length_m'])) for w in foreign):.1f} km; "
      f"geometry vs relations on {db_n} ways both commit: agree {db_agree}, disagree {db_flip}")
    A("")
    A("## Tag overruled (>=5 unanimous relations + geometry agree against the tag)")
    A("")
    A(f"{len(OVERRULED)} ways, {sum(z[7] for z in OVERRULED):.1f} km")
    A("")
    A("## Source mix (ways / km)")
    A("")
    src_ways = defaultdict(int)
    src_km = defaultdict(float)
    for r in ROUT:
        s = DIR[r["way_id"]][2]
        src_ways[s] += 1
        src_km[s] += km(fnum(r["length_m"]))
    for s in sorted(src_ways):
        A(f"- {s}: {src_ways[s]} ways / {src_km[s]:.1f} km")
    (HERE / "stats.md").write_text("\n".join(lines) + "\n")

    print("\n".join(lines[-30:]))
    print("done")


if __name__ == "__main__":
    run()
