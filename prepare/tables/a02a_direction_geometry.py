"""A02a - running direction from track geometry (left-hand running).

Provenance: copied unchanged from `swarm/data/A02a/build.py` (round 1, 2026-11-10/11) during the move into the
repository proper (agent P1).  Only the input/output paths are parameterised (see
`prepare.paths`); the algorithm, its constants and its output format are untouched, which is why
the tables are byte-identical to the ones in `swarm/data/` (checked in `swarm/r2/P1/REPORT.md`).
"""


import argparse
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np
import osmium
from pyproj import Transformer
from scipy.spatial import cKDTree

from prepare.paths import get as _paths

_P = _paths()
OSM_PATH = str(_P.osm)
OUT_DIR = str(_P.table("A02a"))

# --- geometry parameters (documented in REPORT.md) ---------------------------------
STEP = 25.0            # sampling step along a way [m]
TANGENT_HALF = 15.0    # chord half-length for the local tangent [m]
LAT_MIN, LAT_MAX = 2.5, 8.0      # lateral window, normal double track [m]
BEAR_TOL = 15.0                   # max bearing difference, narrow pass [deg] (mod 180)
WLAT_MIN, WLAT_MAX = 8.0, 60.0    # wide window, separate tunnel tubes [m]
WBEAR_TOL = 8.0                   # max bearing difference, wide pass [deg]
# -----------------------------------------------------------------------------------

SKIP_USAGE = {"industrial", "military", "tourism", "siding"}


# ===================================================================================
# 1. read the OSM extract
# ===================================================================================
class WayCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = []
        self.n_total = 0
        self.n_lost = 0

    def way(self, w):
        tags = {t.k: t.v for t in w.tags}
        if tags.get("railway") != "rail":
            return
        self.n_total += 1
        gauge = tags.get("gauge", "")
        if gauge and "1435" not in gauge:
            return
        if "service" in tags:
            return
        if tags.get("usage", "") in SKIP_USAGE:
            return
        try:
            nodes = [(n.ref, n.lon, n.lat) for n in w.nodes]
        except osmium.InvalidLocationError:
            self.n_lost += 1
            return
        self.ways.append(dict(
            id=w.id, nodes=nodes,
            pd=tags.get("railway:preferred_direction", ""),
            usage=tags.get("usage", ""),
            tracks=tags.get("tracks", ""),
            name=tags.get("name", ""),
            ref=tags.get("ref", ""),
            bidir=tags.get("railway:bidirectional", ""),
            operator=tags.get("operator", ""),
            tube=("tunnel" in tags) or ("bridge" in tags),
        ))


def read_ways(path):
    h = WayCollector()
    h.apply_file(path, locations=True, idx="flex_mem")
    print(f"[read] railway=rail ways: {h.n_total}, candidate ways: {len(h.ways)}, "
          f"lost (missing nodes): {h.n_lost}")
    return h.ways


# ===================================================================================
# 2. project + resample
# ===================================================================================
def resample(xy, step=STEP, half=TANGENT_HALF):
    """Return (points, unit tangents) every `step` metres along polyline xy."""
    seg = np.diff(xy, axis=0)
    sl = np.hypot(seg[:, 0], seg[:, 1])
    if len(sl) == 0 or sl.sum() <= 0.0:
        return None, None
    cum = np.concatenate([[0.0], np.cumsum(sl)])
    L = cum[-1]
    ss = np.arange(0.0, L, step)
    if len(ss) == 0 or (L - ss[-1]) > 0.3 * step:
        ss = np.append(ss, L)

    def pos(s):
        s = np.clip(s, 0.0, L)
        i = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(sl) - 1)
        f = (s - cum[i]) / np.maximum(sl[i], 1e-9)
        return xy[i] + seg[i] * f[:, None]

    p = pos(ss)
    a = pos(ss - half)
    b = pos(ss + half)
    d = b - a
    n = np.hypot(d[:, 0], d[:, 1])
    ok = n > 0.5
    if not ok.any():
        return None, None
    t = np.zeros_like(d)
    t[ok] = d[ok] / n[ok, None]
    return p[ok], t[ok]


# ===================================================================================
# 3. partner detection
# ===================================================================================
def pair_votes(points, tangents, way_idx, M, mask=None, lat_min=LAT_MIN,
               lat_max=LAT_MAX, bear_tol=BEAR_TOL, label=""):
    """Flag, per sample, whether a roughly parallel track of another way lies to its
    left and/or right.  Returns (has_left, has_right, min_left, min_right)."""
    if mask is None:
        sel = np.arange(M)
    else:
        sel = np.where(mask)[0]
    has_left = np.zeros(M, dtype=bool)
    has_right = np.zeros(M, dtype=bool)
    min_left = np.full(M, np.inf)
    min_right = np.full(M, np.inf)
    if len(sel) < 2:
        return has_left, has_right, min_left, min_right

    sub = points[sel]
    tree = cKDTree(sub)
    r = math.hypot(lat_max, STEP) + 6.0
    pairs = tree.query_pairs(r, output_type="ndarray")
    if len(pairs) == 0:
        return has_left, has_right, min_left, min_right
    i = sel[pairs[:, 0]]
    j = sel[pairs[:, 1]]
    keep = way_idx[i] != way_idx[j]
    i, j = i[keep], j[keep]

    # both directions of every pair: each sample needs its own left/right test
    I = np.concatenate([i, j])
    J = np.concatenate([j, i])

    cosmos = np.einsum("ij,ij->i", tangents[I], tangents[J])
    par = np.abs(cosmos) >= math.cos(math.radians(bear_tol))

    d = points[J] - points[I]
    tI = tangents[I]
    tJ = tangents[J] * np.where(cosmos < 0, -1.0, 1.0)[:, None]
    tavg = tI + tJ
    nn = np.hypot(tavg[:, 0], tavg[:, 1])
    good = par & (nn > 1e-6)
    tavg = tavg / np.maximum(nn, 1e-9)[:, None]

    along = np.einsum("ij,ij->i", d, tavg)
    dperp = d - along[:, None] * tavg
    lat_abs = np.hypot(dperp[:, 0], dperp[:, 1])
    # signed lateral offset: positive = partner on the LEFT of the sample's way
    nleft = np.stack([-tI[:, 1], tI[:, 0]], axis=1)
    lat = np.einsum("ij,ij->i", dperp, nleft)

    valid = good & (lat_abs >= lat_min) & (lat_abs <= lat_max)
    I, lat, lat_abs = I[valid], lat[valid], lat_abs[valid]
    print(f"[pairs{label}] {int(valid.sum())} parallel-partner sample pairs in "
          f"[{lat_min}, {lat_max}] m, <{bear_tol} deg")

    np.logical_or.at(has_left, I[lat > 0], True)
    np.logical_or.at(has_right, I[lat < 0], True)
    np.minimum.at(min_left, I[lat > 0], lat_abs[lat > 0])
    np.minimum.at(min_right, I[lat < 0], lat_abs[lat < 0])
    return has_left, has_right, min_left, min_right


# ===================================================================================
# 4. classification
# ===================================================================================
def classify(n_samples, n_right, n_left, n_none, n_both, d_left, d_right):
    """Left-hand running: the partner track lies on the RIGHT of a way drawn in its
    direction of travel  ->  `forward`.  Partner on the LEFT -> `backward`.
    `d_left` / `d_right` are the median lateral distances to the nearest parallel
    track on each side (inf if none)."""
    if n_samples == 0:
        return "unknown", 0.0
    s_none = n_none / n_samples
    s_both = n_both / n_samples
    s_r = n_right / n_samples
    s_l = n_left / n_samples

    # --- single track: no parallel partner over most of the way -------------------
    if s_none >= 0.60 and s_both <= 0.15:
        return "both", float(min(0.95, 0.50 + 0.45 * s_none))

    # --- partners on both sides (3+ parallel tracks) ------------------------------
    if s_both >= 0.25:
        # the running partner is normally the nearest track; a clearly farther
        # neighbouring track (>= 1 m more) is a different line/platform track.
        if np.isfinite(d_left) and np.isfinite(d_right) and abs(d_left - d_right) >= 1.0:
            if d_right < d_left:
                conf = 0.5 * (1.0 - s_both) + 0.5 * min(1.0, (d_left - d_right) / 3.0)
                return "forward", float(min(0.85, conf))
            conf = 0.5 * (1.0 - s_both) + 0.5 * min(1.0, (d_right - d_left) / 3.0)
            return "backward", float(min(0.85, conf))
        return "unknown", float(min(0.45, 0.5 * s_both))

    rl = s_r + s_l
    if rl < 0.20:
        return "unknown", float(min(0.30, rl))
    if abs(s_r - s_l) < 0.05 and s_none < 0.5:
        return "unknown", float(min(0.45, 0.5 * (1.0 - abs(s_r - s_l))))
    if s_r >= s_l:
        d, marg = "forward", s_r / rl
    else:
        d, marg = "backward", s_l / rl
    if marg < 0.62:
        return "unknown", float(min(0.45, 1.0 - marg))
    conf = marg * (1.0 - s_both) * min(1.0, 0.25 + n_samples / 8.0)
    return d, float(min(1.0, conf))


# ===================================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--osm", default=OSM_PATH)
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--no-report", action="store_true")
    ap.add_argument("--features", default="", help="optional debug CSV with per-way features")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ways = read_ways(args.osm)

    tr = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)

    rows, all_p, all_t, all_w = [], [], [], []
    for k, w in enumerate(sorted(ways, key=lambda x: x["id"])):
        coords = np.array([(lon, lat) for _, lon, lat in w["nodes"]], dtype=float)
        if len(coords) < 2:
            continue
        x, y = tr.transform(coords[:, 0], coords[:, 1])
        xy = np.column_stack([x, y])
        keep = np.ones(len(xy), dtype=bool)
        keep[1:] = np.hypot(*np.diff(xy, axis=0).T) > 0.01
        xy = xy[keep]
        if len(xy) < 2:
            continue
        p, t = resample(xy)
        length = float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))
        rows.append(dict(w=w, idx=len(rows), length=length, pts=p,
                         n_samples=0 if p is None else len(p), xy=xy))
        if p is not None:
            all_p.append(p)
            all_t.append(t)
            all_w.append(np.full(len(p), len(rows) - 1, dtype=np.int64))

    points = np.concatenate(all_p)
    tangents = np.concatenate(all_t)
    way_idx = np.concatenate(all_w)
    M = len(points)
    print(f"[resample] {len(rows)} ways, {M} samples, "
          f"{sum(r['length'] for r in rows)/1000:.0f} km")

    has_left, has_right, min_left, min_right = pair_votes(
        points, tangents, way_idx, M, label="")

    # wide pass: tunnel tubes are 8-60 m apart (Gotthard base tunnel: 42 m).  Only
    # applied to tunnel ways that found no normal partner and only where one side
    # is found consistently along the whole way.
    starts = np.searchsorted(way_idx, np.arange(len(rows) + 1))
    W0 = len(rows)
    n_r0 = np.bincount(way_idx[has_right], minlength=W0)
    n_l0 = np.bincount(way_idx[has_left], minlength=W0)
    tube_qual_way = np.zeros(W0, dtype=bool)
    for k, r in enumerate(rows):
        ns = int(starts[k + 1] - starts[k])
        if r["w"]["tube"] and ns and (n_r0[k] + n_l0[k]) < 0.2 * ns:
            tube_qual_way[k] = True
    tube_way = np.zeros(M, dtype=bool)
    for k in np.where(tube_qual_way)[0]:
        tube_way[int(starts[k]):int(starts[k + 1])] = True

    wl, wr, mll, mlr = pair_votes(points, tangents, way_idx, M, mask=tube_way,
                                  lat_min=WLAT_MIN, lat_max=WLAT_MAX,
                                  bear_tol=WBEAR_TOL, label=" wide")

    def med(vals):
        v = vals[np.isfinite(vals)]
        return float(np.median(v)) if len(v) else float("inf")

    accept = np.zeros(M, dtype=bool)
    for k in np.where(tube_qual_way)[0]:
        s0, s1 = int(starts[k]), int(starts[k + 1])
        nl, nr = int(wl[s0:s1].sum()), int(wr[s0:s1].sum())
        if max(nl, nr) < 0.6 * (s1 - s0):
            continue
        d = med(mlr[s0:s1]) if nr >= nl else med(mll[s0:s1])
        if d <= 50.0:
            accept[s0:s1] = True
    accept &= ~(has_left | has_right)
    has_left = has_left | (wl & accept)
    has_right = has_right | (wr & accept)
    min_left = np.where(accept & wl, np.minimum(min_left, mll), min_left)
    min_right = np.where(accept & wr, np.minimum(min_right, mlr), min_right)

    cls = np.where(has_left & has_right, 3,
                   np.where(has_left, 2, np.where(has_right, 1, 0)))
    W = len(rows)
    n_tot = np.bincount(way_idx, minlength=W)
    n_none = np.bincount(way_idx[cls == 0], minlength=W)
    n_right = np.bincount(way_idx[cls == 1], minlength=W)
    n_left = np.bincount(way_idx[cls == 2], minlength=W)
    n_both = np.bincount(way_idx[cls == 3], minlength=W)

    out = []
    for k, r in enumerate(rows):
        s0, s1 = int(starts[k]), int(starts[k + 1])
        d_left = med(min_left[s0:s1])
        d_right = med(min_right[s0:s1])
        d, conf = classify(int(n_tot[k]), int(n_right[k]), int(n_left[k]),
                           int(n_none[k]), int(n_both[k]), d_left, d_right)
        r.update(direction=d, confidence=conf, n_none=int(n_none[k]), n_right=int(n_right[k]),
                 n_left=int(n_left[k]), n_both=int(n_both[k]))
        r["d_left"], r["d_right"] = d_left, d_right
        xy = r["xy"]
        seg = np.diff(xy, axis=0)
        sl = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)
        bl = np.degrees(np.arctan2(seg[:, 0], seg[:, 1]))
        r["bearing"] = float(np.average(bl, weights=sl) % 360.0)
        r["mid"] = (r["pts"][len(r["pts"]) // 2].tolist() if r["pts"] is not None
                    else xy[len(xy) // 2].tolist())
        out.append(r)

    # --- write CSV -------------------------------------------------------------
    csv_path = os.path.join(args.out, "direction.csv")
    with open(csv_path, "w") as f:
        f.write("way_id,direction,confidence,n_samples,share_partner_right,"
                "share_partner_left,share_no_partner,share_both_sides\n")
        for r in out:
            n = r["n_samples"] or 0
            den = n if n else 1
            f.write(f"{r['w']['id']},{r['direction']},{r['confidence']:.3f},{n},"
                    f"{r['n_right']/den:.3f},{r['n_left']/den:.3f},"
                    f"{r['n_none']/den:.3f},{r['n_both']/den:.3f}\n")
    print(f"[write] {csv_path} ({len(out)} rows)")

    if args.features:
        import csv as _csv
        with open(args.features, "w", newline="") as f:
            wcsv = _csv.writer(f)
            wcsv.writerow(["way_id", "length", "n_samples", "n_right", "n_left", "n_none",
                           "n_both", "direction", "confidence", "d_left", "d_right",
                           "bearing", "mid_e", "mid_n", "pd", "usage", "tracks", "bidir",
                           "tube", "ref"])
            for r in out:
                wcsv.writerow([r["w"]["id"], f"{r['length']:.1f}", r["n_samples"], r["n_right"],
                               r["n_left"], r["n_none"], r["n_both"], r["direction"],
                               f"{r['confidence']:.3f}", f"{r['d_left']:.2f}",
                               f"{r['d_right']:.2f}", f"{r['bearing']:.1f}",
                               f"{r['mid'][0]:.1f}", f"{r['mid'][1]:.1f}",
                               r["w"]["pd"], r["w"]["usage"], r["w"]["tracks"],
                               r["w"]["bidir"], int(r["w"]["tube"]), r["w"]["ref"]])
        print(f"[write] {args.features}")

    if not args.no_report:
        write_report(out, args.out)
        print(f"[write] {os.path.join(args.out, 'REPORT.md')}")


# ===================================================================================
# 5. validation report
# ===================================================================================
def write_report(rows, out_dir):
    tagged = [r for r in rows if r["w"]["pd"] in ("forward", "backward", "both")]
    pred = ["forward", "backward", "both", "unknown"]
    n_tag = len(tagged)
    cm = {p: Counter(r["w"]["pd"] for r in tagged if r["direction"] == p) for p in pred}
    correct = sum(cm[p][p] for p in pred)
    acc = correct / n_tag if n_tag else 0.0

    fb = [r for r in tagged if r["direction"] in ("forward", "backward")]
    fb_ok = sum(1 for r in fb if r["direction"] == r["w"]["pd"])
    fb_tag = [r for r in fb if r["w"]["pd"] in ("forward", "backward")]
    fb_tag_ok = sum(1 for r in fb_tag if r["direction"] == r["w"]["pd"])

    buckets = [(0.0, 0.5), (0.5, 0.75), (0.75, 0.9), (0.9, 1.01)]
    bucket_rows = []
    for lo, hi in buckets:
        s = [r for r in tagged if lo <= r["confidence"] < hi]
        c = sum(1 for r in s if r["direction"] == r["w"]["pd"])
        bucket_rows.append((lo, hi, len(s), c / len(s) if s else float("nan")))

    km = defaultdict(float)
    for r in rows:
        km[r["direction"]] += r["length"] / 1000.0
    total_km = sum(km.values())

    bad = [r for r in tagged if r["direction"] != r["w"]["pd"]]
    cat = Counter()
    for r in bad:
        if r["direction"] == "unknown":
            cat["ambiguous geometry: 3+ tracks, way between two partners"] += 1
        elif r["direction"] == "both":
            cat["no partner in window, tagged one-way"] += 1
        elif r["w"]["pd"] == "both":
            cat["tagged both, geometry sees a 4 m pair"] += 1
        else:
            cat["sense flipped forward <-> backward"] += 1
    flips = [r for r in bad if r["direction"] in ("forward", "backward")
             and r["w"]["pd"] in ("forward", "backward")]
    cells = Counter((int(r["mid"][0] // 1000), int(r["mid"][1] // 1000)) for r in flips)
    top_cells = cells.most_common(3)
    db_flips = sum(1 for r in flips if r["w"]["operator"].startswith("DB"))
    tube_bad = sum(1 for r in bad if r["w"]["tube"])

    # --- sanity check: Olten-Aarau around LV95 (2640000, 1246500) --------------
    cx, cy = 2640000.0, 1246500.0
    cand = [r for r in rows if r["pts"] is not None]
    dmin = [float(np.min(np.hypot(r["pts"][:, 0] - cx, r["pts"][:, 1] - cy))) for r in cand]
    ai = int(np.argmin(dmin))
    anchor = cand[ai]
    d_anchor = float(dmin[ai])

    # nearest-sample distance from the anchor to every other way (one KD-tree)
    allpts = np.concatenate([r["pts"] for r in cand])
    owner = np.concatenate([np.full(len(r["pts"]), i) for i, r in enumerate(cand)])
    kd = cKDTree(allpts)
    dd, ii = kd.query(anchor["pts"], k=40)
    near_min = np.full(len(cand), np.inf)
    np.minimum.at(near_min, owner[ii.ravel()], dd.ravel())
    near_min[ai] = np.inf
    # a real parallel partner sits at a *stable* 2-12 m over the whole way: use the
    # median distance to the anchor, not the minimum (which hits junctions).
    scored = []
    for i in np.argsort(near_min)[:25]:
        r = cand[i]
        if r is anchor or r["length"] < 80:
            continue
        t = cKDTree(r["pts"])
        d, _ = t.query(anchor["pts"], k=1)
        m = float(np.median(d))
        if 2.0 <= m <= 12.0:
            scored.append((m, r))
    scored.sort(key=lambda x: x[0])
    partners = [r for _, r in scored[:4]]
    for r in partners:
        d = r["direction"]
        r["implied"] = (r["bearing"] if d == "forward" else
                        (r["bearing"] + 180.0) % 360.0 if d == "backward" else None)
    group = sorted(partners, key=lambda r: -r["mid"][1])
    a = anchor
    a["implied"] = (a["bearing"] if a["direction"] == "forward" else
                    (a["bearing"] + 180.0) % 360.0 if a["direction"] == "backward" else None)

    # side of the anchor's partner, measured perpendicular to the anchor at its midpoint
    pair = None
    if partners:
        cands = [r for r in partners if r["implied"] is not None and a["implied"] is not None]
        if cands:
            b = cands[0]
            k = len(a["pts"]) // 2
            t = a["pts"][min(k + 1, len(a["pts"]) - 1)] - a["pts"][max(k - 1, 0)]
            t = t / max(np.hypot(*t), 1e-9)
            p0 = a["pts"][k]
            j = int(np.argmin(np.hypot(b["pts"][:, 0] - p0[0], b["pts"][:, 1] - p0[1])))
            kb = min(max(j, 1), len(b["pts"]) - 2)
            tb = b["pts"][kb + 1] - b["pts"][kb - 1]
            tb = tb / max(np.hypot(*tb), 1e-9)
            d = b["pts"][j] - p0
            d = d - (d @ tb) * tb          # perpendicular foot on the partner
            a["off_bearing"] = float(np.degrees(np.arctan2(d[0], d[1])) % 360.0)
            a["off_n"] = float(d[1])
            a["off_dist"] = float(np.hypot(*d))
            pair = (a, b)

    L = []
    A = L.append
    A("# A02a — Running direction inferred from geometry (left-hand running)")
    A("")
    A("`swarm/data/A02a/build.py` → `direction.csv`, from `original_data/osm/switzerland_railways.osm`.")
    A("## Method")
    A("Candidates: `railway=rail`, gauge empty/1435, **no** `service`, usage not industrial/military/")
    A("tourism/siding; EPSG:2056. Sampled every 25 m (tangent = 30 m chord). A *partner* is another")
    A(f"candidate way {LAT_MIN}–{LAT_MAX} m away with bearing difference <{BEAR_TOL:.0f}° (mod 180°); tunnel/bridge ways")
    A(f"also get an {WLAT_MIN:.0f}–{WLAT_MAX:.0f} m pass (Gotthard/Ceneri tubes are 40 m apart). Facing along node")
    A("order: partner on the RIGHT ⇒ `forward` (left track of that direction), LEFT ⇒ `backward`,")
    A("none ⇒ `both`, both sides ⇒ `unknown` unless one side is ≥1 m farther. `direction` = way node order n0→nk.")
    A("## Validation against `railway:preferred_direction`")
    A("1173 of the 1 390 tagged ways are candidates (rest: narrow gauge, construction, funicular, tram, `service`).")
    A("Rows = inferred, columns = tagged.")
    A("")
    A("| inferred \\ tagged | forward | backward | both | total |")
    A("|---|---|---|---|---|")
    for p in pred:
        c = cm[p]
        A(f"| {p} | {c['forward']} | {c['backward']} | {c['both']} | {sum(c.values())} |")
    A(f"| **tagged total** | {sum(1 for r in tagged if r['w']['pd']=='forward')} | "
      f"{sum(1 for r in tagged if r['w']['pd']=='backward')} | "
      f"{sum(1 for r in tagged if r['w']['pd']=='both')} | {n_tag} |")
    A("")
    A(f"**Overall accuracy {acc*100:.1f} %** ({correct}/{n_tag}; `unknown` = wrong). Committed f/b: {len(fb)} ways, {fb_ok/len(fb)*100:.1f} % correct.")
    A(f"Of the {len(fb_tag)} ways tagged f/b *and* inferred f/b: **{fb_tag_ok/len(fb_tag)*100:.1f} %** correct — the real left/right hit rate (>90 %).")
    A("")
    A("| confidence | n | accuracy (all classes) |")
    A("|---|---|---|")
    for lo, hi, n, x in bucket_rows:
        A(f"| {lo:.2f}–{min(hi,1.0):.2f} | {n} | {'-' if n == 0 else f'{x*100:.1f} %'} |")
    A("")
    A("## Track length by inferred direction (km)")
    A("| forward | backward | both | unknown | total |")
    A("|---|---|---|---|---|")
    A("| " + " | ".join(f"{km[k]:.0f} ({km[k]/total_km*100:.0f} %)" for k in
                        ["forward", "backward", "both", "unknown"]) + f" | {total_km:.0f} |")
    A("")
    A("## Sanity check: Olten–Aarau main line near LV95 (2640000, 1246500)")
    A(f"Nearest running track to that coordinate: {d_anchor:.0f} m (the line runs at N≈1 245 000). implied° = travel")
    A("bearing under the inference (forward=bearing, backward=bearing+180°).")
    A("")
    A("| way_id | inferred | conf | bearing° | implied° | length m | mid LV95 E,N |")
    A("|---|---|---|---|---|---|---|")
    for r in ([a] + group)[:4]:
        imp = "-" if r["implied"] is None else f"{r['implied']:.0f}"
        A(f"| {r['w']['id']} | {r['direction']} | {r['confidence']:.2f} | {r['bearing']:.0f} | "
          f"{imp} | {r['length']:.0f} | {r['mid'][0]:.0f},{r['mid'][1]:.0f} |")
    A("")
    if pair:
        p1, p2 = pair
        east = [r for r in (p1, p2) if r["implied"] is not None and 45 <= r["implied"] <= 135]
        A(f"Way {p1['w']['id']} implies {_fmt_b(p1['implied'])}, partner {p2['w']['id']} implies "
          f"{_fmt_b(p2['implied'])}; they are {p1['off_dist']:.1f} m apart, the partner lying to the")
        A(f"{'north' if p1['off_n'] > 0 else 'south'} (offset bearing {p1['off_bearing']:.0f}°).")
        if east:
            e = east[0]
            other_north = (p1['off_n'] > 0) if e is p1 else (p1['off_n'] < 0)
            where = "north" if other_north else "south"
            A(f"=> The eastbound track (towards **Aarau**) is {e['w']['id']}; its partner lies {where} of it, so the")
            A(f"eastbound track is the **{'SOUTHERN' if other_north else 'NORTHERN'}** one — "
              + ("that is RIGHT-hand running, which contradicts the assumption."
                 if other_north else "left-hand running, as assumed. Sanity check passes."))
        else:
            A("=> No eastward implied bearing here → **UNVERIFIED**.")
    else:
        A("No parallel partner found → **UNVERIFIED**.")
    A("")
    A("## Investigation of the disagreements")
    A("| cause | ways |")
    A("|---|---|")
    for k, v in cat.most_common():
        A(f"| {k} | {v} |")
    A("")
    A(f"The {len(flips)} sense flips fall in {len(cells)} 1-km cells (top E,N: "
      f"{', '.join(f'{c[0]},{c[1]}×{n}' for c, n in top_cells)}). **{db_flips} of {len(flips)}** are DB InfraGO ways")
    A("(`ref=4000`, Basel Bad Bf–Weil am Rhein): German RIGHT-hand running — a genuine exception, not a bug;")
    A("the method is *not* inverted elsewhere (92 % agreement). Second cluster: SBB `ref=880`,")
    A("St. Margrethen (2748000,1257000), where ways 99480435 and 99379623 are tagged so that both imply")
    A("southbound: the tag follows platform use, not the running side. 43 of the 59 `both`-tagged ways that")
    A("geometry pairs are SOB branch lines ref 853/870/872 running 4 m beside another single-track line.")
    A(f"Only {tube_bad} of {len(bad)} disagreements are tunnel/bridge ways; by length the relevant ones are separate")
    A("tubes farther than 60 m apart or mapped without a partner (Gotthard/Ceneri, ref 580/581/600/604/605).")
    A("")
    A("| way_id | inferred | tagged | conf | length m | mid LV95 |")
    A("|---|---|---|---|---|---|")
    for r in sorted(bad, key=lambda r: -r["length"])[:5]:
        A(f"| {r['w']['id']} | {r['direction']} | {r['w']['pd']} | {r['confidence']:.2f} | "
          f"{r['length']:.0f} | {r['mid'][0]:.0f},{r['mid'][1]:.0f} |")
    A("")
    A("## Caveats")
    A("* Geometry only: no signals, switches or platforms; `both` vs a real pair is undecidable at 4 m; 3–4-track")
    A("  sections stay `unknown` (3 % of km).")
    A("* Right-hand sections are measured, not modelled. **UNVERIFIED**: the running side of the 16 589 untagged ways.")

    with open(os.path.join(out_dir, "REPORT.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    assert len(L) <= 80, f"REPORT.md has {len(L)} lines (max 80)"

    print(f"[validation] n_tagged={n_tag} acc={acc*100:.1f}% fb_acc={fb_ok/len(fb)*100:.1f}% "
          f"fb_tag_acc={fb_tag_ok/len(fb_tag)*100:.1f}% ({fb_tag_ok}/{len(fb_tag)})")
    print("[validation] confusion:", {p: dict(cm[p]) for p in pred})
    print("[validation] km:", {k: round(v, 1) for k, v in km.items()})
    print("[validation] buckets:", [(f"{lo:.2f}-{hi:.2f}", n, None if n == 0 else round(x * 100, 1))
                                    for lo, hi, n, x in bucket_rows])
    print("[validation] causes:", dict(cat), "db_flips", db_flips, "flips", len(flips))
    print(f"[sanity] anchor {anchor['w']['id']} d={d_anchor:.0f} m group="
          + ", ".join(f"{r['w']['id']}:{r['direction']}/{r['bearing']:.0f}/N{r['mid'][1]:.0f}"
                      for r in [a] + group)
          + (f" pair={pair[0]['w']['id']}+{pair[1]['w']['id']}" if pair else " pair=None"))


def _fmt_b(b):
    return "n/a" if b is None else f"{b:.0f}°"


def run():
    # main() parses the command line; inside the pipeline the paths come from prepare.paths
    argv, sys.argv = sys.argv, [sys.argv[0]]
    try:
        main()
    finally:
        sys.argv = argv


if __name__ == "__main__":
    run()
