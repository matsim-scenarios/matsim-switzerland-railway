"""DIR - the running-direction table the router reads: H12 fusion + reviewed station corrections.

Input
    <out>/H12/direction_fused.csv          fused per-way direction (H12)
    prepare/corrections/direction_overrides.csv
        way_id,direction,confidence,source,reason
        One row per reviewed correction from the station analyses (`swarm/hotspots/H01..H11`).
        `source` names the station analysis that found it, `reason` is its evidence.
Output
    <out>/DIR/direction_final.csv
        way_id,direction,confidence,source   direction relative to the OSM way's node order,
        `source` = `fused:<H12 source>` or `override:<station analysis>`

Fusion rule (round 1: `swarm/data/DIR/build.py`, unchanged)
    * a correction is applied only if the way is in A01 with `routable=1`, its direction is one of
      forward/backward/both and its confidence is >= 0.65;
    * between two corrections for the same way the one with the higher confidence wins;
    * the row order - and therefore which of two equal-confidence corrections wins - follows the
      order of the correction file.  Do not re-sort that file.

Deterministic: no randomness, output sorted by way id (numeric).
"""

from __future__ import annotations

import csv
import sys
from collections import Counter

from prepare.paths import get as _paths

CONF_MIN = 0.65
VALID = ("forward", "backward", "both")


def fuse(ways_csv, fused_csv, corrections_csv):
    ways = set()
    with open(ways_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            ways.add(r["way_id"])

    out = {}
    with open(fused_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            out[r["way_id"]] = [r["direction"], float(r["confidence"] or 0),
                                "fused:" + r["source"]]

    n_override = n_flip = 0
    with open(corrections_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            w, d = r["way_id"].strip(), r["direction"].strip()
            try:
                c = float(r["confidence"])
            except ValueError:
                continue
            if w not in ways or d not in VALID or c < CONF_MIN:
                continue
            old = out.get(w)
            if old and old[2].startswith("override") and old[1] >= c:
                continue                      # keep the more confident station analysis
            if old and {old[0], d} == {"forward", "backward"}:
                n_flip += 1
            out[w] = [d, c, "override:" + r["source"].strip()]
            n_override += 1
    return out, n_override, n_flip


def run() -> None:
    paths = _paths()
    out_dir = paths.table("DIR")
    out, n_override, n_flip = fuse(
        paths.table("A01") / "ways.csv",
        paths.table("H12") / "direction_fused.csv",
        paths.root / "prepare/corrections/direction_overrides.csv",
    )
    dest = out_dir / "direction_final.csv"
    with open(dest, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["way_id", "direction", "confidence", "source"])
        for k in sorted(out, key=int):
            writer.writerow([k, out[k][0], f"{out[k][1]:.3f}", out[k][2]])
    print(f"DIR: {len(out)} ways, {n_override} corrections applied ({n_flip} flips vs fused), "
          f"{dict(Counter(v[0] for v in out.values()))} -> {dest}", file=sys.stderr)
