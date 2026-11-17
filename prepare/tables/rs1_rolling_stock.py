"""RS1 - per-line rolling stock table handed to the simulation (`vehicles.file`).

The data (length, vmax, acceleration, deceleration per line name) is researched and sourced in
`prepare/corrections/rolling_stock.csv` (round 1: `swarm/data/RS1/REPORT.md`, one quarter-hour of
work per line from fleet lists and dated photos).  This builder only validates that table and
projects it onto the five columns the run class reads.

Round 3 (agent FS1): the **acceleration** column is the mean acceleration from 0 to 100 km/h of the
constant-force / constant-power traction model, not the mean from 0 to vmax (which the engine applied
as a constant).  The derivation and the sources are in the `source_acceleration` column of the table
and in `docs/assumptions.md`; the resulting table is what the reference runs call
`vehicles_fs1.csv` (`scripts/prepare_case.sh` writes it under that name).

Output: `<out>/RS1/vehicles_rs1.csv`
    line_name_prefix,length_m,vmax_kmh,acceleration,deceleration
An empty `line_name_prefix` is the catch-all row used for lines not listed (stock MATSim
`vehiclesFile` semantics: longest matching prefix wins, the run class sorts by prefix length).

Deterministic: the row order is the row order of the correction table.
"""

from __future__ import annotations

import csv
import sys

from prepare.paths import get as _paths

COLUMNS = ["line_name_prefix", "length_m", "vmax_kmh", "acceleration", "deceleration"]
REQUIRED = ["line_name_prefix", "length_m", "vmax_kmh", "acceleration", "deceleration",
            "rolling_stock", "source"]


def read_corrections(path):
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        missing = [c for c in REQUIRED if c not in (reader.fieldnames or [])]
        if missing:
            sys.exit(f"{path}: missing columns {missing}")
        rows = list(reader)
    prefixes = [r["line_name_prefix"] for r in rows]
    if len(set(prefixes)) != len(prefixes):
        sys.exit(f"{path}: duplicate line_name_prefix")
    if prefixes.count("") != 1:
        sys.exit(f"{path}: exactly one catch-all row (empty prefix) is required")
    for r in rows:
        for c in COLUMNS[1:]:
            float(r[c])                      # raises on a typo
        if not r["source"].strip():
            sys.exit(f"{path}: row {r['line_name_prefix']!r} has no source")
    return rows


def run() -> None:
    paths = _paths()
    out = paths.table("RS1") / "vehicles_rs1.csv"
    rows = read_corrections(paths.root / "prepare/corrections/rolling_stock.csv")
    with open(out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh, lineterminator="\n")
        writer.writerow(COLUMNS)
        for r in rows:
            writer.writerow([r[c] for c in COLUMNS])
    print(f"RS1: {len(rows)} line types -> {out}", file=sys.stderr)
