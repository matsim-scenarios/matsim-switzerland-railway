#!/usr/bin/env python3
"""Single entry point of the static-table preparation.

Turns the original open data into the tables the track router and the run class read:

    original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip   --(pt2matsim, GenerateRailsimInput)-->
    matsim_input/tmp/schedule_gtfs_filtered.xml.gz, network_osm.xml.gz
    original_data/osm/switzerland_railways.osm            --(this script)-->  matsim_input/tables/*
    original_data/osm/switzerland-latest.osm.pbf          --(this script)-->  matsim_input/tables/A06/*

Tables (one directory each, see docs/pipeline.md for what every file is used by):

    A01  ways.csv, components.csv          OSM ways + routable components        (A01)
    A02a direction.csv                     running direction from geometry       (A02a)
    A03a platform_tracks.csv               GTFS platform -> OSM track way        (A03a)
    A04  turns.csv, node_degree.csv        legal movements at junctions          (A04)
    A05  segment_block.csv, blocks.csv, ...OSM main signals as block sections     (A05)
    A06  relation_ways.csv, match.csv, ... OSM route=train relations             (A06)
    H12  direction_fused.csv               fused running direction               (H12)
    DIR  direction_final.csv               fused + reviewed corrections          (router input)
    RS1  vehicles_rs1.csv                  per-line rolling stock                (run input)
    OFFMODEL/stop_track_gauge.csv          real vs off-model track per stop       (router input, CH)
    OFFMODEL/leg_distance.csv              real distance of every stop pair       (router input, CH)
    ../olten/unmapped_schedule_gtfs.xml.gz the Olten case schedule (no dwell)    (router input)

Usage
    uv run --python 3.12 --with osmium --with pyproj --with numpy --with scipy --with shapely \\
        prepare/build_tables.py --out matsim_input/tables

    # only some steps (in dependency order), e.g. to re-check the direction tables
    ... prepare/build_tables.py --only DIR --only H12

    # reproducibility check into a scratch directory
    ... prepare/build_tables.py --out /tmp/rebuild/tables

Every step is deterministic (no randomness, sorted iteration, rounded floats); `<out>/MANIFEST.json`
records the md5 of every output and of every input together with the wall time of each step, so a
re-run can be compared file by file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from prepare import paths as pathmod  # noqa: E402

# (step name, module, needs, outputs) - `needs` is what the step consumes, for the manifest and for
# the input check.  Order matters: it is the dependency order of the chain.
STEPS = [
    ("A01", "prepare.tables.a01_ways", ["osm", "schedule"],
     ["A01/ways.csv", "A01/components.csv", "A01/stops_distance.csv"]),
    ("A02a", "prepare.tables.a02a_direction_geometry", ["osm"],
     ["A02a/direction.csv"]),
    ("A04", "prepare.tables.a04_turns", ["osm"],
     ["A04/turns.csv", "A04/node_degree.csv", "A04/stats.txt"]),
    ("A05", "prepare.tables.a05_blocks", ["osm"],
     ["A05/segment_block.csv", "A05/blocks.csv", "A05/boundaries.csv"]),
    ("A06", "prepare.tables.a06_relations", ["pbf", "schedule"],
     ["A06/relation_ways.csv", "A06/match.csv", "A06/way_usage_direction.csv",
      "A06/relations.csv", "A06/relation_stops.csv"]),
    ("A03a", "prepare.tables.a03a_platform_tracks", ["osm", "schedule"],
     ["A03a/platform_tracks.csv"]),
    ("OFFS", "prepare.tables.offmodel_stops", ["osm", "schedule", "A01", "A03a"],
     ["OFFMODEL/stop_track_gauge.csv"]),
    ("OFFL", "prepare.tables.offmodel_legs", ["osm", "schedule", "OFFS"],
     ["OFFMODEL/leg_distance.csv"]),
    ("H12", "prepare.tables.h12_direction_fusion", ["A01", "A02a", "A04", "A05", "A06", "network"],
     ["H12/direction_fused.csv"]),
    ("DIR", "prepare.tables.dir_direction_final", ["A01", "H12"],
     ["DIR/direction_final.csv"]),
    ("RS1", "prepare.tables.rs1_rolling_stock", ["corrections"],
     ["RS1/vehicles_rs1.csv"]),
]

METHODS = {
    "A01": "pyosmium + pyproj, original_data/osm/switzerland_railways.osm",
    "A02a": "pyosmium + pyproj + numpy + scipy (KD-tree), same OSM extract",
    "A03a": "pyosmium + pyproj + numpy + shapely, OSM extract + unmapped schedule",
    "A04": "pyosmium + pyproj + numpy, same OSM extract",
    "A05": "pyosmium + pyproj + numpy, same OSM extract",
    "A06": "pyosmium + pyproj on original_data/osm/switzerland-latest.osm.pbf (relations!)",
    "OFFS": "pyosmium + pyproj: per stop, real standard-gauge track vs off-model track (F1)",
    "OFFL": "pyosmium + pyproj + A* on the OSM railway graph: real length of every stop pair (F1)",
    "H12": "stdlib only, reads the A01/A02a/A04/A05/A06 tables",
    "DIR": "stdlib only, reads H12 + prepare/corrections/direction_overrides.csv",
    "RS1": "stdlib only, reads prepare/corrections/rolling_stock.csv",
}


def md5(path: Path) -> str | None:
    if not path.exists():
        return None
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--osm", help="railway=* OSM extract (default original_data/osm/switzerland_railways.osm)")
    ap.add_argument("--pbf", help="full OSM extract with relations (default original_data/osm/switzerland-latest.osm.pbf)")
    ap.add_argument("--schedule", help="unmapped whole-Switzerland schedule "
                                      "(default matsim_input/tmp/schedule_gtfs_filtered.xml.gz)")
    ap.add_argument("--network", help="pt2matsim network (default matsim_input/tmp/network_osm.xml.gz)")
    ap.add_argument("--out", help="table directory (default matsim_input/tables)")
    ap.add_argument("--only", action="append", default=[], metavar="STEP",
                    help="run only these steps (repeatable), e.g. --only DIR")
    ap.add_argument("--no-case", action="store_true", help="do not cut the Olten case schedule")
    ap.add_argument("--case", default="olten", choices=["olten"], help="case to cut (default olten)")
    args = ap.parse_args(argv)

    paths = pathmod.configure(osm=args.osm, pbf=args.pbf, schedule=args.schedule,
                              network=args.network, out=args.out)
    print(f"repository  {paths.root}")
    print(f"tables      {paths.out}")
    paths.out.mkdir(parents=True, exist_ok=True)

    todo = [s for s in STEPS if not args.only or s[0] in args.only]
    unknown = set(args.only) - {s[0] for s in STEPS}
    if unknown:
        sys.exit(f"unknown step(s) {sorted(unknown)}; choose from {[s[0] for s in STEPS]}")

    # a step needs its input files to exist: fail early instead of after ten minutes of work
    inputs = {"osm": paths.osm, "pbf": paths.pbf, "schedule": paths.schedule,
              "network": paths.network}
    for name, _mod, needs, _outs in todo:
        for n in needs:
            if n in inputs and not inputs[n].exists():
                sys.exit(f"missing input for {name}: {inputs[n]}")

    timings, manifest = {}, {"inputs": {}, "steps": {}, "outputs": {}}
    for n in {x for _s, _m, needs, _o in todo for x in needs}:
        if n in inputs:
            manifest["inputs"][n] = {"path": str(inputs[n]), "md5": md5(inputs[n])}
    manifest["corrections"] = {
        p.name: {"path": str(p), "md5": md5(p)}
        for p in sorted((paths.root / "prepare/corrections").glob("*.csv"))
    }

    for name, module, _needs, outs in todo:
        print(f"\n=== {name}: {METHODS[name]}", file=sys.stderr, flush=True)
        t0 = time.time()
        mod = __import__(module, fromlist=["run"])
        if hasattr(mod, "run"):
            mod.run()
        dt = time.time() - t0
        timings[name] = dt
        manifest["steps"][name] = {
            "method": METHODS[name],
            "module": module + ".py",
            "seconds": round(dt, 1),
            "outputs": {},
        }
        for rel in outs:
            p = paths.out / rel
            h = md5(p)
            if h is None:
                sys.exit(f"{name}: expected output {p} was not written")
            manifest["steps"][name]["outputs"][rel] = {"md5": h, "bytes": p.stat().st_size}
            manifest["outputs"][rel] = {"md5": h, "bytes": p.stat().st_size}

    if not args.no_case:
        print(f"\n=== case {args.case}", file=sys.stderr, flush=True)
        t0 = time.time()
        from prepare.tables import case_olten
        case_olten.run(args.case)
        dt = time.time() - t0
        timings[f"case:{args.case}"] = dt
        case_path = paths.root / f"matsim_input/{args.case}/unmapped_schedule_gtfs.xml.gz"
        manifest["steps"][f"case:{args.case}"] = {
            "method": "stdlib: polygon of original_data/shp/olten/olten.shp + "
                      "prepare/corrections/olten_case_routes.csv",
            "module": "prepare/tables/case_olten.py",
            "seconds": round(dt, 1),
            "outputs": {str(case_path.relative_to(paths.root)):
                        {"md5": md5(case_path), "bytes": case_path.stat().st_size}},
        }

    dest = paths.out / "MANIFEST.json"
    dest.write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8")

    print("\nstep      seconds   outputs")
    for name, dt in timings.items():
        print(f"{name:<9} {dt:8.1f}   {len(manifest['steps'][name]['outputs'])}")
    print(f"\nmanifest  {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
