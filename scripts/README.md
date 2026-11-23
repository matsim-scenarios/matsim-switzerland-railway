# Tools

Round-3 tools in `scripts/` (scoring, verification, audit) and `prepare/` (data steps).  Only the
input paths and defaults differ from the originals named in each file header; the metrics and the
data transformations are unchanged.

| script | what it does | usage |
|---|---|---|
| `evaluate.py` | reference scorer: completion, stop completion, deviation distribution (arrival **and** departure), `valid` flag against a reference schedule | `python3 scripts/evaluate.py <transitSchedule.xml.gz> <events.xml.gz> [--ref refSchedule.xml.gz] [--json out.json]` |
| `punct.py` | **round-3 metric**: per stop arrival and departure, `deviation = actual − published`; `abs_dev_seconds`, `|dev| ≤ 1/5/30 s`, late/early counts, worst early/late arrival, `departures_early`; writes `punct.csv` + `punct.json` | `python3 scripts/punct.py <run-dir> [--ref published.xml.gz] [--top 30]` |
| `late.py` | round-2 metric, kept for comparison: late-arrival statistics with the tolerances `>0 s`, `>60 s`, `>180 s`, max delay, late-seconds; writes `late.csv` + `late.json` | `python3 scripts/late.py <run-dir> [--top 30]` |
| `check_mapping.py` | static quality of a mapped schedule (invalid routes, artificial km, reversals, illegal turns, wrong-direction km, platform match) — run it on the router output **against the router's own network** | `python3 scripts/check_mapping.py <transitSchedule.xml.gz> <transitNetwork.xml.gz> [--json out.json]` |
| `verify_schedule.py` | proves that a mapped schedule is the same timetable as the unmapped reference (same departures, stop sequence, arrival **and** departure times); exits 1 on any difference | `python3 scripts/verify_schedule.py <mapped.xml.gz> [<reference.xml.gz>]` |
| `audit_safety.py` | independent physical-safety audit of a finished run (two bodies on one track, reservations without a body, teleport/acceleration plausibility, platform occupancy); writes `results.json` + `cache/` into `--out` | `python3 scripts/audit_safety.py --exp <run-dir> [--out <run-dir>/audit]` |
| `deadlocks.py` | who blocks whom, cycles, first blocked train (only useful when trains do not finish) | `python3 scripts/deadlocks.py <run-dir> [--top 20]` |

Data steps of the pipeline, in `prepare/` (each one re-runnable on its own; header = provenance):

| script | step (docs/pipeline.md) | usage |
|---|---|---|
| `restore_dwell.py` (+ `dwlib.py`) | 7 — the published GTFS station dwell times | `python3 prepare/restore_dwell.py <schedule.xml.gz> <gtfs.zip> <out.xml.gz>` |
| `verify_dwell.py` | 8 — the schedule *is* the GTFS timetable (arrival and departure, no tolerance) | `python3 prepare/verify_dwell.py <schedule.xml.gz> <gtfs.zip> [--old <old.xml.gz>]` |
| `trackpatch.py` | 9 — passing loops from real OSM tracks (`corrections/second_tracks.json`) | `python3 prepare/trackpatch.py <sched-in> <net-in> <sched-out> <net-out> <spec.json …> [--check]` |
| `routepatch.py` | 10 — route patches F1/F2/F4/F5 (`corrections/f*.json`) | `python3 prepare/routepatch.py <sched-in> <net-in> <sched-out> <net-out> <spec.json …>` |
| `fix_stops.py` (+ `stoplib.py`) | 11 — stop positions inside the platform track | `python3 prepare/fix_stops.py <sched-in> <net-in> <sched-out> <net-out> --vehicles <csv> [--clearance 10]` |
| `fix_offmodel_speed.py` | 12 — off-model link speed from the published running time | `python3 prepare/fix_offmodel_speed.py <schedule> <net-in> <net-out> [--vehicles <csv>]` |
| `fix_ext_length.py` | 13 — off-model hop length 1.096 × straight line (Switzerland) | `python3 prepare/fix_ext_length.py <net-in> <net-out> [--factor 1.096]` |
| `fix_reversals.py` | 6 — honest reversal geometry (round 2) | `python3 prepare/fix_reversals.py --exp <routed-dir> --out <sched.xml.gz>` |
| `build_tables.py` (+ `tables/`) | 2 — the static tables and the Olten case schedule | `uv run --python 3.12 --with … python prepare/build_tables.py --out matsim_input/tables` |

`check_mapping.py` reads the direction table from `matsim_input/tables/DIR/direction_final.csv`;
override the location with `RAIL_TABLES=/path/to/tables`.

Driver scripts (both write only into `matsim_input/`, nothing into `swarm/`):

* `scripts/prepare_case.sh <olten|switzerland>` — steps 1–14: static tables + case schedule →
  rolling stock → `mvn clean package` → track assignment (`ch.sbb.prepare.router.TrackRouter`) →
  reversal post-processing → station dwell times → timetable checks → passing loops → route patches
  → stop positions → off-model speeds → off-model lengths (CH only) → `check_mapping.py`.
* `scripts/run_case.sh <olten|switzerland>` — step 15–16: MATSim run through the jar (slot-limited,
  same semaphore as the project bench) → `evaluate.py` → `punct.py` → `audit_safety.py` →
  `deadlocks.py`.

Requirements: JDK 25 and Maven on PATH (or set `MVN` to the Maven executable), `uv` for the Python builders.
