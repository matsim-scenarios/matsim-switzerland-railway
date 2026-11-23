# Pipeline — from the original open data to a scored railway simulation

This document describes the chain that is in the repository, step by step, with the command, the
inputs, the outputs and the measured run time of each step.  It is the map for the code in
`prepare/`, `src/main/java/` and `scripts/`.  Both cases — **Olten** (612 departures) and
**Switzerland** (2 541 departures) — come out of the same chain.

Round 3 goal: the published Swiss long-distance timetable, simulated on OSM track geometry, must
work — **every train reaches its last stop** and **every train is on time at every stop**, where
"on time" means `actual arrival − published arrival = 0 s`: a train that arrives 3 minutes early is
as wrong as one that arrives 3 minutes late.

Everything is driven by two scripts:

```bash
scripts/prepare_case.sh olten        # data -> tables -> routing -> dwell -> corrections
scripts/run_case.sh     olten        # MATSim run -> score -> punctuality -> audit
scripts/prepare_case.sh switzerland
scripts/run_case.sh     switzerland
```

Nothing in the chain reads from `swarm/` (the working directories of the round-1…3 agents); that tree
is only cited as the provenance of a data decision, and the one place that still looks at it
(`run_case.sh` uses `swarm/bench/slots` as the slot semaphore *if that directory exists*) has a local
fallback (`matsim_input/.slots`).  Proof: a copy of `src/ prepare/ scripts/ docs/ pom.xml` in a
scratch directory with only `original_data/` and `matsim_input/tmp/` linked in (no `swarm/` in the
tree at all) ran `prepare_case.sh olten` (17-18 min) and `run_case.sh olten` (357 s simulation) end to end — see
`swarm/r3/P3/REPORT.md` §5.

## Overview

```
original_data/gtfs/*.zip ──────────────────┐
                                           ├─(1) pt2matsim: GenerateRailsimInput
original_data/osm/switzerland_railways.osm ┘   ─▶ matsim_input/tmp/schedule_gtfs_filtered.xml.gz (1 231 routes, 2 541 departures)
                                                  matsim_input/tmp/network_osm.xml.gz        (665 238 links)

original_data/osm/switzerland_railways.osm ─┐
original_data/osm/switzerland-latest.osm.pbf ├─(2) prepare/build_tables.py ─▶ matsim_input/tables/
original_data/shp/olten/olten.shp ──────────┘        A01 A02a A03a A04 A05 A06 H12 DIR RS1 OFFS OFFL
                                                     + matsim_input/olten/unmapped_schedule_gtfs.xml.gz
                                           │
                                           ├─(3) rolling stock   tables/RS1/vehicles_rs1.csv ─▶ matsim_input/<case>/vehicles_fs1.csv
                                           ├─(4) build           mvn clean package        ─▶ matsim-switzerland-railway-<ver>.jar
                                           ├─(5) TrackRouter                                 ─▶ matsim_input/<case>/routed/
                                           ├─(6) prepare/fix_reversals.py                    ─▶ routed/ (reversal geometry)
                                           ├─(7) prepare/restore_dwell.py                    ─▶ unmapped_schedule.xml.gz (reference)
                                           │                                                    routed/ (published dwell times)
                                           ├─(8) prepare/verify_dwell.py, scripts/verify_schedule.py   (abort on any difference)
                                           ├─(9) prepare/trackpatch.py     (NL1 passing loops)
                                           ├─(10) prepare/routepatch.py    (RC1 F1/F2/F4/F5)
                                           ├─(11) prepare/fix_stops.py     (SP1 stop positions)
                                           ├─(12) prepare/fix_offmodel_speed.py          (CH1)
                                           ├─(13) prepare/fix_ext_length.py + speed again (CH2, CH only)
                                           ├─(14) prepare/bern_platform.py  (BN4: Bern platform 4 on its real track)
                                           │
                                           ├─(15) scripts/check_mapping.py                   ─▶ mapping_check*.json
                                           │
                                           ├─(16) ch.sbb.run.RunSwitzerlandRailway           ─▶ matsim_input/<case>/output/
                                           │        (+ the patched engine/lock from src/main/java)
                                           │
                                           └─(17) scripts/evaluate.py, punct.py, audit_safety.py, deadlocks.py
                                                    ─▶ score.json, punct.csv, punct.json, audit/
```

## Step 0 — Inputs

| file | what | tracked? |
|---|---|---|
| `original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip` | Swiss GTFS timetable, 2024 annual timetable | no (large, licensed separately) |
| `original_data/osm/switzerland_railways.osm` | OSM railway ways (`osmosis --tf accept-ways railway=* --used-node`) | no |
| `original_data/osm/switzerland-latest.osm.pbf` | full OSM extract (only for the `route=train` relations) | no |
| `original_data/shp/{olten,switzerland}/*.shp` | case boundary polygons | yes |
| `matsim_input/tmp/*.xml.gz` | the two pt2matsim outputs of step 1 | no (`*.gz`) |
| `matsim_input/<case>/{config.xml,run.properties}` | MATSim config + run-class switches (`run.properties` = the reference runs' file) | **yes** |
| `prepare/corrections/*.{csv,json}` | the reviewed, hand-made corrections and the track-assignment specs | **yes** |

## Step 1 — GTFS + OSM into MATSim input (`matsim_input/tmp/`)

`src/main/java/ch/sbb/prepare/GenerateRailsimInput.java` uses pt2matsim to

1. convert the GTFS zip into an unmapped transit schedule, keep `rail` routes whose line name starts
   with `IC/IR/EC/RE` and whose stops are inside `original_data/shp/switzerland/switzerland.shp`
   → `matsim_input/tmp/schedule_gtfs_filtered.xml.gz` (1 231 routes, 2 541 departures, 994 stop
   facilities on 288 stations);
2. convert `original_data/osm/switzerland_railways.osm` into a MATSim network (one link per OSM way
   segment and direction, `keepPaths=true`, LV95 metres, `osm:way:id` on every link)
   → `matsim_input/tmp/network_osm.xml.gz` (665 238 links).

```bash
mvn clean package
java -Xmx8g -cp matsim-switzerland-railway-1.0.0-SNAPSHOT.jar ch.sbb.prepare.GenerateRailsimInput switzerland
```

Verified with JDK 25, MATSim 2027.0-2026w45 and pt2matsim 24.4 using the original Swiss GTFS ZIP
and OSM extract. The regenerated network matched the reference after decompression. The filtered
schedule XML matched after sorting the transfer relations. All 2,541 departure signatures and
21,243 scheduled arrivals matched. Pass `--out DIR` to compare against existing inputs in a separate directory.
The project now uses pt2matsim 26.12.1. With the same inputs, it writes a network and filtered schedule
identical to those from 24.4 after decompression.

**Optional alternative to steps 2–15:** `ch.sbb.prepare.platform.GeneratePlatformMappedInput` maps
the step-1 schedule with pt2matsim's platform-aware mapper. The GTFS platform code goes into
the pt2matsim platform-code attribute of each stop facility, and stops are placed on OSM platform links. The output goes to
`matsim_input/platform/` and is run with `matsim_input/platform/config.xml`. This path produces a
simpler, uncorrected input with no track assignment, corrections or the checks from steps 2–15. The
default pipeline does not use it.

**pt2matsim 24.4 loses the station dwell times** (`GtfsConverter.createTransitRouteStop` takes the
*arrival* time for both offsets).  That is why the pipeline restores them in step 7 —
`restore_dwell.py` reads the times from the GTFS zip, not from the converter.

## Step 2 — Static tables (`prepare/build_tables.py`)

One entry point, one module per table in `prepare/tables/`.  Reviewed, hand-made corrections live in
`prepare/corrections/*.csv` (see `docs/assumptions.md`).

```bash
uv run --python 3.12 --with osmium --with pyproj --with numpy --with scipy --with shapely \
    prepare/build_tables.py --out matsim_input/tables            # add --no-case for Switzerland
```

| step | module | reads | writes (→ `matsim_input/tables/`) | time |
|---|---|---|---|---|
| A01 | `a01_ways.py` | OSM extract, unmapped schedule | `A01/ways.csv` (65 016 ways), `A01/components.csv`, `A01/stops_distance.csv` | 25 s |
| A02a | `a02a_direction_geometry.py` | OSM extract | `A02a/direction.csv` (17 762 candidate ways) | 16 s |
| A04 | `a04_turns.py` | OSM extract | `A04/turns.csv`, `A04/node_degree.csv`, `A04/stats.txt` | 13 s |
| A05 | `a05_blocks.py` | OSM extract | `A05/segment_block.csv`, `A05/blocks.csv`, `A05/boundaries.csv` | 17 s |
| A06 | `a06_relations.py` | OSM **pbf**, unmapped schedule | `A06/relation_ways.csv`, `A06/match.csv`, `A06/way_usage_direction.csv`, … | 4 s warm |
| A03a | `a03a_platform_tracks.py` | OSM extract, unmapped schedule | `A03a/platform_tracks.csv` (994 facilities → OSM track) | 24 s |
| OFFS | `offmodel_stops.py` | OSM extract, A01, A03a, schedule | `OFFMODEL/stop_track_gauge.csv` (real vs off-model track per stop) | 13 s |
| OFFL | `offmodel_legs.py` | OSM extract, OFFS, schedule | `OFFMODEL/leg_distance.csv` (real distance of 2 021 stop pairs) | 298 s |
| H12 | `h12_direction_fusion.py` | A01, A02a, A04, A05, A06 | `H12/direction_fused.csv` + evidence tables | 9 s |
| DIR | `dir_direction_final.py` | H12, `corrections/direction_overrides.csv` | `DIR/direction_final.csv` — **router input** | 1 s |
| RS1 | `rs1_rolling_stock.py` | `corrections/rolling_stock.csv` | `RS1/vehicles_rs1.csv` — **run input** (= the reference runs' `vehicles_fs1.csv`, byte-identical) | <1 s |
| case | `case_olten.py` | unmapped schedule, `shp/olten/olten.shp`, `corrections/olten_case_routes.csv` | `matsim_input/olten/unmapped_schedule_gtfs.xml.gz` | 1 s |

Total **420 s** (7 min) warm.  `matsim_input/tables/MANIFEST.json` records the md5 of every input,
every correction table and every output plus the wall time of every step, so a re-run anywhere can
be compared file by file.  The builders are deterministic: a complete re-run reproduces **every**
CSV byte for byte, including the two off-model tables (OFFS md5 `fe357579…`, OFFL `a85bbb5a…`,
identical to the files the reference runs used).

Generated data and its size (all untracked, `.gitignore`):

| path | size | tracked? |
|---|---|---|
| `matsim_input/tables/` (incl. A06 caches) | 59 MB | no |
| `matsim_input/tmp/*.xml.gz` | 700 MB (network), 15 MB (schedule) | no |
| `matsim_input/<case>/unmapped_schedule*.xml.gz` | 0.2 MB (Olten), 15 MB (Switzerland) | no |
| `matsim_input/<case>/routed/` | 75 MB (Olten), 200 MB (Switzerland) | no |
| `matsim_input/<case>/work/` | the numbered intermediates of steps 6-14 | no |
| `matsim_input/<case>/output/` | ~0.4 GB (Olten), ~1.1 GB (Switzerland); `run_case.sh` deletes `ITERS/` and `tmp/` afterwards | no |

## Step 3 — Rolling stock (`matsim_input/<case>/vehicles_fs1.csv`)

`prepare/corrections/rolling_stock.csv` holds one row per line-name prefix (length, vmax, mean
acceleration, deceleration, stock, source).  Round 3 (agent FS1) replaced the **acceleration** by the
mean acceleration from 0→100 km/h of the constant-force/constant-power model, with the per-series
parameters (a0, mass, power) in a `source_acceleration` column; the RS1 builder projects the table
onto the five columns the run class reads, and the pipeline writes it as
`matsim_input/<case>/vehicles_fs1.csv` (the name `run.properties` uses).  The generated file is
**byte-identical** to the round-3 reference table (`md5 499eb6de…`).

## Step 4 — The jar (`mvn clean package`)

One shaded jar in the repository root (~111 MB), `Main-Class: ch.sbb.run.RunSwitzerlandRailway`.
The patched railsim classes in `src/main/java` must be the only copies on the classpath: the shade
configuration filters the railsim artifact's own `RailsimEngine*.class`, `RailsimCalc*.class` and
`PunctualProfileHook*.class`.  Measured: 18-24 s (offline, `~/.m2`).  The engine logs
`SWARM-FIX RailsimEngine active` at start-up and `run_case.sh` greps for that line.

## Step 5 — Track assignment (`ch.sbb.prepare.router.TrackRouter`)

The router assigns every transit route to physical OSM tracks: it minimises a cost over the legal
track graph, using the running direction (DIR), the block/signal table (A05), the legal turns (A04),
the platform tracks (A03a) and the OSM route relations (A06), and it inserts `boundaryExtension`
links where a route leaves the modelled network.  Round-2 additions that are part of it:

* **passing loops** (`Loops.java`, `--loops true --loopPenalty 6.0`): at a loop the two directions
  get different loop tracks, decided from the link usage of a first routing pass;
* **corridor rule R7c** (`--r7cSameLine 0.60 --r9pin true`): a leg of ≥ 20 km whose free-flow time is
  under 60 % of the booked time is re-routed to the slowest path that still fits the booked time, but
  only along the same OSM line (Gotthard mountain line vs base tunnel), and R9 may not re-negotiate
  such a leg;
* **off-model track** (`--stopClass`, `--legDist`, Switzerland case only): metre-gauge and foreign
  track becomes unlimited-capacity links with a realistic length.

```bash
java -Xmx10g -cp matsim-switzerland-railway-<ver>.jar ch.sbb.prepare.router.TrackRouter \
  --schedule matsim_input/<case>/unmapped_schedule_gtfs.xml.gz \
  --network  matsim_input/tmp/network_osm.xml.gz \
  --data     matsim_input/tables --out matsim_input/<case>/routed \
  --loops true --loopPenalty 6.0 --r1 true … --r9 true --r9pin true --maxIter 8 \
  --relBonus 0.85 --dirPenalty 8 --dirEntry 120 --dubious true --softTurn 0 --corridorGain 0
```

Output: `transitSchedule_router.xml.gz`, `transitNetwork.xml.gz`, `transitVehicles.xml.gz` plus the
diagnostics `loops.csv`, `r7c.csv`, `offmodel.csv`, `offmodel_legs.txt`, `legs.csv`, `legs_ids.csv`,
`route_links.csv`, `routing_report.csv`.  Measured: **497 s** (Olten, 8 threads) / **1 796 s**
(Switzerland, 6 threads).  Deterministic and text-identical to the round-2 deliverables
(decompressed md5 of the Olten schedule `6b22828b…`, network `fb6dff72…`, vehicles `4c5de076…`).

`scripts/check_mapping.py` scores the assignment statically, **against the router's own network**
(the corrections below add and split links): Olten `invalid_routes 0`, `illegal_turns 0`,
`reversals_midroute 0`, `wrongdir_km_pct 0.22`, `platform_ok_pct 99.58`, `train_km 102 495.8`,
`artificial_km 0`.

## Step 6 — Honest reversal geometry (`prepare/fix_reversals.py`)

A reversing train's body is rebuilt by the engine onto the departure route.  If the departure does
not retrace the arrival links under the whole body, the body stands on track the train never reserved
(Bern was the measured case).  The post-processing rebuilds those departures — only the `<route>`
link list changes, never a stop or a time.  The router output and the router network are kept as
`transitSchedule_router.xml.gz` / `transitNetwork_router.xml.gz` for the static check.

Measured: Olten **16 s**, 118 reversal route-stops, 93 routes changed (+6 460 m, max +194 m);
Switzerland **36 s**, 173 reversal route-stops, 97 routes changed (+3 382 m), 9 routes reverted as
still unsafe (the IC51 at Delémont: 149.6 m platform for a 377.6 m train).

## Step 7 — The published station dwell times (`prepare/restore_dwell.py`)

pt2matsim 24.4 wrote `arrivalOffset == departureOffset` at every stop (both from the GTFS arrival
time), so 11 194 (CH) / 3 400 (Olten) published dwells were missing: a simulated train left a
station at its *arrival* minute, ran ahead of the timetable and stood on track a punctual train
needed.  The step rewrites only the two offset values (everything else stays byte-identical) and
runs twice:

```bash
python3 prepare/restore_dwell.py matsim_input/<case>/unmapped_schedule_gtfs.xml.gz \
    original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip matsim_input/<case>/unmapped_schedule.xml.gz
python3 prepare/restore_dwell.py matsim_input/<case>/work/0_reversals_schedule.xml.gz \
    original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip matsim_input/<case>/work/1_dwell_schedule.xml.gz
```

The first is the **scoring reference** of the case, the second is the simulated schedule.  New
offsets: `GTFS arrival − GTFS departure of stop 1` / `GTFS departure − GTFS departure of stop 1`;
first stop 0/0, last stop departure = arrival (no trip of the feed has a dwell there).  Route
splitting is implemented (one route per distinct pattern, ids `<id>__d2`…) and was **not needed**:
0 of 1 231 (CH) / 0 of 361 (Olten) routes had departures with different patterns.  Measured: 51 s
(Olten) / 70 s (CH) for both runs together (one streaming pass over `stop_times.txt` per run).

## Step 8 — Timetable checks (`prepare/verify_dwell.py`, `scripts/verify_schedule.py`)

```bash
python3 prepare/verify_dwell.py <schedule> <gtfs.zip> [--old <old schedule>]   # exit 1 on a difference
python3 scripts/verify_schedule.py <mapped> <reference>                        # SIGNATURE IDENTICAL
```

`verify_dwell` compares the absolute times of every departure and every stop with the GTFS (no
tolerance: `departureTime + offset == arrival_time / departure_time`), and with `--old` also proves
that *only* the two offset values changed.  `verify_schedule` compares the mapped schedule with the
reference using the scorer's own signature (departure id, start time, stop sequence by parent
station, arrival **and** departure offsets) and exits non-zero on any difference.  The preparation
aborts if either fails: no train, stop or time can drift.

## Step 9 — Passing loops from real OSM tracks (`prepare/trackpatch.py`)

`prepare/corrections/second_tracks.json` lists five real OSM second tracks the router did not use —
Courtemaîche (the loop of round 2, repaired), Courfaivre, St-Ursanne, Malters,
Les Geneveys-sur-Coffrane.  For each, the way's links are added in both directions and the selected
*physical* direction is routed over the loop, with its stop moved onto the loop link (`<uic>:0:<track>
.link:<link>`; these stops have no published platform number).  One directed link per track made the
whole Neuchâtel–La Chaux-de-Fonds line a single exclusive section (19.70 km, 16 836 s denied); with
the loops the Neuchâtel–LCF corridor went from 26 469 late-seconds to 0.

`--check` validates every route chain, every stop link and the timetable afterwards.  Measured:
CH 35 routes / 120 departures / 150 links added, 0 routes (Olten, only the unused links are added);
57 s (Olten) / 107 s (CH).  The tool is agent RC2's implementation of agent NL1's specs; with NL1's job order it
reproduces NL1's accepted output **byte for byte** (checked in `swarm/r3/P3/REPORT.md` §3).

## Step 10 — Route patches F1/F2/F4/F5 (`prepare/routepatch.py`)

Declarative, streamed patches of the mapped schedule + network (`prepare/corrections/f*.json`):

* **F1** splits the two Heitersberg tunnel bore links (3 306 m, `railway:etcs=1`, one track per
  direction) into 6 block sections of 551 m, so the IC1/IC5 pair booked 25-66 s apart can follow;
* **F2/F4/F5** give selected routes their own copy of a link run at Olten–Aarau, Bern Wylerfeld, the
  Bern platform throat and Aarburg (tracks that exist in OSM; the copies are copies of the existing
  link chain, offset 9-27 m — a documented limitation, `docs/assumptions.md` K3);
* **F3** (Olten platform 9) is **not** applied: it deadlocks at Olten.

Measured: 13 s (Olten) / 32 s (CH); 157 of 361 (Olten) / 512 of 1 231 (CH) routes touched.  Planned
trajectory conflicts (round-3 tool of agent PM1): Olten 163 → 65, Switzerland 573 → 470.

## Step 11 — Stop positions (`prepare/fix_stops.py`)

Stop links are 13-46 m long while trains are 150-378 m, so a train stood with its head at the END of
the stop link and its body reaching back into the station throat.  The step moves every route-stop
that is not a first/last stop forward inside its **own platform track** (bounded by switches and
reversals) to the first link whose end lets the whole body stand inside with 10 m clearance; never
another platform, never past the next stop, timetable and platform share untouched.  Measured: 478
route-stops moved (CH, 117 new facilities), 61 new facilities (Olten), 16 s (Olten) / 35 s (CH); body behind the
platform entry switch 67 771 → 9 153 m (dwell-weighted −84 %).

## Step 12 — Off-model link speeds (`prepare/fix_offmodel_speed.py`)

Off-model (`ext_*`) links got their freespeed from an *arrival→arrival* hop time, i.e. including the
station dwell: Freiburg → Schallstadt was driven at 28 km/h for a leg booked at 120 km/h.  The step
takes the published **running** time (`departureOffset[k] − arrivalOffset[k+1]`) and solves for the
cruise speed at which a vehicle with the line's acceleration/deceleration covers the leg's off-model
links in 0.85 × that time (trapezoidal profile, cap 160 km/h).  Speeds are only ever **raised**: a
link that is already fast enough is left alone, so a stop link is never slowed down.  Measured: CH 19
links raised (all at the cap), Olten 1 link; 6 s (Olten) / 14 s (CH).  `offmodel_speed_fix.csv` lists every changed
link.

## Step 13 — Off-model hop lengths, Switzerland only (`prepare/fix_ext_length.py`)

`TrackRouter.offLength()` gives a foreign hop without a known distance the length *straight line ×
1.25* (402 of the 880 `ext_*` links, 3 784 route-km); the case's own 7 074 modelled hops measure a
median **1.096** (p10 1.02, p90 1.33).  1.25 has no source and only bites where the speed cap binds:
CH1's rule divides the length by 0.85 × booked time with a 160 km/h cap, so an inflated length pushes
the required speed over the cap.  The step sets those 402 lengths to 1.096 × straight line
(3 784 → 3 318 km), then step 12 is re-run on the corrected lengths (19 further links raised, all at
the cap).  Measured: 22 s for both, static infeasible legs 1 090 → 913, Switzerland
`abs_dev_seconds` 109 684 → 103 115 on base11.  **Olten is untouched by construction** (3 `ext_*`
links, none a 1.25 placeholder) and the step is skipped for that case.

## Step 14 — Bern platform 4 on its real track (`prepare/bern_platform.py`, agent BN4)

The case networks are built from `matsim_input/tmp/network_osm.xml.gz` with the links no route uses
dropped.  At Bern platform 4 that removed the western part of the **real 535.0 m track between the
switches** (OSM ways 65596810 + 396181885 + 98445139): the modelled platform track was 93.0 m, so a
150–288 m train stopped with its head 93 m from the eastern switch and its body hung 94–195 m into
the northern throat, where every IC1/IC8/IR15 arrival must pass.  All 61 late Bern arrivals of the
reference run were held by trains standing at platform 4 (35 of them by the 40.7 m single-track lead
`464609`).  The step

1. adds the missing **real** OSM links `1871076911 ↔ 9586428220` (365.0 m, 18 links and 9 nodes) to
   the network, copied verbatim from `network_osm.xml.gz` (id, from/to, length, freespeed, OSM
   attributes, coordinates) — nothing is invented;
2. inserts them into the route of every train that **stands** at platform 4 and reverses there (the
   arrival continues west along the same platform, the departure retraces), and moves those 30
   route-stops to the western end (`8507000:0:4.link:549958`): the head now stops 457.7 m from the
   eastern switch, so the body — and the engine's lagging tail pointer — stay between the switches;
3. leaves first/last stops (the train never stands) and every other stop alone.

`--depth 2` is the depth BN4 measured and shipped (a stop 297 m from the eastern switch deadlocks
through the lagging tail pointer, `swarm/r3/BN4/REPORT.md` §5).  Timetable, platform share and stop
order are unchanged (`verify_dwell` PASSED, `planconf` identical to base12).  Measured: **44 s**;
`abs_dev_seconds` Olten 65 184 → 47 817 (−26.7 %), Bern 32 644 → 18 291 (−44 %), Fribourg
3 442 → 433; Switzerland (with step 13) 99 392 → 92 828 (−6.6 %), Bern 35 102 → 32 705.

## Step 15 — Static quality (`scripts/check_mapping.py`)

Checked twice: the **router output against the router's own network** (the corrections split and add
links, so it cannot be checked against the final network) and the **final schedule against the final
network**.  Olten (router output): `invalid_routes 0`, `illegal_turns 0`, `platform_ok_pct 99.58`,
`train_km 102 495.8`.  Olten (final): `invalid_routes 0`, `platform_ok_pct 99.58`, `train_km
102 526.1`, `illegal_turns 332` — the checker's turn table does not know the reversal retraces and
the parallel-track copies of step 10, exactly as in round 2 (a property of the static checker, not of
the schedule; the simulation and the physical audit are the real test).

## Step 16 — Simulation (`ch.sbb.run.RunSwitzerlandRailway`)

The upstream run class of this repository, extended with a `run.properties` switchboard (it behaves
like the stock class when there is no properties file).  The reference runs' file:

```
endTime=36:00:00            # the last train arrives after 24 h of a service day
vehicles=realistic          # per-line rolling stock from vehicles_fs1.csv
pollInterval=5              # railsim update step [s]
deadlockAvoidance=sectionLock
boundaryCapacity=1000       # Switzerland only: off-model links (metre gauge, abroad)
vehicles.file=vehicles_fs1.csv
sectionLock.stopRule=safe   # a section never ends at a stop used in both directions
sectionLock.dedup=on        # one claim per physical track per section
sectionLock.meet=on         # meet rule at two-track stations
speedProfile=punctual       # just-in-time driving (round 3)
punctual.aim=0.25
punctual.minFactor=0.0
sectionLock.reversalStop=standingArea   # a reversal stop releases the station box (round 3)
sectionLock.retracePass=on
sectionLock.retracePassMargin=120
sectionLock.retracePassAhead=on          # a crossing may also end before its own stop (round 3, agent P4)
engine.speedRefresh=on                   # CH1: the speed cap of a left link is re-evaluated
engine.reachTime=on                      # EN1: the time to a distance includes the cruise phase
engine.headGuard=on                      # EN1: the head only enters a link it has really reached
engine.reversalEnter=off                 # EN1/BN4: the reversal guard, measured as not needed
```

The classes that must win over the railsim jar:

* `src/main/java/ch/sbb/matsim/contrib/railsim/qsimengine/{RailsimEngine,RailsimCalc,
  PunctualProfileHook}.java` — same fully qualified names as the jar classes, patched.  Every hunk is
  marked `SWARM-FIX`; `swarm/r3/src9/CHANGES.md` lists them (reversal releases its arrival-link
  reservations; approved distance cannot go negative; tail pointer no longer lags; the speed cap of a
  left link is re-evaluated; no phantom window stop; a reversing train keeps its platform lead), and
  round 3, agent EN1, added the two fixes that make the tail pointer exact: `engine.reachTime` (the
  time needed for a distance after a deceleration phase includes the cruise phase at the target
  speed) and `engine.headGuard` (`enterLink` moves the head onto the next link only when the head
  really stands at the end of the current one; before that the train teleported up to 500 m forward
  and its recorded body grew to 301 m (Olten) / 657 m (CH) instead of 100 % exact).
* `src/main/java/ch/sbb/run/PunctualSpeedProfile.java` — the just-in-time controller of step 15
  (`speedProfile=punctual`), and `SectionLockDeadlockAvoidance.java` — the section lock with the meet
  rule, the reversal-stop standing area, the time-bounded retrace claim (`sectionLock.retracePass`)
  and its extension for a crossing that ends before its own stop
  (`sectionLock.retracePassAhead`, agent P4).
* `InterlockingDeadlockAvoidance.java`, `JustInTimeSpeedProfile.java` — alternatives selectable from
  `run.properties`, not used by the reference runs.

```bash
HEAP=8g  scripts/run_case.sh olten          # 612 trains, ~10-12 min
HEAP=12g scripts/run_case.sh switzerland    # 2 541 trains, ~20-25 min
```

The per-case `config.xml` has its three input paths pointing at `routed/` (`transitScheduleFile`,
`vehiclesFile`, `inputNetworkFile`) and `runId` `ch_olten` / `ch_switzerland` (that only names the
output files).  `run_case.sh` takes a slot from the same semaphore as the project bench when that
directory exists (`swarm/bench/slots`, limit 7), runs the jar in `matsim_input/<case>/`, then scores,
measures the punctuality, audits and writes the deadlock report.  It deletes `output/ITERS/` and
`output/tmp/` afterwards.

## Step 17 — Scoring, punctuality, audit

```bash
python3 scripts/evaluate.py <schedule> <events> --ref matsim_input/<case>/unmapped_schedule.xml.gz --json score.json
python3 scripts/punct.py matsim_input/<case> --ref matsim_input/<case>/unmapped_schedule.xml.gz
python3 scripts/audit_safety.py --exp matsim_input/<case> --out matsim_input/<case>/audit
python3 scripts/deadlocks.py matsim_input/<case>            # only useful when trains do not finish
```

`evaluate.py --ref` compares the scored schedule with the published one (same departures, same stop
sequence by parent station, same arrival **and** departure times) and marks the run `INVALID`
otherwise; it also reports the deviation statistics.  `punct.py` is the round-3 metric: per scheduled
stop the deviation of the arrival **and** the departure, the three tolerances (`|dev| ≤ 1/5/30 s`),
late and early counts, the worst early/late arrival, and `departures_early` (a train that leaves
before its published departure makes the run invalid).  Its last line is the headline
`abs_dev_seconds = Σ|deviation|` (a missing arrival counts as 3 600 s).  `audit_safety.py` is the
independent physical-safety audit (two bodies on one physical track, a body on an unreserved track,
stale reservations, teleports); off-model resources are exempt when `boundaryCapacity > 1`.

## Measured reference runs (this layout)

| step | Olten | Switzerland |
|---|---:|---:|
| `prepare/build_tables.py` (all tables + case schedule) | 392 s | 335 s |
| rolling stock + `mvn clean package` | 18 s | 17 s |
| `TrackRouter` | 324 s | 1 297 s |
| `prepare/fix_reversals.py` | 10 s | 25 s |
| `prepare/restore_dwell.py` (two runs) | 51 s | 70 s |
| `verify_dwell` (3x) + `verify_schedule` | 56 s | 71 s |
| `prepare/trackpatch.py` (passing loops) | 57 s | 107 s |
| `prepare/routepatch.py` (F1/F2/F4/F5) | 13 s | 32 s |
| `prepare/fix_stops.py` | 16 s | 35 s |
| `prepare/fix_offmodel_speed.py` | 6 s | 14 s |
| `prepare/fix_ext_length.py` + speed again | – | 22 s |
| `prepare/bern_platform.py` (BN4) | 3 s | 3 s |
| `check_mapping.py` (2x) + final checks | 30 s | 90 s |
| **`scripts/prepare_case.sh` total** | **16 min** | **35 min** |
| MATSim/railsim (`RunSwitzerlandRailway`) | **349–394 s** (8 GB) | **783–956 s** (12 GB) |
| `evaluate.py` + `punct.py` | 149 s | 382 s |
| `audit_safety.py` + `deadlocks.py` | 458 s | 959 s |
| **`scripts/run_case.sh` total** | **11–17 min** | **30–38 min** |

Machine: 16 cores / 32 GB, JDK 25, `uv` for the Python builders.  The router is the only thread-using
step (12 threads for Switzerland); MATSim/railsim is single-threaded, so the pipe is I/O-bound in the
scoring and audit steps, which stream the 0.4 GB (Olten) / 1.2 GB (Switzerland) events file.

## Results

Final state of round 3 = `base12` data + CH2's off-model length fix (Switzerland) + BN4's Bern
platform-4 fix, produced by `scripts/prepare_case.sh` and measured by `scripts/run_case.sh`.  The
sources in `src/main/java` carry agent EN1's engine fix and agent P4's extension of the retrace claim
(`run.properties`: `engine.reachTime=on`, `engine.headGuard=on`, `engine.reversalEnter=off`,
`sectionLock.retracePassAhead=on`):

The table uses the service-day reruns after correcting reversal geometry and speed lookahead.
The national total fell from 73,530 to 73,513 seconds of absolute arrival deviation. Olten
reproduced the original total with Java assertions enabled.

| case | trains finished | stop arrivals judged | `abs_dev_seconds` | ≤ 1 s | ≤ 5 s | ≤ 30 s | late > 0 / 30 / 180 s | early > 1 s | worst early / late | `departures_early` |
|---|---:|---:|---:|---:|---:|---:|---|---:|---|---:|
| **Olten** | **612 / 612** | 4 847 / 4 847 | **43 027** | **88.49 %** | 95.15 % | 97.28 % | 1 076 / 132 / 99 | **0** | 0 s / +747 s | **0** |
| **Switzerland** | **2 541 / 2 541** | 18 702 / 18 702 | **73 513** | **87.02 %** | 94.10 % | 98.53 % | 3 923 / 274 / 98 | **0** | 0 s / +842 s | **0** |

(4 847 = 5 459 stops − 612 first stops, 18 702 = 21 243 − 2 541: the first stop of a run has no
arrival to judge.)  Both runs: 0 missing arrivals, `valid: true` in `score.json`, 0 deadlocks
(`deadlocks.py`: 612/2541 completed, 0 stalled, 0 cycles), physical audit 0 body overlaps of 83
(Olten) / 286 (Switzerland) candidate windows, 0 unreserved-body samples (885,456 Olten samples and 2,486,624 Switzerland samples), 0 teleports,
recorded tail within 5 m of an independent reconstruction in **100.00 %** of the state events.  No
arrival of either case is more than 1 s early. The national overlap and reservation checks exclude 880 off-model links on 693 high-capacity boundary resources.

Without the last two steps (they are pure dispatching/engine changes, the data is unchanged): the
same pipeline with base12's sources measured 47 817 (Olten) and 92 828 (Switzerland, P3), with EN1's
engine fix only 48 146 / 92 699 — see `swarm/r3/P4/REPORT.md` §1-§3 for those runs and the row-by-row
comparison.

Reference comparison (round-3 agents, `swarm/r3/P3/REPORT.md` §1-§3 and `swarm/r3/EN1/REPORT.md` §4):

| run | case | trains | `abs_dev_seconds` | ≤ 1 s | late > 180 s | `departures_early` |
|---|---|---:|---:|---:|---:|---:|
| this pipeline | Olten | 612/612 | 43 027 | 88.49 % | 99 | 0 |
| `swarm/r3/exp/BN4-olten` (base12 + BN4, base12 sources) | Olten | 612/612 | 47 817 | 89.87 % | 108 | 0 |
| `swarm/r3/exp/EN1-olten` (same + EN1's engine fix) | Olten | 612/612 | 48 146 | 88.53 % | 107 | 0 |
| this pipeline | Switzerland | 2 541/2 541 | 73 513 | 87.02 % | 98 | 0 |
| `swarm/r3/exp/P3-ch-ref` (base12 + CH2 + BN4) | Switzerland | 2 541/2 541 | 92 828 | 87.61 % | 123 | 0 |
| `swarm/r3/exp/EN1-ch` (same + EN1's engine fix, no CH2 fix upstream) | Switzerland | 2 541/2 541 | 99 214 | 85.85 % | 122 | 0 |

The 8 IC8 at Bern that lost 758 s each in the Olten case (and 17 RE7 + 8 IC8 in Switzerland) are the
retrace-claim fix of agent P4 (`swarm/r3/P4/REPORT.md` §2): 8 of the 8 IC8 arrive within 3 s now and
the 17 RE7 arrive on time; eight following trains pay +53…+174 s at Olten for it (net
−5 119 s Olten, −19 169 s Switzerland).

Determinism: the case runs were compared row by row against the round-3 reference runs and reproduce
them exactly where the settings are identical — the Olten run with only EN1's engine fix reproduces
`swarm/r3/exp/EN1-olten` (`abs_dev_seconds=48146`, `exact1s=88.53%`, `late0=1074 late30=132
late180=107`, `departures_early=0`, same tail/overlap audit), and the same pipeline with base12's
sources reproduces P3's 47 817 / 92 828.  Each case was then run a second time with the retrace-claim
extension (`abs_dev_seconds=43027` Olten, `73530` Switzerland); the two Olten runs took 349 s and
353 s of simulation time.  The preparation is deterministic (P3 §2-§3: every step reproduces the
round-3 tools' output **byte for byte**, md5 list there).

## Where a new dispatching result is edited in

A new lock, engine or speed-profile result changes **three places only**:

1. `src/main/java/ch/sbb/run/…` or `.../qsimengine/…` (the lock/engine/controller sources) — a plain
   file replacement, then `mvn clean package`;
2. `matsim_input/<case>/run.properties` if a new switch is used (and `docs/assumptions.md` §6);
3. the results tables: "Results" in `README.md` and "Results" here, plus the audit line.

No data step has to be re-run for a dispatching change.  A new **data** correction is a new
`prepare/*.py` step plus its spec/table in `prepare/corrections/`, inserted into
`scripts/prepare_case.sh` at the place of the round-3 steps (9-13) — and the whole preparation has to
be re-run for both cases.
