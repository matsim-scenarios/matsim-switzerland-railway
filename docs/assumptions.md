# Modelling assumptions

Every place where the simulation deliberately departs from "raw open data taken literally", with what,
where (file / option), why it is justified, and how many trains it affects.  The published timetable
(GTFS **arrival and departure** times, platform tracks) is ground truth: no train, stop or time is
removed, retimed or shortened.  `scripts/prepare_case.sh` proves that after every preparation
(`prepare/verify_dwell.py`: 0 differences against the GTFS; `scripts/verify_schedule.py`:
`SIGNATURE IDENTICAL`, 612 / 2 541 departures, 5 459 / 21 243 stop arrivals and departures).

Metric: round 3 measures `deviation = actual arrival − published arrival` at **every** stop and wants
**0 s**.  An early train is as wrong as a late one.  A run is invalid if any train departs a stop
before its published departure time (`punct.py`: `departures_early` must be 0).

Cases: **Olten** (612 departures, 5 459 scheduled stop arrivals, each train simulated over its whole
route across Switzerland) and **Switzerland** (2 541 departures, 1 231 routes, 21 243 scheduled stop
arrivals).  Both run on the same OSM network; the Olten case is a cut of the same schedule.  Sources:
`swarm/r3/<agent>/REPORT.md` for round 3, `swarm/r2/*` and `swarm/data/*` for rounds 1–2.

## 1. The timetable: station dwell times (round 3, agent DW1)

| # | what | where | why justified | scale |
|---|---|---|---|---|
| W1 | `arrivalOffset` / `departureOffset` of every stop are the GTFS `arrival_time` / `departure_time` of that stop (first stop 0/0, last stop departure = arrival) | `prepare/restore_dwell.py` (step 7 of `scripts/prepare_case.sh`) | pt2matsim 24.4 wrote both offsets from the **arrival** time (`GtfsConverter.createTransitRouteStop`, `swarm/r3/DW1/REPORT.md`), so every station dwell was lost: a train left at its arrival minute, ran ahead of the timetable and stood on track a punctual train needed | 2 541 / 612 departures, 21 243 / 5 459 stops (11 194 / 3 400 with a dwell > 0) |
| W2 | The dwell-restored unmapped schedule is the scoring reference | `matsim_input/<case>/unmapped_schedule.xml.gz` | the timetable is the yardstick; the reference must be the published one, not the mapping | both cases |
| W3 | Route split rule (one route per distinct arrival/departure pattern), deterministic ids `<id>__d2`… | `prepare/restore_dwell.py` | pt2matsim merges trips with equal stop sequence **and** offsets; a different dwell pattern must not silently change another departure's times | 0 routes split in both cases (measured) |

## 2. Infrastructure and running direction

| # | what | where | why justified | scale |
|---|---|---|---|---|
| A1 | One capacity-1 resource per **physical track section** (a link and its reverse link share one resource; `resources=linkPair`) | `RunSwitzerlandRailway.adjustNetwork` | a track section holds one train at a time – the physical constraint the model rests on. No link gets capacity > 1 inside the modelled network (the only exception is D1) | 665 238 links, all trains |
| A2 | Left-hand running on double track; direction fused from OSM tags, `route=train` relations and track geometry | `prepare/tables/a02a_direction_geometry.py`, `h12_direction_fusion.py`; `tables/DIR/direction_final.csv` | Switzerland runs on the left; the tag alone covers 6.8 % of main-line ways. Hold-out accuracy 96.1 % (443 tagged ways, tag hidden) | 41 119 routable ways; all trains |
| A3 | 895 reviewed per-way direction corrections on top of the fusion | `prepare/corrections/direction_overrides.csv` (each row with source and reason) | station analyses (round 1) looked at the actual tracks in the throats where geometry and relations disagree | 41 119 ways affected only where listed |
| A4 | German/foreign lines (DB Netz, DB InfraGO, SNCF, ÖBB, RFI) use **right**-hand running: the geometry direction is mirrored there | `h12_direction_fusion.py` (`FOREIGN_KEYS`) | mirror image of the Swiss rule; on the 194 ways where geometry and relations both commit, geometry is inverted on 192 | 757 ways / 198.4 km |
| A5 | Untagged track speeds: main/branch 120 km/h, station track without `maxspeed` 80, siding/crossover 60, yard/spur 40 – never above an OSM `maxspeed`, never on a tagged way | router R3 (`Net.lRouteSpeed`) | 4.8 % of main+branch km carry no `maxspeed`; 160 km/h (pt2matsim's default) is physically wrong in a yard | 1 400 links / 42.7 km, 0 tagged ways changed, 0 speeds raised |
| A6 | A legal turn table with a 30° deflection threshold, crossings handled by their OSM tag | `prepare/tables/a04_turns.py`; `tables/A04/turns.csv` | switches diverge ≤ 23° in the data; above 30° a "movement" is a reversal on the spot | all trains (0 illegal turns in both assignments) |
| A7 | A platform-level GTFS stop is mapped to a track by `railway:track_ref`, else a `railway=stop` node, else the adjacent platform edge | `prepare/tables/a03a_platform_tracks.py` | the timetable gives the platform number, OSM the track number | 994 stop facilities; 98.3–99.6 % of route stops on the booked platform track |
| A8 | Blocks = OSM main-signal sections (`tables/A05/segment_block.csv`), used only when `resources=block` (not in the reference runs) | `prepare/tables/a05_blocks.py` | main signals are the only block boundaries visible in open data | 0 in the reference runs |

## 3. Track assignment (the router)

Source: round-2 agents G1, F1, G2, G3 (`swarm/r2/G3/REPORT.md`); code
`src/main/java/ch/sbb/prepare/router/`, `scripts/prepare_case.sh` step 5.

| # | what | where | why justified | scale |
|---|---|---|---|---|
| T1 | The router may choose a different track than pt2matsim's shortest path, may use a parallel track of the same corridor, may serve a long platform way up to 150 m from the platform coordinate, and may insert unlimited-capacity `boundaryExtension` links at route ends | rules R1–R9, R5b, S1 | pt2matsim has no knowledge of running direction, platform use or train length; open data does not say which of two parallel tracks a train uses | all trains; S1: 20 legs, R5b: 46+7 legs |
| T2 | A mid-route reversal costs 300 s in the router's cost model, and is free when no timetable-feasible through path exists | rule R5b | real operation (driver changes ends); the booked slack sits in the adjacent legs | 60 reversals at Bern, ~100 more at termini |
| T3 | **Passing loops**: at a loop the two directions are given the two different loop tracks (`--loops true --loopPenalty 6.0`) | `router/Loops.java` | OSM does not say which of two parallel tracks a direction uses; both directions on one track makes the section lock treat a 2-track loop as one single-track block. Loop detection is pure data | Olten 98 active loops, Switzerland 325 |
| T4 | **Corridor choice by booked time** (Gotthard): a leg ≥ 20 km whose free-flow time is < 60 % of the booked time is re-routed to the slowest path that still fits, only along the same OSM line (`ref`), and not re-negotiated by R9 (`--r9pin`) | `router/TrackRouter.applyCorridorOne` (R7c), `router/Net.wayLineRef` | the 2024 timetable books IC2/IC21/EC over the Gotthard **mountain line** (base tunnel closed 8/2023–9/2024); booked/free-flow 1.73–2.44× (tunnel) vs 1.15–1.35× (mountain) | 35 legs taken (12 route groups), 134 further triggers kept |
| T5 | Full route length is a timetable fact: an unserved alternative is not preferred for being shorter | R9 pin (`--r9pin`) | R9's negotiated re-routing always accepted a shorter path and undid the corridor decision | 35 legs |

## 4. Track-assignment corrections after the router (round 3, applied in this order)

| # | what | where | why justified | scale |
|---|---|---|---|---|
| K1 | **Passing loops from real OSM tracks** at Courtemaîche (repair), Courfaivre, St-Ursanne, Malters, Les Geneveys-sur-Coffrane: the OSM second track's links are added in both directions and one physical direction is routed over it, its stop moved onto it | `prepare/trackpatch.py` + `prepare/corrections/second_tracks.json` (step 9; agent NL1, tool of agent RC2) | each way is in OSM, `routable`, parallel to the platform track, joined at both junctions, and the stop has **no published platform number**; one directed link per track made a whole 19.7 km line one exclusive section (Les Hauts-Geneveys → Neuchâtel: 16 836 s denied) | 35 routes / 120 departures (CH), 0 routes (Olten), 150 links |
| K2 | **Heitersberg tunnel split** into 6 block sections per bore (F1) | `prepare/routepatch.py` + `corrections/f1_neuenhof.json` (step 10; agent RC1) | OSM way 83270426 / 4257794 are one track per direction with `railway:etcs=1`; the router modelled the 3 306 m bore as **one** resource, so the IC1/IC5 pair booked 25–66 s apart could not follow. Splitting into 551 m sections (14 s at 140 km/h) reproduces block signalling | 2 links → 12, 212 route links (Olten), 376 (CH) |
| K3 | **Parallel tracks at Olten–Aarau, Bern Wylerfeld, Bern platform throat, Aarburg** (F2/F4/F5): selected routes get their own copy of a link run, offset 9–27 m | `prepare/routepatch.py` + `corrections/f{2,4,5}_*.json` (step 10; agent RC1) | OSM has several parallel tracks there but the router sent all traffic over one of them. **Caveat, unchanged from round 3:** the copies are *copies of the existing link chain*, not the OSM geometry itself; they stand for tracks that exist in OSM. F3 (Olten platform 9) is **not** applied – it deadlocks at Olten | F2: 45 routes, F4: 8, F5: 6 (Olten); planned conflicts Olten 163 → 65 including F1 |
| K4 | **Stop positions**: every route-stop that is not a first/last stop is moved forward inside its own platform track so that the standing body fits (10 m clearance at the entry switch); never another platform, never past the next stop | `prepare/fix_stops.py` + `stoplib.py` (step 11; agent SP1) | stop links are 13–46 m long while trains are 150–378 m, so a train stood with its head at the END of the stop link and its body reaching back into the throat (body behind the platform entry switch 67 771 → 9 153 m, dwell-weighted −84 %); timetable and platform share unchanged | 478 route-stops (CH) / 61 new facilities (Olten) |
| K5 | **Off-model link speed** from the published *running* time: `freespeed = length / (0.85 × booked running time)`, cap 160 km/h, speeds only raised | `prepare/fix_offmodel_speed.py` (step 12; agent CH1) | the off-model links had their speed derived from an arrival→arrival hop time including the dwell: Freiburg → Schallstadt was driven at 28 km/h for a leg booked at 120 km/h | CH: 19 links raised; Olten: 1 link |
| K6 | **Off-model hop length** 1.096 × straight line instead of the router's blanket 1.25 (Switzerland only), then K5 re-run on the corrected length | `prepare/fix_ext_length.py` + `fix_offmodel_speed.py` (step 13; agent CH2) | the case's own 7 074 modelled hops have median sinuosity 1.096 (p10 1.02, p90 1.33); 1.25 has no source and only bites where the 160 km/h cap binds | 402 links, 3 784 → 3 318 km, 15 RE7 families |
| K7 | **Single-track meets are not "fixed" by inventing track**: no second track at Chambrelien, no tracks at the head-on families of PO1/planconf (Fribourg, Visp, Interlaken West/Ost, Luzern, Arth-Goldau, Bodio, Frutigen, Aarburg) | rejected, `swarm/r3/RC2/REPORT.md` | agent RC2 searched OSM for a parallel track or a usable unused way in every family: no hit; the only real missing track is the second bore of the Lötschberg base tunnel, and wiring it needs two invented connectors | 0 |
| K8 | **Olten station fan** (20 through departures off platform 9 over real OSM tracks): **not** in the pipeline | rejected, `swarm/r3/OL1/REPORT.md`, `swarm/r3/ZH2/REPORT.md` | measured on base10: planned conflicts 65 → 46 but `abs_dev_seconds` 72 032 → 73 967 (+2.7 %) | 0 |

## 5. Rolling stock and train dynamics

| # | what | where | why justified | scale |
|---|---|---|---|---|
| B1 | Per-line rolling stock (length, vmax, mean acceleration, deceleration) instead of 5 prefix classes | `prepare/corrections/rolling_stock.csv` → `tables/RS1/vehicles_rs1.csv` → `matsim_input/<case>/vehicles_fs1.csv`, `run.properties: vehicles.file` | the type decides how long a train occupies a platform track and how fast it clears a throat; fleet lists and dated photos name the stock per line. Deceleration 0.60–0.85 m/s² is engineering judgement, not measured | 23 types, all trains (round 1, D2) |
| B2 | Acceleration = **mean acceleration from 0 to 100 km/h** of the constant-force/constant-power model (e.g. RABe 502 0.74, RABe 511 0.72, Traverso 1.04 m/s²), not the mean from 0 to vmax (0.461 / 0.687 / 0.815) | `corrections/rolling_stock.csv` column `source_acceleration` → `tables/RS1/vehicles_rs1.csv` (round 3, agent FS1, `swarm/r3/FS1/vehicles_fs1_sources.md`) | railsim applies one **constant** acceleration, so a value fitted to 0→200 km/h was applied to a 0→85 km/h hop between two stops 1.5 km apart. 0→100 is the middle of the range a stop-to-stop leg reaches; sources a0/m/P per series | all trains; `abs_dev_seconds` Olten 107 460 → 95 745 on base8 |
| B3 | Trains appear at their first stop and vanish at their last stop; no parking loops or depot movements, no terminal occupancy | railsim behaviour; verified in `swarm/r2/D4/REPORT.md` | no timetable information about depot runs; the timetable starts and ends at a platform | all trains |
| B4 | A train that enters the opposite link of its head link is marked reversible; reversal takes 0 s by default (`reversibleSeconds=0`) | `RunSwitzerlandRailway` | railsim ≥ 2026.0-2025w42 throws on a non-reversible train; a terminus reversal is a real movement with no running time | all reversing trains |

## 6. Engine and dispatching

Sources: `swarm/enginefix/REPORT.md` (round 1), `swarm/r2/H1`, `swarm/r2/V1` (round 2), round 3 agents
ZH1, JT1, JT2, FS2, BN2, BN3, EN1 and IG2 (`swarm/r3/src9/CHANGES.md`, `swarm/r3/EN1/REPORT.md`),
and `swarm/r3/P4/REPORT.md` for the last two rows.

| # | what | where | why justified | scale |
|---|---|---|---|---|
| C1 | Patched `RailsimEngine`: a reversing train releases its reservations on the arrival links; the approved distance cannot become negative (no NaN speed); **round 3**: the tail pointer no longer lags (speed cap "lowest limit under the body" was applied to a body of up to 1 206 m), the speed cap of a left link is re-evaluated (`engine.speedRefresh`), a phantom "stop at the end of the reserved window" command is gone, and a terminus reversal keeps a claim on the platform lead while the train reverses | `src/main/java/ch/sbb/matsim/contrib/railsim/qsimengine/RailsimEngine.java` (`SWARM-FIX`), `PunctualProfileHook.java` | each one is a measured upstream defect (one leaked resource at Luzern blocked 226 trains; the stale tail cost 6 % of the punctuality and doubled the runtime; CH1's stale cap drove RE48 at 40 km/h for 25.9 km) | all trains |
| C2 | Patched `RailsimCalc` (assert on the approved distance) | `.../qsimengine/RailsimCalc.java` | same NaN/negative-distance defect, second place (E2) | all trains |
| C3 | `SectionLockDeadlockAvoidance`: lock only the sections the schedule really uses in both directions, one claim per physical track (`sectionLock.dedup=on`) | `src/main/java/ch/sbb/run/SectionLockDeadlockAvoidance.java`; `run.properties` | railsim's default claims the running direction of the whole remaining route and deadlocks on every one-directional station throat | all trains |
| C4 | Section-lock stop rule `safe`: a section never ends at a stop whose own track is used in both directions | `run.properties: sectionLock.stopRule=safe` | with `any` two opposing trains may enter the same single-track stretch up to their own platform and then refuse each other for ever (Laufen, Courtemaîche) | all trains |
| C5 | Meet rule at a station with two tracks (`sectionLock.meet=on`) | `SectionLockDeadlockAvoidance` (round 2, agent V1) | two opposing trains that are booked to meet at a two-track station may both enter; without it the section lock serialises them | all station meets (9 451 in CH, 3 312 in Olten) |
| C6 | **Reversal stops**: at a stop where a train reverses a section may end when the standing area (the links under the body) is free and not on anybody's admitted way or departure path (`sectionLock.reversalStop=standingArea`) | `SectionLockDeadlockAvoidance` (round 3, agents BN2/BN3) | a reversing train used to hold a claim over the whole station box for its 8–14 minute dwell; punctual trains waited ~800 s for a physically free track (Bern, 63 held arrivals / 39 626 s). Default `off` reproduces the old behaviour | Bern 32 644 → measured −9.2 % on the Olten case |
| C7 | **Time-bounded retrace claim** (`sectionLock.retracePass=on`, `retracePassMargin=120`): a train that only crosses a reversing train's departure lead, ends its section at a stop clear of the lead and is there ≥ 120 s before the reversing train's published departure is no longer refused | `SectionLockDeadlockAvoidance` (round 3, agent BN3) | a safe superset of C6: it removes the Fribourg/other knock-on refusals without letting a train into a lead a reversing train needs | all trains; measured on base10: CH 367 763 → 366 950 (BN3) |
| C8 | **Just-in-time driving** (`speedProfile=punctual`): the train solves for the cruise speed at which it stops at the published arrival second; if the leg cannot be kept even at full speed it drives at full speed | `src/main/java/ch/sbb/run/PunctualSpeedProfile.java` + `PunctualProfileHook` (round 3, agents JT1/JT2), `run.properties: punctual.aim=0.25`, `punctual.minFactor=0.0` | the published running times are far longer than the physics: without the controller 79 % of all arrivals are more than 60 s early; holding before the station instead of at the platform also removes 157 h (Olten) / 326 h (CH) of extra platform occupation. It never makes a train depart before its published departure | all trains; Olten `abs_dev_seconds` 681 227 → 107 460 (base8) |
| C9 | railsim `pollInterval=5` s, `endTime=36:00:00`, one iteration, seed 4711, gzip output | `matsim_input/<case>/config.xml`, `run.properties` | 5 s is the resolution at which a 20 m platform track is not crossed in one step; 36 h covers the last arrival of a service day | all trains |
| C10 | **Not** in the final state: plan-order dispatching (`deadlockAvoidance=planOrder`, agent PO1) and the `justInTime` speed profile of round 2 | rejected, `swarm/r3/PO1/REPORT.md`, `swarm/r2/K1/REPORT.md` | plan-order dispatching cannot operate a station (Bern micro-case 6 of 25 trains, Olten 38 of 612); the round-2 just-in-time profile lost trains | 0 |
| C11 | **The head never moves without driving** (`engine.reachTime=on`, `engine.headGuard=on`): after a deceleration phase the time to reach a distance includes the cruise phase at the target speed, and `enterLink` moves the head onto the next link only when it really stands at the end of the current one (otherwise the update is re-planned, or polled while standing) | `src/main/java/ch/sbb/matsim/contrib/railsim/qsimengine/{RailsimCalc,RailsimEngine}.java`, `PunctualProfileHook.java`, `run.properties` (round 3, agent EN1) | `calcRequiredTime` returned the end of the *speed change*, not the arrival at the link end, so the `ENTER_LINK` event fired while the head was 0–500 m short and `enterLink` moved it forward unconditionally — a teleport: 12.74 % of the state events had a body up to 672 m too long, and a premature event also served a stop and declared the arrival early. With the fix the recorded tail is 100 % consistent with an independent reconstruction and no arrival is more than 5 s early | all trains; tail 100.00 % (was 88.80 % Olten / 79.38 % CH) |
| C12 | **A crossing may end before its own stop** (`sectionLock.retracePassAhead=on`): the time-bounded retrace excuse of C7 also applies when the passing train's section ends at a one-directional link, if the whole way from that link to its next **scheduled stop** is free of other trains (no reservation, no non-retrace claim) and the train is predicted to stand there `retracePassMargin` s before the reversing train's published departure | `SectionLockDeadlockAvoidance.retracePass` (round 3, agent P4) | a section is the maximal run of bidirectional resources, so in the Bern throat it ends 73 m after the platform-4 lead while the train goes on to platform 2, 650 m further; refusing it made 17 RE7 wait 819–824 s each for a lead that was physically free (measured: the way ahead was free from 40 992 s, the train entered at 41 881 s, +874 s). Invariant of C7, with "the place where the train really stands" instead of "where its section ends"; a refused crossing creates no claim | 17 RE7 (CH), 8 IC8 (Olten): Olten `abs_dev_seconds` 48 146 → 43 027 |

## 7. Reversal geometry (post-processing, timetable untouched)

Source: `swarm/r2/E2/REPORT.md`, `swarm/r2/I2/REPORT.md`; step 6 of `scripts/prepare_case.sh`.

| # | what | where | why justified | scale |
|---|---|---|---|---|
| R1 | Every reversal departure is rebuilt to retrace the arrival links under the **whole train body** before it diverges, then leaves at the first switch behind the body | `prepare/fix_reversals.py` | the engine rebuilds a reversing train's body onto the departure route; if the departure does not retrace the arrival links, the body stands on track the train never reserved (Bern was the measured case). Only the `<route>` link list changes | Olten: 93 routes, +6 460 m (max +194 m); Switzerland: 68 reversal routes, 59 kept |
| R2 | A route change is kept only if **every** reversal of the result is honest, and no candidate that adds more than 2 000 m is accepted | `prepare/fix_reversals.py` (`unsafe_reversals`, `--max-added-m`) | at Delémont the crossover is 42 m behind a 149.6 m platform while the IC51 train is 377.6 m long: without the guard the search produces a 121.7 km loop around the Jura | Switzerland: 9 of 68 routes reverted, 0 in Olten |

## 8. Off-model track (both cases; needed for the Switzerland case)

Track that our infrastructure data does not contain (metre gauge, foreign lines, dual-gauge-only
corridors) is modelled as off-model track.  Source: `swarm/r2/F1/REPORT.md` §1–2, round 3 agents CH1
and CH2; data `prepare/tables/offmodel_stops.py`, `offmodel_legs.py` → `tables/OFFMODEL/`.

| # | what | where | why justified | scale |
|---|---|---|---|---|
| D1 | Off-model track becomes unlimited-capacity links (`boundaryCapacity=1000`; the section lock skips resources with capacity > 1) | router off-model model; `run.properties: boundaryCapacity=1000` | the timetable is ground truth and those lines cannot deadlock on track we do not model; unlimited capacity is confined to those links, never to modelled running lines or station tracks | 880 links, 883 of the 2 541 departures (34.8 %); 0 in Olten |
| D2 | A stop without standard-gauge track gets its own off-model node and is never snapped onto a through track | `prepare/tables/offmodel_stops.py` (6 data rules, no station list) → `OFFMODEL/stop_track_gauge.csv` | snapping put RhB trains on the SBB Rheintal and MGB trains on the SBB Visp tracks | 391 of 994 stops, 2 627 of 10 802 route stops |
| D3 | Off-model link length = real OSM distance, else straight line × **1.096** (was 1.25), and speed so that the minimum run time is 0.85 × the booked **running** time (cap 160 km/h) | `prepare/tables/offmodel_legs.py` → `OFFMODEL/leg_distance.csv`; router `offLength`; K5, K6 | 1.096 is the measured median sinuosity of the case's own 7 074 modelled hops; 1.25 was a router default without a source and pushed the required speed over the cap. On the narrow-gauge graph the real distances agree within +0.3 % | 402 links (K6), 2 535 hops + 250 stubs, 18 513 km |
| D4 | A real leg that cannot keep the timetable is off-model too (needs more than booked + 60 s and the off-model substitute fits) | router (`--infeasibleSlack 60`) | Schaffhausen–Bülach via Jestetten, Como–Chiasso, Hochrhein: the data has no usable real path, the timetable does | 56 legs, 0.08 % of all legs |
| D5 | Dual-gauge ways (`gauge=1000;1435`) count as **not** standard gauge for the stop decision | `offmodel_stops.py` | they are metre-gauge corridors in reality | 16 route stops |

## 9. What the model does not do (known limits)

* **Train ends**: trains appear at their first stop and vanish at their last one. No depot run, no
  parking loop and no terminal occupancy is modelled, so a terminus can accept the next train while
  the previous one would still stand there.
* **No node-level interlocking**: resources are track sections (a link pair), not switch areas; the
  section lock works at the granularity of whole sections (up to 25.5 km on non-stop single track).
* **Bern does not fit**: platform tracks between switches are 13–46 m long for 150–378 m trains, and
  Bern platform 4 has 147.6 m for a 187.4 m EC. A train whose body does not fit the platform track
  fouls a switch, and the section lock then refuses a following train – physical, not a claim.
* **Minute-resolution timetable**: GTFS times are whole minutes; a train can be up to 59 s late
  (or early) by rounding alone, and 1 040 of the Switzerland deficits are inside that bound.
* **Off-model track** (D1–D5) has no capacity limit and no conflicts: it is the only place where
  "physics off" is allowed, and it is confined to track we do not model.
* **One service day, one iteration**: no day-to-day propagation, no rolling-stock circulation, no
  passengers and no transfers.
* **Direction data** (A2–A4) is 96 % accurate on a hold-out set; the router's cost – not a hard
  constraint – absorbs the rest. The rolling stock of the generic `IC`/`IR`/`RE` lines and of the EC
  Interlaken run is partly **UNVERIFIED**; deceleration is engineering judgement (B1).
* **Deadlock avoidance is a model rule**, not Swiss interlocking: it can refuse a movement a real
  dispatcher would allow (C3–C7).
