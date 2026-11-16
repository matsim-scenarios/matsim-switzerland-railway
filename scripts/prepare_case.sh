#!/usr/bin/env bash
# Go from the original data to a routed, corrected, ready-to-run case (round 3 pipeline).
#
#   scripts/prepare_case.sh olten
#   scripts/prepare_case.sh switzerland
#
# Steps (details, run times and the evidence for every step: docs/pipeline.md)
#   1. static tables        prepare/build_tables.py                 -> matsim_input/tables/
#                           (A01..DIR, RS1 rolling stock, off-model stop/gauge + leg distance,
#                            the Olten case schedule)
#   2. case schedule        matsim_input/<case>/unmapped_schedule_gtfs.xml.gz  (the times pt2matsim
#                           produces -- WITHOUT the station dwell times)
#   3. rolling stock        tables/RS1/vehicles_rs1.csv               -> matsim_input/<case>/vehicles_fs1.csv
#                           (round 3: acceleration = mean 0->100 km/h, agent FS1)
#   4. build the jar        mvn -o clean package                     -> matsim-switzerland-railway-<ver>.jar
#   5. track assignment     ch.sbb.prepare.router.TrackRouter        -> routed/  (G3's router: passing
#                           loops, corridor rule R7c, off-model track for the Switzerland case)
#   6. reversal geometry    prepare/fix_reversals.py                 -> routed/transitSchedule.xml.gz
#   7. station dwell times  prepare/restore_dwell.py                 -> unmapped_schedule.xml.gz (reference)
#                           and on the routed schedule (the published timetable is ground truth)
#   8. timetable checks     prepare/verify_dwell.py + scripts/verify_schedule.py   (abort on failure)
#   9. passing loops        prepare/trackpatch.py  + corrections/second_tracks.json  (agent NL1)
#  10. route patches        prepare/routepatch.py  + corrections/f{1,2,4,5}_*.json   (agent RC1)
#  11. stop positions       prepare/fix_stops.py                                (agent SP1)
#  12. off-model speeds     prepare/fix_offmodel_speed.py                       (agent CH1)
#  13. off-model lengths    prepare/fix_ext_length.py + fix_offmodel_speed.py   (agent CH2, Switzerland)
#  14. static quality       scripts/check_mapping.py                -> mapping_check*.json
#
# The two pt2matsim inputs (matsim_input/tmp/schedule_gtfs_filtered.xml.gz and
# network_osm.xml.gz) are produced by the upstream class ch.sbb.prepare.GenerateRailsimInput
# from original_data/gtfs/*.zip and original_data/osm/switzerland_railways.osm; see
# docs/pipeline.md step 1.  They are large, slow to rebuild and already part of the input data.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

CASE=${1:-}
case "$CASE" in
  olten|switzerland) ;;
  *) echo "usage: $0 <olten|switzerland>" >&2; exit 2 ;;
esac

MVN=${MVN:-$HOME/.m2/wrapper/dists/apache-maven-3.9.16/56ba1f9f/bin/mvn}
PY=${PY:-uv run --python 3.12 --with osmium --with pyproj --with numpy --with scipy --with shapely python}
HEAP=${HEAP:-10g}
THREADS=${THREADS:-}

GTFS=original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip
DIR="matsim_input/$CASE"
ROUTED="$DIR/routed"
WORK="$DIR/work"
CORR=prepare/corrections

# ---- 0. inputs present?
for f in original_data/osm/switzerland_railways.osm \
         original_data/osm/switzerland-latest.osm.pbf \
         original_data/shp/olten/olten.shp \
         "$GTFS" \
         matsim_input/tmp/schedule_gtfs_filtered.xml.gz \
         matsim_input/tmp/network_osm.xml.gz; do
  [ -e "$f" ] || { echo "missing input: $f (see docs/pipeline.md)" >&2; exit 3; }
done

mkdir -p "$DIR" "$WORK"

# ---- 1. static tables + the Olten case schedule
EXTRA=""
[ "$CASE" = "switzerland" ] && EXTRA="--no-case"
echo "== 1. static tables $(date +%H:%M:%S)"
$PY prepare/build_tables.py --out matsim_input/tables $EXTRA

# ---- 2. the case schedule as pt2matsim wrote it (no station dwell times yet)
echo "== 2. case schedule $(date +%H:%M:%S)"
if [ "$CASE" = "switzerland" ]; then
  # the Switzerland case is the whole filtered schedule, no cutting
  cp -f matsim_input/tmp/schedule_gtfs_filtered.xml.gz "$DIR/unmapped_schedule_gtfs.xml.gz"
fi
GTFS_SCHED="$DIR/unmapped_schedule_gtfs.xml.gz"

# ---- 3. rolling stock (round 3: FS1's acceleration column, see prepare/corrections/rolling_stock.csv)
echo "== 3. rolling stock $(date +%H:%M:%S)"
cp -f matsim_input/tables/RS1/vehicles_rs1.csv "$DIR/vehicles_fs1.csv"

# ---- 4. jar
JAR=$(ls -1t matsim-switzerland-railway-*.jar 2>/dev/null | head -1 || true)
if [ -z "${SKIP_BUILD:-}" ]; then
  echo "== 4. mvn -o clean package $(date +%H:%M:%S)"
  $MVN -o -q -DskipTests clean package
  JAR=$(ls -1t matsim-switzerland-railway-*.jar | head -1)
fi
[ -n "$JAR" ] || { echo "no jar - run: $MVN -o package" >&2; exit 4; }

# ---- 5. track assignment
# G3's router: passing loops on, corridor rule R7c pinned against R9, off-model track for the
# Switzerland case (F1: metre gauge / abroad).  Rule switches from swarm/r2/G3/router/run.sh.
echo "== 5. track assignment ($JAR) $(date +%H:%M:%S)"
OFFMODEL=""
THR=""
[ -n "$THREADS" ] && THR="--threads $THREADS"
[ "$CASE" = "switzerland" ] && OFFMODEL="--stopClass matsim_input/tables/OFFMODEL/stop_track_gauge.csv --legDist matsim_input/tables/OFFMODEL/leg_distance.csv"
java -Xmx$HEAP -cp "$JAR" ch.sbb.prepare.router.TrackRouter \
  --schedule "$GTFS_SCHED" \
  --network  matsim_input/tmp/network_osm.xml.gz \
  --data     matsim_input/tables \
  --out      "$ROUTED" \
  --loops true --loopPenalty 6.0 $OFFMODEL $THR \
  --r1 true --r2 true --r3 true --r4 true --r5 true --r6 true --r7 true --r8 true --r9 true \
  --r9pin true --maxIter 8 --relBonus 0.85 --dirPenalty 8 --dirEntry 120 --dubious true \
  --softTurn 0 --corridorGain 0 2>&1 | tee "$DIR/router.log"

# ---- 6. honest reversal geometry (round-2 E2/I2 geometry fix; the timetable is not touched)
# fix_reversals reads run.properties for the per-line train lengths (vehicles.file), so the router
# output directory gets the two small input files next to the network and the schedule.
echo "== 6. reversal post-processing $(date +%H:%M:%S)"
cp -f "$ROUTED/transitSchedule.xml.gz" "$ROUTED/transitSchedule_router.xml.gz"
cp -f "$ROUTED/transitNetwork.xml.gz" "$ROUTED/transitNetwork_router.xml.gz"
cp -f "$DIR/run.properties" "$ROUTED/run.properties"
cp -f "$DIR/vehicles_fs1.csv" "$ROUTED/vehicles_fs1.csv"
python3 prepare/fix_reversals.py --exp "$ROUTED" \
  --out "$WORK/0_reversals_schedule.xml.gz" \
  --json "$ROUTED/fix_reversals.json" \
  | tee "$DIR/fix_reversals.log"

# ---- 7. the published station dwell times (round 3, agent DW1)
# pt2matsim 24.4 wrote arrivalOffset == departureOffset at every stop; the timetable has separate
# arrival and departure times.  The restored unmapped schedule is the scoring reference, the
# restored routed schedule is what is simulated.
echo "== 7. station dwell times $(date +%H:%M:%S)"
python3 prepare/restore_dwell.py "$GTFS_SCHED" "$GTFS" "$DIR/unmapped_schedule.xml.gz" 2>&1 \
  | tee "$DIR/restore_dwell_reference.log"
python3 prepare/restore_dwell.py "$WORK/0_reversals_schedule.xml.gz" "$GTFS" "$WORK/1_dwell_schedule.xml.gz" 2>&1 \
  | tee "$DIR/restore_dwell_routed.log"
REF="$DIR/unmapped_schedule.xml.gz"

# ---- 8. the timetable must be the published one (arrival AND departure, every stop)
echo "== 8. timetable checks $(date +%H:%M:%S)"
python3 prepare/verify_dwell.py "$REF" "$GTFS" --old "$GTFS_SCHED" | tee "$DIR/verify_dwell_reference.log"
python3 prepare/verify_dwell.py "$WORK/1_dwell_schedule.xml.gz" "$GTFS" --old "$WORK/0_reversals_schedule.xml.gz" \
  | tee "$DIR/verify_dwell_routed.log"
python3 scripts/verify_schedule.py "$WORK/1_dwell_schedule.xml.gz" "$REF" | tee "$DIR/verify_schedule.log"

# ---- 9. passing loops: the real OSM second tracks the router did not use (round 3, agent NL1)
echo "== 9. passing loops $(date +%H:%M:%S)"
python3 prepare/trackpatch.py "$WORK/1_dwell_schedule.xml.gz" "$ROUTED/transitNetwork.xml.gz" \
  "$WORK/2_loops_schedule.xml.gz" "$WORK/2_loops_network.xml.gz" \
  "$CORR/second_tracks.json" --check 2>&1 | tee "$DIR/passing_loops.log"

# ---- 10. route patches F1/F2/F4/F5 (round 3, agent RC1; F3 is rejected, it deadlocks at Olten)
echo "== 10. route patches $(date +%H:%M:%S)"
python3 prepare/routepatch.py "$WORK/2_loops_schedule.xml.gz" "$WORK/2_loops_network.xml.gz" \
  "$WORK/3_patch_schedule.xml.gz" "$WORK/3_patch_network.xml.gz" \
  "$CORR/f1_neuenhof.json" "$CORR/f2_olten_aarau.json" "$CORR/f4_bern.json" "$CORR/f5_meets.json" \
  2>&1 | tee "$DIR/route_patches.log"

# ---- 11. stop positions: the standing train inside its platform track (round 3, agent SP1)
echo "== 11. stop positions $(date +%H:%M:%S)"
python3 prepare/fix_stops.py "$WORK/3_patch_schedule.xml.gz" "$WORK/3_patch_network.xml.gz" \
  "$WORK/4_stops_schedule.xml.gz" "$WORK/4_stops_network.xml.gz" \
  --vehicles "$DIR/vehicles_fs1.csv" --clearance 10 --platbound check --stops through \
  --csv "$DIR/stop_moves.csv" --json "$DIR/stop_moves.json" > "$DIR/fix_stops.log" 2>&1
tail -2 "$DIR/fix_stops.log"

# ---- 12. off-model link speeds from the published running time (round 3, agent CH1)
echo "== 12. off-model speeds $(date +%H:%M:%S)"
python3 prepare/fix_offmodel_speed.py "$WORK/4_stops_schedule.xml.gz" "$WORK/4_stops_network.xml.gz" \
  "$WORK/5_offmodel_network.xml.gz" --vehicles "$DIR/vehicles_fs1.csv" \
  --report "$DIR/offmodel_speed_fix.csv" > "$DIR/fix_offmodel_speed.log" 2>&1
tail -1 "$DIR/fix_offmodel_speed.log"
FINAL_NET="$WORK/5_offmodel_network.xml.gz"

# ---- 13. off-model hop lengths (round 3, agent CH2; Switzerland only -- nothing to correct in Olten)
if [ "$CASE" = "switzerland" ]; then
  echo "== 13. off-model lengths $(date +%H:%M:%S)"
  python3 prepare/fix_ext_length.py "$FINAL_NET" "$WORK/6_length_network.xml.gz" \
    --factor 1.096 --report "$DIR/ext_length_fix.csv" > "$DIR/fix_ext_length.log" 2>&1
  tail -1 "$DIR/fix_ext_length.log"
  python3 prepare/fix_offmodel_speed.py "$WORK/4_stops_schedule.xml.gz" "$WORK/6_length_network.xml.gz" \
    "$WORK/7_offmodel2_network.xml.gz" --vehicles "$DIR/vehicles_fs1.csv" \
    --report "$DIR/offmodel_speed_fix2.csv" > "$DIR/fix_offmodel_speed2.log" 2>&1
  tail -1 "$DIR/fix_offmodel_speed2.log"
  FINAL_NET="$WORK/7_offmodel2_network.xml.gz"
fi

# ---- 14. Bern platform 4 on the real platform track (round 3, agent BN4; the largest single gain)
# The case network kept only the eastern 128.6 m of the 535 m real platform-4 track between its
# switches, so every standing platform-4 train hung 94-195 m into the northern throat.  The step adds
# the real OSM links (verbatim from network_osm) and moves the 30 standing platform-4 stops to the
# western end of the same track (--depth 2 = the depth BN4 shipped).  Timetable and platform unchanged.
echo "== 14. Bern platform 4 $(date +%H:%M:%S)"
python3 prepare/bern_platform.py "$WORK/4_stops_schedule.xml.gz" "$FINAL_NET" \
  "$WORK/8_bern_schedule.xml.gz" "$WORK/8_bern_network.xml.gz" --depth 2 \
  > "$DIR/fix_bern_platform.log" 2>&1
tail -1 "$DIR/fix_bern_platform.log"
FINAL_SCHED="$WORK/8_bern_schedule.xml.gz"
FINAL_NET="$WORK/8_bern_network.xml.gz"

# ---- the case's run inputs
cp -f "$FINAL_SCHED" "$ROUTED/transitSchedule.xml.gz"
cp -f "$FINAL_NET" "$ROUTED/transitNetwork.xml.gz"

# ---- 15. static quality of the assignment
# checked on the ROUTER output -- against the router's OWN network, because the corrections below
# change links (F1 splits a link, F2/F4/F5 add parallel copies, BN4 adds a platform track) -- and on
# the final schedule against the final network.  The differences are the accepted corrections,
# reported, not hidden.
echo "== 15. check_mapping $(date +%H:%M:%S)"
python3 scripts/check_mapping.py "$ROUTED/transitSchedule_router.xml.gz" \
  "$ROUTED/transitNetwork_router.xml.gz" \
  --json "$DIR/mapping_check.json"
python3 scripts/check_mapping.py "$ROUTED/transitSchedule.xml.gz" \
  "$ROUTED/transitNetwork.xml.gz" \
  --json "$DIR/mapping_check_fixed.json" > "$DIR/check_mapping_fixed.log" 2>&1
python3 prepare/verify_dwell.py "$ROUTED/transitSchedule.xml.gz" "$GTFS" \
  > "$DIR/verify_dwell_final.log" 2>&1
grep -E "^VERIFY" "$DIR/verify_dwell_final.log"
python3 scripts/verify_schedule.py "$ROUTED/transitSchedule.xml.gz" "$REF" \
  > "$DIR/verify_schedule_final.log" 2>&1
grep -E "SIGNATURE" "$DIR/verify_schedule_final.log"

echo "ready: scripts/run_case.sh $CASE"
