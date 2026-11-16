#!/usr/bin/env bash
# Run, score and audit one case (round 3: on time = 0 s deviation at every stop).
#
#   scripts/run_case.sh olten              # full run, ~10-20 min, then score + punctuality + audit
#   HEAP=12g scripts/run_case.sh switzerland
#   JAVA_OPTS=-ea scripts/run_case.sh olten
#
# Requires `scripts/prepare_case.sh <case>` to have been run (matsim_input/<case>/routed/).
# Writes, inside matsim_input/<case>/: sim.log, output/, score.json, score.txt, punct.csv, punct.json,
# punct.txt, audit/ (physical-safety audit), deadlocks.txt.
# At most $SLOTS simulations run at once on this machine.  If the project bench directory
# (swarm/bench/slots) exists it is used as the semaphore, so runs of this script and of the bench
# share the limit; otherwise the local matsim_input/.slots is used.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

CASE=${1:-}
case "$CASE" in
  olten|switzerland) ;;
  *) echo "usage: $0 <olten|switzerland>" >&2; exit 2 ;;
esac

SLOTS=${SLOTS:-7}
if [ "$CASE" = "switzerland" ]; then HEAP=${HEAP:-12g}; else HEAP=${HEAP:-8g}; fi
CASE_DIR="matsim_input/$CASE"
JAR=$(ls -1t matsim-switzerland-railway-*.jar 2>/dev/null | head -1 || true)
[ -n "$JAR" ] || { echo "no jar - run scripts/prepare_case.sh $CASE" >&2; exit 3; }
for f in "$CASE_DIR/config.xml" "$CASE_DIR/run.properties" "$CASE_DIR/routed/transitSchedule.xml.gz"; do
  [ -e "$f" ] || { echo "missing $f - run scripts/prepare_case.sh $CASE" >&2; exit 3; }
done

# scoring reference: the dwell-restored published case schedule (prepared by the pipeline itself;
# evaluate.py additionally checks that the run's schedule is the same timetable)
REF="$CASE_DIR/unmapped_schedule.xml.gz"
[ -e "$REF" ] || { echo "missing $REF - run scripts/prepare_case.sh $CASE" >&2; exit 3; }

# ---- slot semaphore (shared with the project bench when it is present, otherwise local)
SLOTDIR=${SLOTDIR:-}
if [ -z "$SLOTDIR" ]; then
  if [ -d swarm/bench/slots ]; then SLOTDIR=swarm/bench/slots; else SLOTDIR=matsim_input/.slots; fi
fi
mkdir -p "$SLOTDIR"
SLOT=""
release() { [ -n "$SLOT" ] && rm -rf "$SLOT"; }
trap release EXIT INT TERM
while [ -z "$SLOT" ]; do
  for i in $(seq 1 "$SLOTS"); do
    d="$SLOTDIR/slot$i"
    if [ -d "$d" ] && [ -f "$d/pid" ] && ! kill -0 "$(cat "$d/pid")" 2>/dev/null; then rm -rf "$d"; fi
    if mkdir "$d" 2>/dev/null; then echo $$ > "$d/pid"; SLOT=$d; break; fi
  done
  [ -z "$SLOT" ] && sleep 5
done

cd "$CASE_DIR"
rm -rf output score.json score.txt punct.csv punct.json punct.txt audit deadlocks.txt
START=$(date +%s)
set +e
java ${JAVA_OPTS:-} -Xmx$HEAP -Djava.awt.headless=true -cp "../../$JAR" \
  ch.sbb.run.RunSwitzerlandRailway config.xml > sim.log 2>&1
RC=$?
set -e
END=$(date +%s)
echo "simulation exit code $RC, wall time $((END-START)) s"
if [ $RC -ne 0 ]; then
  # known railsim bug: the mobsim finishes, the analysis writer then dies on a NaN train state
  ITEV=$(ls output/ITERS/it.0/*.events.xml.gz 2>/dev/null | head -1 || true)
  if [ -n "$ITEV" ] && grep -q "RailsimCsvWriter.writeTrainStatesCsv" sim.log; then
    echo "WARNING: crash AFTER the simulation in the train-state writer; scoring from $ITEV"
    cp "$ITEV" output/crashed.output_events.xml.gz
  else
    grep -E "Exception|ERROR" sim.log | head -15; tail -5 sim.log
    echo "SIMULATION FAILED (see $CASE_DIR/sim.log)"; exit 4
  fi
fi
# the patched engine prints this line in its constructor; without it the jar's class was active
grep -q "SWARM-FIX RailsimEngine active" sim.log \
  && echo "engine: patched RailsimEngine active" \
  || { echo "WARNING: patched RailsimEngine NOT active (see sim.log)"; }

EVENTS=$(ls output/*output_events.xml.gz output/*output_events.xml.zst output/*output_events.xml 2>/dev/null | head -1 || true)
[ -n "$EVENTS" ] || { echo "no events file in $CASE_DIR/output" >&2; exit 5; }

SCHED=$(python3 - <<'EOF'
import re
c = open('config.xml').read()
m = re.search(r'name="transitScheduleFile"\s+value="([^"]+)"', c)
print(m.group(1) if m else 'routed/transitSchedule.xml.gz')
EOF
)
# ---- score: the bench scorer (schedule identity + late arrivals) and the round-3 metric
echo "== score $(date +%H:%M:%S)"
python3 ../../scripts/evaluate.py "$SCHED" "$EVENTS" --ref "../../$REF" --json score.json | tee score.txt
python3 ../../scripts/punct.py "$(pwd)" --ref "../../$REF" > punct.txt 2>&1 || true
tail -1 punct.txt

# ---- physical-safety audit (E2/I2: 0 body overlaps, no unreserved body, no stale reservation)
echo "== audit $(date +%H:%M:%S)"
python3 ../../scripts/audit_safety.py --exp "$(pwd)" --out audit > audit.log 2>&1 || { tail -5 audit.log; echo "AUDIT FAILED"; }

# ---- deadlock report (who blocks whom); cheap, from the events cache of the audit
echo "== deadlocks $(date +%H:%M:%S)"
python3 ../../scripts/deadlocks.py "$(pwd)" > deadlocks.txt 2>&1 || true

# disk hygiene: drop the per-iteration copies (they duplicate the events file)
rm -rf output/ITERS output/tmp
