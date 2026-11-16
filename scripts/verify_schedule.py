#!/usr/bin/env python3
"""Verify that a mapped schedule is the same timetable as the unmapped reference.

Uses the scorer's own signature definition (`scripts/evaluate.py`): per departure
(line, departure id, start time, [(station, arrOffset, depOffset), ...]), station = stop id
without the `.link:NNN` suffix and without the platform part.  This is the check that a track
assignment or a post-processing step did not remove, retime or shorten a single train or stop.

Usage: python3 scripts/verify_schedule.py <mapped.xml.gz> [<reference.xml.gz>]
Exit 0 = identical, 1 = differences.

(round 2: agent D4's `verify_schedule.py`, now reading this repository's scorer)
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location("ev", os.path.join(HERE, "evaluate.py"))
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)

mapped = sys.argv[1]
ref = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "matsim_input/tmp/schedule_gtfs_filtered.xml.gz")

m = ev.read_schedule(mapped)
r = ev.read_schedule(ref)
sm, sr = ev.signature(m), ev.signature(r)
print(f"mapped: {len(m)} departures, {sum(len(d['stops']) for d in m.values())} scheduled stop arrivals")
print(f"ref   : {len(r)} departures, {sum(len(d['stops']) for d in r.values())} scheduled stop arrivals")
if sm == sr:
    print(f"SIGNATURE IDENTICAL ({len(sr)} distinct departure signatures)")
    sys.exit(0)
missing = sum((sr[k] - sm.get(k, 0)) for k in sr if sr[k] > sm.get(k, 0))
extra = sum((sm[k] - sr.get(k, 0)) for k in sm if sm[k] > sr.get(k, 0))
print(f"DIFFERENT: {missing} reference departures missing/changed, {extra} unknown")
for k in list(sr)[:3]:
    if sr.get(k, 0) != sm.get(k, 0):
        print("  ref-only", k[0], k[1], k[2], len(k[3]))
        break
sys.exit(1)
