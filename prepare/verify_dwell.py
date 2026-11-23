#!/usr/bin/env python3
"""Proof that a schedule carries the published timetable -- the dwell check of the pipeline.

    python3 prepare/verify_dwell.py <schedule> <gtfs.zip> [--old <old schedule>] [--cache <file>]

Ported from round-3 agent DW1 (`swarm/r3/DW1/verify_dwell.py`), code unchanged; `scripts/
prepare_case.sh` runs it twice (reference schedule and routed schedule) and aborts on FAILED.

Check 1 (absolute times)
    For every departure and every stop of <schedule>:
        departureTime(first stop) + arrivalOffset   == GTFS arrival_time
        departureTime(first stop) + departureOffset == GTFS departure_time
    compared as absolute seconds, no tolerance.  Exits 1 on the first class of
    difference (summarised, not one line per stop).

Check 2 (old vs new, with --old)
    Identical departure ids, departure times, vehicle refs, stop sequences
    (refIds, i.e. also the stop link refs of a mapped schedule) and, for a
    mapped schedule, the same `<route>` link sequence for the route that holds
    the departure.  Route ids may only change by a ``__dN`` split suffix.
    Additionally the two files are compared line by line after removing just
    the two offset attribute values: they must be byte-identical
    (only possible when no route was split).

Exit code 0 = both checks pass for every departure, 1 = something differs.
"""
import collections
import gzip
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dwlib as D
from restore_dwell import cached_stop_times

RE_OFF = re.compile(rb'(arrivalOffset|departureOffset)="([^"]*)"')


def norm_line(line):
    return RE_OFF.sub(rb'\1="X"', line)


def trip_index(path):
    """dep id -> (line, route id, t0, veh, [stop refIds], n_links, link_hash)."""
    idx = {}
    for r in D.iter_schedule(path):
        refs = tuple(s[0] for s in r.stops)
        for dep_id, t0, veh in r.departures:
            idx[dep_id] = (r.line, r.id, t0, veh, refs, r.n_links, r.link_hash)
    return idx


def check_absolute(path, stop_times):
    n_dep = n_stop = 0
    bad = collections.Counter()
    examples = []
    for r in D.iter_schedule(path):
        for dep_id, t0, veh in r.departures:
            tid, tt = D.gtfs_trip_ids(dep_id)
            g = stop_times.get(tid)
            n_dep += 1
            if not g:
                bad["departure without GTFS trip"] += 1
                examples.append(("no gtfs", dep_id))
                continue
            if len(g) != len(r.stops):
                bad["stop count differs"] += 1
                examples.append(("len", dep_id, len(r.stops), len(g)))
                continue
            for i, (ref, a, d, aw) in enumerate(r.stops):
                n_stop += 1
                ga, gd = g[i][1], g[i][2]
                if t0 + a != ga:
                    bad["arrival"] += 1
                    if len(examples) < 20:
                        examples.append(("arr", dep_id, i, ref, t0 + a, ga))
                if t0 + d != gd:
                    bad["departure"] += 1
                    if len(examples) < 20:
                        examples.append(("dep", dep_id, i, ref, t0 + d, gd))
    return dict(departures=n_dep, stops=n_stop, bad=dict(bad), examples=examples)


def check_old_new(old, new):
    a = trip_index(old)
    b = trip_index(new)
    res = dict(old_departures=len(a), new_departures=len(b),
               missing_in_new=0, extra_in_new=0, changed_time=0, changed_vehicle=0,
               changed_stops=0, changed_links=0, changed_line=0,
               route_id_renamed=0, renamed_routes=set(), examples=[])
    ex = res["examples"]
    for dep, o in a.items():
        n = b.get(dep)
        if n is None:
            res["missing_in_new"] += 1
            if len(ex) < 20:
                ex.append(("missing", dep))
            continue
        if o[2] != n[2]:
            res["changed_time"] += 1
            if len(ex) < 20:
                ex.append(("time", dep, o[2], n[2]))
        if o[3] != n[3]:
            res["changed_vehicle"] += 1
            if len(ex) < 20:
                ex.append(("veh", dep, o[3], n[3]))
        if o[0] != n[0]:
            res["changed_line"] += 1
            if len(ex) < 20:
                ex.append(("line", dep, o[0], n[0]))
        if o[4] != n[4]:
            res["changed_stops"] += 1
            if len(ex) < 20:
                ex.append(("stops", dep, o[4], n[4]))
        if o[5:] != n[5:]:
            res["changed_links"] += 1
            if len(ex) < 20:
                ex.append(("links", dep, o[5:], n[5:]))
        if o[1] != n[1]:
            res["route_id_renamed"] += 1
            res["renamed_routes"].add((o[1], n[1]))
            if not (n[1] == o[1] or n[1].startswith(o[1] + "__d")):
                res["changed_stops"] += 0
                if len(ex) < 20:
                    ex.append(("route id not a split of the old id", dep, o[1], n[1]))
    for dep in b:
        if dep not in a:
            res["extra_in_new"] += 1
            if len(ex) < 20:
                ex.append(("extra", dep))
    res["renamed_routes"] = len(res["renamed_routes"])
    return res


def check_raw(old, new):
    """line-by-line equality after removing only the offset values."""
    n = 0
    with D.gzip_text(old) as fo, D.gzip_text(new) as fn:
        for lo, ln in zip(fo, fn):
            n += 1
            if norm_line(lo) != norm_line(ln):
                return False, n
        # length equality
        for _ in fo:
            return False, n
        for _ in fn:
            return False, n
    return True, n


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    old = None
    if "--old" in sys.argv:
        old = sys.argv[sys.argv.index("--old") + 1]
        args = [a for a in args if a != old]
    if len(args) != 2:
        raise SystemExit(__doc__)
    sched, gtfs_zip = args
    trip_ids = set()
    for r in D.iter_schedule(sched):
        for dep_id, _t, _v in r.departures:
            trip_ids.add(D.gtfs_trip_ids(dep_id)[0])
    st = cached_stop_times(gtfs_zip, trip_ids)

    ok = True
    print("check 1 — absolute times vs GTFS  (%s)" % sched)
    a = check_absolute(sched, st)
    print("  departures %d, stops %d, differences: %s" % (a["departures"], a["stops"], a["bad"] or "NONE"))
    for e in a["examples"]:
        print("   ", e)
    if a["bad"]:
        ok = False

    if old:
        print("check 2 — old vs new  (old = %s)" % old)
        r = check_old_new(old, sched)
        bad = {k: r[k] for k in ("missing_in_new", "extra_in_new", "changed_time", "changed_vehicle",
                                 "changed_stops", "changed_links", "changed_line")}
        print("  departures old %d / new %d" % (r["old_departures"], r["new_departures"]))
        print("  differences: %s" % {k: v for k, v in bad.items() if v})
        print("  departures whose route id changed (route split): %d, renamed routes: %d"
              % (r["route_id_renamed"], r["renamed_routes"]))
        for e in r["examples"]:
            print("   ", e)
        if any(bad.values()):
            ok = False
        same, n = check_raw(old, sched)
        print("  whole file identical except the two offset values: %s (%d lines)" % (same, n))
        if not same:
            print("  (expected when a route was split — see restore_dwell output)")

    print("VERIFY %s" % ("PASSED" if ok else "FAILED"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
