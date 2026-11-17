#!/usr/bin/env python3
"""Pipeline step: restore the published GTFS station dwell times in a MATSim transit schedule.

    python3 prepare/restore_dwell.py <schedule-in.xml.gz> <gtfs.zip> <schedule-out.xml.gz>

Ported from round-3 agent DW1 (`swarm/r3/DW1/restore_dwell.py`), code unchanged; it is run twice by
`scripts/prepare_case.sh` -- once on the unmapped case schedule (which becomes the scoring
reference) and once on the routed, reversal-fixed schedule.  Background (round 3): pt2matsim 24.4
loses the dwell times (`arrivalOffset == departureOffset` at every stop), and a train that leaves a
station at its arrival minute runs ahead of the timetable and blocks punctual trains.

What it changes
---------------
Only the `arrivalOffset` / `departureOffset` attribute values of the `<stop>`
elements of every `<routeProfile>`.  Everything else (DOCTYPE, indentation,
attribute order and spacing, `<route>` link sequences, stop facility ids,
`linkRefId`s, departure ids/times/vehicleRefs, `<attributes>`, gzip) is copied
through byte for byte, so MATSim reads the result exactly like the input.

New offsets (relative to the first stop, whose departure is the departureTime
of the departure):
    arrivalOffset   = GTFS arrival of that stop   - GTFS departure of stop 1
    departureOffset = GTFS departure of that stop - GTFS departure of stop 1
First stop: arrival = departure = 0.  Last stop: departure = arrival
(no trip of the feed has a dwell at its first or last stop; verified).

Route splitting
---------------
pt2matsim merges trips with the same stop sequence AND the same offsets into
one `<transitRoute>` with several `<departure>`s.  If the departures of a route
do not all have the same GTFS arrival/departure pattern, the route is split
into one route per pattern: the first pattern (in file order of the
departures) keeps the old id, the others get `<old id>__d2`, `__d3`, ...  Each
copy carries the full `<route>` link sequence and only its own departures.

Standard library only.
"""
import collections
import gzip
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dwlib as D

RE_A = re.compile(rb'arrivalOffset="[^"]*"')
RE_D = re.compile(rb'departureOffset="[^"]*"')
RE_ROUTE_ID = re.compile(rb'<transitRoute id="([^"]*)"')
RE_DEP_ID = re.compile(rb'<departure id="([^"]*)"')


def station_key(stop_id):
    """platform-independent key: '8500127:0:3' -> '8500127'."""
    return D.strip_link(stop_id).replace("Parent", "").split(":")[0]


def build_plan(routes, stop_times, log=sys.stderr):
    """-> (plan, stats); plan[route_id] = [Segment]."""
    plan = {}
    stats = collections.Counter()
    for r in routes:
        if not r.departures:
            stats["routes_without_departures"] += 1
            plan[r.id] = [Segment(r.id, [(a, d) for _, a, d, _ in r.stops], [])]
            continue
        groups = collections.OrderedDict()   # pattern -> [dep ids]
        pattern_of = {}
        for dep_id, t0, _veh in r.departures:
            tid, tt = D.gtfs_trip_ids(dep_id)
            g = stop_times.get(tid)
            if not g:
                raise SystemExit("restore_dwell: no GTFS stop_times for trip %r (departure %r)"
                                 % (tid, dep_id))
            if len(g) != len(r.stops):
                raise SystemExit(
                    "restore_dwell: departure %r has %d GTFS stops but the route has %d "
                    "(cannot restore the dwell times one-to-one)" % (dep_id, len(g), len(r.stops)))
            for i, (ref, a, d, aw) in enumerate(r.stops):
                gs = g[i][0]
                if D.strip_link(ref) != gs and station_key(ref) != station_key(gs):
                    raise SystemExit(
                        "restore_dwell: departure %r stop %d: route stop %r vs GTFS stop %r"
                        % (dep_id, i, ref, gs))
            if g[0][2] != t0:
                raise SystemExit("restore_dwell: departure %r starts at %s in the schedule but "
                                 "at %s in GTFS" % (dep_id, D.hms_str(t0), D.hms_str(g[0][2])))
            gd0 = g[0][2]
            pat = tuple((ga - gd0, gd - gd0) for _, ga, gd, _ in g)
            # first/last stop rules
            pat = ((0.0, 0.0),) + pat[1:-1] + ((pat[-1][0], pat[-1][0]),) if len(pat) > 1 else ((0.0, 0.0),)
            key = tuple((round(a, 3), round(d, 3)) for a, d in pat)
            groups.setdefault(key, []).append(dep_id)
            pattern_of[dep_id] = key
        segs = []
        for k, (key, deps) in enumerate(groups.items()):
            new_id = r.id if k == 0 else "%s__d%d" % (r.id, k + 1)
            segs.append(Segment(new_id, list(key), deps))
        plan[r.id] = segs
        stats["routes"] += 1
        stats["departures"] += len(r.departures)
        if len(segs) > 1:
            stats["routes_split"] += 1
            stats["routes_created_by_split"] += len(segs) - 1
            log.write("split %s into %d routes: %s\n" % (
                r.id, len(segs), [s.id for s in segs]))
    return plan, stats


class Segment(object):
    __slots__ = ("id", "offsets", "dep_ids")

    def __init__(self, id, offsets, dep_ids):
        self.id = id
        self.offsets = offsets      # [(arr_s, dep_s)]
        self.dep_ids = dep_ids      # [dep id] (bytes later)

    def __repr__(self):
        return "Segment(%s, %d deps)" % (self.id, len(self.dep_ids))


def emit_block(fout, block, plan, stats):
    """Rewrite one buffered <transitRoute>...</transitRoute> block."""
    rid = RE_ROUTE_ID.search(block[0]).group(1).decode()
    segs = plan.get(rid)
    if segs is None:
        raise SystemExit("restore_dwell: no plan for route %r" % rid)
    stop_lines = []
    dep_lines = []
    tmpl = []
    for line in block[1:]:
        ls = line.lstrip()
        if ls.startswith(b"<stop "):
            stop_lines.append(line)
        elif ls.startswith(b"<departure "):
            dep_lines.append(line)
        else:
            tmpl.append(line)
    if len(stop_lines) != len(segs[0].offsets):
        raise SystemExit("restore_dwell: route %r has %d <stop> lines but the plan %d "
                         "(file and parser disagree)" % (rid, len(stop_lines), len(segs[0].offsets)))
    if len(dep_lines) != sum(len(s.dep_ids) for s in segs):
        raise SystemExit("restore_dwell: route %r: %d <departure> lines, plan has %d"
                         % (rid, len(dep_lines), sum(len(s.dep_ids) for s in segs)))
    dep_by_id = {}
    for line in dep_lines:
        m = RE_DEP_ID.search(line)
        dep_by_id[m.group(1).decode()] = line

    for seg in segs:
        stats["routes_out"] += 1
        if seg.id == rid:
            fout.write(block[0])
        else:
            fout.write(block[0].replace(b'id="%s"' % rid.encode(), b'id="%s"' % seg.id.encode(), 1))
        stop_idx = 0
        for line in tmpl:
            fout.write(line)
            if b"<routeProfile" in line:
                for i, (a, d) in enumerate(seg.offsets):
                    l = stop_lines[i]
                    l = RE_A.sub(b'arrivalOffset="%s"' % D.hms_str(a).encode(), l)
                    l = RE_D.sub(b'departureOffset="%s"' % D.hms_str(d).encode(), l)
                    fout.write(l)
                    stop_idx += 1
            elif b"<departures" in line:
                for did in seg.dep_ids:
                    fout.write(dep_by_id[did])
        if stop_idx != len(stop_lines):
            raise SystemExit("restore_dwell: route %r: emitted %d stops, expected %d"
                             % (rid, stop_idx, len(stop_lines)))


def rewrite(in_path, out_path, plan, stats):
    fin = D.gzip_text(in_path)
    if out_path.endswith(".gz"):
        fout = gzip.open(out_path, "wb", compresslevel=6)
    else:
        fout = open(out_path, "wb")
    in_block = False
    block = []
    try:
        for line in fin:
            if not in_block:
                if line.lstrip().startswith(b"<transitRoute "):
                    in_block = True
                    block = [line]
                else:
                    fout.write(line)
            else:
                block.append(line)
                if b"</transitRoute>" in line:
                    in_block = False
                    emit_block(fout, block, plan, stats)
                    block = []
        if in_block:
            raise SystemExit("restore_dwell: unterminated <transitRoute> block")
    finally:
        fout.close()
        fin.close()


def restore(in_path, gtfs_zip, out_path, stop_times=None, log=sys.stderr):
    routes, n_dep = D.read_schedule(in_path)
    if stop_times is None:
        trip_ids = set()
        for r in routes:
            for dep_id, _t, _v in r.departures:
                trip_ids.add(D.gtfs_trip_ids(dep_id)[0])
        log.write("streaming %s for %d trip ids ...\n" % (gtfs_zip, len(trip_ids)))
        stop_times = D.load_gtfs_stop_times(gtfs_zip, trip_ids, verbose=True)
    plan, pstats = build_plan(routes, stop_times, log=log)
    stats = collections.Counter()
    stats.update(pstats)
    rewrite(in_path, out_path, plan, stats)
    return stats


CACHE = os.environ.get("DWELL_CACHE") or None


def cached_stop_times(gtfs_zip, trip_ids, cache=CACHE):
    """Read the stop_times of trip_ids from the local json cache if it covers
    them, else stream the zip once and refresh the cache."""
    import json
    if cache and os.path.exists(cache):
        with gzip.open(cache, "rt") as f:
            data = json.load(f)
        if set(data) >= set(trip_ids):
            return {k: [tuple(r) for r in v] for k, v in data.items() if k in trip_ids}
    st = D.load_gtfs_stop_times(gtfs_zip, trip_ids, verbose=True)
    if cache:
        import json
        with gzip.open(cache, "wt") as f:
            json.dump({k: [list(r) for r in v] for k, v in st.items()}, f)
    return st


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 3:
        raise SystemExit(__doc__)
    in_path, gtfs_zip, out_path = args
    routes, _ = D.read_schedule(in_path)
    trip_ids = set(D.gtfs_trip_ids(dep_id)[0] for r in routes for dep_id, _t, _v in r.departures)
    st = cached_stop_times(gtfs_zip, trip_ids)
    stats = restore(in_path, gtfs_zip, out_path, stop_times=st)
    print("restore_dwell: %s -> %s" % (in_path, out_path))
    print("  routes %d (out %d), departures %d" % (
        stats["routes"], stats["routes_out"], stats["departures"]))
    print("  routes split: %d (extra routes %d)" % (
        stats["routes_split"], stats["routes_created_by_split"]))


if __name__ == "__main__":
    main()
