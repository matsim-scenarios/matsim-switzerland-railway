#!/usr/bin/env python3
"""Reference scorer for a MATSim/railsim run (pipeline step: score).

Round-3 agent DW1's copy of swarm/bench/evaluate.py, taken over as the repository's scorer: the
schedules carry the GTFS dwell times, so a stop has TWO published times (arrival and departure).
Differences to the bench scorer (all original output keys are kept unchanged):

  * the reference check compares the published ARRIVAL *and* DEPARTURE offset of every stop
    (bench evaluate.py already does: signature = station, arrivalOffset, departureOffset, start
    time), and it can pick the reference from the case name: --case olten|ch;
  * the deviation of a stop is reported un-clipped (early is negative), plus the departure
    deviation (departure event - (departureTime + departureOffset)), so earliness can be measured;
  * score_abs = mean |arrival deviation| = the round-3 objective (0 s at every stop).

Usage: evaluate.py <transitSchedule.xml(.gz)> <events.xml(.gz)> [--json out.json] [--stuck N]
                   [--ref refSchedule.xml.gz] [--case olten|ch]

Ground truth is the schedule: every (line, route, departure) must reach every stop of its route.
`--ref` = the untouched reference schedule; the scored schedule must contain exactly the same
departures with the same stop sequence (by parent station) and the same arrival/departure times,
otherwise INVALID.

Metrics
  completion      = departures whose vehicle produced an arrival event at the LAST stop / all departures
  stop_completion = realised stop arrivals / scheduled stop arrivals
  arrival delay   = t_arrival_event - (departureTime + arrivalOffset)   [s], negative values clipped to 0 for means
  score           = mean over ALL scheduled stop arrivals of min(delay, PENALTY), where a missing arrival counts as
                    PENALTY seconds (3600). Lower is better. This is the single number to minimise.
                    (An arrival more than one hour late counts like a missing one - it cannot count worse.)
  on_time_3min_pct= share of ALL scheduled stop arrivals that happened less than 180 s late (the SBB punctuality
                    definition; missing arrivals count as not punctual). Higher is better.
"""
import os, sys, gzip, json, re, statistics, argparse
import xml.etree.ElementTree as ET

PENALTY = 3600.0


def op(path):
    if path.endswith('.gz'):
        return gzip.open(path, 'rb')
    if path.endswith('.zst'):  # MATSim >= 2026 default output compression
        import subprocess
        return subprocess.Popen(['zstd', '-dc', path], stdout=subprocess.PIPE).stdout
    return open(path, 'rb')


def hms(s):
    if s is None:
        return None
    p = s.split(':')
    return int(p[0]) * 3600 + int(p[1]) * 60 + float(p[2])


def station(stop_id):
    """Platform-/link-independent station key: '8500218:0:7.link:123' -> '8500218'."""
    s = stop_id.split('.link:')[0]
    s = s.replace('Parent', '')
    return s.split(':')[0]


def read_schedule(path):
    """-> dict[(line, route, dep)] = dict(t0, stops=[(facility, station, arrOff, depOff)], line_name)"""
    deps = {}
    line = route = None
    stops = []
    line_name = None
    for ev, el in ET.iterparse(op(path), events=('start', 'end')):
        tag = el.tag
        if ev == 'start':
            if tag == 'transitLine':
                line = el.get('id'); line_name = el.get('name')
            elif tag == 'transitRoute':
                route = el.get('id'); stops = []
            continue
        if tag == 'stop' and route is not None:
            a, d = hms(el.get('arrivalOffset')), hms(el.get('departureOffset'))
            if a is None: a = d
            if d is None: d = a
            stops.append((el.get('refId'), station(el.get('refId')), a, d))
        elif tag == 'departure' and route is not None:
            deps[(line, route, el.get('id'))] = dict(t0=hms(el.get('departureTime')), stops=stops, line_name=line_name)
        elif tag == 'transitRoute':
            route = None; el.clear()
        elif tag == 'transitLine':
            line = None; el.clear()
    return deps


def signature(deps):
    """Timetable signature that must be invariant: per departure the (station, arr, dep) list + start time."""
    sig = {}
    for (line, route, dep), d in deps.items():
        key = (line, dep, d['t0'], tuple((s[1], s[2], s[3]) for s in d['stops']))
        sig[key] = sig.get(key, 0) + 1
    return sig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('schedule'); ap.add_argument('events')
    ap.add_argument('--json'); ap.add_argument('--stuck', type=int, default=15)
    ap.add_argument('--ref')
    ap.add_argument('--case', choices=('olten', 'switzerland'),
                    help='pick the default reference matsim_input/<case>/unmapped_schedule.xml.gz')
    a = ap.parse_args()
    if not a.ref and a.case:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        a.ref = os.path.join(root, 'matsim_input', a.case, 'unmapped_schedule.xml.gz')

    deps = read_schedule(a.schedule)
    valid, why = True, ''
    if a.ref:
        ref = signature(read_schedule(a.ref)); cur = signature(deps)
        if ref != cur:
            missing = sum((ref[k] - cur.get(k, 0)) for k in ref if ref[k] > cur.get(k, 0))
            extra = sum((cur[k] - ref.get(k, 0)) for k in cur if cur[k] > ref.get(k, 0))
            valid, why = False, f'timetable differs from reference: {missing} reference departures missing/changed, {extra} unknown'

    veh2dep = {}          # vehicle id -> (line, route, dep)
    arrivals = {}         # (line, route, dep) -> list of (time, facility)
    departs = {}          # (line, route, dep) -> list of (time, facility)
    last_seen = {}        # vehicle -> (time, link)
    t_last = 0.0
    for ev, el in ET.iterparse(op(a.events), events=('end',)):
        if el.tag != 'event':
            continue
        typ = el.get('type'); t = float(el.get('time')); t_last = max(t_last, t)
        if typ == 'TransitDriverStarts':
            key = (el.get('transitLineId'), el.get('transitRouteId'), el.get('departureId'))
            veh2dep[el.get('vehicleId')] = key
            arrivals.setdefault(key, [])
            departs.setdefault(key, [])
        elif typ == 'VehicleArrivesAtFacility':
            key = veh2dep.get(el.get('vehicle'))
            if key is not None:
                arrivals[key].append((t, el.get('facility')))
        elif typ == 'VehicleDepartsAtFacility':
            key = veh2dep.get(el.get('vehicle'))
            if key is not None:
                departs[key].append((t, el.get('facility')))
        elif typ in ('entered link', 'left link', 'vehicle enters traffic'):
            v = el.get('vehicle')
            if v in veh2dep:
                last_seen[v] = (t, el.get('link'), typ)
        el.clear()

    dep2veh = {v: k for k, v in veh2dep.items()}
    n_dep = len(deps); n_started = 0; n_done = 0
    n_stops = 0; n_arr = 0
    delays = []; final_delays = []; stuck = []; dep_delays = []
    by_cat = {}
    for key, d in deps.items():
        stops = d['stops']; n_stops += len(stops)
        cat = re.match(r'[A-Za-z]*', d['line_name'] or '').group(0) or '?'
        c = by_cat.setdefault(cat, dict(departures=0, completed=0))
        c['departures'] += 1
        arr = arrivals.get(key)
        if arr is None:
            stuck.append(dict(line=d['line_name'], route=key[1], departure=key[2], sched_start=d['t0'], state='never started',
                              stops_done=0, stops=len(stops)))
            continue
        n_started += 1
        # match arrivals to stops in order
        i = 0
        for (t, fac) in arr:
            j = i
            while j < len(stops) and stops[j][0] != fac:
                j += 1
            if j == len(stops):
                continue
            delays.append(t - (d['t0'] + stops[j][2]))
            if j == len(stops) - 1:
                final_delays.append(delays[-1])
            i = j + 1
            n_arr += 1
        # departure deviations (same greedy in-order matching on the facility id)
        j = 0
        for (t, fac) in departs.get(key, []):
            while j < len(stops) and stops[j][0] != fac:
                j += 1
            if j == len(stops):
                continue
            dep_delays.append(t - (d['t0'] + stops[j][3]))
            j += 1
        if i == len(stops):
            n_done += 1; c['completed'] += 1
        else:
            v = dep2veh.get(key); ls = last_seen.get(v, (None, None, None))
            stuck.append(dict(line=d['line_name'], route=key[1], departure=key[2], sched_start=d['t0'], state='stuck',
                              stops_done=i, stops=len(stops), next_stop=stops[i][0], last_time=ls[0], last_link=ls[1]))

    pos = [max(x, 0.0) for x in delays]
    pos_sorted = sorted(pos)
    q = lambda p: pos_sorted[min(len(pos_sorted) - 1, int(p * len(pos_sorted)))] if pos_sorted else None
    missing = n_stops - n_arr
    score = (sum(min(x, PENALTY) for x in pos) + PENALTY * missing) / n_stops if n_stops else None
    score_uncapped = (sum(pos) + PENALTY * missing) / n_stops if n_stops else None
    dp = sorted(dep_delays)
    def dq(p):
        return dp[min(len(dp) - 1, int(p * len(dp)))] if dp else None
    res = dict(
        valid=valid, invalid_reason=why,
        departures=n_dep, started=n_started, completed=n_done,
        completion_pct=round(100.0 * n_done / n_dep, 3) if n_dep else None,
        stop_arrivals_scheduled=n_stops, stop_arrivals_realised=n_arr,
        stop_completion_pct=round(100.0 * n_arr / n_stops, 3) if n_stops else None,
        mean_arrival_delay_s=round(statistics.fmean(pos), 2) if pos else None,
        median_arrival_delay_s=round(q(0.5), 2) if pos else None,
        p95_arrival_delay_s=round(q(0.95), 2) if pos else None,
        max_arrival_delay_s=round(max(pos), 2) if pos else None,
        share_arrivals_delay_gt_180s=round(sum(1 for x in pos if x > 180) / len(pos), 4) if pos else None,
        mean_final_stop_delay_s=round(statistics.fmean([max(x, 0) for x in final_delays]), 2) if final_delays else None,
        min_raw_delay_s=round(min(delays), 2) if delays else None,
        early_arrivals_gt_60s=sum(1 for x in delays if x < -60),
        last_event_time_s=t_last,
        on_time_3min_pct=round(100.0 * sum(1 for x in pos if x < 180) / n_stops, 3) if n_stops else None,
        arrivals_gt_1h_late=sum(1 for x in pos if x > 3600),
        score_uncapped=round(score_uncapped, 2) if score_uncapped is not None else None,
        score=round(score, 2) if score is not None else None,
        by_category=by_cat,
        # ---- round-3 additions (deviation = actual - published, EARLY IS NEGATIVE) ----
        mean_arrival_deviation_s=round(statistics.fmean(delays), 2) if delays else None,
        mean_abs_arrival_deviation_s=round(statistics.fmean(abs(x) for x in delays), 2) if delays else None,
        score_abs=round((sum(abs(x) for x in delays) + PENALTY * missing) / n_stops, 2) if n_stops else None,
        median_arrival_deviation_s=round(sorted(delays)[len(delays) // 2], 2) if delays else None,
        p05_arrival_deviation_s=round(sorted(delays)[int(0.05 * len(delays))], 2) if delays else None,
        p95_arrival_deviation_s=round(sorted(delays)[min(len(delays) - 1, int(0.95 * len(delays)))], 2) if delays else None,
        worst_early_s=round(min(delays), 2) if delays else None,
        early_gt_5s=sum(1 for x in delays if x < -5),
        early_gt_30s=sum(1 for x in delays if x < -30),
        early_gt_60s=sum(1 for x in delays if x < -60),
        early_gt_180s=sum(1 for x in delays if x < -180),
        exact_0s=sum(1 for x in delays if abs(x) < 0.5),
        late_gt_1s=sum(1 for x in delays if x > 1),
        late_gt_60s=sum(1 for x in delays if x > 60),
        late_gt_180s=sum(1 for x in delays if x > 180),
        share_in_30s=round(sum(1 for x in delays if abs(x) <= 30) / len(delays), 4) if delays else None,
        share_in_60s=round(sum(1 for x in delays if abs(x) <= 60) / len(delays), 4) if delays else None,
        departure_deviations=len(dep_delays),
        mean_departure_deviation_s=round(statistics.fmean(dep_delays), 2) if dep_delays else None,
        median_departure_deviation_s=round(dp[len(dp) // 2], 2) if dp else None,
        max_departure_delay_s=round(max(dep_delays), 2) if dep_delays else None,
        worst_early_departure_s=round(min(dep_delays), 2) if dep_delays else None,
        late_departures_gt_60s=sum(1 for x in dep_delays if x > 60),
        early_departures_gt_60s=sum(1 for x in dep_delays if x < -60),
    )
    print(json.dumps(res, indent=1))
    stuck.sort(key=lambda s: (s.get('last_link') or '', s['sched_start']))
    print(f'\n# not completed: {len(stuck)} (showing {min(a.stuck, len(stuck))})')
    for s in stuck[:a.stuck]:
        print('  ', s)
    if a.json:
        res['not_completed'] = stuck
        json.dump(res, open(a.json, 'w'), indent=1)
    if not valid:
        print('\nINVALID RUN:', why)
        sys.exit(2)


if __name__ == '__main__':
    main()
