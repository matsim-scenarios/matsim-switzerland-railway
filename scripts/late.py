#!/usr/bin/env python3
"""Late-arrival statistics for one MATSim/railsim experiment.

Usage:  python3 scripts/late.py <experiment-dir> [--top 30]

Reads the schedule named in <experiment-dir>/config.xml (transit module,
``transitScheduleFile``) and the events of the run
(``output/*output_events.xml.gz``, or ``output/ITERS/it.*/*.events.xml.gz``),
in ONE streaming pass, and writes:

  <experiment-dir>/late.csv   one row per scheduled stop of every departure
  <experiment-dir>/late.json  aggregate late-arrival statistics
  (plus the same summary on stdout)

Vehicle -> departure matching and arrival-event -> stop matching are done
exactly like ``swarm/bench/evaluate.py`` (greedy in-order matching on the
facility id).  Delay = actual - scheduled in seconds (negative = early).

Standard library only.
"""

import argparse
import glob
import gzip
import json
import os
import re
import statistics
import sys
import xml.etree.ElementTree as ET

PENALTY = 3600.0  # only used to report delays above one hour separately


# ------------------------------------------------------------------ helpers
def op(path):
    if path.endswith(".gz"):
        return gzip.open(path, "rb")
    if path.endswith(".zst"):
        import subprocess
        return subprocess.Popen(["zstd", "-dc", path], stdout=subprocess.PIPE).stdout
    return open(path, "rb")


def hms(s):
    if s is None:
        return None
    p = s.split(":")
    return int(p[0]) * 3600 + int(p[1]) * 60 + float(p[2])


def station(stop_id):
    """Platform-/link-independent station key (same as evaluate.py)."""
    s = stop_id.split(".link:")[0]
    s = s.replace("Parent", "")
    return s.split(":")[0]


def first_existing(patterns):
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def hhmmss(t):
    if t is None:
        return "-"
    return "%02d:%02d:%02d" % (int(t) // 3600, (int(t) % 3600) // 60, int(t) % 60)


# ------------------------------------------------------------------ inputs
def read_config(exp_dir):
    cfg = os.path.join(exp_dir, "config.xml")
    sched = None
    if os.path.exists(cfg):
        for _, el in ET.iterparse(cfg, events=("end",)):
            if el.tag == "module" and el.get("name") == "transit":
                for p in el.findall("param"):
                    if p.get("name") == "transitScheduleFile":
                        sched = p.get("value")
                el.clear()
    cand = os.path.join(exp_dir, sched) if sched else None
    if cand and os.path.exists(cand):
        return cand
    for name in ("transitSchedule.xml.gz", "transitSchedule.xml"):
        p = os.path.join(exp_dir, name)
        if os.path.exists(p):
            return p
    raise SystemExit("late.py: no schedule found in %s (config says %r)" % (exp_dir, sched))


def find_events(exp_dir):
    p = first_existing([
        os.path.join(exp_dir, "output", "*output_events.xml.gz"),
        os.path.join(exp_dir, "output", "ITERS", "it.*", "*.events.xml.gz"),
        os.path.join(exp_dir, "output", "*.events.xml.gz"),
    ])
    if not p:
        raise SystemExit("late.py: no events file under %s/output" % exp_dir)
    return p


def read_schedule(path):
    """-> departures dict, facilities dict (id -> (name, station, linkRefId))"""
    deps = {}
    facilities = {}
    line = route = None
    stops = []
    line_name = None
    for ev, el in ET.iterparse(op(path), events=("start", "end")):
        tag = el.tag
        if ev == "start":
            if tag == "transitLine":
                line = el.get("id")
                line_name = el.get("name")
            elif tag == "transitRoute":
                route = el.get("id")
                stops = []
            elif tag == "stopFacility":
                facilities[el.get("id")] = (
                    el.get("name") or "",
                    station(el.get("id")),
                    el.get("linkRefId"),
                )
            continue
        if tag == "stop" and route is not None:
            a, d = hms(el.get("arrivalOffset")), hms(el.get("departureOffset"))
            if a is None:
                a = d
            if d is None:
                d = a
            stops.append((el.get("refId"), station(el.get("refId")), a, d))
        elif tag == "departure" and route is not None:
            deps[(line, route, el.get("id"))] = dict(
                t0=hms(el.get("departureTime")), stops=stops, line_name=line_name)
        elif tag == "transitRoute":
            route = None
            el.clear()
        elif tag == "transitLine":
            line = None
            el.clear()
    return deps, facilities


# ------------------------------------------------------------------ events
def scan_events(path, deps):
    """One pass. -> (arrivals, departures, started, finished, t_last)

    arrivals/departures: key -> {stop_index: time}, matched greedily in stop
    order exactly like evaluate.py.

    The events file is parsed line by line (one <event .../> per line) with a
    bytes regex: ~8x faster than ElementTree on a 400 MB events file.
    """
    RE_ATTR = re.compile(rb'([A-Za-z_]+)="([^"]*)"')
    veh2dep = {}
    arrivals = {}
    departures = {}
    ptr_a = {}
    ptr_d = {}
    t_last = 0.0
    n_events = 0
    NA = b"VehicleArrivesAtFacility"
    ND = b"VehicleDepartsAtFacility"
    NS = b"TransitDriverStarts"
    for line in op(path):
        if NA in line:
            typ = 1
        elif ND in line:
            typ = 2
        elif NS in line:
            typ = 3
        else:
            continue
        if b"<event" not in line:
            continue
        a = dict(RE_ATTR.findall(line))
        t = float(a.get(b"time") or 0.0)
        if t > t_last:
            t_last = t
        if typ == 3:
            key = (a.get(b"transitLineId", b"").decode(),
                   a.get(b"transitRouteId", b"").decode(),
                   a.get(b"departureId", b"").decode())
            veh2dep[a.get(b"vehicleId", b"")] = key
            arrivals.setdefault(key, {})
            departures.setdefault(key, {})
            ptr_a.setdefault(key, 0)
            ptr_d.setdefault(key, 0)
        else:
            key = veh2dep.get(a.get(b"vehicle"))
            if key is None:
                continue
            d = deps.get(key)
            if d is None:
                continue
            stops = d["stops"]
            fac = a.get(b"facility")
            if typ == 1:
                i = ptr_a[key]
                j = i
                while j < len(stops) and stops[j][0].encode() != fac:
                    j += 1
                if j < len(stops):
                    arrivals[key][j] = t
                    ptr_a[key] = j + 1
            else:
                i = ptr_d[key]
                j = i
                while j < len(stops) and stops[j][0].encode() != fac:
                    j += 1
                if j < len(stops):
                    departures[key][j] = t
                    ptr_d[key] = j + 1
        n_events += 1
    return arrivals, departures, veh2dep, ptr_a, t_last, n_events


# ------------------------------------------------------------------ analysis
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--top", type=int, default=30)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    exp = a.experiment.rstrip("/")

    sched_path = read_config(exp)
    events_path = find_events(exp)
    deps, facilities = read_schedule(sched_path)
    arrivals, departures, veh2dep, ptr_a, t_last, n_events = scan_events(events_path, deps)

    rows = []
    delays = []            # all scheduled stops that have an arrival event
    delayed_keys = []      # (delay, key, idx) for ranking
    final_delays = []
    late_by_station = {}
    late_by_line = {}
    n_sched = 0
    n_arr = 0
    n_done = 0
    n_started = 0
    trains_late = [0, 0, 0]  # number of trains with >=1 arrival late by >0/>60/>180
    for key, d in deps.items():
        line, route, dep = key
        stops = d["stops"]
        n_sched += len(stops)
        name = d["line_name"] or line
        arr = arrivals.get(key, {})
        depp = departures.get(key, {})
        if key not in ptr_a:
            # departure never started (no TransitDriverStarts)
            for i, s in enumerate(stops):
                rows.append(row_of(line, name, route, dep, i, stops, None, None, None, None, facilities))
            continue
        n_started += 1
        done = ptr_a[key] >= len(stops)
        if done:
            n_done += 1
        tr_late = [0, 0, 0]
        for i, s in enumerate(stops):
            fac, st, sched_a, sched_d = s
            ta = arr.get(i)
            td = depp.get(i)
            da = dd = None
            if ta is not None:
                da = ta - (d["t0"] + sched_a)
                delays.append(da)
                n_arr += 1
                delayed_keys.append((da, key, i))
                if da > 0:
                    nm = facilities.get(fac, ("?",))[0]
                    late_by_station.setdefault(nm, [0, 0.0])
                    late_by_station[nm][0] += 1
                    late_by_station[nm][1] += da
                    late_by_line.setdefault(name, [0, 0.0])
                    late_by_line[name][0] += 1
                    late_by_line[name][1] += da
                    if da > 60:
                        tr_late[1] += 1
                    if da > 180:
                        tr_late[2] += 1
                if i == len(stops) - 1:
                    final_delays.append(da)
            if td is not None:
                dd = td - (d["t0"] + sched_d)
            if da is not None and da > 0:
                tr_late[0] += 1
            rows.append(row_of(line, name, route, dep, i, stops, ta, da, td, dd, facilities, sched_a, sched_d,
                               d["t0"]))
        for k in range(3):
            if tr_late[k]:
                trains_late[k] += 1

    pos = [max(x, 0.0) for x in delays]
    pos_sorted = sorted(pos)
    q = lambda p: pos_sorted[min(len(pos_sorted) - 1, int(p * len(pos_sorted)))] if pos_sorted else None
    fpos = [max(x, 0.0) for x in final_delays]
    n_all = len(delays)
    missing = n_sched - n_arr

    def block(vals, n):
        p = [max(x, 0.0) for x in vals]
        if not n:
            return dict(n=0)
        return dict(
            n=n,
            late_gt_0=sum(1 for x in p if x > 0),
            late_gt_0_pct=round(100.0 * sum(1 for x in p if x > 0) / n, 2),
            late_gt_60=sum(1 for x in p if x > 60),
            late_gt_60_pct=round(100.0 * sum(1 for x in p if x > 60) / n, 2),
            late_gt_180=sum(1 for x in p if x > 180),
            late_gt_180_pct=round(100.0 * sum(1 for x in p if x > 180) / n, 2),
            max_delay_s=round(max(p), 1) if p else None,
            mean_positive_delay_s=round(statistics.fmean([x for x in p if x > 0]), 2) if any(x > 0 for x in p) else 0.0,
            sum_positive_delay_s=round(sum(p), 1),
            gt_1h=sum(1 for x in p if x > PENALTY),
        )

    delayed_keys.sort(key=lambda r: -r[0])
    worst = []
    for da, key, i in delayed_keys[:a.top]:
        d = deps[key]
        fac = d["stops"][i][0]
        worst.append(dict(
            delay_s=round(da, 1), line=d["line_name"] or key[0], route=key[1], departure=key[2],
            stop_index=i, n_stops=len(d["stops"]),
            facility=fac, station=facilities.get(fac, ("?",))[0],
            sched_arr=round(d["t0"] + d["stops"][i][2], 1),
            act_arr=round(d["t0"] + d["stops"][i][2] + da, 1),
        ))

    res = dict(
        experiment=exp,
        schedule=sched_path,
        events=events_path,
        last_event_time_s=round(t_last, 1),
        departures=len(deps),
        started=n_started,
        completed=n_done,
        completion_pct=round(100.0 * n_done / len(deps), 3) if deps else None,
        scheduled_arrivals=n_sched,
        realised_arrivals=n_arr,
        missing_arrivals=missing,
        stop_completion_pct=round(100.0 * n_arr / n_sched, 3) if n_sched else None,
        arrivals=block(delays, n_all),
        last_stop=block(final_delays, len(final_delays)),
        trains_with_late_arrival=dict(
            gt_0=trains_late[0], gt_60=trains_late[1], gt_180=trains_late[2]),
        n_trains=len(deps),
        late_by_station_top=[
            dict(station=k, late_arrivals=v[0], late_seconds=round(v[1], 1))
            for k, v in sorted(late_by_station.items(), key=lambda kv: -kv[1][1])[:15]],
        late_by_line_top=[
            dict(line=k, late_arrivals=v[0], late_seconds=round(v[1], 1))
            for k, v in sorted(late_by_line.items(), key=lambda kv: -kv[1][1])[:15]],
        worst_arrivals=worst,
    )

    # ---- write late.csv
    csv_path = os.path.join(exp, "late.csv")
    with open(csv_path, "w") as f:
        f.write("line,route,departure,stop_index,n_stops,facility,station_name,sched_arr,act_arr,arr_delay,"
                "sched_dep,act_dep,dep_delay\n")
        for r in rows:
            f.write(r + "\n")

    json_path = os.path.join(exp, "late.json")
    with open(json_path, "w") as f:
        json.dump(res, f, indent=1)

    if not a.quiet:
        print_summary(res, a.top)
    print("wrote %s" % csv_path, file=sys.stderr)
    print("wrote %s" % json_path, file=sys.stderr)


def row_of(line, name, route, dep, i, stops, ta, da, td, dd, facilities, sched_a=None, sched_d=None, t0=0.0):
    fac, st, sa, sd = stops[i]
    sa = t0 + sa
    sd = t0 + sd
    return "%s,%s,%s,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s" % (
        csvq(name), csvq(route), csvq(dep), i, len(stops), csvq(fac),
        csvq(facilities.get(fac, ("?",))[0]),
        fmt(sa), fmt(ta), fmt(da), fmt(sd), fmt(td), fmt(dd))


def fmt(v):
    if v is None:
        return ""
    return "%.0f" % v


def csvq(s):
    s = "" if s is None else str(s)
    if any(c in s for c in ',"\n'):
        return '"%s"' % s.replace('"', '""')
    return s


def print_summary(r, top):
    a = r["arrivals"]
    out = []
    out.append("== late.py  %s" % r["experiment"])
    out.append("schedule %s" % r["schedule"])
    out.append("events   %s  (last event %s)" % (r["events"], hhmmss(r["last_event_time_s"])))
    out.append("")
    out.append("departures %d   started %d   completed %d (%.2f%%)"
               % (r["departures"], r["started"], r["completed"], r["completion_pct"]))
    out.append("scheduled stop arrivals %d   realised %d   missing %d"
               % (r["scheduled_arrivals"], r["realised_arrivals"], r["missing_arrivals"]))
    out.append("")
    out.append("ALL %d realised stop arrivals:" % a["n"])
    out.append("  late >   0 s : %5d  (%6.2f %%)" % (a["late_gt_0"], a["late_gt_0_pct"]))
    out.append("  late >  60 s : %5d  (%6.2f %%)" % (a["late_gt_60"], a["late_gt_60_pct"]))
    out.append("  late > 180 s : %5d  (%6.2f %%)" % (a["late_gt_180"], a["late_gt_180_pct"]))
    out.append("  > 1 h        : %5d" % a["gt_1h"])
    out.append("  max delay    : %s s (%.1f min)" % (a["max_delay_s"], (a["max_delay_s"] or 0) / 60.0))
    out.append("  mean positive: %.1f s   sum positive: %.0f s (%.1f h)"
               % (a["mean_positive_delay_s"], a["sum_positive_delay_s"], a["sum_positive_delay_s"] / 3600.0))
    b = r["last_stop"]
    out.append("LAST STOP (%d arrivals): >0 %d (%.2f%%), >60 %d (%.2f%%), >180 %d (%.2f%%), max %s s, sum %.0f s"
               % (b["n"], b["late_gt_0"], b["late_gt_0_pct"], b["late_gt_60"], b["late_gt_60_pct"],
                  b["late_gt_180"], b["late_gt_180_pct"], b["max_delay_s"], b["sum_positive_delay_s"]))
    t = r["trains_with_late_arrival"]
    out.append("TRAINS (%d) with >=1 late arrival: >0 %d, >60 %d, >180 %d"
               % (r["n_trains"], t["gt_0"], t["gt_60"], t["gt_180"]))
    out.append("")
    out.append("late arrivals per station (top 15, by late seconds):")
    for s in r["late_by_station_top"]:
        out.append("  %-38s %5d arrivals  %8.0f s" % (s["station"][:38], s["late_arrivals"], s["late_seconds"]))
    out.append("late arrivals per line (top 15, by late seconds):")
    for s in r["late_by_line_top"]:
        out.append("  %-38s %5d arrivals  %8.0f s" % (s["line"][:38], s["late_arrivals"], s["late_seconds"]))
    out.append("")
    out.append("worst %d arrivals:" % top)
    out.append("   delay  line                          departure           stop             sched      act")
    for w in r["worst_arrivals"]:
        out.append("  %6.0f  %-28s  %-18s %2d/%-2d %-12s %s  %s"
                   % (w["delay_s"], (w["line"] or "")[:28], w["departure"], w["stop_index"] + 1, w["n_stops"],
                      w["station"][:12], hhmmss(w["sched_arr"]), hhmmss(w["act_arr"])))
    print("\n".join(out))


if __name__ == "__main__":
    main()
