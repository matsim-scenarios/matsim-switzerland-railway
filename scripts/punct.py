#!/usr/bin/env python3
"""Punctuality metric for MATSim/railsim runs: LATE **and** EARLY at every stop.

Usage:  python3 scripts/punct.py <experiment-dir> [--ref <published schedule.xml.gz>] [--top 30]

Round 1-2 only punished late arrivals.  The goal now is ``deviation = actual - published == 0``
at every stop, so an early train is as wrong as a late one.

Reads
  * the schedule the run used: the file named in ``<exp>/config.xml`` (transit module,
    ``transitScheduleFile``), else ``transitSchedule.xml.gz`` / ``transitSchedule.xml`` in <exp>;
  * the events: ``<exp>/output/*output_events.xml.gz`` (or ``output/ITERS/it.*/*.events.xml.gz``),
    streamed in one pass;
  * optional ``--ref``: the untouched published schedule.  Published times then come from it,
    matched by departure id and stop index (the run's schedule may be base7d, the ref is what
    the timetable says).

Per scheduled stop of every departure:  published arrival / departure, actual arrival /
departure (VehicleArrivesAtFacility / VehicleDepartsAtFacility), dev_arr, dev_dep.
The first stop of a run has no arrival to judge (a train materialises there at its departure
time) -> only its departure is scored.  The last stop has no departure.

Vehicle -> departure and event -> stop matching are done exactly like
``swarm/bench/evaluate.py`` and ``swarm/r2/tools/late.py``: greedy in stop order on the
facility id, so repeated visits of one platform are matched to the right stop index.

Writes  <exp>/punct.csv, <exp>/punct.json  and prints the summary.
HEADLINE (last line): abs_dev_seconds = sum |dev_arr| over all judged scheduled arrivals,
a missing arrival counting as 3600 s.

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

PENALTY = 3600.0


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
    s = stop_id.split(".link:")[0].replace("Parent", "")
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
    t = int(round(t))
    return "%02d:%02d:%02d" % (t // 3600, (t % 3600) // 60, t % 60)


def csvq(s):
    s = "" if s is None else str(s)
    if any(c in s for c in ',"\n'):
        return '"%s"' % s.replace('"', '""')
    return s


def fmt(v):
    return "" if v is None else "%.1f" % v


def pct(k, n):
    return (100.0 * k / n) if n else 0.0


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
    raise SystemExit("punct.py: no schedule found in %s (config says %r)" % (exp_dir, sched))


def find_events(exp_dir):
    p = first_existing([
        os.path.join(exp_dir, "output", "*output_events.xml.gz"),
        os.path.join(exp_dir, "output", "ITERS", "it.*", "*.events.xml.gz"),
        os.path.join(exp_dir, "output", "*.events.xml.gz"),
    ])
    if not p:
        raise SystemExit("punct.py: no events file under %s/output" % exp_dir)
    return p


def read_schedule(path):
    """-> departures {(line,route,dep): dict(t0, stops, line_name, route_desc)}, facilities."""
    deps = {}
    facilities = {}
    line = route = None
    stops = []
    line_name = None
    route_desc = None
    for ev, el in ET.iterparse(op(path), events=("start", "end")):
        tag = el.tag
        if ev == "start":
            if tag == "transitLine":
                line = el.get("id")
                line_name = el.get("name")
            elif tag == "transitRoute":
                route = el.get("id")
                route_desc = el.get("description")
                stops = []
            elif tag == "stopFacility":
                facilities[el.get("id")] = (el.get("name") or "", station(el.get("id")),
                                            el.get("linkRefId"))
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
                t0=hms(el.get("departureTime")), stops=stops, line_name=line_name,
                route_desc=route_desc)
        elif tag == "transitRoute":
            route = None
            el.clear()
        elif tag == "transitLine":
            line = None
            el.clear()
    return deps, facilities


def read_ref(path):
    """-> {departure_id: dict(t0, stops, line_name, line)} from the published schedule."""
    deps, fac = read_schedule(path)
    out = {}
    for (line, route, dep), d in deps.items():
        out[dep] = dict(t0=d["t0"], stops=d["stops"], line_name=d["line_name"], line=line,
                        route=route)
    return out, fac


# ------------------------------------------------------------------ events
RE_ATTR = re.compile(rb'([A-Za-z_]+)="([^"]*)"')


def scan_events(path, deps):
    veh2dep = {}
    arrivals = {}
    departures = {}
    ptr_a = {}
    ptr_d = {}
    t_last = 0.0
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
                j = ptr_a[key]
                while j < len(stops) and stops[j][0].encode() != fac:
                    j += 1
                if j < len(stops):
                    arrivals[key][j] = t
                    ptr_a[key] = j + 1
            else:
                j = ptr_d[key]
                while j < len(stops) and stops[j][0].encode() != fac:
                    j += 1
                if j < len(stops):
                    departures[key][j] = t
                    ptr_d[key] = j + 1
    return arrivals, departures, veh2dep, ptr_a, t_last


# ------------------------------------------------------------------ analysis
def block(dev, n_sched):
    """Statistics of a list of signed deviations (realised only) + the missing count."""
    n = len(dev)
    if not n:
        return dict(n=0, realised=0, missing=n_sched, share_within_1s_pct=0.0)
    absd = [abs(x) for x in dev]
    early = [-x for x in dev if x < 0]
    late = [x for x in dev if x > 0]
    s = sorted(dev)
    q = lambda p: s[min(n - 1, int(p * n))]
    return dict(
        n=n_sched, realised=n, missing=max(n_sched - n, 0),
        within_1s=sum(1 for x in absd if x <= 1.0),
        within_1s_pct=round(pct(sum(1 for x in absd if x <= 1.0), n_sched), 3),
        within_5s=sum(1 for x in absd if x <= 5.0),
        within_5s_pct=round(pct(sum(1 for x in absd if x <= 5.0), n_sched), 3),
        within_30s=sum(1 for x in absd if x <= 30.0),
        within_30s_pct=round(pct(sum(1 for x in absd if x <= 30.0), n_sched), 3),
        within_60s=sum(1 for x in absd if x <= 60.0),
        within_60s_pct=round(pct(sum(1 for x in absd if x <= 60.0), n_sched), 3),
        within_180s=sum(1 for x in absd if x <= 180.0),
        within_180s_pct=round(pct(sum(1 for x in absd if x <= 180.0), n_sched), 3),
        early_gt_1=sum(1 for x in dev if x < -1.0),
        early_gt_5=sum(1 for x in dev if x < -5.0),
        early_gt_30=sum(1 for x in dev if x < -30.0),
        early_gt_60=sum(1 for x in dev if x < -60.0),
        early_gt_180=sum(1 for x in dev if x < -180.0),
        late_gt_0=len(late),
        late_gt_5=sum(1 for x in late if x > 5.0),
        late_gt_30=sum(1 for x in late if x > 30.0),
        late_gt_60=sum(1 for x in late if x > 60.0),
        late_gt_180=sum(1 for x in late if x > 180.0),
        late_gt_3600=sum(1 for x in late if x > 3600.0),
        mean_abs_s=round(statistics.fmean(absd), 2),
        mean_signed_s=round(statistics.fmean(dev), 2),
        median_signed_s=round(q(0.5), 2),
        p05_s=round(q(0.05), 2), p95_s=round(q(0.95), 2),
        min_s=round(min(dev), 1), max_s=round(max(dev), 1),
        sum_early_s=round(sum(early), 1), sum_late_s=round(sum(late), 1),
        sum_abs_s=round(sum(absd), 1),
        n_early_trains=0,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment")
    ap.add_argument("--ref")
    ap.add_argument("--top", type=int, default=30)
    a = ap.parse_args()
    exp = a.experiment.rstrip("/")

    sched_path = read_config(exp)
    events_path = find_events(exp)
    deps, facilities = read_schedule(sched_path)
    refmap = ref_deps = None
    n_ref_matched = n_ref_missing = 0
    if a.ref:
        refmap, ref_fac = read_ref(a.ref)
        for fac, v in ref_fac.items():
            facilities.setdefault(fac, v)

    # ---- replace published times by the reference where asked, keyed by departure id
    if refmap is not None:
        for key, d in deps.items():
            r = refmap.get(key[2])
            if r is not None and len(r["stops"]) == len(d["stops"]):
                d["ref_t0"], d["ref_stops"] = r["t0"], r["stops"]
                n_ref_matched += 1
            else:
                n_ref_missing += 1

    arrivals, departures, veh2dep, ptr_a, t_last = scan_events(events_path, deps)

    rows = []
    dev_arr, dev_dep = [], []
    n_sched_arr = n_sched_dep = 0
    n_arr = n_dep_ev = 0
    n_departures = len(deps)
    n_started = n_done = 0
    abs_sum = 0.0
    n_judged_missing = 0
    by_station = {}
    by_line = {}
    worst = []
    trains_ok = 0            # trains with every judged arrival within 1 s
    trains_bad = []

    for key, d in deps.items():
        line, route, dep = key
        stops = d["stops"]
        r_stops = d.get("ref_stops", stops)
        t0 = d.get("ref_t0", d["t0"])
        name = d["line_name"] or line
        arr = arrivals.get(key, {})
        depp = departures.get(key, {})
        started = key in ptr_a
        if started:
            n_started += 1
            if ptr_a[key] >= len(stops):
                n_done += 1
        worst_tr = 0.0
        for i, s in enumerate(stops):
            fac, st, sa, sd = r_stops[i] if i < len(r_stops) else s
            sa = t0 + (sa if sa is not None else 0.0)
            sd = t0 + (sd if sd is not None else 0.0)
            ta, td = arr.get(i), depp.get(i)
            judge_a = started and i > 0
            judge_d = started and i < len(stops) - 1
            da = (ta - sa) if (judge_a and ta is not None) else None
            dd = (td - sd) if (judge_d and td is not None) else None
            if judge_a:
                n_sched_arr += 1
                if ta is not None:
                    n_arr += 1
                    dev_arr.append(da)
                    abs_sum += abs(da)
                    worst_tr = max(worst_tr, abs(da))
                else:
                    n_judged_missing += 1
                    abs_sum += PENALTY
            if judge_d:
                n_sched_dep += 1
                if td is not None:
                    n_dep_ev += 1
                    dev_dep.append(dd)
            if ta is not None:
                nm = facilities.get(fac, ("?",))[0]
                if i > 0:
                    for tgt, k in ((by_station, nm), (by_line, name)):
                        e = tgt.setdefault(k, [0, 0.0, 0.0, 0])
                        e[0] += 1
                        e[1] += abs(da)
                        e[2] += da
                        e[3] += 1 if da < -5 else 0
                if i > 0 and da is not None:
                    worst.append((abs(da), da, name, route, dep, i, len(stops), st, nm,
                                  sa, ta, sd, td))
            rows.append("%s,%s,%s,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d" % (
                csvq(dep), csvq(name), csvq(route), i, len(stops), csvq(fac),
                csvq(facilities.get(fac, ("?",))[0]),
                fmt(sa), fmt(ta), fmt(da), fmt(sd), fmt(td), fmt(dd),
                1 if (judge_a and ta is not None) else 0,
                1 if (judge_d and td is not None) else 0))
        if started:
            if worst_tr <= 1.0 and ptr_a[key] >= len(stops):
                trains_ok += 1
            else:
                trains_bad.append((worst_tr, name, dep, ptr_a[key], len(stops)))

    A = block(dev_arr, n_sched_arr)
    D = block(dev_dep, n_sched_dep)
    A["late_gt_0"] = sum(1 for x in dev_arr if x > 0)
    A["trains_all_within_1s"] = trains_ok
    A["n_trains_judged"] = n_started

    st_top = sorted(by_station.items(), key=lambda kv: -kv[1][1])[:a.top]
    li_top = sorted(by_line.items(), key=lambda kv: -kv[1][1])[:a.top]
    worst.sort(key=lambda r: -r[0])

    res = dict(
        experiment=exp, schedule=sched_path, events=events_path,
        ref=a.ref, ref_matched=n_ref_matched, ref_missing=n_ref_missing,
        last_event_time_s=round(t_last, 1),
        departures=n_departures, started=n_started, completed=n_done,
        completion_pct=round(pct(n_done, n_departures), 3),
        judged_arrivals=n_sched_arr, realised_arrivals=n_arr,
        missing_arrivals=n_judged_missing,
        stop_completion_pct=round(pct(n_arr, n_sched_arr), 3),
        judged_departures=n_sched_dep, realised_departures=n_dep_ev,
        departures_before_published=sum(1 for x in dev_dep if x < 0),
        abs_dev_seconds=round(abs_sum, 1),
        arrivals=A, departures_block=D,
        by_station=[dict(station=k, arrivals=v[0], sum_abs_dev_s=round(v[1], 1),
                         mean_signed_s=round(v[2] / v[0], 1), early_gt_5=v[3])
                    for k, v in st_top],
        by_line=[dict(line=k, arrivals=v[0], sum_abs_dev_s=round(v[1], 1),
                      mean_signed_s=round(v[2] / v[0], 1), early_gt_5=v[3])
                 for k, v in li_top],
        worst_arrivals=[dict(abs_dev_s=round(r[0], 1), dev_s=round(r[1], 1), line=r[2],
                             route=r[3], departure=r[4], stop_index=r[5], n_stops=r[6],
                             station=r[8], facility=r[7], sched_arr=round(r[9], 1),
                             act_arr=round(r[10], 1), sched_dep=round(r[11], 1),
                             act_dep=round(r[12], 1) if r[12] is not None else None)
                            for r in worst[:a.top]],
    )

    csv_path = os.path.join(exp, "punct.csv")
    with open(csv_path, "w") as f:
        f.write("departure,line,route,stop_index,n_stops,facility,station,sched_arr,act_arr,"
                "dev_arr,sched_dep,act_dep,dev_dep,arr_judged,dep_judged\n")
        f.write("\n".join(rows))
        f.write("\n")
    json_path = os.path.join(exp, "punct.json")
    with open(json_path, "w") as f:
        json.dump(res, f, indent=1)

    print("wrote %s" % csv_path, file=sys.stderr)
    print("wrote %s" % json_path, file=sys.stderr)
    print_summary(res, a.top, exp)   # printed last: the HEADLINE line must be the last line


def print_summary(r, top, exp):
    A, D = r["arrivals"], r["departures_block"]
    o = []
    o.append("== punct.py  %s" % exp)
    o.append("schedule %s%s" % (r["schedule"],
                                "" if not r["ref"] else
                                "   published times from --ref %s (%d matched, %d unmatched)"
                                % (r["ref"], r["ref_matched"], r["ref_missing"])))
    o.append("events   %s   (last event %s)" % (r["events"], hhmmss(r["last_event_time_s"])))
    o.append("")
    o.append("trains finished %d / %d (%.2f%%)   started %d"
             % (r["completed"], r["departures"], r["completion_pct"], r["started"]))
    o.append("stop arrivals realised %d / scheduled %d (%.2f%%)   missing %d"
             % (r["realised_arrivals"], r["judged_arrivals"], r["stop_completion_pct"],
                r["missing_arrivals"]))
    o.append("trains with every judged arrival within 1 s: %d / %d started"
             % (A.get("trains_all_within_1s", 0), r["started"]))
    o.append("")

    def table(b, label, n_sched):
        t = ["%s (n=%d scheduled%s):" % (label, n_sched,
                                        "" if b.get("realised") is None else
                                        ", %d realised" % b["realised"])]
        t.append("  |dev| <=   1 s : %6d  (%6.2f %%)" % (b["within_1s"], b["within_1s_pct"]))
        t.append("  |dev| <=   5 s : %6d  (%6.2f %%)" % (b["within_5s"], b["within_5s_pct"]))
        t.append("  |dev| <=  30 s : %6d  (%6.2f %%)" % (b["within_30s"], b["within_30s_pct"]))
        t.append("  |dev| <=  60 s : %6d  (%6.2f %%)" % (b["within_60s"], b["within_60s_pct"]))
        t.append("  |dev| <= 180 s : %6d  (%6.2f %%)" % (b["within_180s"], b["within_180s_pct"]))
        t.append("  early >  5 s   : %6d      > 30 s : %6d     > 60 s : %6d     > 180 s : %6d"
                 % (b["early_gt_5"], b["early_gt_30"], b["early_gt_60"], b["early_gt_180"]))
        t.append("  late  >  0 s   : %6d      > 30 s : %6d     > 60 s : %6d     > 180 s : %6d"
                 % (b["late_gt_0"], b["late_gt_30"], b["late_gt_60"], b["late_gt_180"]))
        t.append("  mean |dev| %.1f s   mean signed %+.1f s   median %+.1f s   p05/p95 %+.0f/%+.0f s"
                 % (b["mean_abs_s"], b["mean_signed_s"], b["median_signed_s"], b["p05_s"],
                    b["p95_s"]))
        t.append("  sum early %.0f s   sum late %.0f s   worst early %+.0f s   worst late %+.0f s"
                 % (b["sum_early_s"], b["sum_late_s"], b["min_s"], b["max_s"]))
        return t

    o += table(A, "ARRIVAL deviation (actual - published)", r["judged_arrivals"])
    o.append("")
    o += table(D, "DEPARTURE deviation", r["judged_departures"])
    o.append("  departures BEFORE the published departure time: %d  (must be 0)"
             % r["departures_before_published"])
    o.append("")
    o.append("worst %d arrivals by |dev|:" % top)
    o.append("   |dev|   dev  line                          departure          stop            "
             "sched      act")
    for w in r["worst_arrivals"]:
        o.append("  %6.0f %6.0f  %-28s %-18s %2d/%-2d %-12s %s  %s"
                 % (w["abs_dev_s"], w["dev_s"], (w["line"] or "")[:28], w["departure"],
                    w["stop_index"] + 1, w["n_stops"], (w["station"] or "")[:12],
                    hhmmss(w["sched_arr"]), hhmmss(w["act_arr"])))
    o.append("")
    o.append("per station, top %d by sum |dev|:" % top)
    for s in r["by_station"]:
        o.append("  %-34s %5d arrivals  sum|dev| %8.0f s  mean signed %+7.1f s  early>5 %4d"
                 % ((s["station"] or "?")[:34], s["arrivals"], s["sum_abs_dev_s"],
                    s["mean_signed_s"], s["early_gt_5"]))
    o.append("per line, top %d by sum |dev|:" % top)
    for s in r["by_line"]:
        o.append("  %-34s %5d arrivals  sum|dev| %8.0f s  mean signed %+7.1f s  early>5 %4d"
                 % ((s["line"] or "?")[:34], s["arrivals"], s["sum_abs_dev_s"],
                    s["mean_signed_s"], s["early_gt_5"]))
    o.append("")
    o.append("HEADLINE abs_dev_seconds=%.0f exact1s=%.2f%% within5=%.2f%% within30=%.2f%% "
             "within60=%.2f%% within180=%.2f%% mean_abs=%.1f mean_signed=%+.1f early5=%d "
             "late0=%d late30=%d late180=%d missing=%d departures_early=%d"
             % (r["abs_dev_seconds"], A["within_1s_pct"], A["within_5s_pct"],
                A["within_30s_pct"], A["within_60s_pct"], A["within_180s_pct"],
                A["mean_abs_s"], A["mean_signed_s"], A["early_gt_5"], A["late_gt_0"],
                A["late_gt_30"], A["late_gt_180"], r["missing_arrivals"],
                r["departures_before_published"]))
    print("\n".join(o))


if __name__ == "__main__":
    main()
