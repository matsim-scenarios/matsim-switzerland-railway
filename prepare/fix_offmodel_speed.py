#!/usr/bin/env python3
"""Pipeline step: fix the running time of off-model (ext_*) links: the published running time,
not the dwell-inclusive time, with a kinematic rest-to-rest profile.

Ported from round-3 agent CH1 (`swarm/r3/CH1/fix_offmodel_speed.py`), code unchanged; pipeline
step 10 of `docs/pipeline.md` (both cases: the Olten case has 3 off-model links, 1 of them too slow).

Why
  Off-model links (metre gauge, foreign lines, dual-gauge-only corridors: F1's model, r2) get their
  freespeed from the timetable: freespeed = length / (0.85 x scheduled hop time). The hop time that
  went into the network was measured from the ARRIVAL at the origin stop to the ARRIVAL at the
  destination stop, so wherever the published dwell is long the link is slower than the published
  RUNNING time (Freiburg Hbf -> Schallstadt: 10 003 m at 28.2 km/h for a booked 300 s leg = 120 km/h,
  4.25x too slow). A second defect: the 25 km/h lower clamp of F1's rule makes short hops slower than
  the published time even with a short dwell.

What this does (data only, no geometry, no timetable)
  For every leg of every transit route that runs on off-model links, take the published RUNNING time
  (departureOffset[k] - arrivalOffset[k+1]) and solve for the cruise speed at which a vehicle with the
  line's acceleration/deceleration covers the leg's off-model links in 0.85 x that time (rest to rest,
  trapezoidal profile, U = min(vmax, 160 km/h)). 0.85 is F1's accepted margin; acceleration/deceleration
  come from the rolling stock table (longest line-name prefix). Speeds are only ever RAISED (a link
  that is already fast enough is left alone), and the leg with the shortest booked time wins if an
  off-model link is used by several legs.

Usage:
  python3 prepare/fix_offmodel_speed.py <schedule.xml.gz> <network-in.xml.gz> <network-out.xml.gz> \
      [--vehicles vehicles_fs1.csv] [--factor 0.85] [--vmax 160] [--report out.csv]

Standard library only.
"""
import argparse
import collections
import csv
import gzip
import math
import re
import xml.etree.ElementTree as ET


def op(p):
    return gzip.open(p, 'rb') if p.endswith('.gz') else open(p, 'rb')


def hms(s):
    if s is None:
        return None
    p = s.split(':')
    return int(p[0]) * 3600 + int(p[1]) * 60 + float(p[2])


def read_network(path):
    links = {}
    for ev, el in ET.iterparse(op(path), events=('end',)):
        if el.tag == 'link':
            links[el.get('id')] = dict(length=float(el.get('length')),
                                       fs=float(el.get('freespeed')),
                                       attrs=dict(el.attrib))
        el.clear()
    return links


def is_ext(lid, ln):
    return lid.startswith('ext_') or 'extn_' in ln['attrs'].get('from', '')


def read_routes(path):
    line = None
    route = None
    links, stops = [], []
    for ev, el in ET.iterparse(op(path), events=('start', 'end')):
        if ev == 'start':
            if el.tag == 'transitLine':
                line = el.get('name') or el.get('id')
            elif el.tag == 'transitRoute':
                route = el.get('id')
                links, stops = [], []
            continue
        if el.tag == 'transitRoute':
            yield line, route, links, stops
            route = None
            el.clear()
        elif el.tag == 'transitLine':
            line = None
            el.clear()
        elif route is not None:
            if el.tag == 'link':
                links.append(el.get('refId'))
            elif el.tag == 'stop':
                stops.append((el.get('refId'), hms(el.get('arrivalOffset')),
                              hms(el.get('departureOffset'))))


def read_vehicles(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((r['line_name_prefix'], float(r['length_m']), float(r['vmax_kmh']),
                         float(r['acceleration']), float(r['deceleration'])))
    rows.sort(key=lambda r: -len(r[0]))
    return rows


def vehicle(rows, line):
    for p, ln, vm, a, d in rows:
        if line and line.startswith(p):
            return ln, vm, a, d
    return rows[-1][1:]


def t_kin(L, v, a, d):
    """Rest-to-rest time over length L with cruise speed v (trapezoidal / triangular)."""
    if v <= 0:
        return float('inf')
    da = v * v / (2 * a)
    dd = v * v / (2 * d)
    if da + dd <= L:
        return v / a + v / d + (L - da - dd) / v
    # triangular: no cruise phase
    vp = math.sqrt(2 * L * a * d / (a + d))
    return vp / a + vp / d


def solve_v(L, target, a, d, vcap):
    """Smallest cruise speed whose rest-to-rest time over L is <= target, capped at vcap."""
    if L <= 0:
        return vcap
    if t_kin(L, vcap, a, d) >= target:
        return vcap
    lo, hi = 0.0, vcap
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if t_kin(L, mid, a, d) > target:
            lo = mid
        else:
            hi = mid
    return hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('schedule')
    ap.add_argument('net_in')
    ap.add_argument('net_out')
    ap.add_argument('--vehicles', default='matsim_input/tables/RS1/vehicles_rs1.csv')
    ap.add_argument('--factor', type=float, default=0.85)
    ap.add_argument('--vmax', type=float, default=160.0)
    ap.add_argument('--report', default='matsim_input/tables/RS1/offmodel_speed_fix.csv')
    a = ap.parse_args()

    links = read_network(a.net_in)
    vehs = read_vehicles(a.vehicles)

    # ext link -> best (shortest target time) plan
    best = {}
    legs = 0
    for line, rid, rlinks, stops in read_routes(a.schedule):
        if len(stops) < 2:
            continue
        slink = [s[0].split('.link:')[1] if '.link:' in s[0] else None for s in stops]
        idx = {}
        for i, l in enumerate(rlinks):
            idx.setdefault(l, i)
        for k in range(len(stops) - 1):
            A, B = idx.get(slink[k]), idx.get(slink[k + 1])
            if A is None or B is None:
                continue
            seg = rlinks[A + 1:B + 1]
            booked = stops[k + 1][1] - stops[k][2]
            if booked <= 0:
                continue
            exts = [l for l in seg if l in links and is_ext(l, links[l])]
            if not exts:
                continue
            legs += 1
            real = [l for l in seg if l in links and not is_ext(l, links[l])]
            L_ext = sum(links[l]['length'] for l in exts)
            t_real = sum(links[l]['length'] / links[l]['fs'] for l in real)
            _, vm, acc, dec = vehicle(vehs, line)
            vcap = min(a.vmax, vm) / 3.6
            target = a.factor * booked - t_real
            v = solve_v(L_ext, target, acc, dec, vcap)
            need = L_ext / v if v > 0 else float('inf')
            for l in exts:
                old = links[l]['fs']
                # only raise: a link that already allows the published running time stays
                if L_ext / old <= target + 0.5:
                    continue
                b = best.get(l)
                if b is None or v > b['v']:
                    best[l] = dict(v=v, line=line, from_pair=(slink[k], slink[k + 1]),
                                   booked=booked, L=L_ext, t_real=t_real, target=target,
                                   a=acc, d=dec, capped=(v >= vcap - 1e-9),
                                   old=old, old_t=L_ext / old)
    print(f"== {a.schedule}: {legs} legs with off-model links, {len(best)} off-model links too slow,"
          f" {sum(1 for b in best.values() if b['capped'])} of them capped at {a.vmax:.0f} km/h")

    # rewrite the network: text-level surgery on <link> elements, everything else untouched
    with op(a.net_in) as fi:
        data = fi.read().decode('utf-8')
    rows = []

    def repl(m):
        el = m.group(0)
        lid = re.search(r'\bid="([^"]+)"', el)
        if not lid or lid.group(1) not in best:
            return el
        b = best[lid.group(1)]
        rows.append(dict(link=lid.group(1), line=b['line'], length_m=round(b['L'], 1),
                         booked_s=round(b['booked'], 1),
                         old_kmh=round(b['old'] * 3.6, 1),
                         old_time_s=round(b['old_t'], 1),
                         new_kmh=round(b['v'] * 3.6, 1),
                         new_time_s=round(b['L'] / b['v'], 1),
                         target_s=round(b['target'], 1),
                         capped=int(b['capped']),
                         from_stop=str(b['from_pair'][0]), to_stop=str(b['from_pair'][1])))
        return re.sub(r'\bfreespeed="[^"]*"', 'freespeed="%.4f"' % b['v'], el, count=1)

    data = re.sub(r'<link\b[^>]*>', repl, data)
    n = len(rows)
    with (gzip.open(a.net_out, 'wb') if a.net_out.endswith('.gz') else open(a.net_out, 'wb')) as out:
        out.write(data.encode('utf-8'))
    with open(a.report, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['link', 'line', 'length_m', 'booked_s', 'old_kmh',
                                          'old_time_s', 'new_kmh', 'new_time_s', 'target_s',
                                          'capped', 'from_stop', 'to_stop'])
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: -(r['old_time_s'] - r['new_time_s'])))
    print(f"   wrote {a.net_out} ({n} links changed), report {a.report}")
    print("   worst 15 (old time -> new time, booked, target):")
    for r in sorted(rows, key=lambda r: -(r['old_time_s'] - r['new_time_s']))[:15]:
        print(f"     {r['link']:>9s} {r['line']:6s} {r['length_m']:9.0f} m  {r['old_time_s']:7.1f}"
              f" -> {r['new_time_s']:6.1f} s  booked {r['booked_s']:6.0f}  target {r['target_s']:6.1f}"
              f"  {r['old_kmh']:5.1f} -> {r['new_kmh']:5.1f} km/h{'  CAPPED' if r['capped'] else ''}")


if __name__ == '__main__':
    main()
