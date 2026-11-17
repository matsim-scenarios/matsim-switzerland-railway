#!/usr/bin/env python3
"""Static quality check of a MAPPED transit schedule (no simulation). OWNED BY THE ORCHESTRATOR - do not edit.

Usage: check_mapping.py <transitSchedule.xml.gz> <transitNetwork.xml.gz> [--json out.json] [--no-conflicts] [--list N]

Reports per-schedule metrics that predict whether trains will block each other in railsim:
  invalid_routes        routes whose link sequence is disconnected / stop links missing or out of order (must be 0)
  artificial_km         km on links that are not OSM track (no osm:way:id, not a boundary extension) - teleport links
  reversals_midroute    U-turns (link followed by its reverse) NOT at a stop link            (should be 0)
  reversals_at_stop     U-turns at a stop link (terminus reversal - plausible)
  illegal_turns         transitions through a junction that A04 marks physically impossible (should be 0)
  wrongdir_km_pct       share of train-km run against the normal running direction of the track (matsim_input/tables/DIR/direction_final.csv,
                        conf >= 0.6; --a02a = old geometry-only table)
  platform_ok_pct       share of route stops (that have a known platform track) placed on that platform track
  service_km_pct        share of train-km on yard/spur/siding track
  headon_pairs          number of train pairs that, by timetable interpolation (free-flow speed profile scaled to the scheduled leg time), are on the SAME physical track
                        segment in OPPOSITE directions within 60 s of each other (deadlock predictor; lower is better)
All "train-km" figures are weighted by the number of departures.
"""
import os, sys, gzip, json, csv, argparse, collections, bisect
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.environ.get('RAIL_TABLES', ROOT + '/matsim_input/tables')


def op(p):
    return gzip.open(p, 'rb') if p.endswith('.gz') else open(p, 'rb')


def hms(s):
    a = s.split(':'); return int(a[0]) * 3600 + int(a[1]) * 60 + float(a[2])


def norm_code(c):
    c = (c or '').strip()
    out = set()
    for part in c.replace(';', '/').split('/'):
        part = part.strip()
        digits = ''.join(ch for ch in part if ch.isdigit())
        out.add(digits.lstrip('0') if digits else part)
    out.discard('')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('schedule'); ap.add_argument('network')
    ap.add_argument('--json'); ap.add_argument('--no-conflicts', action='store_true'); ap.add_argument('--list', type=int, default=8); ap.add_argument('--a02a', action='store_true', help='use the old geometry-only direction table')
    a = ap.parse_args()

    # ---- network ----
    links = {}  # id -> (from, to, length, wayid or None, modes)
    for ev, el in ET.iterparse(op(a.network), events=('end',)):
        if el.tag == 'link':
            way = None; ext = False
            for at in el.iter('attribute'):
                if at.get('name') == 'osm:way:id': way = at.text
                if at.get('name') == 'boundaryExtension': ext = True
            links[el.get('id')] = (el.get('from'), el.get('to'), float(el.get('length')), way, ext or el.get('id').startswith('ext_'), max(float(el.get('freespeed')), 1.0))
            el.clear()
    # ---- data products ----
    ways = {}
    for r in csv.DictReader(open(DATA + '/A01/ways.csv')):
        ways[r['way_id']] = (r['klass'], r['track_ref'], r['first_node'])
    direction = {}
    import os
    dirfile = DATA + '/DIR/direction_final.csv' if os.path.exists(DATA + '/DIR/direction_final.csv') and not a.a02a else DATA + '/A02a/direction.csv'
    for r in csv.DictReader(open(dirfile)):
        if r['direction'] in ('forward', 'backward') and float(r['confidence']) >= 0.6:
            direction[r['way_id']] = r['direction']
    legal = {}; junction = set()
    for r in csv.DictReader(open(DATA + '/A04/turns.csv')):
        junction.add(r['node_id'])
        k = (r['node_id'], r['from_node'], r['to_node'])
        legal[k] = max(legal.get(k, 0), int(r['legal']))
        legal[(r['node_id'], r['to_node'], r['from_node'])] = legal[k]
    plat = {}
    for r in csv.DictReader(open(DATA + '/A03a/platform_tracks.csv')):
        if r['method'] in ('track_ref', 'stop_node') and r['way_ids']:
            plat[r['stop_id']] = (set(r['way_ids'].split(';')), norm_code(r['platform_code']))
    # node order of ways (forward = in way order): from the OSM-derived network we only know segments; use A05 segment table
    seg_fwd = set()
    try:
        for r in csv.DictReader(open(DATA + '/A05/segment_block.csv')):
            seg_fwd.add((r['from_node'], r['to_node']))
    except FileNotFoundError:
        pass

    # ---- schedule ----
    fac_link = {}
    res = collections.Counter(); km = collections.Counter()
    bad = []; trains = []  # (t0, [(link, tin, tout)])
    line = None
    for ev, el in ET.iterparse(op(a.schedule), events=('start', 'end')):
        if ev == 'start':
            if el.tag == 'transitLine': line = el.get('name')
            continue
        if el.tag == 'stopFacility':
            fac_link[el.get('id')] = el.get('linkRefId')
        elif el.tag == 'transitRoute':
            rid = el.get('id')
            stops = [(s.get('refId'), hms(s.get('arrivalOffset') or s.get('departureOffset')), hms(s.get('departureOffset') or s.get('arrivalOffset')))
                     for s in el.find('routeProfile')]
            rl = [l.get('refId') for l in el.find('route')] if el.find('route') is not None else []
            deps = [hms(d.get('departureTime')) for d in el.find('departures')]
            n = len(deps); res['routes'] += 1; res['departures'] += n; res['route_stops'] += len(stops) * n
            ok = bool(rl) and all(l in links for l in rl)
            if ok:
                for i in range(len(rl) - 1):
                    if links[rl[i]][1] != links[rl[i + 1]][0]: ok = False; break
            # stop links in order
            pos = []; j = 0
            if ok:
                for (sid, arr, dep) in stops:
                    L = fac_link.get(sid)
                    try:
                        j = rl.index(L, j) if L else -1
                    except ValueError:
                        j = -1
                    if j < 0: ok = False; break
                    pos.append(j)
            if not ok:
                res['invalid_routes'] += 1; res['invalid_departures'] += n
                bad.append((line, rid, 'invalid link sequence / stops')); el.clear(); continue
            stop_idx = set(pos)
            length = 0.0; cum = [0.0]; tcum = [0.0]
            for i, l in enumerate(rl):
                f, t, ln, way, ext, fs = links[l]
                length += ln; cum.append(length); tcum.append(tcum[-1] + ln / min(fs, 55.6))
                if way is None and not ext: km['artificial'] += ln * n
                elif ext: km['extension'] += ln * n
                else:
                    kl = ways.get(way, ('?', '', ''))[0]
                    if kl in ('yard', 'spur', 'siding'): km['service'] += ln * n
                    d = direction.get(way)
                    if d and seg_fwd:
                        fwd = (f, t) in seg_fwd
                        if (d == 'forward') != fwd: km['wrongdir'] += ln * n
                        km['dirknown'] += ln * n
                if i + 1 < len(rl):
                    f2, t2 = links[rl[i + 1]][0], links[rl[i + 1]][1]
                    if t2 == f and f2 == t:  # reversal
                        if i in stop_idx: res['reversals_at_stop'] += n
                        else:
                            res['reversals_midroute'] += n
                            if len(bad) < 2000: bad.append((line, rid, f'mid-route reversal at link {l}'))
                    elif t in junction and legal.get((t, f, t2), 0) == 0 and way is not None and links[rl[i + 1]][3] is not None:
                        res['illegal_turns'] += n
                        if len(bad) < 2000: bad.append((line, rid, f'illegal turn at node {t} ({l} -> {rl[i + 1]})'))
            km['total'] += length * n
            for (sid, arr, dep), p in zip(stops, pos):
                base = sid.split('.link:')[0]
                if base in plat:
                    res['stops_with_platform'] += n
                    wset, codes = plat[base]
                    way = links[rl[p]][3]
                    tr = norm_code(ways.get(way, ('', '', ''))[1]) if way else set()
                    if way in wset or (codes and tr & codes): res['stops_platform_ok'] += n
            # timetable interpolation per link
            times = []
            for k in range(len(pos) - 1):
                i0, i1 = pos[k], pos[k + 1]
                t0, t1 = stops[k][2], stops[k + 1][1]
                d0, d1 = tcum[i0 + 1], tcum[i1 + 1]   # free-flow running time profile, scaled to the scheduled leg time
                span = max(d1 - d0, 1e-6)
                for i in range(i0 + 1, i1 + 1):
                    times.append((rl[i], t0 + (tcum[i] - d0) / span * (t1 - t0), t0 + (tcum[i + 1] - d0) / span * (t1 - t0)))
            for d in deps:
                trains.append((d, times))
            el.clear()

    out = dict(res)
    tot = max(km['total'], 1e-9)
    out.update(train_km=round(km['total'] / 1000, 1), artificial_km=round(km['artificial'] / 1000, 1),
               extension_km=round(km['extension'] / 1000, 1),
               service_km_pct=round(100 * km['service'] / tot, 2),
               wrongdir_km_pct=round(100 * km['wrongdir'] / max(km['dirknown'], 1e-9), 2),
               platform_ok_pct=round(100 * res['stops_platform_ok'] / max(res['stops_with_platform'], 1), 2))
    for k in ('invalid_routes', 'reversals_midroute', 'reversals_at_stop', 'illegal_turns'):
        out.setdefault(k, 0)

    if not a.no_conflicts:
        occ = collections.defaultdict(list)  # undirected segment -> (tin, tout, dir, train)
        for ti, (d, times) in enumerate(trains):
            for (l, tin, tout) in times:
                f, t, ln, way, ext, fs = links[l]
                if ext or way is None: continue
                key = (f, t) if f < t else (t, f)
                occ[key].append((d + tin - 60, d + tout + 60, f < t, ti))
        pairs = set()
        for key, lst in occ.items():
            if len(lst) < 2: continue
            fw = sorted(x for x in lst if x[2]); bw = sorted(x for x in lst if not x[2])
            if not fw or not bw: continue
            starts = [x[0] for x in bw]
            for (s, e, _, ti) in fw:
                # any backward interval overlapping (s, e)
                j = bisect.bisect_left(starts, e)
                for q in range(j - 1, -1, -1):
                    bs, be, _, tj = bw[q]
                    if be > s: pairs.add((ti, tj) if ti < tj else (tj, ti))
                    if e - bs > 3600: break
        out['headon_pairs'] = len(pairs)
        involved = collections.Counter()
        for x, y in pairs: involved[x] += 1; involved[y] += 1
        out['trains_in_headon'] = len(involved)

    print(json.dumps(out, indent=1))
    if bad:
        print(f'\n# problems: {len(bad)} (showing {min(a.list, len(bad))})')
        for b in bad[:a.list]: print('  ', b)
    if a.json: json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
