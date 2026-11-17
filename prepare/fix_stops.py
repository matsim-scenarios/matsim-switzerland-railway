#!/usr/bin/env python3
"""Pipeline step: move every route-stop of a mapped schedule inside its platform track.

  python3 prepare/fix_stops.py <schedule-in> <network-in> <schedule-out> <network-out> \
          --vehicles <vehicles.csv> [--clearance 10] [--stops through|all] [--rule tail|centre] \
          [--csv moves.csv] [--json stats.json] [--dry-run]

Ported from round-3 agent SP1 (`swarm/r3/SP1/tools/fix_stops.py` + `stoplib.py`), code unchanged;
pipeline step 9 of `docs/pipeline.md`.  Why: stop links are 13-46 m long while trains are 150-378 m,
so a train stood with its head at the END of the stop link and its body reaching back into the
station throat (body behind the platform entry switch 67 771 -> 9 153 m, dwell-weighted).

Rule (round-2 S1, generalised):
  railsim stops a train with its HEAD at the end of the stop link and the body hanging back.  For every
  route-stop that is not a first or a last stop the platform track is the maximal chain of consecutive
  links of the train's OWN route around the stop link, bounded by switches (physical degree >= 3 in A04
  or a schedule divergence/convergence, ignoring the reverse of the own link) and by a reversal.  If the
  body does not fit inside this chain, the stop is moved forward along the same route, inside the same
  platform track -- never to another platform, never past the next stop -- to the first link whose end
  lets the whole body stand inside with the tail `--clearance` metres clear of the entry switch.  If the
  platform track is shorter than the train the head goes to the far end of the platform track (the
  position that fouls least).

  `--rule centre` instead centres the train on the published stop coordinate (head = stop point + L/2),
  bounded by the platform track; it is reported as a variant.

Not touched: the timetable (every arrivalOffset / departureOffset / departureTime stays), the route link
sequences, the network, the platform numbers, the stop order.  Only the facility a moved route-stop
refers to changes: id `<platform>.link:<newlink>`, name / stopAreaId / isBlocking of the old facility, and
the coordinate = the projection of the published platform coordinate onto the new stop link (so that a
facility's `linkRefId` and its coordinate always agree, and one facility per (platform, link) is shared by
every route that stops there).

First and last stops are not moved (they have no standing time: a train is created at its first stop at
the departure second and removed at its last stop on arrival -- `base7d/README.md` "Engine check"), and
reversal stops are moved only up to the reversal point, which is the far end of their platform track.
"""
import argparse
import collections
import gzip
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import stoplib as G  # noqa: E402

LINK_RE = re.compile(r'<link refId="([^"]+)"\s*/>')
LINE_RE = re.compile(r'<transitLine id="([^"]+)" name="([^"]*)"\s*>(.*?)</transitLine>', re.S)
TR_RE = re.compile(r'(<transitRoute id="([^"]+)">)(.*?)(</transitRoute>)', re.S)
PROFILE_RE = re.compile(r'(<routeProfile>)(.*?)(</routeProfile>)', re.S)
STOP_RE = re.compile(r'<stop refId="([^"]+)"')
FAC_RE = re.compile(r'<stopFacility id="([^"]+)"([^>]*?)/>')
ROUTE_RE = re.compile(r'<route>(.*?)</route>', re.S)


def project_on_link(net, link, xy):
    """nearest point of the link segment to xy (net coordinates are the same system as the stop x/y)."""
    a = net.xy.get(net.frm(link))
    b = net.xy.get(net.to(link))
    if a is None or b is None or xy is None:
        return None
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 <= 1e-9:
        return a
    t = ((xy[0] - a[0]) * dx + (xy[1] - a[1]) * dy) / L2
    t = max(0.0, min(1.0, t))
    return (a[0] + t * dx, a[1] + t * dy)


def plat_ok_fn(net, plat, wayref, base):
    """the platform test of swarm/bench/check_mapping.py: the stop link's OSM way must be one of the
    platform's ways (A03a rows with method track_ref/stop_node) or carry a matching track_ref."""
    rec = plat.get(base) if plat else None
    if not rec:
        return None
    ways, codes = rec

    def ok(link):
        w = net.links.get(link, {}).get('way')
        if w is None:
            return False
        if w in ways:
            return True
        return bool(codes & (wayref.get(w) or set()))
    return ok


def plan_route(net, sw, links, stop_links, stops, L, clearance, rule, move_first_last, fac,
               plat=None, wayref=None):
    """-> list of moves for one route."""
    pos = G.stop_positions(net, links, stop_links)
    n = len(stop_links)
    moves = []
    for si, k in enumerate(pos):
        if k is None:
            continue
        first, last = si == 0, si == n - 1
        if (first or last) and not move_first_last:
            continue
        if net.links.get(links[k], {}).get('boundary'):
            continue
        g = G.stop_geometry(net, links, k, L, sw)
        need = L + clearance
        if g['inside'] >= need:
            continue
        j, i = g['j'], g['i']
        if plat is not None:
            # never leave the published platform: cap the head at the last link of the platform stretch
            pf = plat_ok_fn(net, plat, wayref, G.platform_of(stops[si]))
            if pf is not None:
                i_plat = k
                for x in range(k + 1, i + 1):
                    if not pf(links[x]):
                        break
                    i_plat = x
                i = i_plat
        k2 = None
        if rule == 'centre':
            # arc length of the published stop coordinate along the platform track, head = that + L/2
            f = fac.get(stops[si])
            xy = (float(f['x']), float(f['y'])) if f and f.get('x') else None
            target = None
            if xy is not None:
                acc0 = 0.0
                best = None
                for x in range(j, i + 1):
                    p = project_on_link(net, links[x], xy)
                    if p is None:
                        break
                    d = (p[0] - xy[0]) ** 2 + (p[1] - xy[1]) ** 2
                    along = 0.0
                    a = net.xy.get(net.frm(links[x]))
                    if a is not None:
                        along = ((p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2) ** 0.5
                    if best is None or d < best[0]:
                        best = (d, acc0 + along)
                    acc0 += net.length(links[x])
                if best is not None:
                    target = best[1] + L / 2.0
            if target is not None:
                acc = 0.0
                for x in range(j, i + 1):
                    acc += net.length(links[x])
                    if x >= k and acc >= target:
                        k2 = x
                        break
        else:
            acc = g['inside']
            for x in range(k + 1, i + 1):
                acc += net.length(links[x])
                if acc >= need:
                    k2 = x
                    break
            if k2 is None:
                k2 = i                      # platform track shorter than the train: far end
        if k2 is None or k2 == k:
            continue
        nxt = [p for p in pos[si + 1:] if p is not None]
        if nxt and k2 >= nxt[0]:
            k2 = nxt[0] - 1
            if k2 <= k:
                continue
        adv = sum(net.length(links[x]) for x in range(k + 1, k2 + 1))
        inside2 = g['inside'] + adv
        if g['body_out'] - max(0.0, L - inside2) < 2.0:
            continue                     # move gains < 2 m: not worth a new stop facility
        moves.append(dict(si=si, k=k, k2=k2, old_link=links[k], new_link=links[k2],
                          inside=g['inside'], chain_len=g['chain_len'], body_out=g['body_out'],
                          advance=adv, inside_after=inside2,
                          body_out_after=max(0.0, L - inside2),
                          j=j, i=i, L=L, reversal=G.is_reversal_stop(net, links, k)))
    return moves


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('schedule_in')
    ap.add_argument('network_in')
    ap.add_argument('schedule_out')
    ap.add_argument('network_out')
    ap.add_argument('--vehicles', required=True)
    ap.add_argument('--clearance', type=float, default=10.0)
    ap.add_argument('--stops', choices=('through', 'all'), default='through')
    ap.add_argument('--rule', choices=('tail', 'centre'), default='tail')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--json', default=None)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--stations', default=None, help='comma-separated station names to restrict to')
    ap.add_argument('--platbound', choices=('check', 'off'), default='check',
                    help="check (default): never leave the published platform (check_mapping's platform "
                         "test on the way of the target link); off: only the platform track bounds the move")
    a = ap.parse_args()

    net = G.load_net(a.network_in)
    fac, lines = G.load_schedule(a.schedule_in)
    specs = G.load_vehicle_specs(a.vehicles)
    all_routes = [r['links'] for ln in lines for r in ln['routes']]
    sw = G.Switches(net, all_routes, G.load_degree())
    plat = G.load_platform_tracks(all_methods=False) if a.platbound == 'check' else None
    wayref = G.load_way_trackrefs()

    text = gzip.open(a.schedule_in, 'rt').read() if a.schedule_in.endswith('.gz') \
        else open(a.schedule_in).read()
    existing = {m.group(1) for m in FAC_RE.finditer(text)}
    fac_attrs = {m.group(1): m.group(2) for m in FAC_RE.finditer(text)}

    stats = collections.Counter()
    rows = []
    new_facs = {}
    route_new = {}
    for lm in LINE_RE.finditer(text):
        line_name = lm.group(2)
        L = G.train_length(line_name, specs)
        for tm in TR_RE.finditer(lm.group(3)):
            rid = tm.group(2)
            content = tm.group(3)
            rm = ROUTE_RE.search(content)
            links = LINK_RE.findall(rm.group(1))
            pm = PROFILE_RE.search(content)
            stop_ids = STOP_RE.findall(pm.group(2))
            stop_links = [fac[s]['link'] if s in fac else None for s in stop_ids]
            route = next((r for r in lines if r['id'] == rid), None)
            dwells = [max(0.0, route['dep'][x] - route['arr'][x]) for x in range(len(stop_ids))] \
                if route else [0.0] * len(stop_ids)
            ndep = route['n_dep'] if route else 0
            stats['routes'] += 1
            stats['route_stops'] += len(stop_ids)
            if any(x is None for x in stop_links) or any(s not in fac for s in stop_ids):
                stats['missing_facility'] += 1
                continue
            moves = plan_route(net, sw, links, stop_links, stop_ids, L, a.clearance, a.rule,
                               a.stops == 'all', fac, plat, wayref)
            if a.stations:
                keep = {x.strip() for x in a.stations.split(',')}
                moves = [m for m in moves if fac[stop_ids[m['si']]]['name'] in keep]
            if not moves:
                continue
            newrefs = list(stop_ids)
            for mv in moves:
                old = stop_ids[mv['si']]
                base = G.platform_of(old)
                nid = '%s.link:%s' % (base, mv['new_link'])
                newrefs[mv['si']] = nid
                if nid not in existing and nid not in new_facs:
                    attrs = fac_attrs.get(old, '')
                    attrs = re.sub(r'linkRefId="[^"]*"', 'linkRefId="%s"' % mv['new_link'], attrs)
                    xy = project_on_link(net, mv['new_link'],
                                         (float(fac[old]['x']), float(fac[old]['y'])))
                    if xy is not None:
                        attrs = re.sub(r'\sx="[^"]*"', ' x="%.6f"' % xy[0], attrs, count=1)
                        attrs = re.sub(r'\sy="[^"]*"', ' y="%.6f"' % xy[1], attrs, count=1)
                    new_facs[nid] = attrs
                rows.append(dict(mv, line=line_name, route=rid, stop=old, new_stop=nid,
                                 name=fac[old]['name'], platform=base,
                                 first=mv['si'] == 0, last=mv['si'] == len(stop_ids) - 1,
                                 dwell=dwells[mv['si']], n_dep=ndep))
                stats['moved'] += 1
                stats['advance_m'] += mv['advance']
                stats['body_before'] += mv['body_out']
                stats['body_after'] += mv['body_out_after']
                if mv['reversal']:
                    stats['moved_reversal'] += 1
                if mv['body_out_after'] <= 0:
                    stats['fixed'] += 1
            route_new[(lm.group(1), rid)] = newrefs

    # ---------------- rewrite the schedule text ----------------
    out = []
    pos = 0
    for lm in LINE_RE.finditer(text):
        line_id = lm.group(1)
        body = lm.group(3)
        new_body = []
        p2 = 0
        for tm in TR_RE.finditer(body):
            rid = tm.group(2)
            refs = route_new.get((line_id, rid))
            if not refs:
                continue
            content = tm.group(3)
            pm = PROFILE_RE.search(content)
            prof = pm.group(2)
            old_ids = STOP_RE.findall(prof)
            if refs == old_ids:
                continue
            newprof = []
            pp = 0
            ctr = 0
            for m in STOP_RE.finditer(prof):
                newprof.append(prof[pp:m.start()])
                newprof.append('<stop refId="%s"' % refs[ctr])
                pp = m.end()
                ctr += 1
            newprof.append(prof[pp:])
            content_new = content[:pm.start(2)] + ''.join(newprof) + content[pm.end(2):]
            new_body.append(body[p2:tm.start(3)] + content_new)
            p2 = tm.end(3)
        new_body.append(body[p2:])
        out.append(text[pos:lm.start(3)] + ''.join(new_body))
        pos = lm.end(3)
    out.append(text[pos:])
    new_text = ''.join(out)

    if new_facs:
        anchor = '<transitStops>'
        i = new_text.index(anchor) + len(anchor)
        add = ''.join('\n\t\t<stopFacility id="%s"%s/>' % (k, v) for k, v in new_facs.items())
        new_text = new_text[:i] + add + new_text[i:]

    # ---------------- report ----------------
    print('%s: %d routes, %d route-stops, %d moves; head advanced %.0f m (mean %.1f m); '
          'body outside %.0f -> %.0f m; fully fixed %d, reversal stops moved %d, new facilities %d'
          % (os.path.basename(a.schedule_in), stats['routes'], stats['route_stops'], stats['moved'],
             stats['advance_m'], stats['advance_m'] / max(1, stats['moved']),
             stats['body_before'], stats['body_after'], stats['fixed'], stats['moved_reversal'],
             len(new_facs)))
    by = collections.Counter()
    mb = collections.Counter()
    for r in rows:
        by[r['name']] += 1
        mb[r['name']] += r['advance']
    print('stations (moves / head advance m): %s'
          % ', '.join('%s %d/%.0f' % (k, by[k], mb[k]) for k in sorted(by, key=lambda z: -mb[z])[:15]))
    if a.csv and rows:
        import csv as _csv
        with open(a.csv, 'w', newline='') as f:
            cols = list(rows[0].keys())
            wr = _csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            wr.writeheader()
            wr.writerows(rows)
        print('wrote %s' % a.csv)
    if a.json:
        json.dump(dict(stats=dict(stats), moves=rows), open(a.json, 'w'), indent=1)
    if a.dry_run:
        print('dry run: nothing written')
        return
    gzip.open(a.schedule_out, 'wt').write(new_text)
    gzip.open(a.network_out, 'wt').write(
        gzip.open(a.network_in, 'rt').read() if a.network_in.endswith('.gz')
        else open(a.network_in).read())
    print('wrote %s (%d new stop facilities) and %s (unchanged network)'
          % (a.schedule_out, len(new_facs), a.network_out))


if __name__ == '__main__':
    main()
