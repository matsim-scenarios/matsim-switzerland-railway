#!/usr/bin/env python3
"""Reversal post-processing (pipeline step 4): honest reversal geometry.

Round 2 agent E2's geometry fix (`fix_reversals2.py`) as delivered by agent I2
(`swarm/r2/I2/fix_reversals3.py`), unchanged except for this header; it is a pipeline step now,
run by `scripts/prepare_case.sh` directly after the track router and before `verify_schedule.py`.

For every reversal route-stop (route[k+1] == opposite(route[k]) and route[k] carries a
scheduled stop) whose departure currently retraces LESS than the train length, rebuild the
departure segment so that

  * the departure retraces the ARRIVAL links under the whole body (>= train length), so the
    whole train stands on track it arrived over, and
  * it then leaves the arrival track at the first node behind the body that offers a
    continuation which is not the arrival track itself (the first switch behind the body),
  * and continues to the next scheduled stop of the route (unchanged stop sequence).

Timetable (stops, times, departures) is not touched. Only the link list of the affected
<route> elements changes; the network is untouched.

Usage (normally called by scripts/prepare_case.sh):
  python3 prepare/fix_reversals.py --exp DIR --out OUT.xml.gz [--json stats.json] [--dry-run]
                                   [--max-added-m 2000]
  # DIR holds transitNetwork.xml.gz, transitSchedule.xml.gz (router output) and run.properties

I2 addition 1 (both cases): a route is only kept when EVERY reversal of the thrown route is honest
afterwards - the departure retraces the arrival links under the whole body (retrace >= train length)
or the stop link itself is >= train length. Otherwise the route is restored to the original: such a
"fix" buys nothing for the engine (the body is still rebuilt onto the departure route) but can cost a
lot. Measured in Switzerland: 3 IC51 routes at Delémont (149.6 m platform, 377.6 m train) were extended
by 17 m, which made their Moutier-side approach links "used in both directions" and merged the RE56
section on the Grenchen-Moutier-Delémont corridor from 25.5 km to 30.1 km (sections_ch.py); the run
then lost ~2 400 s on that corridor. Reverted, the sections are 25.5 km again. For the Olten case every
accepted fix is honest, so this gate changes nothing there (byte-identical output).

I2 addition 2 (Switzerland case): E2's search prefers the "first switch behind the body". At
Delémont the crossover to the departure line is only ~42 m behind the platform while the IC51
train (377.6 m) reaches 348 m past it, so no switch behind the body leads to the next stop: the
Dijkstra then returns a 137 km loop around the whole Jura, which would add 121.7 km to a 15 km
departure segment. A candidate is therefore rejected when it adds more than --max-added-m
(default 2000 m) to the route, and the next (shorter retrace) node is tried; if none is
acceptable the route is left UNCHANGED (counted as `failed`, why='connection path only as a
detour'). Every legitimate fix found by E2 in the Olten case adds <= 194 m (max 222 m in
Switzerland), so this guard cannot change an accepted Olten route. Routes that already retrace
>= train length are never touched.
"""
import argparse
import collections
import gzip
import heapq
import json
import os
import re
import sys

VEH = {'IC': 205., 'EC': 195., 'IR': 150., 'RE': 120., 'S': 100.}


def body_links(net, route, k, length):
    """arrival link indices covering >= length, walking back from the stop link k."""
    idx = [k]
    tot = net[route[k]][2]
    j = k
    while tot < length and j > 0:
        j -= 1
        tot += net[route[j]][2]
        idx.append(j)
    return idx, tot


def load_vehicle_lengths(exp):
    """E2: per-line rolling stock from the experiment's vehicles file (longest prefix wins)."""
    specs = []
    props = os.path.join(exp, 'run.properties')
    vfile = None
    if os.path.exists(props):
        for line in open(props):
            if line.startswith('vehicles.file='):
                vfile = line.split('=', 1)[1].strip()
    if vfile:
        path = vfile if os.path.isabs(vfile) else os.path.join(exp, vfile)
        if os.path.exists(path):
            for line in open(path):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                p = [x.strip() for x in line.split(',')]
                if len(p) != 5:
                    continue
                try:
                    specs.append((p[0], float(p[1])))
                except ValueError:
                    continue
            print('per-line vehicle lengths from %s: %d' % (path, len(specs)))
    return specs


def load_net(path):
    import xml.etree.ElementTree as ET
    links = {}
    for ev, el in ET.iterparse(gzip.open(path, 'rb'), events=('end',)):
        if el.tag == 'link':
            links[el.get('id')] = (el.get('from'), el.get('to'), float(el.get('length')),
                                   float(el.get('freespeed')))
            el.clear()
    return links


def train_length(line_name, specs=None):
    """E2: per-line length if the vehicles file has a row for the line, else the type default."""
    if specs:
        best = None
        for prefix, length in specs:
            if prefix == '' or line_name.startswith(prefix):
                if best is None or len(prefix) > len(best[0]):
                    best = (prefix, length)
        if best is not None:
            return best[1]
    t = 'other'
    for p in ('EC', 'IC', 'IR', 'RE', 'S'):
        if line_name.startswith(p) and (t == 'other' or len(p) > len(t)):
            t = p
    return VEH.get(t, 100.)


class Net:
    def __init__(self, links):
        self.links = links                       # id -> (from, to, length, freespeed)
        self.out = collections.defaultdict(list)
        for lid, (a, b, l, fs) in links.items():
            self.out[a].append(lid)
        self.opp = {}
        pair = {}
        for lid, (a, b, l, fs) in links.items():
            k = (a, b) if a <= b else (b, a)
            pair.setdefault(k, []).append(lid)
        for k, ids in pair.items():
            for x in ids:
                for y in ids:
                    if x != y and links[x][0] == links[y][1] and links[x][1] == links[y][0]:
                        self.opp[x] = y

    def opposite(self, lid):
        return self.opp.get(lid)

    def tt(self, lid):
        a, b, l, fs = self.links[lid]
        return l / max(fs, 1.0)

    def shortest_path(self, start_node, target_node, banned, banned_first, max_cost=1e9, prefer=None):
        """Dijkstra over links, cost = travel time. Returns list of link ids or None.

        E2: links of the original departure route are charged at 35 % of their travel time, so the
        splice stays on the track the router chose instead of switching to a parallel track (a
        parallel track makes the resource "used in both directions" and inflates the section-lock
        sections: measured 3614 -> 6872 km of locked sections, which deadlocks the section lock).
        """
        if start_node == target_node:
            return []
        prefer = prefer or set()

        def cost(lid):
            return self.tt(lid) * (0.35 if lid in prefer else 1.0)

        dist = {}
        prev = {}
        pq = []
        for lid in self.out.get(start_node, ()):
            if lid in banned or lid in banned_first:
                continue
            c = cost(lid)
            if c < dist.get(self.links[lid][1], 1e18):
                dist[self.links[lid][1]] = c
                prev[self.links[lid][1]] = (start_node, lid)
                heapq.heappush(pq, (c, self.links[lid][1]))
        while pq:
            d, node = heapq.heappop(pq)
            if d > dist.get(node, 1e18):
                continue
            if node == target_node:
                path = []
                cur = node
                while cur != start_node:
                    p, lid = prev[cur]
                    path.append(lid)
                    cur = p
                return path[::-1]
            if d > max_cost:
                continue
            for lid in self.out.get(node, ()):
                if lid in banned:
                    continue
                nb = self.links[lid][1]
                nd = d + cost(lid)
                if nd < dist.get(nb, 1e18) - 1e-12:
                    dist[nb] = nd
                    prev[nb] = (node, lid)
                    heapq.heappush(pq, (nd, nb))
        return None


LINK_RE = re.compile(r'<link refId="([^"]+)"\s*/>')
ROUTE_RE = re.compile(r'(<route>)(.*?)(</route>)', re.S)
LINE_RE = re.compile(r'<transitLine id="([^"]+)" name="([^"]*)"\s*>(.*?)</transitLine>', re.S)
TROUTE_RE = re.compile(r'(<transitRoute id="([^"]+)">)(.*?)(</transitRoute>)', re.S)
FAC_RE = re.compile(r'<stopFacility id="([^"]+)"[^>]*?linkRefId="([^"]+)"')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exp', required=True)
    ap.add_argument('--out', default=None)
    ap.add_argument('--json', default=None)
    ap.add_argument('--max-added-m', type=float, default=2000.,
                    help='reject a candidate that adds more than this to the route (I2: avoid 121 km detours)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    specs = load_vehicle_lengths(args.exp)
    net = Net(load_net(os.path.join(args.exp, "transitNetwork.xml.gz")))
    sched_path = os.path.join(args.exp, 'transitSchedule.xml.gz')
    text = gzip.open(sched_path, 'rt').read()
    fac = dict(FAC_RE.findall(text))
    print('network %d links, %d stop facilities' % (len(net.links), len(fac)))

    # find transit routes and process their link lists
    stats = collections.Counter()
    rows = []
    out_parts = []
    pos = 0
    for lm in LINE_RE.finditer(text):
        line_name = lm.group(2)
        L = train_length(line_name, specs)
        line_body = lm.group(3)
        # process each transitRoute inside the line
        new_block = []
        p2 = 0
        for tm in TROUTE_RE.finditer(line_body):
            rid = tm.group(2)
            content = tm.group(3)
            rms = list(ROUTE_RE.finditer(content))
            assert len(rms) == 1, 'expected exactly one <route> in transitRoute %s' % rid
            rmx = rms[0]
            links = LINK_RE.findall(rmx.group(2))
            stop_facs = re.findall(r'<stop refId="([^"]+)"', content)
            stop_links = [fac.get(f) for f in stop_facs]

            new_links, changes = fix_route(net, links, stop_links, L, rid, stats, rows,
                                           args.max_added_m)
            if changes:
                body_new = ''.join('\t\t\t\t<link refId="%s"/>\n' % x for x in new_links)
                content_new = content[:rmx.start(2)] + '\n' + body_new + '\t\t\t' + content[rmx.end(2):]
                new_block.append(line_body[p2:tm.start(3)] + content_new)
            else:
                new_block.append(line_body[p2:tm.end(3)])
            p2 = tm.end(3)
        new_block.append(line_body[p2:])
        out_parts.append(text[pos:lm.start(3)] + ''.join(new_block))
        pos = lm.end(3)
    out_parts.append(text[pos:])
    new_text = ''.join(out_parts)

    print('\nreversal route-stops: %d, fixed: %d, skipped (already honest): %d, failed: %d '
          '(%d only as a detour, %d retrace < train length); routes reverted as still unsafe: %d'
          % (stats['reversals'], stats['fixed'], stats['ok'], stats['failed'],
             stats['failed_detour'], stats['failed_retrace'], stats.get('reverted_unsafe', 0)))
    print('routes changed: %d, added metres: %.0f (median %.0f, max %.0f)'
          % (stats['routes'], stats['added_m'],
             sorted(r['added'] for r in rows if r['changed'])[len([r for r in rows if r['changed']]) // 2]
             if any(r['changed'] for r in rows) else 0,
             max([r['added'] for r in rows if r['changed']] or [0])))
    for r in rows:
        if not r['changed'] or args.json:
            continue
    for r in rows:
        if r.get('failed'):
            print('  FAILED %-45s %-14s %s' % (r['route'][:45], r['station'], r['why']))

    if args.json:
        json.dump(dict(stats=dict(stats), rows=rows), open(args.json, 'w'), indent=1)
        print('wrote %s' % args.json)
    if args.out and not args.dry_run:
        gzip.open(args.out, 'wt').write(new_text)
        print('wrote %s' % args.out)


def path_length(net, path):
    return sum(net.links[x][2] for x in path) if path else 0.0


def unsafe_reversals(net, links, stop_links, L):
    """I2: reversal stops whose departure neither retraces the whole body nor sits on a long enough stop link."""
    bad = []
    for i in range(len(links) - 1):
        if links[i] not in stop_links or net.opposite(links[i]) != links[i + 1]:
            continue
        stop_len = net.links[links[i]][2]
        retr = 0.0
        o = 0
        while i - o >= 0 and i + 1 + o < len(links) and net.opposite(links[i - o]) == links[i + 1 + o]:
            retr += net.links[links[i + 1 + o]][2]
            o += 1
        if retr < L - 1e-6 and stop_len < L - 1e-6:
            bad.append((i, retr, stop_len))
    return bad


def fix_route(net, links, stop_links, L, rid, stats, rows, max_added=2000.):
    """Return (new links, changed) for one route."""
    orig = list(links)
    changed = False
    route_added = 0.0
    # process reversals from the end so indices stay valid
    rev = []
    for i in range(len(links) - 1):
        if net.opposite(links[i]) == links[i + 1]:
            rev.append(i)
    for i in reversed(rev):
        stats['reversals'] += 1
        if i >= len(links) - 1:
            continue
        if links[i] not in stop_links:
            stats['not_at_stop'] += 1
            continue
        body_idx, tot = body_links(net.links, links, i, L)
        body = [links[j] for j in body_idx]
        retr = 0.0
        o = 0
        while i - o >= 0 and i + 1 + o < len(links) and net.opposite(links[i - o]) == links[i + 1 + o]:
            retr += net.links[links[i + 1 + o]][2]
            o += 1
        if retr >= L:
            stats['ok'] += 1
            rows.append(dict(route=rid, station=None, changed=False, retrace=retr, L=L))
            continue
        # next stop link after the reversal
        nxt = None
        for lid in links[i + 1:]:
            if lid in stop_links:
                nxt = lid
                break
        if nxt is None:
            stats['failed'] += 1
            rows.append(dict(route=rid, station=None, changed=False, failed=True,
                             why='no next stop link'))
            continue
        # target node = start node of the next stop link
        target = net.links[nxt][0]
        prefix = []
        ok = True
        for lid in body:
            ov = net.opposite(lid)
            if ov is None:
                ok = False
                break
            prefix.append(ov)
        if not ok:
            stats['failed'] += 1
            rows.append(dict(route=rid, station=None, changed=False, failed=True,
                             why='arrival link without opposite'))
            continue
        banned = set(body) | set(prefix) | {nxt}
        if net.opposite(nxt):
            banned.add(net.opposite(nxt))
        # first node behind the whole body, extended while it has no continuation
        j = links.index(nxt, i + 1)
        m = len(body) - 1
        path = None
        detour = 0.0
        short_retrace = 0.0
        while m >= 0:
            pre = prefix[:m + 1]                  # retrace of body[0..m]
            A = net.links[body[m]][0]
            # the arrival track continuation further back (must not be taken if alternatives exist)
            further = None
            if body_idx[m] - 1 >= 0:
                further = net.opposite(links[body_idx[m] - 1])
            banned_first = {further} if further else set()
            leave = links[i + 1:j + 1]
            cand = net.shortest_path(A, target, banned | set(pre), banned_first, prefer=set(leave))
            if cand is None:
                # no continuation that is not the arrival track: allow it (last resort) at this node
                cand = net.shortest_path(A, target, banned | set(pre), set(), prefer=set(leave))
            if cand is not None:
                # I2: a "path" that only exists as a huge detour is not a departure track. Measure the
                # detour against the departure segment it replaces (the connection path itself is often
                # tens of km long when the next stop is far away - that is normal, not a detour).
                added_cand = (path_length(net, prefix[:m + 1]) + path_length(net, cand)
                              + net.links[nxt][2] + sum(net.links[x][2] for x in links[j + 1:])
                              - sum(net.links[x][2] for x in links))
                retrace_new = path_length(net, prefix[:m + 1])
                honest = retrace_new >= L - 1e-6 or net.links[links[i]][2] >= L - 1e-6
                if added_cand <= max_added and (honest or added_cand <= 0.5):
                    path = cand
                    break
                if added_cand > max_added:
                    detour = max(detour, added_cand)
                else:
                    short_retrace = max(short_retrace, retrace_new)
            m -= 1
        if path is None:
            stats['failed'] += 1
            if detour:
                stats['failed_detour'] += 1
                why = 'connection path only as a detour (+%.0f m > +%.0f m)' % (detour, max_added)
            elif short_retrace:
                stats['failed_retrace'] += 1
                why = 'retrace only %.0f m < train length %.0f m' % (short_retrace, L)
            else:
                why = 'no path from switch behind body to next stop'
            rows.append(dict(route=rid, station=None, changed=False, failed=True, why=why,
                             detour=detour))
            continue
        # splice: route[0..i] + prefix[0..m] + path + next stop link + rest after nxt
        new = links[:i + 1] + prefix[:m + 1] + path + [nxt] + links[j + 1:]
        # validation: connected, no duplicate consecutive, the number of reversals is unchanged
        # (E2: the old check flagged the *other* reversal of a route that reverses twice - Basel and
        # Bern for the ICE - because it scanned the whole spliced list.)
        valid = True
        why = ''
        revs_old = sum(1 for k in range(len(links) - 1) if net.opposite(links[k]) == links[k + 1])
        revs_new = sum(1 for k in range(len(new) - 1) if net.opposite(new[k]) == new[k + 1])
        for k in range(len(new) - 1):
            if net.links[new[k]][1] != net.links[new[k + 1]][0]:
                valid = False
                why = 'disconnected at %d (%s->%s)' % (k, new[k], new[k + 1])
                break
            if new[k] == new[k + 1]:
                valid = False
                why = 'duplicate consecutive %s' % new[k]
                break
        if valid and revs_new != revs_old:
            valid = False
            why = 'reversal count changed %d -> %d' % (revs_old, revs_new)
        if not valid:
            stats['failed'] += 1
            rows.append(dict(route=rid, station=None, changed=False, failed=True, why=why))
            continue
        added = sum(net.links[x][2] for x in new) - sum(net.links[x][2] for x in links)
        route_added = route_added + added
        stats['fixed'] += 1
        stats['routes'] += 1
        stats['added_m'] += added
        rows.append(dict(route=rid, changed=True, retrace=retr, L=L, added=added,
                         prefix_links=m + 1, path_links=len(path),
                         old_segment=links[i + 1:j + 1], new_segment=new[i + 1:i + 1 + (m + 1) + len(path) + 1]))
        links = new
        changed = True
    if changed:
        # I2: keep the change only if every reversal of the final route is honest
        bad = unsafe_reversals(net, links, stop_links, L)
        if bad:
            stats['reverted_unsafe'] = stats.get('reverted_unsafe', 0) + 1
            stats['fixed'] -= 1
            stats['routes'] -= 1
            stats['added_m'] -= route_added
            rows.append(dict(route=rid, station=None, changed=False, reverted=True,
                             why='unsafe reversal after fix: idx %d retrace %.0f m, stop %.0f m, L %.0f m'
                                 % (bad[0][0], bad[0][1], bad[0][2], L)))
            return orig, False
    return links, changed


if __name__ == '__main__':
    main()
