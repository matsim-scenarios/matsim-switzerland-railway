#!/usr/bin/env python3
"""Declarative track-assignment patches for the mapped transit schedule + network.

    python3 prepare/routepatch.py <schedule-in> <network-in> <schedule-out> <network-out> <spec.json...>

Ported from round-3 agent RC1 (`swarm/r3/RC1/routepatch.py`), code unchanged except the repository
root (computed from this file) and the junction table (`matsim_input/tables/A04/turns.csv` instead
of round-2's `swarm/data/A04/turns.csv`, byte-identical).  Pipeline step 8 of `docs/pipeline.md`
applies RC1's specs F1 (Heitersberg block sections), F2 (Olten-Aarau second track), F4 (Bern
Wylerfeld / Bern platform throat) and F5 (Aarburg) from `prepare/corrections/`.

Each spec is a small JSON file with a list of `actions`, applied in order.  The schedule and the
network are streamed and rewritten line by line (MATSim writes one element per line), so nothing
else in the files changes and memory stays small even for the Switzerland schedule.

Actions
-------
1. split_link — replace one network link by N equal pieces (block sections of a long link, e.g. a
   3.3 km tunnel link that carries two trains 45 s apart).
     {"type": "split_link", "name": "Heitersberg bore A", "link": "366010", "parts": 6}
   New nodes are interpolated along the link; new link ids are "<link>s1".."<link>sN"; every route
   that uses the link gets the pieces instead.  A stop on the link is an error.

2. duplicate_run — give selected routes their own copy of a link run (a parallel track), used where
   OSM has several parallel tracks but the router sent all traffic over one of them.
     {"type": "duplicate_run", "name": "Bern Wylerfeld track 2", "from": "532996", "to": "20534",
      "lines": ["IC6", "IR17"], "suffix": ".R", "offset": 9.0, "tag": "b2"}
   `lines` selects by transit line name, `suffix` by route id ending.  Only routes that contain
   `from` … `to` in this order (this travel direction, no stops in between) are rewritten.  The copy
   keeps each link's length/freespeed/capacity; its interior nodes are the originals shifted by
   `offset` m perpendicular to the local direction, and the first/last copy link keep the original
   junction nodes.

3. move_stop — move one stop of selected routes to another stop facility of the same station
     {"type": "move_stop", "name": "...", "old": "8500218:0:9.link:47462",
      "new_link": "47732", "platform": "8", "lines": ["IR35"]}
   The facility is created if missing (id "<uic>:0:<platform>.link:<new_link>", same name/x/y).

At the end every route is re-read and checked: connected link chain, stop links present and in
order, no link missing from the network (the same test as swarm/bench/check_mapping.py).
"""
import gzip
import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

NET_DTD = '<!DOCTYPE network SYSTEM "http://www.matsim.org/files/dtd/network_v2.dtd">'
SCHED_DTD = '<!DOCTYPE transitSchedule SYSTEM "http://www.matsim.org/files/dtd/transitSchedule_v2.dtd">'

RE_NODE = re.compile(r'<node id="([^"]+)" x="([^"]+)" y="([^"]+)"')
RE_LINK = re.compile(r'<link id="([^"]+)" from="([^"]+)" to="([^"]+)" length="([^"]+)" '
                     r'freespeed="([^"]+)"(?: capacity="([^"]+)")?(?: permlanes="([^"]+)")?')
RE_ROUTE = re.compile(r'<transitRoute id="([^"]+)"')
RE_TL = re.compile(r'<transitLine id="([^"]+)"[^>]*name="([^"]*)"')
RE_STOPF = re.compile(r'<stopFacility id="([^"]+)"[^>]*linkRefId="([^"]+)"[^>]*name="([^"]*)"')


def op(path, mode='rt'):
    return gzip.open(path, mode) if path.endswith('.gz') else open(path, mode)


def link_xml(lid, fr, to, length, fs, cap, lanes, attrs=''):
    head = ('\t\t<link id="%s" from="%s" to="%s" length="%.6f" freespeed="%.4f" capacity="%s" '
            'permlanes="%s" oneway="1" modes="rail" >' % (lid, fr, to, length, fs, cap, lanes))
    return head + ('\n' + attrs.rstrip('\n') if attrs else '')


def node_xml(nid, x, y):
    return '\t\t<node id="%s" x="%.6f" y="%.6f" >' % (nid, x, y)


def _orig_node_id(run_ids, k):
    return _NET_LINK[run_ids[k - 1]]['to']


def make_copy(run_ids, off, tag, new_nodes, new_links):
    """Parallel chain: every interior node is a new node shifted `off` m perpendicular to the
    alignment; only the two end nodes (the junctions where the copy branches off and rejoins)
    are the original ones, so the copy shares no railSim resource with the track it parallels.
    """
    node, link = _NET_NODE, _NET_LINK
    pts = [node[link[run_ids[0]]['fr']]]
    for x in run_ids:
        pts.append(node[link[x]['to']])
    m = len(run_ids)                      # pts has m+1 entries
    shifted = []
    for k, p in enumerate(pts):
        if k == 0 or k == m:
            shifted.append(p)
            continue
        q, r0 = pts[k + 1], pts[k - 1]
        dx, dy = q[0] - r0[0], q[1] - r0[1]
        nn = math.hypot(dx, dy) or 1.0
        shifted.append((p[0] - dy / nn * off, p[1] + dx / nn * off))
    ids = {}                              # node position -> node id
    ids[0] = link[run_ids[0]]['fr']
    for k in range(1, m):
        nid = 'n%s_%s_%d' % (run_ids[0], tag, k)
        ids[k] = nid
        new_nodes.setdefault(run_ids[k - 1], []).append(node_xml(nid, *shifted[k]))
        node[nid] = shifted[k]
    ids[m] = link[run_ids[-1]]['to']
    cids = []
    for k, x in enumerate(run_ids):
        cid = '%s_%s_%s' % (x, tag, run_ids[0])
        cids.append(cid)
        new_links.setdefault(x, []).append(
            link_xml(cid, ids[k], ids[k + 1], link[x]['length'], link[x]['fs'],
                     link[x]['cap'], link[x]['lanes'], link[x].get('attrs', '')))
    return cids


_NET_NODE = {}
_NET_LINK = {}
_JUNCTIONS = set()


def load_junctions(path=None):
    global _JUNCTIONS
    if _JUNCTIONS:
        return _JUNCTIONS
    path = path or (ROOT + '/matsim_input/tables/A04/turns.csv')
    try:
        import csv as _csv
        with open(path) as f:
            for r in _csv.DictReader(f):
                _JUNCTIONS.add(r['node_id'])
    except OSError:
        print('WARNING: %s not found - junction nodes will be shifted' % path)
    return _JUNCTIONS


# ---------------------------------------------------------------- pass 1: read
def read_network(path):
    node, link = {}, {}
    cur, attrs = None, []
    for s in op(path):
        t = s.strip()
        if cur is not None:
            if t.startswith('</link>'):
                link[cur]['attrs'] = '\n'.join(attrs)
                cur = None
            elif t:
                attrs.append(s.rstrip('\n'))
            continue
        if t.startswith('<node '):
            m = RE_NODE.match(t)
            if m:
                node[m.group(1)] = (float(m.group(2)), float(m.group(3)))
        elif t.startswith('<link id='):
            m = RE_LINK.match(t)
            if m:
                cur = m.group(1)
                attrs = []
                link[cur] = dict(fr=m.group(2), to=m.group(3), length=float(m.group(4)),
                                 fs=float(m.group(5)), cap=m.group(6) or '9999.0',
                                 lanes=m.group(7) or '1.0', attrs='')
    return node, link


def read_schedule(path):
    """-> routes {route_id: dict(line=, links=[], stops=[(refId)], deps=[dep ids])}, fac {}"""
    routes, fac, order = {}, {}, []
    cur, line = None, None
    for s in op(path):
        t = s.strip()
        if t.startswith('<stopFacility '):
            m = RE_STOPF.match(t)
            if m:
                fac[m.group(1)] = (m.group(2), m.group(3))
        elif t.startswith('<transitLine '):
            m = RE_TL.match(t)
            line = m.group(2) if m else None
        elif t.startswith('<transitRoute '):
            m = RE_ROUTE.match(t)
            cur = dict(id=m.group(1), line=line, links=[], stops=[], deps=[])
        elif cur is not None:
            if t.startswith('<stop ') and 'arrivalOffset' in t:
                m = re.search(r'refId="([^"]+)"', t)
                if m:
                    cur['stops'].append(m.group(1))
            elif t.startswith('<link ') and 'refId' in t:
                m = re.search(r'refId="([^"]+)"', t)
                if m:
                    cur['links'].append(m.group(1))
            elif t.startswith('<departure '):
                cur['deps'].append(t)
            elif t.startswith('</transitRoute>'):
                routes[cur['id']] = cur
                cur = None
    return routes, fac


# ---------------------------------------------------------------- apply
def run(sched_in, net_in, sched_out, net_out, specs):
    node, link = read_network(net_in)
    _NET_NODE.clear(); _NET_NODE.update(node)
    _NET_LINK.clear(); _NET_LINK.update(link)
    routes, fac = read_schedule(sched_in)

    # patches are keyed by route id: {position in links: ('replace', [ids]) | ('drop',)}
    route_patch = {}
    stop_patch = {}          # (route_id, old facility) -> new facility id
    new_route_ids = {}       # route id -> new link id list (whole run replaced)
    new_nodes = {}           # node id -> [xml lines]  (inserted after that node's element)
    new_links = {}           # link id -> [xml lines]  (inserted after that link's element)
    drop_links = set()
    new_fac = {}             # facility id -> xml line, inserted after `after` facility
    fac_after = {}
    actions_done = []

    for sp in specs:
        spec = json.load(open(sp))
        for act in spec['actions']:
            typ = act['type']
            if typ == 'split_link':
                lid = act['link']
                if lid not in link:
                    print('  split_link %s: not in this network - skipped' % lid)
                    continue
                if lid in drop_links or any(x.startswith(lid + 's') for x in link):
                    print('  split_link %s: already split - skipped' % lid)
                    continue
                if any(v[0] == lid for v in fac.values()):
                    raise SystemExit('split_link %s: link carries a stop facility' % lid)
                L = link[lid]
                n = int(act['parts'])
                a, b = node[L['fr']], node[L['to']]
                ids, mids = [], []
                for k in range(1, n):
                    nid = '%ss%d' % (lid, k)
                    x = a[0] + (b[0] - a[0]) * k / n
                    y = a[1] + (b[1] - a[1]) * k / n
                    mids.append((nid, x, y))
                    new_nodes.setdefault(L['fr'], []).append(node_xml(nid, x, y))
                seq = [L['fr']] + [m[0] for m in mids] + [L['to']]
                for k in range(n):
                    cid = '%ss%d' % (lid, k + 1)
                    ids.append(cid)
                    new_links.setdefault(lid, []).append(
                        link_xml(cid, seq[k], seq[k + 1], L['length'] / n, L['fs'], L['cap'],
                                 L['lanes'], L.get('attrs', '')))
                drop_links.add(lid)
                hit = 0
                for rid, r in routes.items():
                    ids_r = r['links']
                    for i, x in enumerate(ids_r):
                        if x == lid:
                            route_patch.setdefault(rid, {})[i] = ('replace', ids, 'split')
                            hit += 1
                actions_done.append(('split_link', lid, n, hit))
                print('  split_link %s into %d pieces of %.1f m: %d route links rewritten'
                      % (lid, n, L['length'] / n, hit))

            elif typ == 'duplicate_run':
                frm, to = act['from'], act['to']
                cands = [r for r in routes.values() if frm in r['links']]
                if not cands:
                    print('  duplicate_run %s: no route contains %s - skipped' % (act['name'], frm))
                    continue
                run_ids = None
                for r in cands:
                    ids = r['links']
                    i0 = ids.index(frm)
                    try:
                        i1 = ids.index(to, i0)
                    except ValueError:
                        continue
                    run_ids = ids[i0:i1 + 1]
                    break
                if run_ids is None:
                    print('  duplicate_run %s: no route has %s..%s - skipped' % (act['name'], frm, to))
                    continue
                if not all(x in link for x in run_ids):
                    print('  duplicate_run %s: run leaves the network - skipped' % act['name'])
                    continue
                sel = []
                skipped = []
                for rid, r in routes.items():
                    if act.get('lines') and r['line'] not in act['lines']:
                        continue
                    if act.get('suffix') and not rid.endswith(act['suffix']):
                        continue
                    ids = r['links']
                    if frm not in ids:
                        continue
                    i0 = ids.index(frm)
                    try:
                        i1 = ids.index(to, i0)
                    except ValueError:
                        continue
                    if ids[i0:i1 + 1] != run_ids:
                        continue
                    # a stop inside the run would have to be moved too -> skip that route
                    stoplinks = [fac.get(s, ('?',))[0] for s in r['stops']]
                    if any(f in stoplinks for f in run_ids):
                        skipped.append(rid)
                        continue
                    sel.append((rid, i0, i1))
                if skipped:
                    print('  duplicate_run %s: %d routes skipped (a stop lies in the run): %s'
                          % (act['name'], len(skipped), ','.join(sorted(set(
                              routes[x]['line'] for x in skipped)))))
                if not sel:
                    print('  duplicate_run %s: no route selected - skipped' % act['name'])
                    continue
                off = float(act.get('offset', 9.0))
                tag = act.get('tag', 'c')
                cids = make_copy(run_ids, off, tag, new_nodes, new_links)
                for rid, i0, i1 in sel:
                    route_patch.setdefault(rid, {})[i0] = ('replace', cids, 'copy')
                    for k in range(i0 + 1, i1 + 1):
                        route_patch[rid][k] = ('drop', None, None)
                actions_done.append(('duplicate_run', act['name'], len(cids), len(sel)))
                print('  duplicate_run %s: %d links, offset %.1f m, %d routes (%s)'
                      % (act['name'], len(cids), off, len(sel),
                         ','.join(sorted({routes[r]['line'] for r, a, b in sel}))))

            elif typ == 'duplicate_corridor':
                corridor = set(act['chain'])
                to = act['to']
                off = float(act.get('offset', 9.0))
                tag = act.get('tag', 'c')
                copies = {}       # (entry link) -> (run ids, copy ids)
                sel = 0
                for rid, r in routes.items():
                    if act.get('lines') and r['line'] not in act['lines']:
                        continue
                    if act.get('suffix') and not rid.endswith(act['suffix']):
                        continue
                    ids = r['links']
                    if to not in ids:
                        continue
                    i1 = len(ids) - 1 - ids[::-1].index(to)
                    i0 = i1
                    while i0 > 0 and ids[i0 - 1] in corridor:
                        i0 -= 1
                    run_ids = ids[i0:i1 + 1]
                    if i0 == i1 and act.get('min_links', 2) > 1:
                        continue
                    stoplinks = [fac.get(x, ('?',))[0] for x in r['stops']]
                    if any(f in stoplinks for f in run_ids):
                        continue
                    key = run_ids[0]
                    if key not in copies:
                        copies[key] = (run_ids, make_copy(run_ids, off, tag, new_nodes, new_links))
                    cids = copies[key][1]
                    route_patch.setdefault(rid, {})[i0] = ('replace', cids, 'copy')
                    for k in range(i0 + 1, i1 + 1):
                        route_patch[rid][k] = ('drop', None, None)
                    sel += 1
                if not sel:
                    print('  duplicate_corridor %s: no route selected - skipped' % act['name'])
                    continue
                actions_done.append(('duplicate_corridor', act['name'], len(copies), sel))
                print('  duplicate_corridor %s: %d distinct entries, %d routes rewritten'
                      % (act['name'], len(copies), sel))

            elif typ == 'move_stop':
                old = act['old']
                if old not in fac:
                    print('  move_stop %s: %s not in schedule - skipped' % (act['name'], old))
                    continue
                uic = old.split(':')[0]
                newid = '%s:0:%s.link:%s' % (uic, act['platform'], act['new_link'])
                n = 0
                for rid, r in routes.items():
                    if act.get('lines') and r['line'] not in act['lines']:
                        continue
                    if old not in r['stops']:
                        continue
                    stop_patch[(rid, old)] = newid
                    n += 1
                if n and newid not in fac:
                    x, y, name = None, None, fac[old][1]
                    # clone the old facility line
                    newfac_xml = None
                    for s in op(sched_in):
                        if 'id="%s"' % old in s and s.strip().startswith('<stopFacility'):
                            s2 = s.replace('id="%s"' % old, 'id="%s"' % newid)
                            s2 = re.sub(r'linkRefId="[^"]+"', 'linkRefId="%s"' % act['new_link'], s2)
                            newfac_xml = s2
                            break
                    new_fac[newid] = newfac_xml
                    fac_after[newid] = old
                    fac[newid] = (act['new_link'], name)
                actions_done.append(('move_stop', old, newid, n))
                print('  move_stop %s: %s -> %s on %d routes' % (act['name'], old, newid, n))
            else:
                raise SystemExit('unknown action %r' % typ)

    # ---------- pass 2: write the network (new nodes go into <nodes>, new links into <links>)
    all_nodes = [x for v in new_nodes.values() for x in v]
    all_links = [x for v in new_links.values() for x in v]
    with op(net_out, 'wt') as f:
        skipping = False
        for s in op(net_in):
            t = s.strip()
            if skipping:
                if t.startswith('</link>'):
                    skipping = False
                continue
            if t.startswith('<link id='):
                m = RE_LINK.match(t)
                if m and m.group(1) in drop_links:
                    skipping = True
                    continue
                f.write(s)
                continue
            f.write(s)
            if t.startswith('<nodes'):
                for x in all_nodes:
                    f.write(x + '\n\t\t</node>\n')
            elif t.startswith('<links'):
                for x in all_links:
                    f.write(x + '\n\t\t</link>\n')

    # ---------- pass 3: write the schedule
    written_new = set()
    with op(sched_out, 'wt') as f:
        cur_route = None
        link_pos = 0
        in_route = False
        for s in op(sched_in):
            t = s.strip()
            if t.startswith('<transitRoute '):
                m = RE_ROUTE.match(t)
                cur_route = m.group(1)
                link_pos = 0
                f.write(s)
                continue
            if t.startswith('<stopFacility '):
                m = re.match(r'<stopFacility id="([^"]+)"', t)
                fid = m.group(1)
                fid_new = None
                for nid, after in fac_after.items():
                    if after == fid and nid in new_fac and nid not in written_new:
                        fid_new = nid
                f.write(s)
                if fid_new:
                    f.write(new_fac[fid_new])
                    written_new.add(fid_new)
                continue
            if cur_route is not None and t.startswith('<stop ') and 'arrivalOffset' in t:
                m = re.search(r'refId="([^"]+)"', t)
                old = m.group(1)
                new = stop_patch.get((cur_route, old))
                if new:
                    s = s.replace('refId="%s"' % old, 'refId="%s"' % new, 1)
                f.write(s)
                continue
            if cur_route is not None and t.startswith('<link ') and 'refId' in t:
                p = route_patch.get(cur_route, {}).get(link_pos)
                if p is None:
                    f.write(s)
                elif p[0] == 'replace':
                    indent = s[:len(s) - len(s.lstrip())]
                    for x in p[1]:
                        f.write('%s<link refId="%s"/>\n' % (indent, x))
                link_pos += 1
                continue
            if t.startswith('</transitRoute>'):
                cur_route = None
            f.write(s)

    # ---------- pass 4: verify
    routes2, fac2 = read_schedule(sched_out)
    node2, link2 = read_network(net_out)
    bad = 0
    for rid, r in routes2.items():
        seq = r['links']
        ok = bool(seq) and all(x in link2 for x in seq)
        if ok:
            ok = all(link2[x]['fr'] in node2 and link2[x]['to'] in node2 for x in seq)
        if ok:
            for i in range(len(seq) - 1):
                if link2[seq[i]]['to'] != link2[seq[i + 1]]['fr']:
                    ok = False
                    break
        if ok:
            j = 0
            for sid in r['stops']:
                l = fac2.get(sid, (None,))[0]
                try:
                    j = seq.index(l, j) + 1
                except (ValueError, TypeError):
                    if l is not None:
                        ok = False
                        break
        if not ok:
            bad += 1
            if bad <= 5:
                print('  INVALID route %s (%s)' % (rid, r['line']))
    print('network: %d links, %d nodes; schedule: %d routes, %d facilities; invalid routes %d'
          % (len(link2), len(node2), len(routes2), len(fac2), bad))
    print('actions: %s' % actions_done)
    print('wrote %s  %s' % (sched_out, net_out))
    return bad


if __name__ == '__main__':
    if len(sys.argv) < 6:
        raise SystemExit(__doc__)
    sys.exit(1 if run(*sys.argv[1:5], sys.argv[5:]) else 0)
