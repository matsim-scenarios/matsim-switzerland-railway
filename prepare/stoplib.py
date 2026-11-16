#!/usr/bin/env python3
"""SP1 shared geometry: stop positions, platform tracks, train bodies, dwell.

Ported from round-3 agent SP1 (`swarm/r3/SP1/tools/stoplib.py`), code unchanged except the table
paths (matsim_input/tables/ instead of round-1/2's swarm/data/; the files are byte-identical).

Extends round-2 `swarm/r2/S1/stopgeom.py` (same platform-track definition, same switch test) with
  * the per-stop dwell of the published timetable (arrivalOffset / departureOffset of every route-stop),
  * the number of departures of a route (the weight of a route-stop in the timetable),
  * the links/track actually under the standing body (not only the metres outside the platform track),
  * the OSM platform ways of a stop (A03a), used to keep a moved stop on its published platform.

Definitions (unchanged from S1):
  platform track of a route-stop k = maximal chain links[j..i] of the train's own route around k,
    bounded by switches (physical degree >= 3 in A04 or a schedule divergence/convergence, ignoring the
    reverse of the train's own link) and by a reversal.
  railsim stops a train with its HEAD at the end of the stop link, the body hanging back:
    head = end of links[k];  inside = sum(len(links[j..k]));  body_out = max(0, L - inside).

Dwell: dwell = departureOffset - arrivalOffset (seconds).  The **first** stop of a run has no arrival to
wait for (the train is created there at its departure second) and the **last** stop is left in the second
the train arrives (or at the published arrival if it is early), see `base7d/README.md` §"Engine check";
`standing(s)` classifies a stop accordingly.
"""
import collections
import csv
import gzip
import os
import re
import xml.etree.ElementTree as ET

ROOT = os.environ.get('STOPLIB_ROOT') or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
DEGREE_CSV = os.path.join(ROOT, 'matsim_input', 'tables', 'A04', 'node_degree.csv')
PLATFORM_CSV = os.path.join(ROOT, 'matsim_input', 'tables', 'A03a', 'platform_tracks.csv')
WAYS_CSV = os.path.join(ROOT, 'matsim_input', 'tables', 'A01', 'ways.csv')


def op(path):
    return gzip.open(path, 'rb') if path.endswith('.gz') else open(path, 'rb')


def hms(s):
    if not s:
        return 0.0
    a = s.split(':')
    return int(a[0]) * 3600 + int(a[1]) * 60 + float(a[2])


def load_degree(path=None):
    """node id -> physical degree (>=3 only in the file; an absent node is degree 2)."""
    deg = {}
    p = path or DEGREE_CSV
    if not os.path.exists(p):
        return deg
    with open(p) as f:
        for r in csv.DictReader(f):
            deg[r['node_id']] = int(r['degree'])
    return deg


class Net:
    def __init__(self, links):
        self.links = links
        self.out = collections.defaultdict(list)
        self.inn = collections.defaultdict(list)
        for lid, d in links.items():
            self.out[d['frm']].append(lid)
            self.inn[d['to']].append(lid)

    def frm(self, lid):
        return self.links[lid]['frm']

    def to(self, lid):
        return self.links[lid]['to']

    def length(self, lid):
        return self.links[lid]['length']

    def opposite(self, lid):
        d = self.links.get(lid)
        if d is None:
            return None
        for x in self.out.get(d['to'], ()):
            e = self.links[x]
            if e['to'] == d['frm']:
                return x
        return None

    def between(self, a, b):
        if a is None or b is None:
            return None
        return self.links[a]['to'] if self.links[a]['to'] == self.links[b]['frm'] else None


def load_net(path):
    links = {}
    nodes = {}
    for ev, el in ET.iterparse(op(path), events=('end',)):
        tag = el.tag
        if tag == 'link':
            way = None
            for at in el.findall('attributes/attribute'):
                if at.get('name') == 'osm:way:id':
                    way = (at.text or '').strip()
            links[el.get('id')] = dict(frm=el.get('from'), to=el.get('to'),
                                       length=float(el.get('length')),
                                       freespeed=float(el.get('freespeed') or 0.0),
                                       boundary=(el.get('boundaryExtension') == 'true'
                                                 or el.get('id', '').startswith('ext_')),
                                       way=way)
            el.clear()
        elif tag == 'node':
            nodes[el.get('id')] = (float(el.get('x') or 0.0), float(el.get('y') or 0.0))
            el.clear()
    n = Net(links)
    n.xy = nodes
    return n


def load_schedule(path):
    """-> facilities {id: dict}, lines [{id,name,routes:[{id,links,stops,[dwell],[dep_offset],[n_dep]}]}]

    Per route-stop: arr (arrivalOffset), dep (departureOffset), dwell = dep - arr, n_dep = departures.
    """
    fac = {}
    lines = []
    for ev, el in ET.iterparse(op(path), events=('end',)):
        tag = el.tag
        if tag == 'stopFacility':
            fac[el.get('id')] = dict(id=el.get('id'), link=el.get('linkRefId'),
                                     x=el.get('x'), y=el.get('y'), name=el.get('name'),
                                     area=el.get('stopAreaId'), blocking=el.get('isBlocking'))
            el.clear()
        elif tag == 'transitLine':
            cur_line = dict(id=el.get('id'), name=el.get('name'), routes=[])
            for r in el.findall('transitRoute'):
                prof = r.find('routeProfile')
                stops, arrs, deps = [], [], []
                if prof is not None:
                    for s in prof.findall('stop'):
                        stops.append(s.get('refId'))
                        a = hms(s.get('arrivalOffset'))
                        d = hms(s.get('departureOffset'))
                        arrs.append(a)
                        deps.append(d)
                cur_line['routes'].append(dict(
                    id=r.get('id'),
                    links=[l.get('refId') for l in r.findall('route/link')],
                    stops=stops, arr=arrs, dep=deps,
                    n_dep=len(r.findall('departures/departure')),
                    first_dep=min([hms(d.get('departureTime')) for d in r.findall('departures/departure')],
                                  default=0.0)))
            lines.append(cur_line)
            el.clear()
    return fac, lines


def platform_of(fid):
    """'8500218:0:8.link:47732' -> '8500218:0:8'"""
    return fid.split('.link:')[0] if '.link:' in fid else fid


def norm_codes(s):
    out = set()
    for part in re.split(r'[;/|]', (s or '').strip()):
        part = part.strip().lstrip('0')
        if part:
            out.add(part)
    return out


def load_way_trackrefs(path=None):
    d = {}
    p = path or WAYS_CSV
    if not os.path.exists(p):
        return d
    with open(p) as f:
        for r in csv.DictReader(f):
            d[r['way_id']] = norm_codes(r.get('track_ref'))
    return d


def load_platform_tracks(path=None, all_methods=True):
    """stop base id -> (way ids set, platform codes set).

    `all_methods=True` keeps every row that has way ids (track_ref, stop_node, platform_ref); the
    orchestrator's check_mapping.py only uses track_ref/stop_node, which is the stricter subset used for
    the `platform_ok` metric.
    """
    d = {}
    p = path or PLATFORM_CSV
    if not os.path.exists(p):
        return d
    with open(p) as f:
        for r in csv.DictReader(f):
            ways = {w for w in (r.get('way_ids') or '').split(';') if w}
            if not ways:
                continue
            if not all_methods and r.get('method') not in ('track_ref', 'stop_node'):
                continue
            d[r['stop_id']] = (ways, norm_codes(r.get('platform_code')))
    return d


VEH = {'IC': 205., 'EC': 195., 'IR': 150., 'RE': 120., 'S': 100.}


def load_vehicle_specs(path):
    """csv (prefix,length,vmax,a,d) -> list of (prefix, length). Missing path -> builtin fallback."""
    specs = []
    if not path or not os.path.exists(path):
        return specs
    with open(path) as f:
        for row in f:
            row = row.strip()
            if not row or row.startswith('#') or row.startswith('line_name_prefix'):
                continue
            q = [x.strip() for x in row.split(',')]
            if len(q) < 2:
                continue
            try:
                specs.append((q[0], float(q[1])))
            except ValueError:
                pass
    return specs


def vehicles_from_runprops(expdir):
    props = os.path.join(expdir, 'run.properties')
    if not os.path.exists(props):
        return None
    for line in open(props):
        if line.startswith('vehicles.file='):
            v = line.split('=', 1)[1].strip()
            return v if os.path.isabs(v) else os.path.join(expdir, v)
    return None


def train_length(line_name, specs):
    best = None
    for prefix, length in specs or ():
        if prefix == '' or line_name.startswith(prefix):
            if best is None or len(prefix) > len(best[0]):
                best = (prefix, length)
    if best:
        return best[1]
    t = 'other'
    for p in ('EC', 'IC', 'IR', 'RE', 'S'):
        if line_name.startswith(p) and (t == 'other' or len(p) > len(t)):
            t = p
    return VEH.get(t, 100.)


class Switches:
    def __init__(self, net, all_routes, degree):
        self.net = net
        self.degree = degree
        self.out_used = collections.defaultdict(set)
        self.in_used = collections.defaultdict(set)
        for links in all_routes:
            for lid in links:
                d = net.links.get(lid)
                if d is None:
                    continue
                self.out_used[d['frm']].add(lid)
                self.in_used[d['to']].add(lid)

    def pdeg(self, node):
        d = self.degree.get(node)
        if d is not None:
            return d
        return len({self.net.links[x]['to'] for x in self.net.out.get(node, ())} |
                   {self.net.links[x]['frm'] for x in self.net.inn.get(node, ())})

    def is_switch(self, node):
        if node is None:
            return True
        if self.pdeg(node) >= 3:
            return True
        if len(self.out_used.get(node, ())) > 1:
            return True
        if len(self.in_used.get(node, ())) > 1:
            return True
        return False

    def break_forward(self, node, incoming):
        if node is None:
            return True
        if self.pdeg(node) >= 3:
            return True
        alt = self.out_used.get(node, set()) - {self.net.opposite(incoming)}
        return len(alt) > 1

    def break_backward(self, node, outgoing):
        if node is None:
            return True
        if self.pdeg(node) >= 3:
            return True
        alt = self.in_used.get(node, set()) - {self.net.opposite(outgoing)}
        return len(alt) > 1


def stop_positions(net, links, stop_links, reuse=True):
    pos = []
    cur = 0
    for sl in stop_links:
        j = None
        if sl is not None:
            try:
                j = links.index(sl, cur) if reuse else None
            except ValueError:
                try:
                    j = links.index(sl)
                except ValueError:
                    j = None
        pos.append(j)
        if j is not None:
            cur = j
    return pos


def platform_chain(net, links, k, switches):
    j = k
    while j > 0:
        node = net.between(links[j - 1], links[j])
        if switches.break_backward(node, links[j]):
            break
        j -= 1
    i = k
    while i < len(links) - 1:
        node = net.between(links[i], links[i + 1])
        if switches.break_forward(node, links[i]):
            break
        if links[i + 1] == net.opposite(links[i]):
            break
        i += 1
    return j, i


def stop_geometry(net, links, k, L, switches):
    j, i = platform_chain(net, links, k, switches)
    inside = sum(net.length(links[x]) for x in range(j, k + 1))
    ahead = sum(net.length(links[x]) for x in range(k + 1, i + 1))
    chain_len = inside + ahead
    entry = net.frm(links[j])
    return dict(j=j, i=i, inside=inside, ahead=ahead, chain_len=chain_len,
                body_out=max(0.0, L - inside), entry_node=entry,
                entry_deg=switches.pdeg(entry),
                exit_node=net.to(links[i]), exit_deg=switches.pdeg(net.to(links[i])))


def body_links(net, links, k, L):
    """[(index, link, metres of the train on that link)] from the head backwards, one entry per link."""
    out = []
    left = L
    x = k
    head = net.length(links[k])
    while left > 1e-9 and x >= 0:
        ln = net.length(links[x])
        take = min(left, ln)
        out.append((x, links[x], take))
        left -= take
        x -= 1
    return out


def is_reversal_stop(net, links, k):
    """the route reverses at this stop (the next link is the reverse of the stop link)."""
    if k + 1 >= len(links):
        return False
    return links[k + 1] == net.opposite(links[k])


def platform_links(net, links, k, j, i, plat, wayref, base_id):
    """maximal run of links within [j..i] that are on the published platform (A03a ways / track_ref).

    -> (pj, pi) or None when the platform ways are unknown for this stop."""
    rec = plat.get(base_id) if plat else None
    if not rec:
        return None
    ways, codes = rec

    def ok(lid):
        w = net.links.get(lid, {}).get('way')
        if w is None:
            return False
        if w in ways:
            return True
        return bool(codes & wayref.get(w, set()))

    if not ok(links[k]):
        # the mapped stop link itself is not on the A03a platform: trust A03a only as an upper bound
        return None
    pj = k
    while pj > j and ok(links[pj - 1]):
        pj -= 1
    pi = k
    while pi < i and ok(links[pi + 1]):
        pi += 1
    return pj, pi


def standing(kind_first, kind_last):
    """does the train stand (block) at this stop?  first: created at the departure second, no arrival
    wait; last: removed on arrival (waits only if early); through: stands the published dwell."""
    if kind_first:
        return 'first'
    if kind_last:
        return 'last'
    return 'through'
