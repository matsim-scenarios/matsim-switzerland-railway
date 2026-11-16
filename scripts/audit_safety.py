#!/usr/bin/env python3
"""D5: independent physical-safety audit of a railsim experiment (no simulation).

Reads only the experiment's network, schedule and output events.

Checks
 1  body reconstruction from railsimTrainStateEvent + the route the train really
    drove (LinkEnterEvent order), body = head position backwards over the vehicle length
 2  two bodies on the same physical track (link-pair resource) at the same time
 3  body over a track that the train has not reserved; reservations held >10 min
    with no body on the track
 4  node co-incidences: two trains through one node within 5 s on different link pairs
 5  plausibility: speed/acceleration vs type parameters, negative speed, teleports,
    arrival order
 6  start/end of run facts: insertion delay, disappearance, platform occupancy

I2 copy (Switzerland case): railsim resources that are off-model are exempt. The run class gives
every link with the network attribute boundaryExtension="true" (F1: metre gauge / abroad, 884 links
in the Switzerland network) train capacity boundaryCapacity=1000 instead of 1, so two trains may
legitimately be on such a track at the same time and such a link needs no capacity-1 reservation.
Resource = unordered node pair (linkPair), therefore a resource is exempt when its links carry the
boundaryExtension attribute. Exempt resources are skipped in (2) body overlaps, (3a) unreserved
body and (3b) stale/open reservations, and counted in the log line "I2: ... exempt". Everything
else is unchanged from E2's audit_e2.py (per-line vehicle lengths).

Usage: python3 scripts/audit_safety.py [--exp matsim_input/<case>] [--out matsim_input/<case>/audit]
                                       [--parse-only] [--skip-parse]

(round 2: E2's/D5's audit, delivered by agent I2 as audit_i2.py; off-model resources exempt)
Writes audit.json + audit.txt into --out; exit code 0 even when it finds a violation, read the JSON.
"""
import argparse
import bisect
import collections
import gzip
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from array import array

TOL_RES_S = 2.0          # event times are rounded up to the sim step
OVERLAP_M = 0.5          # metres: below this two bodies are considered not overlapping
STALE_S = 600.0          # 10 min
NODE_WINDOW_S = 5.0
GAP_S = 15.0             # state events are ~10 s apart; longer gaps are stand-stills

VEH = {  # run.properties vehicles=realistic, prepare/corrections/rolling_stock.csv
    'IC': dict(length=205., vmax=55.556, accel=0.45, decel=0.60),
    'EC': dict(length=195., vmax=55.556, accel=0.45, decel=0.60),
    'IR': dict(length=150., vmax=44.444, accel=0.60, decel=0.65),
    'RE': dict(length=120., vmax=44.444, accel=0.70, decel=0.70),
    'S':  dict(length=100., vmax=38.889, accel=0.85, decel=0.85),
    'other': dict(length=100., vmax=38.889, accel=0.70, decel=0.70),
}
TERMINI_NAMES = ['Zürich HB', 'Luzern', 'Basel SBB', 'Genève-Aéroport', 'Interlaken Ost',
                 'Brig', 'Chur', 'St. Gallen', 'Lugano', 'Chiasso', 'Bern']


# --------------------------------------------------------------------- helpers

def hms(s):
    if s is None:
        return None
    p = [int(x) for x in s.split(':')]
    return p[0] * 3600 + p[1] * 60 + p[2]


ATTR = re.compile(r'([\w:.-]+)="([^"]*)"')


def log(*a, **k):
    print(*a, **k, flush=True)


def ga(line, name):
    """Value of attribute `name` in an event line (fast path)."""
    i = line.find(' ' + name + '="')
    if i < 0:
        return None
    i += len(name) + 3
    j = line.find('"', i)
    return line[i:j]


def parse_event_line(line):
    t = line.find('type="')
    if t < 0:
        return None
    return line[t + 6:line.find('"', t + 6)]


def load_network(path):
    links = {}
    nodes = {}
    boundary = set()
    for ev, el in ET.iterparse(gzip.open(path, 'rb'), events=('end',)):
        if el.tag == 'link':
            lid = el.get('id')
            links[lid] = (el.get('from'), el.get('to'), float(el.get('length')),
                          float(el.get('freespeed')))
            for at in el.findall('attributes/attribute'):
                if at.get('name') in ('boundaryExtension', 'offModel') and \
                        (at.text or '').strip().lower() in ('true', '1', 'yes'):
                    boundary.add(lid)
            el.clear()
        elif el.tag == 'node':
            nodes[el.get('id')] = (float(el.get('x')), float(el.get('y')))
            el.clear()
    return links, nodes, boundary


def load_schedule(path):
    """veh -> dict(route links, offsets, stops, type, line, dep_time)."""
    stops = {}          # facility id -> (name, x, y, link)
    trains = {}
    for ev, el in ET.iterparse(gzip.open(path, 'rb'), events=('end',)):
        if el.tag == 'stopFacility':
            stops[el.get('id')] = (el.get('name'), float(el.get('x')), float(el.get('y')),
                                   el.get('linkRefId'))
            el.clear()
        elif el.tag == 'transitLine':
            line = el
            lname = line.get('name')
            ltype = None
            for p in ('EC', 'IC', 'IR', 'RE', 'S'):
                if lname.startswith(p) and (ltype is None or len(p) > len(ltype)):
                    ltype = p
            ltype = ltype or 'other'
            for route in line.findall('transitRoute'):
                # E2: the <route> element of this schedule lists every link as a child
                # <link refId=.../> (no startLinkId/endLinkId attributes), so D5's parse is complete.
                rnode = route.find('route')
                links = [l.get('refId') for l in rnode.findall('link')]
                stoplist = []
                for s in route.findall('routeProfile/stop'):
                    stoplist.append(dict(fac=s.get('refId'),
                                         arr=hms(s.get('arrivalOffset')),
                                         dep=hms(s.get('departureOffset'))))
                for dep in route.findall('departures/departure'):
                    vid = 'line_%s_%s' % (lname, dep.get('id'))
                    dep_t = hms(dep.get('departureTime'))
                    my_stops = []
                    for s in stoplist:
                        s2 = dict(s)
                        s2['arr_abs'] = dep_t + s['arr']
                        s2['dep_abs'] = dep_t + s['dep']
                        my_stops.append(s2)
                    trains[vid] = dict(route=links, stops=my_stops, type=ltype,
                                       line=lname, route_id=route.get('id'),
                                       dep_id=dep.get('id'), dep_time=dep_t,
                                       station=stops.get(stoplist[0]['fac'], ('?',))[0] if stoplist else '?')
            el.clear()
    return stops, trains


def route_offsets(links, net):
    offs = []
    o = 0.0
    for lid in links:
        offs.append(o)
        o += net[lid][2]
    return offs, o


def load_vehicle_specs(exp):
    """E2 fix: per-line rolling stock from run.properties vehicles.file (vehicles_rs1.csv).

    D5's audit.py used one length per vehicle *type*, which over-/underestimates the body of every
    line that has its own row (IR35 134.5 m vs 150 m, IC5 377.6 m vs 205 m): the recorded tail then
    disagrees with the reconstruction and the overlap/unreserved checks see bodies that do not exist.
    """
    specs = []
    props = os.path.join(exp, 'run.properties')
    vfile = None
    if os.path.exists(props):
        for line in open(props):
            if line.startswith('vehicles.file='):
                vfile = line.split('=', 1)[1].strip()
    if not vfile:
        return specs
    path = vfile if os.path.isabs(vfile) else os.path.join(exp, vfile)
    if not os.path.exists(path):
        log('E2: vehicles file %s not found, using type lengths' % path)
        return specs
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        p = [x.strip() for x in line.split(',')]
        if len(p) != 5:
            continue
        try:
            specs.append(dict(prefix=p[0], length=float(p[1]), vmax=float(p[2]) / 3.6,
                              accel=float(p[3]), decel=float(p[4])))
        except ValueError:
            continue  # header
    log('E2: %d per-line vehicle specs from %s' % (len(specs), path))
    return specs


def spec_for(specs, line):
    best = None
    for s in specs:
        if s['prefix'] == '' or (line or '').startswith(s['prefix']):
            if best is None or len(s['prefix']) > len(best['prefix']):
                best = s
    return best


# ------------------------------------------------------------------ event parse

def parse_events(evpath, cache, parse_only=False):
    """Stream the events once into small gz caches, tolerant to a truncated file."""
    os.makedirs(cache, exist_ok=True)
    files = {k: gzip.open(os.path.join(cache, k + '.tsv.gz'), 'wt') for k in
             ('states', 'linkstates', 'stops', 'linkenter', 'traffic')}
    n = collections.Counter()
    w = {k: files[k].write for k in files}
    try:
        with gzip.open(evpath, 'rt') as f:
            for line in f:
                typ = parse_event_line(line)
                if typ is None:
                    continue
                if typ == 'railsimTrainStateEvent':
                    n[typ] += 1
                    w['states']('%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' % (
                        ga(line, 'exactTime') or ga(line, 'time'), ga(line, 'headLink'),
                        ga(line, 'headPosition'), ga(line, 'speed'), ga(line, 'acceleration'),
                        ga(line, 'vehicle'), ga(line, 'tailLink'), ga(line, 'tailPosition')))
                elif typ == 'railsimLinkStateChangeEvent':
                    n[typ] += 1
                    w['linkstates']('%s\t%s\t%s\t%s\n' % (
                        ga(line, 'time'), ga(line, 'link'),
                        '0' if ga(line, 'state') == 'EMPTY' else '1', ga(line, 'vehicle')))
                elif typ == 'VehicleArrivesAtFacility' or typ == 'VehicleDepartsAtFacility':
                    n[typ] += 1
                    w['stops']('%s\t%s\t%s\t%s\n' % (
                        ga(line, 'time'), typ[7], ga(line, 'facility'), ga(line, 'vehicle')))
                elif typ == 'entered link' or typ == 'link enter':
                    n['entered link'] += 1
                    w['linkenter']('%s\t%s\t%s\n' % (
                        ga(line, 'time'), ga(line, 'link'), ga(line, 'vehicle')))
                elif typ == 'vehicle enters traffic':
                    n[typ] += 1
                    w['traffic']('%s\tE\t%s\t%s\n' % (
                        ga(line, 'time'), ga(line, 'link'), ga(line, 'vehicle')))
                elif typ == 'vehicle leaves traffic':
                    n[typ] += 1
                    w['traffic']('%s\tL\t%s\t%s\n' % (
                        ga(line, 'time'), ga(line, 'link'), ga(line, 'vehicle')))
    except (EOFError, OSError, ValueError) as e:
        log('events file truncated while parsing (partial cache): %s' % e, file=sys.stderr)
    finally:
        for f in files.values():
            f.close()
    log('event counts: %s' % dict(n))
    return n


# ----------------------------------------------------------------------- train

class Train:
    __slots__ = ('vid', 'type', 'length', 'vmax', 'accel', 'decel', 'links', 'offs',
                 'cum_end', 'ts', 'ss', 'vs', 'as_', 't_first', 't_last', 'lens',
                 'offs_by_link')

    def __init__(self, vid, typ, route, net, spec=None):
        self.vid = vid
        self.type = typ
        p = dict(VEH[typ])
        if spec is not None:
            p.update(spec)
        self.length = p['length']
        self.vmax = p['vmax']
        self.accel = p['accel']
        self.decel = p['decel']
        self.links = route
        self.lens = [net[l][2] for l in route]
        self.offs, tot = route_offsets(route, net)
        self.offs_by_link = {}
        for off, lid in zip(self.offs, route):
            self.offs_by_link.setdefault(lid, []).append(off)
        self.cum_end = tot
        self.ts = array('d')
        self.ss = array('d')
        self.vs = array('d')
        self.as_ = array('d')
        self.t_first = None
        self.t_last = None

    def add(self, t, s, v, a):
        if self.ts and t < self.ts[-1]:
            return
        self.ts.append(t)
        self.ss.append(s)
        self.vs.append(v)
        self.as_.append(a)
        if self.t_first is None:
            self.t_first = t
        self.t_last = t

    def finish(self):
        # monotone s (a tiny backwards step would break the bisect)
        for i in range(1, len(self.ss)):
            if self.ss[i] < self.ss[i - 1]:
                self.ss[i] = self.ss[i - 1]

    def s_at(self, t):
        ts, ss, vs = self.ts, self.ss, self.vs
        if t <= ts[0]:
            return ss[0]
        if t >= ts[-1]:
            return ss[-1]
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return ss[0]
        t0, t1 = ts[i - 1], ts[i]
        dt = t1 - t0
        # POSITION events come every ~10 s. A longer gap is a stand-still (stop,
        # waiting for reservation, insertion): hold the last known position
        # instead of spreading the next movement over the whole gap.
        if dt > GAP_S or t1 <= t0:
            return ss[i - 1]
        return ss[i - 1] + (ss[i] - ss[i - 1]) * (t - t0) / dt

    def t_at_s(self, s):
        """First time the head reaches route distance s (None if never)."""
        ts, ss = self.ts, self.ss
        if s <= ss[0]:
            return ts[0]
        if s > ss[-1]:
            return None
        i = bisect.bisect_left(ss, s)
        if i <= 0:
            return ts[0]
        s0, s1 = ss[i - 1], ss[i]
        dt = ts[i] - ts[i - 1]
        if s1 <= s0 or dt > GAP_S:
            return ts[i]
        return ts[i - 1] + dt * (s - s0) / (s1 - s0)

    def body(self, t):
        """[(route index, a, b)] body coverage in link-local coordinates."""
        s = self.s_at(t)
        lo, hi = s - self.length, s
        i0 = bisect.bisect_right(self.offs, lo) - 1
        i1 = bisect.bisect_right(self.offs, hi) - 1
        out = []
        n = len(self.links)
        for i in range(max(0, i0), min(i1, n - 1) + 1):
            off = self.offs[i]
            a = max(lo, off) - off
            b = min(hi, off + self.lens[i]) - off
            if b - a > 1e-6:
                out.append((i, a, b))
        return out

    def set_lens(self, net):
        self.lens = [net[l][2] for l in self.links]


def canon_interval(link, net, a, b):
    """Directional (a,b) -> canonical coordinate from the smaller node id."""
    frm, to, ln = net[link][0], net[link][1], net[link][2]
    if frm <= to:
        return a, b
    return ln - b, ln - a


def res_key(link, net):
    frm, to = net[link][0], net[link][1]
    return (frm, to) if frm <= to else (to, frm)


# --------------------------------------------------------------------- results

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exp', default='matsim_input/olten')
    ap.add_argument('--out', default=None, help='default <exp>/audit')
    ap.add_argument('--skip-parse', action='store_true')
    args = ap.parse_args()
    if args.out is None:
        args.out = os.path.join(args.exp, 'audit')
    os.makedirs(args.out, exist_ok=True)

    def case_file(name):
        """the network/schedule of a pipeline case sits in <case>/routed/, an experiment has it at its root"""
        p = os.path.join(args.exp, name)
        return p if os.path.exists(p) else os.path.join(args.exp, 'routed', name)

    net, nodes, boundary_links = load_network(case_file('transitNetwork.xml.gz'))
    # I2: resources of off-model links (capacity > 1 at run time) are exempt from the capacity-1 checks
    # only off-model links that really get unlimited capacity at run time (run.properties boundaryCapacity > 1)
    boundary_capacity = 1
    props = os.path.join(args.exp, 'run.properties')
    if os.path.exists(props):
        for line in open(props):
            if line.startswith('boundaryCapacity='):
                try:
                    boundary_capacity = float(line.split('=', 1)[1].strip())
                except ValueError:
                    pass
    exempt = {res_key(l, net) for l in boundary_links if l in net} if boundary_capacity > 1 else set()
    log('I2: boundaryCapacity=%g: %d off-model links (boundaryExtension/offModel) on %d resources '
        'are exempt from the overlap and reservation checks' % (boundary_capacity, len(boundary_links), len(exempt)))
    stops, sched = load_schedule(case_file('transitSchedule.xml.gz'))
    log('network %d links %d nodes, schedule %d trains, %d stop facilities'
          % (len(net), len(nodes), len(sched), len(stops)))

    import glob
    evpath = os.path.join(args.exp, 'output', 'ITERS', 'it.0', 'ch_olten.0.events.xml.gz')
    if not os.path.exists(evpath):
        found = []
        for pat in ('*output_events.xml.gz', '*.events.xml.gz'):
            found += glob.glob(os.path.join(args.exp, 'output', pat))
        found = sorted(set(found), key=os.path.getmtime, reverse=True)
        if not found:
            sys.exit('no events file in %s/output (run scripts/run_case.sh first)' % args.exp)
        evpath = found[0]
    cache = os.path.join(args.out, 'cache')
    if not args.skip_parse:
        parse_events(evpath, cache)

    R = {}  # results

    # ---------------- load states into Train objects
    specs = load_vehicle_specs(args.exp)
    trains = {}
    for vid, info in sched.items():
        tr = Train(vid, info['type'], info['route'], net, spec_for(specs, info.get('line')))
        trains[vid] = tr
    nstate = 0
    orphan = 0
    tail_check = dict(n=0, agree=0, worst=0.0, worst_case=None, over50=0)
    gap_stat = collections.Counter()
    with gzip.open(os.path.join(cache, 'states.tsv.gz'), 'rt') as f:
        for line in f:
            t, hl, hp, v, a, vid, tl, tp = line.rstrip('\n').split('\t')
            tr = trains.get(vid)
            if tr is None or hl == 'null':
                orphan += 1
                continue
            t = float(t); hp = float(hp); v = float(v); a = float(a)
            if tr.ts:
                dt = t - tr.ts[-1]
                gap_stat['lt15' if dt <= GAP_S else ('lt60' if dt <= 60 else 'gt60')] += 1
            prev_s = tr.ss[-1] if tr.ss else 0.0
            best = None
            # candidate s for this head link: first offset at or after prev position
            for off in tr.offs_by_link.get(hl, ()):
                cand = off + hp
                if cand >= prev_s - 5.0:
                    best = cand
                    break
            if best is None:
                offs = tr.offs_by_link.get(hl)
                if offs:
                    best = offs[0] + hp
            if best is None:
                best = prev_s
            tr.add(t, best, v, a)
            # compare my reconstructed tail with the recorded one
            if tl != 'null' and tl in tr.offs_by_link:
                cand = [(off + float(tp)) for off in tr.offs_by_link[tl]]
                if cand:
                    rec_s = min(cand, key=lambda x: abs(x - (best - tr.length)))
                    d = abs((best - tr.length) - rec_s)
                    tail_check['n'] += 1
                    if d < 5.0:
                        tail_check['agree'] += 1
                    if d > 50:
                        tail_check['over50'] += 1
                    if d > tail_check['worst']:
                        tail_check['worst'] = d
                        tail_check['worst_case'] = dict(train=vid, t=t, d=d,
                                                        head=hl, tail=tl)
            nstate += 1
    for tr in trains.values():
        tr.finish()
    active = [tr for tr in trains.values() if tr.ts]
    R['state_events'] = nstate
    R['gap'] = dict(gap_stat)
    tail_check['worst_case'] = tail_check['worst_case']
    R['tail_vs_reconstruction'] = tail_check
    log('state events %d, trains with events %d / %d; recorded tail within 5 m of '
          'reconstruction in %.2f%% of %d events (max diff %.1f m); gaps %s'
          % (nstate, len(active), len(trains),
             100.0 * tail_check['agree'] / max(1, tail_check['n']), tail_check['n'],
             tail_check['worst'], dict(gap_stat)))

    # ---------------- per train per link body intervals + reservation timelines
    res_t = collections.defaultdict(lambda: array('d'))
    res_v = collections.defaultdict(lambda: array('i'))
    res_b = collections.defaultdict(lambda: array('b'))
    veh_index = {}
    veh_names = []

    def vindex(vid):
        i = veh_index.get(vid)
        if i is None:
            i = len(veh_names)
            veh_index[vid] = i
            veh_names.append(vid)
        return i

    with gzip.open(os.path.join(cache, 'linkstates.tsv.gz'), 'rt') as f:
        for line in f:
            t, link, state, vid = line.rstrip('\n').split('\t')
            if link not in net:
                continue
            rk = res_key(link, net)
            res_t[rk].append(float(t))
            res_v[rk].append(vindex(vid))
            res_b[rk].append(1 if state == '1' else 0)
    # holder timeline per resource (capacity 1 everywhere: EMPTY or EXHAUSTED);
    # events are written in time order, so each resource's arrays are sorted
    holder = {}
    for rk in res_t:
        ts = res_t[rk]
        vs = res_v[rk]
        bs = res_b[rk]
        tl_t = array('d')
        tl_h = array('i')
        for t, vi, b in zip(ts, vs, bs):
            h = vi if b else -1
            if tl_h and tl_h[-1] == h:
                continue
            tl_t.append(t)
            tl_h.append(h)
        holder[rk] = (tl_t, tl_h)
    nlv = sum(len(x) for x in res_t.values())
    log('link-state events %d on %d resources' % (nlv, len(res_t)))

    link_iv = collections.defaultdict(list)   # resource -> [(t0,t1,train,link)]
    for tr in active:
        L = tr.length
        lastt = tr.t_last
        for i, lid in enumerate(tr.links):
            off = tr.offs[i]
            if off > tr.ss[-1]:
                break
            t0 = tr.t_at_s(off)
            if t0 is None:
                continue
            t1 = tr.t_at_s(off + net[lid][2] + L)
            if t1 is None:
                t1 = lastt
            t0 = max(t0, tr.t_first)
            t1 = min(t1, lastt)
            if t1 - t0 > 0.01:
                link_iv[res_key(lid, net)].append((t0, t1, tr.vid, lid))

    # ---------------- 2. body overlaps on one resource
    overlaps = []
    pair_windows = 0
    log('overlap candidates: %d resources with >=2 intervals'
        % sum(1 for x in link_iv.values() if len(x) > 1))
    for rk, ivs in link_iv.items():
        if rk in exempt:
            continue
        if len(ivs) < 2:
            continue
        ivs.sort()
        for i in range(len(ivs)):
            t0a, t1a, ta, la = ivs[i]
            for j in range(i + 1, len(ivs)):
                t0b, t1b, tb, lb = ivs[j]
                if t0b >= t1a:
                    break
                if ta == tb:
                    continue
                wa, wb = max(t0a, t0b), min(t1a, t1b)
                if wb - wa < 0.05:
                    continue
                pair_windows += 1
                if pair_windows % 20000 == 0:
                    log('  %d candidate windows ...' % pair_windows)
                if pair_windows > 400000:
                    log('  too many windows, stopping overlap scan')
                    break
                tr_a, tr_b = trains[ta], trains[tb]
                worst = 0.0
                dur = 0.0
                nt = int(min(60, max(2, wb - wa)))
                for k in range(nt + 1):
                    t = wa + (wb - wa) * k / nt
                    ca = [(a, b) for (ii, a, b) in tr_a.body(t)
                          if res_key(tr_a.links[ii], net) == rk
                          for a, b in [canon_interval(tr_a.links[ii], net, a, b)]]
                    cb = [(a, b) for (ii, a, b) in tr_b.body(t)
                          if res_key(tr_b.links[ii], net) == rk
                          for a, b in [canon_interval(tr_b.links[ii], net, a, b)]]
                    m = 0.0
                    for a0, a1 in ca:
                        for b0, b1 in cb:
                            ov = min(a1, b1) - max(a0, b0)
                            if ov > m:
                                m = ov
                    if m > worst:
                        worst = m
                    if m > OVERLAP_M:
                        dur += (wb - wa) / nt
                if worst > OVERLAP_M:
                    overlaps.append(dict(t0=wa, t1=wb, dur=dur, metres=worst,
                                         a=ta, b=tb, link_a=la, link_b=lb,
                                         node_a=net[la][0] + '/' + net[la][1]))
    R['overlaps'] = len(overlaps)
    R['overlap_pairs'] = pair_windows
    overlaps.sort(key=lambda o: -o['metres'])

    def nearest_stop(nd):
        x, y = nodes.get(nd, (0, 0))
        best = None
        for sid, (nm, sx, sy, lk) in stops.items():
            d = (sx - x) ** 2 + (sy - y) ** 2
            if best is None or d < best[0]:
                best = (d, nm if nm else sid)
        return best[1] if best else nd

    for o in overlaps:
        o['station'] = nearest_stop(net[o['link_a']][0])
    R['overlaps_worst'] = overlaps[:12]
    log('overlaps: %d of %d candidate windows' % (len(overlaps), pair_windows))

    # ---------------- 3a. body without reservation
    def holder_at(rk, t):
        tl = holder.get(rk)
        if not tl:
            return None
        i = bisect.bisect_right(tl[0], t) - 1
        if i < 0:
            return None
        h = tl[1][i]
        return None if h < 0 else veh_names[h]

    unres = []
    nsamp = 0
    for tr in active:
        # sample every 5 s between first and last event
        t = tr.t_first
        while t < tr.t_last:
            nsamp += 1
            for i, a, b in tr.body(t):
                if b - a <= 1e-6:
                    continue
                if res_key(tr.links[i], net) in exempt:
                    continue
                h = holder_at(res_key(tr.links[i], net), t)
                if h != tr.vid:
                    unres.append((t, tr.vid, tr.links[i], h))
                    break
            t += 5.0
    # merge consecutive samples per (vid, link) into intervals
    unres.sort(key=lambda x: (x[1], x[2], x[0]))
    merged = []
    for t, vid, lid, h in unres:
        if merged and merged[-1][1] == vid and merged[-1][2] == lid and t - merged[-1][3] <= 6:
            merged[-1][3] = t
        else:
            merged.append([t, vid, lid, t, h])
    R['unreserved_samples'] = len(unres)
    R['unreserved_intervals'] = len(merged)
    R['unreserved_worst'] = sorted([dict(t0=m[0], t1=m[3], dur=m[3] - m[0], train=m[1],
                                         link=m[2], holder=m[4])
                                    for m in merged if m[3] - m[0] > 5],
                                   key=lambda d: -d['dur'])[:12]
    R['unreserved_held_by_other'] = sum(1 for m in merged if m[4] is not None)
    R['unreserved_samples_checked'] = nsamp
    by_link = collections.Counter()
    dur_link = collections.Counter()
    for m in merged:
        by_link[m[2]] += 1
        dur_link[m[2]] += m[3] - m[0]
    R['unreserved_by_link'] = [dict(link=l, intervals=c, seconds=dur_link[l],
                                    station=nearest_stop(net[l][0]))
                               for l, c in by_link.most_common(10)]
    R['unreserved_dur_hist'] = dict(
        le10=sum(1 for m in merged if m[3] - m[0] <= 10),
        le60=sum(1 for m in merged if 10 < m[3] - m[0] <= 60),
        gt60=sum(1 for m in merged if m[3] - m[0] > 60))
    log('unreserved body samples %d of %d samples in %d intervals (%d intervals where another train held the track)'
        % (len(unres), nsamp, len(merged), R['unreserved_held_by_other']))
    log('  by link: %s' % R['unreserved_by_link'][:5])
    log('  duration histogram: %s' % R['unreserved_dur_hist'])

    # ---------------- 3b. stale reservations
    cover = collections.defaultdict(list)   # (res, train) -> [(t0,t1)]
    for rk, ivs in link_iv.items():
        for t0, t1, vid, lid in ivs:
            cover[(rk, vid)].append((t0, t1))
    stale = []
    for rk, (tl_t, tl_h) in holder.items():
        if rk in exempt:
            continue
        for k in range(len(tl_t)):
            hi = tl_h[k]
            if hi < 0:
                continue
            t0 = tl_t[k]
            vid = veh_names[hi]
            t1 = tl_t[k + 1] if k + 1 < len(tl_t) else 1e18
            if t1 - t0 < STALE_S:
                continue
            cv = sorted(cover.get((rk, vid), []))
            cov = 0.0
            for a, b in cv:
                a, b = max(a, t0), min(b, t1)
                if b > a:
                    cov += b - a
            if (t1 - t0) - cov > STALE_S:
                stale.append(dict(res=rk, train=vid, t0=t0, t1=min(t1, 1e9),
                                  dur=min(t1, 1e9) - t0, covered=cov))
    stale.sort(key=lambda s: -(s['dur'] - s['covered']))
    R['stale_reservations'] = len(stale)
    R['stale_worst'] = stale[:12]
    log('stale reservations >10 min: %d' % len(stale))

    # ---------------- reservations still held at the end / after a train vanished
    open_after = []
    last_ev = {tr.vid: tr.t_last for tr in active}
    for rk, (tl_t, tl_h) in holder.items():
        if rk in exempt:
            continue
        if len(tl_h) and tl_h[-1] >= 0:
            open_after.append((tl_t[-1], veh_names[tl_h[-1]], rk))
    R['open_at_end'] = len(open_after)
    R['open_at_end_sample'] = open_after[:10]
    after = []
    for rk, (tl_t, tl_h) in holder.items():
        if rk in exempt:
            continue
        for k in range(len(tl_t)):
            hi = tl_h[k]
            if hi < 0:
                continue
            vid = veh_names[hi]
            t1 = tl_t[k + 1] if k + 1 < len(tl_t) else 1e18
            le = last_ev.get(vid)
            if le is not None and t1 > le + TOL_RES_S:
                after.append((le, t1, vid, rk))
    R['held_after_last_event'] = len(after)

    # ---------------- 4. node co-incidences
    move = collections.defaultdict(list)   # node -> [(t, vid, in, out)]
    prev_link = {}
    with gzip.open(os.path.join(cache, 'linkenter.tsv.gz'), 'rt') as f:
        for line in f:
            t, lid, vid = line.rstrip('\n').split('\t')
            if lid not in net:
                continue
            p = prev_link.get(vid)
            prev_link[vid] = lid
            if p is None:
                continue
            move[net[lid][0]].append((float(t), vid, p, lid))
    log('node passages: %d on %d nodes' % (sum(len(v) for v in move.values()), len(move)))
    node_hits = collections.Counter()
    node_pairs = collections.Counter()
    node_nolink = collections.Counter()
    node_1s = collections.Counter()
    node_all = collections.Counter()
    for nd, evs in move.items():
        if len(evs) < 2:
            continue
        evs.sort()
        for i in range(len(evs)):
            t, vid, pin, pout = evs[i]
            for j in range(i + 1, len(evs)):
                t2, v2, qi, qo = evs[j]
                if t2 - t > NODE_WINDOW_S:
                    break
                if v2 == vid:
                    continue
                node_all[nd] += 1
                if (pin, pout) == (qi, qo):
                    continue
                node_pairs[nd] += 1
                node_hits[nd] += 1
                if t2 - t < 1.0:
                    node_1s[nd] += 1
                if not ({pin, pout} & {qi, qo}):
                    node_nolink[nd] += 1
    R['node_pairs'] = sum(node_pairs.values())
    R['node_pairs_1s'] = sum(node_1s.values())
    R['node_pairs_nolink'] = sum(node_nolink.values())
    R['node_pairs_all'] = sum(node_all.values())
    R['nodes_with_pairs'] = len(node_pairs)

    def nearest_stop(nd):
        x, y = nodes.get(nd, (0, 0))
        best = None
        for sid, (nm, sx, sy, lk) in stops.items():
            d = (sx - x) ** 2 + (sy - y) ** 2
            if best is None or d < best[0]:
                best = (d, nm if nm else sid)
        return best[1] if best else nd

    top = []
    for nd, c in node_hits.most_common(20):
        top.append(dict(node=nd, station=nearest_stop(nd), pairs=c, pairs_1s=node_1s[nd],
                        pairs_nolink=node_nolink[nd],
                        xy=nodes.get(nd)))
    R['node_top20'] = top
    log('node co-incidences %d (1s %d, no shared link %d) on %d nodes'
          % (R['node_pairs'], R['node_pairs_1s'], R['node_pairs_nolink'], len(node_pairs)))

    # ---------------- 5. plausibility
    per_type = collections.defaultdict(lambda: dict(n=0, vmax=-1e9, a=-1e9, d=-1e9, neg=0,
                                                    dv_max=-1e9, dv_min=1e9, dv_bad=0))
    tele = []
    back = []
    for tr in active:
        p = per_type[tr.type]
        p['n'] += 1
        for v, a in zip(tr.vs, tr.as_):
            if v > p['vmax']:
                p['vmax'] = v
            if a > p['a']:
                p['a'] = a
            if a < p['d']:
                p['d'] = a
            if v < -0.01:
                p['neg'] += 1
        for i in range(1, len(tr.ts)):
            dt = tr.ts[i] - tr.ts[i - 1]
            dv = tr.vs[i] - tr.vs[i - 1]
            if 0.4 <= dt <= 3.0:
                rate = dv / dt
                if rate > p['dv_max']:
                    p['dv_max'] = rate
                if rate < p['dv_min']:
                    p['dv_min'] = rate
                if rate > tr.accel * 1.2 + 0.05 or rate < -tr.decel * 1.2 - 0.05:
                    p['dv_bad'] += 1
            ds = tr.ss[i] - tr.ss[i - 1]
            if dt <= 0:
                continue
            vmax = max(tr.vs[i], tr.vs[i - 1], 0)
            allowed = vmax * dt + 0.5 * tr.accel * dt * dt + 50.0
            # at a reversal the leading end of the train becomes the other end:
            # the head jumps by ~one train length along the route path
            reversal = abs(ds - tr.length) < 0.1 * tr.length + 20 and tr.vs[i] < 2
            if ds > allowed + 50 and not reversal:
                tele.append(dict(train=tr.vid, t=tr.ts[i], dt=dt, ds=ds, allowed=allowed))
            if ds < -3.0:
                back.append(dict(train=tr.vid, t=tr.ts[i], dt=dt, ds=ds))
    R['motion_types'] = {k: v for k, v in per_type.items()}
    R['teleports'] = len(tele)
    R['teleports_worst'] = sorted(tele, key=lambda d: -d['ds'])[:10]
    R['backwards'] = len(back)
    R['backwards_worst'] = sorted(back, key=lambda d: d['ds'])[:10]
    log('teleports %d, backwards steps %d' % (len(tele), len(back)))

    # arrival order vs schedule stops
    arr = collections.defaultdict(list)
    with gzip.open(os.path.join(cache, 'stops.tsv.gz'), 'rt') as f:
        for line in f:
            t, kind, fac, vid = line.rstrip('\n').split('\t')
            if kind == 'A':
                arr[vid].append((float(t), fac))
    bad_order = []
    for vid, evs in arr.items():
        info = sched.get(vid)
        if not info:
            continue
        order = {}
        for k, s in enumerate(info['stops']):
            order.setdefault(s['fac'], k)
        last = -1
        for t, fac in evs:
            k = order.get(fac)
            if k is None:
                continue
            if k < last:
                bad_order.append(dict(train=vid, t=t, fac=fac))
                break
            last = k
    R['arrival_order_bad'] = len(bad_order)
    R['arrival_order_worst'] = bad_order[:10]
    log('arrival order violations %d' % len(bad_order))

    # ---------------- 6. start / end of run
    traffic = collections.defaultdict(list)
    with gzip.open(os.path.join(cache, 'traffic.tsv.gz'), 'rt') as f:
        for line in f:
            t, kind, lid, vid = line.rstrip('\n').split('\t')
            traffic[vid].append((kind, float(t), lid))
    ins_delay = []
    for vid, evs in traffic.items():
        info = sched.get(vid)
        if not info:
            continue
        ent = [e for e in evs if e[0] == 'E']
        if not ent:
            ins_delay.append(dict(train=vid, delay=None))
            continue
        ins_delay.append(dict(train=vid, delay=ent[0][1] - info['dep_time'],
                              link=ent[0][2], t=ent[0][1], sched=info['dep_time'],
                              type=info['type'], line=info['line']))
    known = [d for d in ins_delay if d['delay'] is not None]
    known.sort(key=lambda d: -d['delay'])
    R['insertion'] = dict(
        n=len(known), no_insert=len(ins_delay) - len(known),
        late0=sum(1 for d in known if d['delay'] > 0.5),
        late60=sum(1 for d in known if d['delay'] > 60),
        late180=sum(1 for d in known if d['delay'] > 180),
        max=known[0]['delay'] if known else None,
        worst=known[:12])
    log('insertion: %d trains, %d >0s, %d >60s, %d >180s, max %.0fs'
          % (len(known), R['insertion']['late0'], R['insertion']['late60'],
             R['insertion']['late180'], R['insertion']['max'] or 0))

    # disappearance
    last_arr = {}
    for vid, evs in arr.items():
        last_arr[vid] = max(t for t, _ in evs)
    dis = []
    leaves = sum(1 for vid, evs in traffic.items() for e in evs if e[0] == 'L')
    for tr in active:
        la = last_arr.get(tr.vid)
        sched_last = sched.get(tr.vid, {}).get('stops')
        sched_arr = sched_last[-1]['arr_abs'] if sched_last else None
        dis.append(dict(train=tr.vid, t_last=tr.t_last, last_arr=la,
                        hold=(tr.t_last - la) if la is not None else None,
                        sched_arr=sched_arr,
                        late=(la - sched_arr) if (la is not None and sched_arr is not None) else None,
                        tail_neg=None))
    holds = [d['hold'] for d in dis if d['hold'] is not None]
    holds.sort()
    R['disappear'] = dict(n=len(dis), leaves_traffic=leaves,
                          hold_p50=holds[len(holds) // 2] if holds else None,
                          hold_max=max(holds) if holds else None,
                          hold_over60=sum(1 for h in holds if h > 60),
                          late0=sum(1 for d in dis if d['late'] is not None and d['late'] > 0.5),
                          late60=sum(1 for d in dis if d['late'] is not None and d['late'] > 60),
                          late180=sum(1 for d in dis if d['late'] is not None and d['late'] > 180),
                          late_max=max((d['late'] for d in dis if d['late'] is not None), default=None),
                          worst=sorted(dis, key=lambda d: -(d['late'] or -1e9))[:10])
    log('disappearance: %d trains, %d leave-traffic events, final-stop hold p50 %s s, max %s s'
          % (len(dis), leaves, R['disappear']['hold_p50'], R['disappear']['hold_max']))

    # "phantom" body at insertion: how much of the train hangs off the network
    phantom = []
    for tr in active:
        if not tr.ts:
            continue
        # first state event tail position not recorded, but body starts at s=0
        first_s = tr.ss[0]
        if first_s < tr.length - 1:
            phantom.append(dict(train=tr.vid, type=tr.type, off=tr.length - first_s,
                                first_link=tr.links[0], link_len=net[tr.links[0]][2]))
    phantom.sort(key=lambda d: -d['off'])
    R['phantom_insertion'] = dict(n=len(phantom), worst=phantom[:10],
                                  over100=sum(1 for p in phantom if p['off'] > 100))

    # platform occupancy at termini. A platform TRACK is a station+platform key
    # ("8507000:0:8"); its two direction links carry two different stop facilities.
    def platkey(fac):
        return fac.split('.link:')[0]

    departs = collections.defaultdict(list)
    with gzip.open(os.path.join(cache, 'stops.tsv.gz'), 'rt') as f:
        for line in f:
            t, kind, fac, vid = line.rstrip('\n').split('\t')
            if kind == 'D':
                departs[(vid, fac)].append(float(t))
    # per platform track: sim occupancy intervals from the events
    fac_iv = collections.defaultdict(list)
    for vid, evs in arr.items():
        for t, fac in evs:
            ds = departs.get((vid, fac))
            t1 = min(ds) if ds else last_ev.get(vid, t)
            fac_iv[platkey(fac)].append((t, max(t1, t), vid))

    by_station = collections.defaultdict(set)
    for fac, (nm, x, y, lk) in stops.items():
        by_station[nm].add(platkey(fac))

    term_stats = {}
    for nm in TERMINI_NAMES:
        pks = by_station.get(nm, set())
        term = [d for d in dis if sched.get(d['train'], {}).get('stops')
                and platkey(sched[d['train']]['stops'][-1]['fac']) in pks]
        term.sort(key=lambda d: -(d['late'] or -1e9))
        # sim occupancy overlaps on the same platform track
        ov = 0
        for pk in pks:
            ivs = sorted(fac_iv.get(pk, []))
            for i in range(len(ivs)):
                for j in range(i + 1, len(ivs)):
                    if ivs[j][0] >= ivs[i][1]:
                        break
                    if ivs[i][2] != ivs[j][2]:
                        ov += 1
        term_stats[nm] = dict(n_term=len(term),
                              hold_p50=(sorted(d['hold'] for d in term if d['hold'] is not None)[len(term) // 2]
                                        if term else None),
                              platform_tracks=len(pks),
                              late_max=max((d['late'] for d in term if d['late'] is not None), default=None),
                              late0=sum(1 for d in term if d['late'] is not None and d['late'] > 0.5),
                              sim_overlaps=ov,
                              worst=term[:5])
    R['termini'] = term_stats

    # how often does the published timetable put the next departure on the same
    # platform track within X minutes after a terminating arrival?
    turn = collections.Counter()
    turn_list = []
    for nm in TERMINI_NAMES:
        pks = by_station.get(nm, set())
        dep_by_fac = collections.defaultdict(list)
        for vid, info in sched.items():
            for s in info['stops'][:-1]:      # the last stop has no departure
                if platkey(s['fac']) in pks:
                    dep_by_fac[platkey(s['fac'])].append((s['dep_abs'], vid))
        for pk, lst in dep_by_fac.items():
            lst.sort()
        for vid, info in sched.items():
            if not info['stops']:
                continue
            last = info['stops'][-1]
            pk = platkey(last['fac'])
            if pk not in pks:
                continue
            lst = dep_by_fac.get(pk, [])
            nxt = [(t, v) for t, v in lst if t >= last['arr_abs'] + 1 and v != vid]
            if not nxt:
                continue
            gap = nxt[0][0] - last['arr_abs']
            turn['n'] += 1
            if gap <= 300:
                turn['lt300'] += 1
            if gap <= 600:
                turn['lt600'] += 1
            if gap <= 1800:
                turn['lt1800'] += 1
            if gap <= 1800:
                turn_list.append(dict(station=nm, platform=pk, arr=last['arr_abs'],
                                      next_dep=nxt[0][0], gap=gap, train=vid,
                                      next_train=nxt[0][1]))
    turn_list.sort(key=lambda d: d['gap'])
    R['turnaround'] = dict(turn)
    R['turnaround_worst'] = turn_list[:15]
    log('terminal same-platform next departures: %s' % dict(turn))

    with open(os.path.join(args.out, 'results.json'), 'w') as f:
        json.dump(R, f, indent=1, default=str)
    log('wrote %s' % os.path.join(args.out, 'results.json'))


if __name__ == '__main__':
    main()
