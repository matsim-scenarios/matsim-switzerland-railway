#!/usr/bin/env python3
"""Pipeline step: Bern platform 4 stands on the real platform track (round 3, agent BN4).

    python3 prepare/bern_platform.py <schedule-in> <network-in> <schedule-out> <network-out> \\
            [--osm-net matsim_input/tmp/network_osm.xml.gz] [--depth 2] [--dry-run]

Ported from `swarm/r3/BN4/tools/apply_bern.py` (agent BN4), code unchanged except the input/output
arguments (explicit files instead of `--case/--base`, and gzip-aware writing).  Step 14 of
`scripts/prepare_case.sh`, after the off-model work; `--depth 2` is the depth BN4 measured and
shipped.  Measured effect (base12 data, sources unchanged): `abs_dev_seconds` Olten 65 184 -> 47 817
(-26.7 %), Switzerland 104 833 -> 99 392 (-5.2 %), Bern -44 % / -7 %, both cases complete, no
deadlock, audit clean.

Original header follows.

The case networks are built from `matsim_input/tmp/network_osm.xml.gz`, but the links no route
uses are dropped.  At Bern platform 4 (stop facilities `8507000:0:4.*`) that removed the whole
western part of the real platform track (OSM ways 98445139 + 396181885 + 65596810, 535 m between
the switches 794139835 and 817895789): the modelled track is only the eastern 128.6 m, so every
train that stops there has to stop with its head at node 1871076911 — 93 m from the eastern
switch — and its 150-288 m body hangs over that switch into the northern throat.

This tool
  1. adds the missing real track links 1871076911 <-> 800046404 (8 segments, 204.2 m: link
     366060/366061 and the 7 segments of OSM way 396181885) to the case network, copied verbatim
     (geometry, length, freespeed, OSM attributes) from `network_osm.xml.gz`;
  2. inserts them into the route of every train that *stands* at platform 4 and reverses there
     (arrival from the east continues 204.2 m west along the platform; the departure retraces
     them), and moves that route-stop's facility to the westernmost link (508218, head at node
     800046404 = 297.1 m from the eastern switch);
  3. leaves first/last stops (the train never stands) and every other stop untouched.

Nothing is retimed: arrivalOffset / departureOffset and the number of stops stay identical.

"""
import argparse
import collections
import copy
import gzip
import os
import shutil
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

OSM_NET = os.path.join(ROOT, 'matsim_input/tmp/network_osm.xml.gz')

NODES = ['3198451514', '817895792', '1871076908', '817895807', '817895909', '817895779',
         '800046348', '800046404']

# westward chain from 1871076911 to 800046404 (arrival), and back (departure)
ARRIVE = ['366061', '508230', '508228', '508226', '508224', '508222', '508220', '508218']
DEPART = ['508217', '508219', '508221', '508223', '508225', '508227', '508229', '366060']
# --depth 2: one more real link west (OSM way 98445139, 800046404 -> 9586428220, 160.6 m), so that
# even the engine's lagging tail pointer stays on the platform ("phantom reservation", see REPORT).
ARRIVE2 = ARRIVE + ['549958']
DEPART2 = ['549957'] + DEPART
NODES2 = NODES + ['9586428220']
STOP_LINK = ARRIVE[-1]            # 508218 (800046348 -> 800046404), head at 800046404
OLD_STOP_LINK = '366063'          # the truncated platform track's western end
PLATFORM = '8507000:0:4'
LINKS = sorted(set(ARRIVE + DEPART + ARRIVE2 + DEPART2))


def op(p):
    return gzip.open(p, 'rb') if p.endswith('.gz') else open(p, 'rb')


def read_osm_elements(want_links, want_nodes):
    links, nodes = {}, {}
    for ev, el in ET.iterparse(op(OSM_NET), events=('end',)):
        if el.tag == 'link' and el.get('id') in want_links:
            links[el.get('id')] = copy.deepcopy(el)
        elif el.tag == 'node' and el.get('id') in want_nodes:
            nodes[el.get('id')] = copy.deepcopy(el)
        if el.tag in ('link', 'node'):
            el.clear()
    return links, nodes


DOCTYPE = {
    'network': '<!DOCTYPE network SYSTEM "http://www.matsim.org/files/dtd/network_v2.dtd">',
    'transitSchedule': '<!DOCTYPE transitSchedule SYSTEM '
                       '"http://www.matsim.org/files/dtd/transitSchedule_v2.dtd">',
}


def write_xml(tree, path):
    """Matsim XML readers pick their delegate from the DOCTYPE — keep it."""
    import io
    buf = io.StringIO()
    buf.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    buf.write(DOCTYPE[tree.getroot().tag])
    buf.write('\n')
    tree.write(buf, encoding='unicode')
    data = buf.getvalue().encode('UTF-8')
    if path.endswith('.gz'):
        with gzip.open(path, 'wb') as f:
            f.write(data)
    else:
        with open(path, 'wb') as f:
            f.write(data)


def hms(s):
    a = s.split(':')
    return int(a[0]) * 3600 + int(a[1]) * 60 + float(a[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('schedule_in')
    ap.add_argument('network_in')
    ap.add_argument('schedule_out')
    ap.add_argument('network_out')
    ap.add_argument('--osm-net', default=OSM_NET)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--depth', type=int, default=2, choices=(1, 2))
    a = ap.parse_args()
    global ARRIVE, DEPART, NODES, STOP_LINK
    if a.depth == 2:
        ARRIVE, DEPART, NODES = ARRIVE2, DEPART2, NODES2
        STOP_LINK = ARRIVE[-1]
    tag = os.path.basename(os.path.dirname(os.path.abspath(a.schedule_in))) or 'case'

    # ---------- network ----------
    ntree = ET.parse(op(a.network_in))
    nroot = ntree.getroot()
    nodes_el = nroot.find('nodes')
    links_el = nroot.find('links')
    have_nodes = {n.get('id') for n in nodes_el}
    have_links = {l.get('id') for l in links_el}
    miss_links = [l for l in LINKS if l not in have_links]
    miss_nodes = [n for n in NODES if n not in have_nodes]
    print(f'{tag}: network has {len(have_links)} links; adding {len(miss_links)} links, '
          f'{len(miss_nodes)} nodes')
    if not miss_links and not miss_nodes:
        # nothing to do (already patched): copy the inputs unchanged
        shutil.copyfile(a.network_in, a.network_out)
        shutil.copyfile(a.schedule_in, a.schedule_out)
        print('nothing to add: inputs copied unchanged')
        return
    osm_links, osm_nodes = read_osm_elements(set(miss_links), set(miss_nodes))
    for n in miss_nodes:
        if n not in osm_nodes:
            raise SystemExit(f'node {n} not found in {a.osm_net}')
        nodes_el.append(osm_nodes[n])
    for l in miss_links:
        if l not in osm_links:
            raise SystemExit(f'link {l} not found in {a.osm_net}')
        links_el.append(osm_links[l])

    # ---------- schedule ----------
    stree = ET.parse(op(a.schedule_in))
    sroot = stree.getroot()
    stops_el = sroot.find('transitStops')
    facilities = {f.get('id'): f for f in stops_el}
    if PLATFORM not in {f.split('.link:')[0] for f in facilities}:
        raise SystemExit('platform 4 facilities not found')

    new_fac_id = f'{PLATFORM}.link:{STOP_LINK}'
    # the head stops at the end of the stop link: coordinate = that node
    end_node = osm_links[STOP_LINK].get('to')
    nx = ny = None
    for n in osm_nodes.values():
        if n.get('id') == end_node:
            nx, ny = float(n.get('x')), float(n.get('y'))
    if nx is None:
        raise SystemExit(f'new stop node {end_node} not found')

    moved_routes = 0
    moved_stops = 0
    checked_routes = 0
    for line in sroot.findall('transitLine'):
        for route in line.findall('transitRoute'):
            prof = route.find('routeProfile')
            links_parent = route.find('route')
            rl = [l.get('refId') for l in links_parent.findall('link')]
            stops = prof.findall('stop')
            # route indices of the stops
            pos, cur, ok = [], 0, True
            for s in stops:
                fid = s.get('refId')
                link = facilities[fid].get('linkRefId') if fid in facilities else None
                try:
                    i = rl.index(link, cur)
                except ValueError:
                    ok = False
                    break
                pos.append(i)
                cur = i
            if not ok:
                continue
            checked_routes += 1
            for si, s in enumerate(stops):
                fid = s.get('refId')
                if not fid.startswith(PLATFORM + '.'):
                    continue
                if facilities[fid].get('linkRefId') != OLD_STOP_LINK:
                    continue
                dwell = hms(s.get('departureOffset') or '0') - hms(s.get('arrivalOffset') or '0')
                k = pos[si]
                rev = (k + 1 < len(rl) and rl[k + 1] == '366062')
                if dwell <= 0 or not rev:
                    print(f'  skip {route.get("id")[:40]} stop {si}: dwell {dwell:.0f} rev {rev} '
                          f'(first/last stop, train does not stand)')
                    continue
                # insert the westward approach after the old stop link and the departure retrace
                insert = ARRIVE + DEPART
                for off, lid in enumerate(insert):
                    e = ET.SubElement(links_parent, 'link')
                    e.set('refId', lid)
                # move the newly added elements into position k+1..k+len(insert)
                all_links = links_parent.findall('link')
                added = all_links[-len(insert):]
                for e in added:
                    links_parent.remove(e)
                for off, e in enumerate(added):
                    links_parent.insert(k + 1 + off, e)
                s.set('refId', new_fac_id)
                pos = [i + len(insert) if i > k else i for i in pos]
                rl = [l.get('refId') for l in links_parent.findall('link')]
                moved_routes += 1
                moved_stops += 1
                break  # one platform-4 stand per route

    if moved_stops:
        if new_fac_id in facilities:
            facilities[new_fac_id].set('x', f'{nx:.6f}')
            facilities[new_fac_id].set('y', f'{ny:.6f}')
        else:
            e = ET.SubElement(stops_el, 'stopFacility')
            src = next(f for f in facilities.values() if f.get('id').startswith(PLATFORM + '.'))
            e.set('id', new_fac_id)
            e.set('x', f'{nx:.6f}')
            e.set('y', f'{ny:.6f}')
            e.set('linkRefId', STOP_LINK)
            e.set('name', src.get('name'))
            e.set('stopAreaId', src.get('stopAreaId'))
            e.set('isBlocking', src.get('isBlocking') or 'false')
    print(f'{tag}: {checked_routes} routes checked, {moved_stops} platform-4 standing stops moved '
          f'to {new_fac_id} (head node {end_node} at {nx:.0f},{ny:.0f})')

    if a.dry_run:
        print('dry run: nothing written')
        return
    write_xml(ntree, a.network_out)
    write_xml(stree, a.schedule_out)
    print('wrote', a.network_out, a.schedule_out)


if __name__ == '__main__':
    main()
