#!/usr/bin/env python3
"""Pipeline step: correct the length of the off-model (ext_*) hops that use the router's blanket
detour factor (Switzerland case; the Olten case has no such link).

Ported from round-3 agent CH2 (`swarm/r3/CH2/fix_ext_length.py`), code unchanged; pipeline step 11
of `docs/pipeline.md`, run on the Switzerland case only.

Why
  `TrackRouter.offLength()` (src/main/java/ch/sbb/prepare/router/TrackRouter.java:496-504) gives a
  foreign / metre-gauge / unmodelled hop the length  straight line x offDetour  whenever the real
  distance is not in the leg-distance table, with offDetour = 1.25 (TrackRouter.java:90).  402 of
  the 880 ext links of the Switzerland case are such placeholders (3 784 route-km).

  The simulated network itself shows what a Swiss railway hop really looks like: over the 7 074
  legs that run on modelled OSM track the length of the track between two stops is
  1.096 x the straight line between the two stops (median; p10 1.02, p90 1.33).  A blanket 1.25
  is 14 % above that and *only* matters where the speed is capped: CH1's off-model rule sets
  freespeed = length / (0.85 x published running time) capped at 160 km/h, so an inflated length
  pushes the required speed over the cap and the leg becomes slower than the published time in the
  model although the published time is drivable on the real line.  (The speed itself is
  re-solved after this script by CH1's `fix_offmodel_speed.py`.)

What this does (data only, no timetable, no speed change here)
  For every ext_* link whose length is exactly 1.25 x the straight line between its two nodes
  (> 15 m) the length becomes  straight line x <factor>  (default 1.096, the measured median of
  the modelled hops).  Links whose length comes from the leg-distance table are untouched, and
  every other attribute (freespeed, capacity, ...) is preserved; the file is rewritten by text
  surgery so that everything else stays byte-identical.

Usage:
  python3 prepare/fix_ext_length.py <network-in.xml.gz> <network-out.xml.gz> \
      [--factor 1.096] [--report fixreport.csv]

Standard library only.
"""
import argparse
import csv
import gzip
import math
import re
import xml.etree.ElementTree as ET


def op(p, mode='rb'):
    return gzip.open(p, mode) if p.endswith('.gz') else open(p, mode)


def read_nodes(path):
    nodes = {}
    for ev, el in ET.iterparse(op(path), events=('end',)):
        if el.tag == 'node':
            nodes[el.get('id')] = (float(el.get('x')), float(el.get('y')))
            el.clear()
    return nodes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('net_in')
    ap.add_argument('net_out')
    ap.add_argument('--factor', type=float, default=1.096)
    ap.add_argument('--old-factor', type=float, default=1.25)
    ap.add_argument('--report', default='fixreport.csv')
    a = ap.parse_args()

    nodes = read_nodes(a.net_in)
    with op(a.net_in) as f:
        data = f.read().decode('utf-8')

    rows = []

    def repl(m):
        el = m.group(0)
        lid = re.search(r'\bid="([^"]+)"', el)
        if not lid or not lid.group(1).startswith('ext_'):
            return el
        fnode = re.search(r'\bfrom="([^"]+)"', el)
        tnode = re.search(r'\bto="([^"]+)"', el)
        ln = re.search(r'\blength="([^"]+)"', el)
        if not (fnode and tnode and ln):
            return el
        p, q = nodes.get(fnode.group(1)), nodes.get(tnode.group(1))
        if p is None or q is None:
            return el
        chord = math.hypot(q[0] - p[0], q[1] - p[1])
        old = float(ln.group(1))
        if chord <= 15.0 or abs(old - a.old_factor * chord) > 1.0:
            return el                      # not a blanket-detour placeholder
        new = a.factor * chord
        rows.append(dict(link=lid.group(1), from_node=fnode.group(1), to_node=tnode.group(1),
                         chord_m=round(chord, 1), old_length_m=round(old, 1),
                         new_length_m=round(new, 1), ratio_old=round(old / chord, 3)))
        return re.sub(r'\blength="[^"]*"', 'length="%.1f"' % new, el, count=1)

    data = re.sub(r'<link\b[^>]*>', repl, data)
    with op(a.net_out, 'wb') as f:
        f.write(data.encode('utf-8'))
    with open(a.report, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else
                           ['link', 'from_node', 'to_node', 'chord_m', 'old_length_m',
                            'new_length_m', 'ratio_old'])
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: r['old_length_m'] - r['new_length_m']))
    print(f'{a.net_in}: {len(rows)} ext links changed '
          f'({sum(r["old_length_m"] for r in rows) / 1000:.0f} -> '
          f'{sum(r["new_length_m"] for r in rows) / 1000:.0f} km), factor '
          f'{a.old_factor} -> {a.factor}')
    print('wrote', a.net_out, 'report', a.report)


if __name__ == '__main__':
    main()
