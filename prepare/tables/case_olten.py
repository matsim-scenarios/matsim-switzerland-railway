"""Cut the Olten case schedule out of the whole-Switzerland unmapped schedule.

The Olten case is the set of long-distance trains that use the Olten rail area, each simulated over
its WHOLE route across Switzerland.  A route belongs to the case if

  1. one of its stop facilities lies inside `original_data/shp/olten/olten.shp`, or
  2. it is listed in `prepare/corrections/olten_case_routes.csv` - routes that run through the
     Olten area on track without a stop inside the polygon (in practice IC81/IC8 Bern-Zürich
     services on the Rothrist-Mattstetten line).  Round 1 selected them from their *mapped* path
     (`trimSchedule` in `GenerateRailsimInput`, point-in-polygon on the route's links); the mapped
     whole-Switzerland schedule needed to recompute that automatically is not part of the
     repository, so the 32 rows are carried as a reviewed correction with the reason spelled out.

Everything else (all routes of the selected lines, all their departures, all stop facilities they
use) is kept unchanged: no train is removed, retimed or shortened, and no stop is edited.

Output: `matsim_input/olten/unmapped_schedule_gtfs.xml.gz`, byte-identical (uncompressed) to the
round-1 reference `swarm/bench/ref/unmapped_schedule.xml.gz`.  It carries the times pt2matsim
produced, i.e. WITHOUT the station dwell times; `scripts/prepare_case.sh` restores those in step 6
and writes the scoring reference `matsim_input/<case>/unmapped_schedule.xml.gz`.
"""

from __future__ import annotations

import csv
import gzip
import re
import struct
import sys
from pathlib import Path

from prepare.paths import get as _paths


# ------------------------------------------------------------------ shapefile (no extra dependency)
def read_polygon_rings(shp: Path):
    """Rings of the (one) polygon in an ESRI shapefile, as lists of (x, y)."""
    data = shp.read_bytes()
    records, off = [], 100
    while off < len(data):
        content_words = struct.unpack(">i", data[off + 4:off + 8])[0]
        content = data[off + 8:off + 8 + content_words * 2]
        off += 8 + content_words * 2
        shape_type = struct.unpack("<i", content[0:4])[0]
        if shape_type == 0:
            continue
        if shape_type != 5:
            sys.exit(f"{shp}: expected polygon (5), got shape type {shape_type}")
        nparts, npts = struct.unpack("<ii", content[36:44])
        parts = struct.unpack("<%di" % nparts, content[44:44 + 4 * nparts])
        base = 44 + 4 * nparts
        pts = [struct.unpack("<dd", content[base + 16 * i:base + 16 * i + 16]) for i in range(npts)]
        for i in range(nparts):
            records.append(pts[parts[i]:(parts[i + 1] if i + 1 < nparts else npts)])
    return records


def inside_rings(rings, x, y) -> bool:
    """Even-odd rule over all rings of all polygons."""
    count = 0
    for ring in rings:
        hit, n = False, len(ring)
        for i in range(n):
            x1, y1 = ring[i]
            x2, y2 = ring[(i + 1) % n]
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                hit = not hit
        count += hit
    return count % 2 == 1


# ------------------------------------------------------------------ schedule
def read_correction_routes(path: Path):
    if not path.exists():
        return set()
    with open(path, newline="", encoding="utf-8") as fh:
        return {r["route_id"].strip() for r in csv.DictReader(fh) if r["route_id"].strip()}


def cut(full_schedule_gz: Path, rings, extra_routes, dest_gz: Path):
    text = gzip.open(full_schedule_gz, "rt", encoding="utf-8").read()

    facilities = {
        m.group(1): (float(m.group(2)), float(m.group(3)))
        for m in re.finditer(r'<stopFacility id="([^"]+)"[^>]*x="([^"]+)" y="([^"]+)"', text)
    }
    in_area = {sid for sid, (x, y) in facilities.items() if inside_rings(rings, x, y)}

    selected = set(extra_routes)
    n_by_stop = 0
    for m in re.finditer(r'<transitRoute id="([^"]+)">(.*?)</transitRoute>', text, re.S):
        if set(re.findall(r'<stop refId="([^"]+)"', m.group(2))) & in_area:
            selected.add(m.group(1))
            n_by_stop += 1

    text = re.sub(r'[ \t]*<transitRoute id="([^"]+)">.*?</transitRoute>\n?',
                  lambda m: m.group(0) if m.group(1) in selected else "", text, flags=re.S)
    text = re.sub(r'[ \t]*<transitLine id="[^"]+"[^>]*>\s*</transitLine>\n?', "", text)
    used = set(re.findall(r'<stop refId="([^"]+)"', text))
    text = re.sub(r'[ \t]*<stopFacility id="([^"]+)"[^>]*?(?:/>|>.*?</stopFacility>)\n?',
                  lambda m: m.group(0) if m.group(1) in used else "", text, flags=re.S)

    dest_gz.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(dest_gz, "wt", encoding="utf-8") as fh:
        fh.write(text)
    return (len(re.findall("<transitRoute ", text)), len(re.findall("<departure ", text)),
            len(re.findall("<stopFacility ", text)), n_by_stop)


def run(case: str = "olten") -> None:
    paths = _paths()
    if case != "olten":
        sys.exit(f"case_olten: unknown case {case!r}")
    rings = read_polygon_rings(paths.root / "original_data/shp/olten/olten.shp")
    extra = read_correction_routes(paths.root / "prepare/corrections/olten_case_routes.csv")
    dest = paths.root / "matsim_input/olten/unmapped_schedule_gtfs.xml.gz"
    routes, departures, facilities, n_by_stop = cut(paths.schedule, rings, extra, dest)
    print(f"case {case}: {routes} routes ({n_by_stop} by stop-in-area + {routes - n_by_stop} by "
          f"correction), {departures} departures, {facilities} stop facilities -> {dest}",
          file=sys.stderr)
