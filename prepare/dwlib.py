#!/usr/bin/env python3
"""Shared helpers for `prepare/restore_dwell.py` (restore the GTFS station dwell times).

Ported from round-3 agent DW1 (`swarm/r3/DW1/dwlib.py`), code unchanged; it is the reader/writer
library of the dwell step (pipeline step 6 of `docs/pipeline.md`).

  * streaming GTFS readers (the Swiss feed: BOM, every field quoted, times may
    exceed 24:00:00, stop_times.txt is ~1 GB -> read it ONCE, keep only the
    trip ids that are asked for);
  * a schedule reader that keeps *everything* of a transitSchedule.xml(.gz)
    (routes, stop sequences, departures, line ids/names);
  * a text-level rewriter that touches only the arrivalOffset/departureOffset
    attribute values of <stop> elements, so the file stays byte-identical
    everywhere else (DOCTYPE, indentation, attribute order, gzip).

Standard library only.
"""
import gzip
import hashlib
import io
import re
import sys
import zipfile


# ------------------------------------------------------------------ time
def hms(s):
    """'01:31:00' / '25:07:30' -> seconds (float), None stays None."""
    if s is None or s == "":
        return None
    p = s.split(":")
    if len(p) != 3:
        raise ValueError("bad time %r" % (s,))
    return int(p[0]) * 3600 + int(p[1]) * 60 + float(p[2])


def hms_str(sec):
    """seconds -> 'HH:MM:SS' (MATSim transit schedule format, hours may be >24)."""
    sec = int(round(sec))
    neg = sec < 0
    sec = abs(sec)
    out = "%02d:%02d:%02d" % (sec // 3600, (sec % 3600) // 60, sec % 60)
    return ("-" + out) if neg else out


def gzip_text(path):
    """open a possibly gzipped text file for reading (binary, utf-8 decode later)."""
    if path.endswith(".gz"):
        return gzip.open(path, "rb")
    return open(path, "rb")


# ------------------------------------------------------------------ schedule
STOP_RE = re.compile(
    r'<stop\s+refId="([^"]*)"\s+arrivalOffset="([^"]*)"\s+departureOffset="([^"]*)"'
    r'\s+awaitDeparture="([^"]*)"\s*/>')
ROUTE_RE = re.compile(r'<transitRoute id="([^"]*)">')
LINK_RE = re.compile(r'<link refId="([^"]*)"')
LINE_RE = re.compile(r'<transitLine id="([^"]*)"(?: name="([^"]*)")?\s*>')
DEP_RE = re.compile(r'<departure id="([^"]*)" departureTime="([^"]*)" vehicleRefId="([^"]*)"')


class Route(object):
    __slots__ = ("line", "line_name", "id", "stops", "departures", "order",
                 "link_hash", "n_links", "_h")

    def __init__(self):
        self.line = None
        self.line_name = None
        self.id = None
        self.stops = []        # [(refId, arr_sec, dep_sec, await)]
        self.departures = []   # [(id, time_sec), ...]
        self.order = 0
        self.link_hash = None  # sha1 of the <route> link sequence (None = no links)
        self.n_links = 0


def iter_schedule(path, with_lines=True):
    """Generator of Route (memory-light): one Route per <transitRoute> block.

    Line-based: the files are written by MATSim's XML writer, one element per
    line (verified for all four schedules of round 3).  Route -> transitLine is
    resolved through the enclosing <transitLine>, <link> sequences are reduced
    to a sha1 (the mapped Switzerland schedule has 2.9 M links).
    """
    cur = None
    line = (None, None)
    with gzip_text(path) as f:
        for raw in f:
            s = raw.decode("utf-8", "replace")
            if cur is None:
                if with_lines and "<transitLine " in s:
                    m = LINE_RE.search(s)
                    if m:
                        line = (m.group(1), m.group(2))
                elif s.lstrip().startswith("<transitRoute "):
                    m = ROUTE_RE.search(s)
                    cur = Route()
                    cur.id = m.group(1)
                    cur.line, cur.line_name = line
                    cur._h = hashlib.sha1()
                continue
            if "</transitRoute>" in s:
                if cur.n_links:
                    cur.link_hash = cur._h.hexdigest()
                del cur._h
                yield cur
                cur = None
                continue
            if "<stop " in s:
                m = STOP_RE.search(s)
                if not m:
                    raise SystemExit("unexpected <stop> line in %s: %r" % (path, s[:200]))
                cur.stops.append((m.group(1), hms(m.group(2)), hms(m.group(3)), m.group(4)))
            elif "<departure " in s:
                m = DEP_RE.search(s)
                if not m:
                    raise SystemExit("unexpected <departure> line in %s: %r" % (path, s[:200]))
                cur.departures.append((m.group(1), hms(m.group(2)), m.group(3)))
            elif "<link " in s:
                m = LINK_RE.search(s)
                if m:
                    cur.n_links += 1
                    cur._h.update(m.group(1).encode() + b"\n")


def read_schedule(path, with_lines=True):
    """-> (routes: list[Route] in file order, n_departures)."""
    routes = list(iter_schedule(path, with_lines))
    return routes, sum(len(r.departures) for r in routes)


# ------------------------------------------------------------------ gtfs
def _csv_rows(fp):
    """yield lists of fields from a quoted GTFS csv, skipping the BOM header."""
    first = True
    for line in fp:
        s = line.decode("utf-8-sig", "replace").rstrip("\r\n")
        if first:
            first = False
        if not s:
            continue
        yield s


def gtfs_trip_ids(dep_id):
    """'373.TA.91-1-D-j24-1.1258.H_08:32:00' -> ('373....H', '08:32:00')."""
    trip, _, t = dep_id.rpartition("_")
    return trip, t


def load_gtfs_stop_times(zip_path, trip_ids, verbose=False):
    """Stream stop_times.txt once -> {trip_id: [(stop_id, arr_sec, dep_sec, seq)]}.

    Only the requested trip ids are kept.
    """
    want = set(trip_ids)
    out = {}
    n = 0
    with zipfile.ZipFile(zip_path) as z:
        with z.open("stop_times.txt") as raw:
            fp = io.BufferedReader(raw, 1 << 20)
            for i, s in enumerate(_csv_rows(fp)):
                if i == 0:      # header
                    continue
                # naive split is safe here: no field of stop_times contains a comma
                f = s.split('","')
                if len(f) < 5:
                    continue
                tid = f[0][1:] if f[0].startswith('"') else f[0]
                if tid not in want:
                    continue
                n += 1
                out.setdefault(tid, []).append((
                    f[3], hms(f[1]), hms(f[2]), int(f[4])))
    for v in out.values():
        v.sort(key=lambda r: r[3])
    if verbose:
        print("stop_times: kept %d rows for %d/%d trips" % (n, len(out), len(want)), file=sys.stderr)
    return out


def load_gtfs_trips(zip_path, trip_ids, wanted_tokens=("route_id", "trip_id", "trip_headsign",
                                                       "trip_short_name", "block_id")):
    """Stream trips.txt once -> {trip_id: {col: value}} for the requested ids."""
    want = set(trip_ids)
    out = {}
    with zipfile.ZipFile(zip_path) as z:
        with z.open("trips.txt") as raw:
            fp = io.BufferedReader(raw, 1 << 20)
            hdr = None
            for s in _csv_rows(fp):
                f = [x.strip('"') for x in s.split('","')]
                if hdr is None:
                    hdr = [x.strip('\ufeff').strip('"') for x in f]
                    idx = {k: hdr.index(k) for k in wanted_tokens if k in hdr}
                    ti = hdr.index("trip_id")
                    continue
                if f[ti] in want:
                    out[f[ti]] = {k: f[j] for k, j in idx.items()}
    return out


def strip_link(ref_id):
    """'8507000:0:8.link:505936' -> '8507000:0:8' (mapped schedule stop refId)."""
    return ref_id.split(".link:")[0]
