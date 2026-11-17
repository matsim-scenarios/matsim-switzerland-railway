#!/usr/bin/env python3
"""Diagnose WHERE and WHY trains got stuck in a finished MATSim + railsim run.

Usage:  python3 scripts/deadlocks.py <experiment-dir> [--top 20]

Reads (from <experiment-dir>):
  output/*output_events.xml.gz                  MATSim + railsim events
  output/*output_network.xml.gz | transitNetwork.xml.gz
  transitSchedule.xml.gz | output/*output_transitSchedule.xml.gz
  score.json (optional, list ``not_completed``)

Writes:
  <experiment-dir>/deadlocks.json
  <experiment-dir>/deadlocks.txt   (same text as stdout)

Method (details and limitations: swarm/tools/DEADLOCKS_README.md):
  * a train occupies every link from its tail to its head and reserves a
    braking-distance look-ahead up to its next scheduled stop;
  * the authoritative occupancy source is ``railsimLinkStateChangeEvent``
    (state EXHAUSTED = reserved by <vehicle>, EMPTY = released), aggregated
    per physical track resource (``railsimResourceId``), so a link and its
    reverse link are one track;
  * railsim's ``SimpleDeadlockAvoidance`` additionally claims the *direction*
    of every capacity-1 resource ahead of a train until an opposite claim.
    That is invisible in reservation events but blocks trains on tracks that
    are EMPTY; it is replayed here (R03 section 2);
  * a train is "stalled" when its head position does not change again before
    the end of the run and it never reaches its last stop;
  * "wanted link" = next link of its scheduled route after the head link.

Standard library only.
"""

import argparse
import glob
import gzip
import json
import math
import os
import pickle
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

LOG_PREFIX = "[deadlocks]"
DEFAULT_TOP = 20
STATION_NEAR_M = 1000.0     # "station throat"
STATION_MAX_M = 3000.0      # "nearest station" search radius


def log(msg):
    print("%s %s" % (LOG_PREFIX, msg), file=sys.stderr, flush=True)


def hms(t):
    if t is None:
        return "-"
    t = int(round(float(t)))
    return "%02d:%02d:%02d" % (t // 3600, (t % 3600) // 60, t % 60)


def parse_hms(s):
    if not s:
        return 0.0
    parts = [float(p) for p in s.split(":")]
    while len(parts) < 3:
        parts.append(0.0)
    return parts[0] * 3600.0 + parts[1] * 60.0 + parts[2]


def open_text(path):
    if path.endswith(".gz"):
        return gzip.open(path, "rb")
    return open(path, "rb")


def first_existing(patterns):
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


# --------------------------------------------------------------------------
# schedule
# --------------------------------------------------------------------------
class Route(object):
    __slots__ = ("line_id", "line_name", "route_id", "links", "stops", "deps")

    def __init__(self, line_id, line_name, route_id):
        self.line_id = line_id
        self.line_name = line_name
        self.route_id = route_id
        self.links = []
        self.stops = []          # [(facility_id, arrival_offset, departure_offset)]
        self.deps = []           # [(departure_id, departure_time, vehicle_ref)]


def parse_schedule(path):
    """Return (facilities, routes, departures).

    facilities: id -> (name, x, y, linkRefId)
    routes:     (line_id, route_id) -> Route
    departures: dep_id -> dict(line_id, line_name, route_id, t0, vehicle_ref, route)
    """
    facilities = {}
    routes = {}
    departures = {}
    with open_text(path) as f:
        for ev, el in ET.iterparse(f, events=("end",)):
            tag = el.tag
            if tag == "stopFacility":
                facilities[el.get("id")] = (
                    el.get("name") or "",
                    float(el.get("x") or 0.0),
                    float(el.get("y") or 0.0),
                    el.get("linkRefId"),
                )
                el.clear()
            elif tag == "transitLine":
                line_id = el.get("id")
                line_name = el.get("name") or line_id
                for rel in el.findall("transitRoute"):
                    route = Route(line_id, line_name, rel.get("id"))
                    for s in rel.findall("routeProfile/stop"):
                        route.stops.append((
                            s.get("refId"),
                            parse_hms(s.get("arrivalOffset")),
                            parse_hms(s.get("departureOffset")),
                        ))
                    rl = rel.find("route")
                    if rl is not None:
                        for l in rl.findall("link"):
                            route.links.append(sys.intern(l.get("refId")))
                    for d in rel.findall("departures/departure"):
                        dep = dict(
                            line_id=line_id,
                            line_name=line_name,
                            route_id=route.route_id,
                            t0=parse_hms(d.get("departureTime")),
                            vehicle_ref=d.get("vehicleRefId"),
                            route=route,
                        )
                        departures[d.get("id")] = dep
                        route.deps.append((d.get("id"), dep["t0"], d.get("vehicleRefId")))
                    routes[(line_id, route.route_id)] = route
                el.clear()
            # NB: nested <attributes>/<route>/<stop> elements are deliberately
            # NOT cleared; their parent (transitLine/link) reads them first.
    return facilities, routes, departures


# --------------------------------------------------------------------------
# network
# --------------------------------------------------------------------------
def parse_network(path):
    """Return dicts: node xy, link from/to/length/way/resource, resource->links."""
    node_xy = {}
    l_from = {}
    l_to = {}
    l_len = {}
    l_way = {}
    l_res = {}
    has_resource_attr = False
    with open_text(path) as f:
        for ev, el in ET.iterparse(f, events=("end",)):
            tag = el.tag
            if tag == "node":
                nid = el.get("id")
                node_xy[nid] = (float(el.get("x") or 0.0), float(el.get("y") or 0.0))
                el.clear()
            elif tag == "link":
                lid = el.get("id")
                l_from[lid] = el.get("from")
                l_to[lid] = el.get("to")
                l_len[lid] = float(el.get("length") or 0.0)
                res = None
                way = None
                for a in el.findall("attributes/attribute"):
                    name = a.get("name")
                    if name == "railsimResourceId":
                        res = a.text
                        has_resource_attr = True
                    elif name == "osm:way:id":
                        way = a.text
                l_way[lid] = way
                l_res[lid] = res
                el.clear()
    if not has_resource_attr:
        # fallback: a physical track = the pair of opposite links between nodes
        for lid in l_from:
            a, b = l_from[lid], l_to[lid]
            l_res[lid] = "geo:%s|%s" % ((a, b) if a <= b else (b, a))
    res_links = defaultdict(list)
    for lid, r in l_res.items():
        if r is None:
            r = "link:" + lid
            l_res[lid] = r
        res_links[r].append(lid)
    reverse = {}
    for r, links in res_links.items():
        if len(links) == 2:
            a, b = links
            if l_from[a] == l_to[b] and l_to[a] == l_from[b]:
                reverse[a] = b
                reverse[b] = a
    return dict(node_xy=node_xy, l_from=l_from, l_to=l_to, l_len=l_len,
                l_way=l_way, l_res=l_res, res_links=dict(res_links),
                reverse=reverse, has_resource_attr=has_resource_attr)


# --------------------------------------------------------------------------
# events
# --------------------------------------------------------------------------
def scan_events(path, routes, link_res, dep_window):
    """Single streaming pass over the events file.

    Besides the plain event facts this reconstructs railsim's
    ``SimpleDeadlockAvoidance`` state (R03 section 2): every successful link
    reservation also claims the *direction* of every capacity-1 resource ahead
    on the train's route until an opposite-direction claim is met.  A train is
    denied a link either because the resource is reserved (capacity 1) or
    because the resource's direction is claimed by an opposite-direction train.

    Memory stays O(#trains + #currently reserved links + #currently claimed
    route links), independent of the event count.
    """
    driver2veh = {}
    veh_leg = {}              # vehicle -> (line_id, route_id)
    veh_dep = {}              # vehicle -> departure id
    dep2veh = {}              # departure id -> vehicle
    entered = {}              # vehicle -> time it entered the network
    enter_link = {}
    last = {}                 # vehicle -> tuple state
    stall = {}                # vehicle -> last time its head position changed
    last_pos = {}
    route_idx = {}            # vehicle -> index in route link list
    arrivals = defaultdict(list)   # vehicle -> [(time, facility)]
    arrived = set()           # vehicles that emitted PersonArrivalEvent ("arrival")
    aborted = {}              # vehicle -> (time, link)
    stuck = defaultdict(list)      # vehicle -> [(time, link or None)]
    owner = {}                # resource -> vehicle (current reservation)
    dep_block = {}            # departure id -> first blocked link at departure
    mismatches = Counter()
    end_time = 0.0
    nevents = 0

    # --- reconstructed SimpleDeadlockAvoidance.conflictPoints ---
    claim_dir = {}            # resource -> link id that owns the direction
    claim_trains = {}         # resource -> set(vehicle) that claimed it
    claim_first = {}          # resource -> first vehicle that claimed it
    claim_end = {}            # vehicle -> furthest route index it claims
    stand_snapshot = {}       # vehicle -> blockage at start of its last standstill
    stand_since = {}
    standing = {}

    def claim_snapshot(res):
        o = owner.get(res)
        if o is not None:
            return dict(reason="reserved", link=None, primary=o, blockers=[o],
                        direction=claim_dir.get(res))
        d = claim_dir.get(res)
        s = claim_trains.get(res)
        prim = claim_first.get(res)
        if s and prim not in s:
            prim = min(s)
        return dict(reason="direction claim" if d else "free", link=d,
                    primary=prim,
                    blockers=sorted(s) if d else [],
                    direction=d)

    def claim_walk(veh, link, t):
        """Replay DeadlockAvoidance.onReserve for one newly reserved link."""
        leg = veh_leg.get(veh)
        route = routes.get(leg)
        if route is None:
            return
        links = route.links
        i = route_idx.get(veh, -1)
        try:
            idx = links.index(link, i - 2 if i > 1 else 0)
        except ValueError:
            try:
                idx = links.index(link)
            except ValueError:
                mismatches["reserved_link_not_in_route"] += 1
                return
        j = idx if idx > claim_end.get(veh, -1) else claim_end.get(veh, -1) + 1
        n = len(links)
        while j < n:
            lj = links[j]
            r = link_res.get(lj)
            if r is None:
                break
            d = claim_dir.get(r)
            if d is not None and d != lj:
                break
            claim_dir[r] = lj
            s = claim_trains.get(r)
            if s is None:
                s = set()
                claim_trains[r] = s
                claim_first[r] = veh
            s.add(veh)
            claim_end[veh] = j
            j += 1

    def claim_release(veh, link):
        r = link_res.get(link)
        if r is None:
            return
        s = claim_trains.get(r)
        if s is None:
            return
        s.discard(veh)
        if not s:
            claim_trains.pop(r, None)
            claim_dir.pop(r, None)
            claim_first.pop(r, None)

    def wanted_link(veh):
        st = last.get(veh)
        if st is None:
            return None
        leg = veh_leg.get(veh)
        route = routes.get(leg)
        if route is None:
            return None
        links = route.links
        idx = route_idx.get(veh, -1)
        head = st[1]
        if idx < 0 or idx >= len(links) or links[idx] != head:
            try:
                idx = links.index(head)
            except ValueError:
                return None
        if idx + 1 >= len(links):
            return None
        return links[idx + 1]

    def blocker_now(veh):
        w = wanted_link(veh)
        if w is None:
            return None
        r = link_res.get(w)
        if r is None:
            return None
        snap = claim_snapshot(r)
        if snap["reason"] == "reserved" and snap["primary"] == veh:
            snap["reason"] = "self"
        snap["wanted_link"] = w
        snap["resource"] = r
        return snap

    def check_departure_window(veh, dep):
        """Why could this train not reserve its whole departure window?

        ``RailsimEngine.updateDeparture`` only enters traffic when
        ``blockLinkTracks`` reserved everything up to ``reserveDist+10`` - the
        whole window up to the next scheduled stop, not just the first link.
        Snapshot the first blocked link of that window before this train's own
        reservations happen.
        """
        win = dep_window.get(dep)
        if not win:
            return
        for i, l in win:
            r = link_res.get(l)
            if r is None:
                continue
            o = owner.get(r)
            if o is not None and o != veh:
                dep_block[dep] = (i, l, "reserved", o)
                return
            d = claim_dir.get(r)
            if d is not None and d != l:
                s = claim_trains.get(r)
                prim = claim_first.get(r)
                if s and prim not in s:
                    prim = min(s)
                dep_block[dep] = (i, l, "direction claim", prim)
                return
        dep_block[dep] = (0, win[0][1], "free", None)

    def advance(veh, link):
        leg = veh_leg.get(veh)
        if leg is None:
            return
        route = routes.get(leg)
        if route is None:
            return
        links = route.links
        i = route_idx.get(veh, -1)
        n = i + 1
        if n < len(links) and links[n] == link:
            route_idx[veh] = n
            return
        stop = min(len(links), n + 64)
        for j in range(n, stop):
            if links[j] == link:
                route_idx[veh] = j
                return
        try:
            route_idx[veh] = links.index(link)
        except ValueError:
            mismatches["entered_link_not_in_route"] += 1

    with open_text(path) as f:
        it = ET.iterparse(f, events=("start", "end"))
        root = None
        for ev, el in it:
            if ev == "start":
                if root is None:
                    root = el
                continue
            if el.tag != "event":
                el.clear()
                continue
            a = el.attrib
            ty = a.get("type")
            t = float(a.get("time") or 0.0)
            if t > end_time:
                end_time = t
            veh = a.get("vehicle")
            if ty == "TransitDriverStarts":
                v = a.get("vehicleId")
                driver2veh[a.get("driverId")] = v
                veh_leg[v] = (a.get("transitLineId"), a.get("transitRouteId"))
                veh_dep[v] = a.get("departureId")
                dep2veh[a.get("departureId")] = v
            elif ty == "departure":
                v = driver2veh.get(a.get("person"))
                dep = veh_dep.get(v)
                if dep is not None and dep not in dep_block:
                    check_departure_window(v, dep)
            elif ty == "vehicle enters traffic":
                entered[veh] = t
                enter_link[veh] = a.get("link")
            elif ty == "railsimTrainStateEvent":
                hpos = float(a.get("headPosition") or 0.0)
                key = (a.get("headLink"), hpos)
                changed = last_pos.get(veh) != key
                speed = float(a.get("speed") or 0.0)
                tspeed = float(a.get("targetSpeed") or 0.0)
                if changed:
                    stall[veh] = t
                    last_pos[veh] = key
                    standing[veh] = False
                last[veh] = (t, key[0], hpos,
                             a.get("tailLink"), float(a.get("tailPosition") or 0.0),
                             speed, tspeed)
                if not standing.get(veh) and speed == 0.0 and tspeed == 0.0:
                    standing[veh] = True
                    stand_since[veh] = t
                    stand_snapshot[veh] = blocker_now(veh)
            elif ty == "railsimLinkStateChangeEvent":
                link = a.get("link")
                res = link_res.get(link)
                if res is None:
                    mismatches["link_state_change_unknown_link"] += 1
                else:
                    st = a.get("state")
                    if st == "EXHAUSTED":
                        prev = owner.get(res)
                        if prev is not None and prev != veh:
                            mismatches["resource_two_owners"] += 1
                        owner[res] = veh
                        claim_walk(veh, link, t)
                    elif st == "EMPTY":
                        claim_release(veh, link)
                        cur = owner.get(res)
                        if cur == veh:
                            del owner[res]
                        elif cur is not None:
                            mismatches["empty_but_other_owner"] += 1
            elif ty == "entered link":
                advance(veh, a.get("link"))
            elif ty == "VehicleArrivesAtFacility":
                arrivals[veh].append((t, a.get("facility")))
            elif ty == "arrival":
                v = driver2veh.get(a.get("person"))
                if v is not None:
                    arrived.add(v)
            elif ty == "vehicle aborts":
                aborted[veh] = (t, a.get("link"))
            elif ty == "stuckAndAbort":
                v = driver2veh.get(a.get("person"))
                stuck[v].append((t, a.get("link")))
            nevents += 1
            el.clear()
            if (nevents & 0x7FFFF) == 0 and root is not None:
                root.clear()
                log("  ... %d events" % nevents)
    log("  %d events, %d trains with state events, %d links reserved at end, "
        "%d resources direction-claimed, %d departures checked"
        % (nevents, len(last), len(owner), len(claim_dir), len(dep_block)))
    return dict(driver2veh=driver2veh, veh_leg=veh_leg, veh_dep=veh_dep,
                dep2veh=dep2veh, entered=entered, enter_link=enter_link,
                last=last, stall=stall, route_idx=route_idx,
                arrivals=arrivals, arrived=arrived, aborted=aborted,
                stuck=dict(stuck), end_owner=dict(owner), end_time=end_time,
                dep_block=dep_block,
                mismatches=dict(mismatches), nevents=nevents,
                claim_dir=dict(claim_dir),
                claim_trains={r: sorted(s) for r, s in claim_trains.items()},
                claim_first=dict(claim_first),
                stand_snapshot=stand_snapshot, stand_since=stand_since)


# --------------------------------------------------------------------------
# geometry helpers
# --------------------------------------------------------------------------
def make_station_index(facilities):
    seen = set()
    out = []
    for fid, (name, x, y, link) in facilities.items():
        key = (name, round(x, 0), round(y, 0))
        if key in seen:
            continue
        seen.add(key)
        out.append((name, x, y))
    return out


def nearest_station(stations, x, y, max_m=STATION_MAX_M):
    best = None
    bestd2 = max_m * max_m
    for name, sx, sy in stations:
        d2 = (sx - x) ** 2 + (sy - y) ** 2
        if d2 < bestd2:
            bestd2 = d2
            best = (name, math.sqrt(d2))
    return best


def link_point(net, lid, pos=None):
    """(x, y) on link lid, pos metres from its from-node (default: midpoint)."""
    if not lid:
        return None
    fr = net["l_from"].get(lid)
    to = net["l_to"].get(lid)
    a = net["node_xy"].get(fr)
    b = net["node_xy"].get(to)
    if a is None or b is None:
        return None
    if pos is None:
        pos = net["l_len"].get(lid, 0.0) / 2.0
    L = net["l_len"].get(lid, 0.0)
    if L <= 0:
        return a
    f = max(0.0, min(1.0, pos / L))
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)


def link_vec(net, lid):
    if not lid:
        return None
    a = net["node_xy"].get(net["l_from"].get(lid))
    b = net["node_xy"].get(net["l_to"].get(lid))
    if a is None or b is None:
        return None
    return (b[0] - a[0], b[1] - a[1])


# --------------------------------------------------------------------------
# analysis
# --------------------------------------------------------------------------
def analyse(exp_dir, top, cache=None):
    t_start = time.time()
    events_path = first_existing([
        os.path.join(exp_dir, "output", "*output_events.xml.gz"),
        os.path.join(exp_dir, "output", "ITERS", "it.*", "*.events.xml.gz"),
        os.path.join(exp_dir, "output", "*.events.xml.gz"),
    ])
    network_path = first_existing([
        os.path.join(exp_dir, "output", "*output_network.xml.gz"),
        os.path.join(exp_dir, "transitNetwork.xml.gz"),
        os.path.join(exp_dir, "input", "transitNetwork.xml.gz"),
    ])
    schedule_path = first_existing([
        os.path.join(exp_dir, "transitSchedule.xml.gz"),
        os.path.join(exp_dir, "output", "*output_transitSchedule.xml.gz"),
        os.path.join(exp_dir, "input", "transitSchedule.xml.gz"),
    ])
    score_path = os.path.join(exp_dir, "score.json")
    if not events_path or not network_path or not schedule_path:
        raise SystemExit("missing input file(s): events=%s network=%s schedule=%s"
                         % (events_path, network_path, schedule_path))
    log("events   %s" % events_path)
    log("network  %s" % network_path)
    log("schedule %s" % schedule_path)

    if cache and os.path.exists(cache):
        log("loading cached parse/scan from %s" % cache)
        with open(cache, "rb") as f:
            facilities, routes, departures, net, ev = pickle.load(f)
    else:
        log("parsing schedule ...")
        facilities, routes, departures = parse_schedule(schedule_path)
        log("  %d stop facilities, %d routes, %d departures"
            % (len(facilities), len(routes), len(departures)))

        log("parsing network ...")
        net = parse_network(network_path)
        log("  %d nodes, %d links, %d resources (%d reverse pairs)"
            % (len(net["node_xy"]), len(net["l_from"]),
               len(net["res_links"]), len(net["reverse"])))

        # ---- departure windows: everything up to the next scheduled stop ----
        dep_window = {}
        for dep_id, d in departures.items():
            links = d["route"].links
            end = len(links)
            stops = d["route"].stops
            if len(stops) >= 2:
                nextlink = facilities.get(stops[1][0], (None, None, None, None))[3]
                if nextlink:
                    try:
                        end = links.index(nextlink, 1) + 1
                    except ValueError:
                        end = min(len(links), 200)
            end = min(end, 400)
            dep_window[dep_id] = [(i, links[i]) for i in range(min(end, len(links)))]

        log("scanning events (this is the slow part) ...")
        ev = scan_events(events_path, routes, net["l_res"], dep_window)
        log("  done in %.1fs" % (time.time() - t_start))
        if cache:
            log("writing cache %s" % cache)
            with open(cache, "wb") as f:
                pickle.dump((facilities, routes, departures, net, ev), f,
                            protocol=pickle.HIGHEST_PROTOCOL)

    # ---- score.json: authoritative unfinished list ----
    unfinished = {}
    score_meta = {}
    score_loaded = os.path.exists(score_path)
    if score_loaded:
        with open(score_path) as f:
            score = json.load(f)
        for e in score.get("not_completed", []):
            unfinished[e["departure"]] = e
        score_meta = dict(departures=score.get("departures"),
                          started=score.get("started"),
                          completed=score.get("completed"),
                          completion_pct=score.get("completion_pct"))
        log("score.json: %d not_completed of %s departures"
            % (len(unfinished), score.get("departures")))

    stations = make_station_index(facilities)
    end_time = ev["end_time"]

    # ---- per-departure records ----
    trains = []
    dep2veh = ev["dep2veh"]
    for dep_id, d in departures.items():
        trains.append(dict(
            departure=dep_id, line_id=d["line_id"], line=d["line_name"],
            route_id=d["route_id"], scheduled_start=d["t0"],
            vehicle=dep2veh.get(dep_id), route=d["route"],
            score=unfinished.get(dep_id),
        ))
    log("  %d departures, %d with TransitDriverStarts" % (len(trains), len(dep2veh)))

    stalled = []
    insertion = []
    after_end = []
    completed = []
    for rec in trains:
        veh = rec["vehicle"]
        route = rec["route"]
        rec["n_stops"] = len(route.stops)
        rec["from_stop"] = route.stops[0][0] if route.stops else None
        rec["to_stop"] = route.stops[-1][0] if route.stops else None
        rec["from_name"] = facilities.get(rec["from_stop"], ("?",))[0]
        rec["to_name"] = facilities.get(rec["to_stop"], ("?",))[0]
        rec["direction"] = "%s -> %s" % (rec["from_name"], rec["to_name"])
        se = rec["score"]
        arr = ev["arrivals"].get(veh, []) if veh else []
        rec["n_arrivals"] = len(arr)
        # greedy match of arrival events to the scheduled stop sequence
        matched = 0
        for (ta, fac) in arr:
            if matched < len(route.stops) and fac == route.stops[matched][0]:
                matched += 1
        rec["stops_done"] = se.get("stops_done", matched) if se else matched
        if rec["stops_done"] and arr:
            k = min(rec["stops_done"], len(arr)) - 1
            rec["last_stop_reached"] = arr[k][1]
            rec["last_stop_time"] = arr[k][0]
        else:
            rec["last_stop_reached"] = None
            rec["last_stop_time"] = None
        if score_loaded:
            is_unfinished = se is not None
        else:
            is_unfinished = veh is None or veh not in ev["arrived"]
        rec["unfinished"] = is_unfinished
        if veh is None:
            if rec["scheduled_start"] >= end_time:
                rec["status"] = "after_sim_end"
                after_end.append(rec)
            else:
                rec["status"] = "never_started"
                insertion.append(rec)
            continue
        if veh not in ev["entered"]:
            rec["status"] = "insertion_blocked"
            insertion.append(rec)
        elif not is_unfinished:
            rec["status"] = "completed"
            completed.append(rec)
        else:
            rec["status"] = "stalled"
            last = ev["last"].get(veh)
            if last is None:
                rec["status"] = "stalled_no_state"
            rec["stall_time"] = ev["stall"].get(veh)
            if last is not None:
                rec["last_state"] = last
                rec["head_link"] = last[1]
                rec["head_pos"] = last[2]
                rec["tail_link"] = last[3]
                rec["speed"] = last[5]
            stalled.append(rec)

    log("  completed=%d stalled=%d insertion_blocked=%d after_sim_end=%d"
        % (len(completed), len(stalled), len(insertion), len(after_end)))

    # ---- wanted link + blockers ----
    end_owner = ev["end_owner"]
    claim_dir = ev["claim_dir"]
    claim_trains = ev["claim_trains"]
    claim_first = ev["claim_first"]

    def snapshot(res):
        if res is None:
            return dict(reason="unknown", primary=None, blockers=[], direction=None)
        o = end_owner.get(res)
        if o is not None:
            return dict(reason="reserved", primary=o, blockers=[o],
                        direction=claim_dir.get(res))
        d = claim_dir.get(res)
        if d is not None:
            return dict(reason="direction claim", primary=claim_first.get(res),
                        blockers=list(claim_trains.get(res, [])), direction=d)
        return dict(reason="free", primary=None, blockers=[], direction=None)

    for rec in stalled:
        veh = rec["vehicle"]
        route = rec["route"]
        links = route.links
        head = rec.get("head_link")
        idx = ev["route_idx"].get(veh, -1)
        if head is not None:
            if idx < 0 or idx >= len(links) or links[idx] != head:
                try:
                    idx = links.index(head)
                except ValueError:
                    idx = None
        else:
            idx = None
        rec["route_index"] = idx
        wanted = links[idx + 1] if idx is not None and idx + 1 < len(links) else None
        rec["wanted_link"] = wanted
        rec["wanted_osm_way"] = net["l_way"].get(wanted) if wanted else None
        rec["wanted_coords"] = link_point(net, wanted) if wanted else None
        res = net["l_res"].get(wanted) if wanted else None
        rec["wanted_resource"] = res
        if wanted:
            rec["contested_reverse"] = net["reverse"].get(wanted)
        if rec.get("head_link"):
            rec["head_coords"] = link_point(net, rec["head_link"], rec.get("head_pos"))
        rec["self_uturn"] = bool(wanted and rec.get("head_link")
                                 and net["reverse"].get(rec["head_link"]) == wanted)

        snap_end = snapshot(res)
        if rec["self_uturn"]:
            snap_end = dict(reason="self U-turn", primary=None, blockers=[],
                            direction=snap_end.get("direction"))
        if snap_end["primary"] == veh:
            snap_end["primary"] = None
            snap_end["blockers"] = [b for b in snap_end["blockers"] if b != veh]
        rec["blocker"] = snap_end["primary"]
        rec["blocker_reason_end"] = snap_end["reason"]
        rec["blockers_end"] = snap_end["blockers"]
        rec["direction_owner_end"] = snap_end["direction"]

        snap_stall = ev["stand_snapshot"].get(veh)
        if snap_stall is not None:
            prim = snap_stall.get("primary")
            if prim == veh:
                prim = None
            rec["blocker_at_stall"] = prim
            rec["blocker_reason_stall"] = snap_stall.get("reason")
            rec["blockers_stall"] = [b for b in snap_stall.get("blockers", []) if b != veh]
            rec["stall_wanted_link"] = snap_stall.get("wanted_link")
            rec["stall_wanted_resource"] = snap_stall.get("resource")
            if (rec.get("head_link") and rec.get("stall_wanted_link")
                    and net["reverse"].get(rec["head_link"]) == rec["stall_wanted_link"]):
                rec["blocker_at_stall"] = None
                rec["blocker_reason_stall"] = "self U-turn"
                rec["blockers_stall"] = []
        else:
            rec["blocker_at_stall"] = None
            rec["blocker_reason_stall"] = None
            rec["blockers_stall"] = []
        rec["still_moving_at_end"] = bool(rec.get("stall_time") is not None
                                          and rec["stall_time"] >= end_time - 0.5)

    # ---- insertion-blocked trains: first blocked link of the departure window ----
    for rec in insertion:
        if rec["status"] != "insertion_blocked" or rec["vehicle"] is None:
            continue
        route = rec["route"]
        rec["wanted_link"] = route.links[0] if route.links else None
        db = ev.get("dep_block", {}).get(rec["departure"])
        if db is not None:
            j, link, reason, primary = db
            rec["blocked_at_departure_by"] = primary
            rec["blocked_at_departure_reason"] = reason
            rec["blocked_at_departure_link"] = link
            rec["blocked_at_departure_index"] = j
        else:
            rec["blocked_at_departure_by"] = None
            rec["blocked_at_departure_reason"] = "unknown"
            rec["blocked_at_departure_link"] = rec["wanted_link"]
            rec["blocked_at_departure_index"] = 0
        res = net["l_res"].get(rec["wanted_link"]) if rec["wanted_link"] else None
        rec["wanted_resource"] = res
        rec["wanted_osm_way"] = net["l_way"].get(rec["wanted_link"]) if rec["wanted_link"] else None
        rec["wanted_coords"] = link_point(net, rec["wanted_link"]) if rec["wanted_link"] else None
        snap_end = snapshot(res)
        rec["blocker"] = snap_end["primary"]
        rec["blocker_reason_end"] = snap_end["reason"]

    by_veh = {}
    for rec in trains:
        if rec["vehicle"]:
            by_veh[rec["vehicle"]] = rec

    def pick_claimant(blockers):
        """One representative of a direction-claim set.

        A railsim direction claim is held by a *set* of trains; it is released
        only when the last of them has passed.  For the single wait-for edge we
        pick the claimant that stalled earliest (a deterministic, causal
        choice); the full set is kept in blockers_*.
        """
        if not blockers:
            return None
        cand = [b for b in blockers
                if b in by_veh and by_veh[b].get("stall_time") is not None]
        if cand:
            return min(cand, key=lambda b: (by_veh[b]["stall_time"], b))
        return blockers[0]

    for rec in stalled:
        if rec.get("blocker_reason_stall") == "direction claim":
            rec["blocker_at_stall"] = pick_claimant(rec.get("blockers_stall"))
        if rec.get("blocker_reason_end") == "direction claim":
            rec["blocker"] = pick_claimant(rec.get("blockers_end"))

    # ---- wait-for graph at end of run (who holds/reserves the wanted track) ----
    node_set = {rec["vehicle"] for rec in stalled}
    next_of = {}
    for rec in stalled:
        b = rec.get("blocker")
        next_of[rec["vehicle"]] = b if (b is not None and b in node_set
                                        and b != rec["vehicle"]) else None

    def find_cycles(edges):
        done = set()
        cycles = []
        cyc_of = {}
        for start in node_set:
            if start in done:
                continue
            path = []
            pos = {}
            v = start
            while v is not None and v in node_set and v not in done and v not in pos:
                pos[v] = len(path)
                path.append(v)
                v = edges.get(v)
            if v is not None and v in pos:
                c = path[pos[v]:]
                cid = len(cycles)
                cycles.append(c)
                for u in c:
                    cyc_of[u] = cid
            done.update(path)
        return cycles, cyc_of

    cycles, _ = find_cycles(next_of)
    cycles.sort(key=lambda c: min(ev["stall"].get(v, 1e18) for v in c))
    cycle_nodes = {v: i for i, c in enumerate(cycles) for v in c}

    chain_end = {}
    for rec in stalled:
        v = rec["vehicle"]
        if v in cycle_nodes:
            continue
        seen = set()
        cur = v
        end = None
        while cur is not None and cur not in seen:
            seen.add(cur)
            if cur in cycle_nodes:
                end = ("cycle", cycle_nodes[cur])
                break
            nxt = next_of.get(cur)
            if nxt is None:
                r = by_veh[cur]
                reason = r.get("blocker_reason_end")
                if reason == "self U-turn":
                    end = ("open", "self")
                elif r.get("blocker") is None:
                    end = ("open", "free")
                else:
                    end = ("open", "moving")
                break
            cur = nxt
        chain_end[v] = end or ("loop", cur)
    victim_count = Counter(e[1] for e in chain_end.values() if e[0] == "cycle")

    # ---- stall-time graph: who blocked each train when it stopped for good ----
    # railsim's directional claims reach far ahead, so the train that finally
    # stops you may not have stopped itself yet; this is the causal graph.
    stall_next = {}
    for rec in stalled:
        b = rec.get("blocker_at_stall")
        stall_next[rec["vehicle"]] = b if (b is not None and b in node_set
                                           and b != rec["vehicle"]) else None
    st_cycles, st_cycle_of = find_cycles(stall_next)

    parent = {v: v for v in node_set}

    def uf_find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def uf_union(a, b):
        ra, rb = uf_find(a), uf_find(b)
        if ra != rb:
            parent[rb] = ra

    for v, b in stall_next.items():
        if b:
            uf_union(v, b)
    comps = defaultdict(list)
    for v in node_set:
        comps[uf_find(v)].append(v)

    def geometry(members):
        wanted = [m.get("wanted_link") for m in members]
        contested = wanted[0] if wanted else None
        xy = link_point(net, contested) if contested else None
        st = nearest_station(stations, xy[0], xy[1]) if xy else None
        near = bool(st and st[1] <= STATION_NEAR_M)
        opposite = False
        same = False
        for i in range(len(members)):
            for j in range(i + 1, len(members)):
                a, b = wanted[i], wanted[j]
                if not a or not b:
                    continue
                if a == b:
                    same = True
                elif net["l_res"].get(a) == net["l_res"].get(b):
                    opposite = True
                else:
                    va, vb = link_vec(net, a), link_vec(net, b)
                    if va and vb:
                        dot = va[0] * vb[0] + va[1] * vb[1]
                        na, nb = math.hypot(*va), math.hypot(*vb)
                        if na > 0 and nb > 0 and dot < -0.5 * na * nb:
                            opposite = True
                        else:
                            same = True
        if near:
            cls = "station throat"
        elif opposite:
            cls = "head-on on plain line"
        elif same:
            cls = "follows"
        else:
            cls = "other"
        return cls, contested, xy, st

    def member_detail(rec):
        hc = rec.get("head_coords")
        nst = nearest_station(stations, hc[0], hc[1]) if hc else None
        return dict(
            vehicle=rec["vehicle"], line=rec["line"], direction=rec["direction"],
            stall_time=rec.get("stall_time"),
            head_link=rec.get("head_link"),
            head_pos=round(rec.get("head_pos") or 0.0, 1),
            head_coords=hc,
            osm_way=net["l_way"].get(rec.get("head_link")),
            wanted_link=rec.get("wanted_link"),
            wanted_osm_way=rec.get("wanted_osm_way"),
            wanted_coords=rec.get("wanted_coords"),
            near_station=nst[0] if nst else None,
            station_distance_m=round(nst[1], 1) if nst else None,
            blocker=rec.get("blocker_at_stall"),
            blocker_reason=rec.get("blocker_reason_stall"),
        )

    cascades = []
    for root, members in comps.items():
        members.sort(key=lambda v: ev["stall"].get(v, 1e18))
        first_rec = by_veh[members[0]]
        cyc_members = [v for v in members if v in st_cycle_of]
        if cyc_members:
            cid = st_cycle_of[cyc_members[0]]
            root_nodes = list(st_cycles[cid])
            kind = "cycle"
            cls, contested, xy, st = geometry([by_veh[v] for v in root_nodes])
        else:
            seen = set()
            cur = members[0]
            while cur is not None and cur not in seen and stall_next.get(cur) is not None:
                seen.add(cur)
                cur = stall_next.get(cur)
            root_nodes = [cur] if cur else []
            r = by_veh.get(cur) if cur else None
            reason = (r or {}).get("blocker_reason_stall")
            if reason in ("self", "self U-turn"):
                kind = "self U-turn"
                cls = "terminus reversal (self-block)"
            elif reason in ("reserved", "direction claim"):
                kind = "blocked by moving train"
                cls = "blocked by moving train"
            else:
                kind = "free track"
                cls = "blocked by free track"
            contested = (r or {}).get("wanted_link")
            xy = link_point(net, contested) if contested else None
            st = nearest_station(stations, xy[0], xy[1]) if xy else None
        cascades.append(dict(
            id=0, time=ev["stall"].get(members[0], 0.0),
            kind=kind, classification=cls,
            first_train=first_rec["vehicle"], first_train_line=first_rec["line"],
            first_train_departure=first_rec["departure"].rsplit("_", 1)[-1][:5],
            n_trains=len(members), victims=max(0, len(members) - len(root_nodes)),
            root_trains=root_nodes,
            members=[member_detail(by_veh[v]) for v in members],
            root_members=[member_detail(by_veh[v]) for v in root_nodes],
            contested_link=contested, contested_coords=xy,
            contested_osm_way=net["l_way"].get(contested) if contested else None,
            near_station=st[0] if st else None,
            station_distance_m=round(st[1], 1) if st else None,
        ))
    # a train that was still moving at 24:00 is not a deadlock
    for c in cascades:
        if any(by_veh[v].get("still_moving_at_end") for v in c["root_trains"]):
            c["kind"] = "still running at end"
            c["classification"] = "still running at end (sim ended)"
    cascades.sort(key=lambda c: c["time"] or 1e18)
    for i, c in enumerate(cascades):
        c["id"] = i + 1
        c["is_root"] = c["kind"] != "blocked by moving train" and \
            c["kind"] != "still running at end"
    roots = [c for c in cascades if c["is_root"]]
    for i, c in enumerate(roots):
        c["root_id"] = i + 1

    # insertion-blocked incidents: group by (first blocked link, blocker then)
    ins_groups = defaultdict(list)
    for rec in insertion:
        if rec["status"] != "insertion_blocked":
            continue
        ins_groups[(rec.get("blocked_at_departure_link"),
                    rec.get("blocked_at_departure_by"))].append(rec)
    ins_incidents = []
    for (link, blocker), recs in ins_groups.items():
        recs.sort(key=lambda r: r["scheduled_start"])
        first = recs[0]
        xy = link_point(net, link) if link else None
        st = nearest_station(stations, xy[0], xy[1]) if xy else None
        ins_incidents.append(dict(
            kind="insertion", time=first["scheduled_start"],
            classification="insertion blocked",
            trains=[r["vehicle"] or r["departure"] for r in recs],
            lines=sorted({r["line"] for r in recs}),
            count=len(recs), contested_link=link,
            contested_coords=xy,
            contested_osm_way=net["l_way"].get(link) if link else None,
            near_station=st[0] if st else None,
            station_distance_m=round(st[1], 1) if st else None,
            blocker_at_departure=blocker,
            blocker_reason=first.get("blocked_at_departure_reason"),
            blocker_is_stalled=(blocker in node_set) if blocker else None,
            members=[],
        ))
    ins_incidents.sort(key=lambda r: r["time"])

    # ---- area statistics ----
    area_roots = Counter()
    area_stalled = Counter()
    for r in roots:
        area_roots[r["near_station"] or "(no station within 3 km)"] += 1
    for rec in stalled:
        xy = link_point(net, rec["head_link"]) if rec.get("head_link") else None
        st = nearest_station(stations, xy[0], xy[1]) if xy else None
        rec["nearest_station"] = st[0] if st else None
        rec["station_distance_m"] = round(st[1], 1) if st else None
        area_stalled[rec["nearest_station"] or "(none)"] += 1

    # ---- hour histogram ----
    hours = Counter()
    for rec in stalled:
        t = rec.get("stall_time")
        if t is not None:
            hours[int(t // 3600)] += 1
    for rec in insertion:
        if rec["status"] == "insertion_blocked":
            hours[int(rec["scheduled_start"] // 3600)] += 1

    # ---- first 10 things that went wrong ----
    events10 = []
    for rec in stalled:
        events10.append((rec.get("stall_time") or 1e18, "STALL", rec))
    for inc in ins_incidents:
        events10.append((inc["time"], "INSERTION BLOCKED", inc))
    events10.sort(key=lambda x: x[0])
    first10 = []
    for t, kind, obj in events10[:10]:
        if kind == "STALL":
            first10.append(dict(
                time=t, kind="STALL", line=obj["line"], train=obj["vehicle"],
                departure=obj["departure"].rsplit("_", 1)[-1][:5],
                head_link=obj.get("head_link"),
                wanted_link=obj.get("wanted_link"),
                wanted_osm_way=obj.get("wanted_osm_way"),
                nearest_station=obj.get("nearest_station"),
                station_distance_m=obj.get("station_distance_m"),
                direction=obj["direction"],
                blocker=obj.get("blocker_at_stall"),
                blocker_reason=obj.get("blocker_reason_stall"),
                blockers=obj.get("blockers_stall"),
            ))
        else:
            e = {k: v for k, v in obj.items() if k not in ("members", "trains", "kind")}
            e["kind"] = "INSERTION BLOCKED"
            e["count"] = obj["count"]
            first10.append(e)

    # ---- assemble output ----
    def clean_rec(rec):
        out = {}
        for k in ("vehicle", "line", "route_id", "departure", "scheduled_start",
                  "status", "direction", "stops_done", "n_stops",
                  "last_stop_reached", "last_stop_time", "stall_time",
                  "head_link", "head_pos", "head_coords", "tail_link",
                  "wanted_link", "wanted_osm_way", "wanted_coords",
                  "wanted_resource", "blocker", "route_index",
                  "nearest_station", "station_distance_m",
                  "blocker_reason_end", "blockers_end", "direction_owner_end",
                  "blocker_at_stall", "blocker_reason_stall", "blockers_stall",
                  "stall_wanted_link", "stall_wanted_resource",
                  "self_uturn", "still_moving_at_end",
                  "blocked_at_departure_by", "blocked_at_departure_reason",
                  "blocked_at_departure_link", "blocked_at_departure_index"):
            if k in rec:
                out[k] = rec[k]
        if out.get("last_stop_reached"):
            out["last_stop_name"] = facilities.get(out["last_stop_reached"], ("?",))[0]
        if out.get("wanted_coords"):
            out["wanted_coords"] = [round(v, 1) for v in out["wanted_coords"]]
        if out.get("head_coords"):
            out["head_coords"] = [round(v, 1) for v in out["head_coords"]]
        return out

    result = dict(
        experiment=os.path.abspath(exp_dir),
        inputs=dict(events=events_path, network=network_path,
                    schedule=schedule_path,
                    score=score_path if os.path.exists(score_path) else None),
        network=dict(nodes=len(net["node_xy"]), links=len(net["l_from"]),
                     resources=len(net["res_links"]),
                     reverse_pairs=len(net["reverse"]),
                     has_railsim_resource_attribute=net["has_resource_attr"]),
        counts=dict(scheduled=len(trains), with_driver_start=len(dep2veh),
                    entered_network=len(ev["entered"]), completed=len(completed),
                    unfinished=len(unfinished) if unfinished else
                    len([r for r in trains if r["status"] != "completed"]),
                    stalled=len(stalled),
                    insertion_blocked=sum(1 for r in insertion
                                          if r["status"] == "insertion_blocked"),
                    after_sim_end=len(after_end),
                    in_cycles=len(cycle_nodes), cycles=len(cycles),
                    root_deadlocks=len(roots), cascades=len(cascades),
                    insertion_incidents=len(ins_incidents),
                    victims=sum(victim_count.values()),
                    chain_end_moving=sum(1 for e in chain_end.values()
                                         if e == ("open", "moving")),
                    chain_end_self=sum(1 for e in chain_end.values()
                                       if e == ("open", "self")),
                    chain_end_free=sum(1 for e in chain_end.values()
                                       if e == ("open", "free")),
                    still_moving_at_end=sum(1 for r in stalled
                                            if r.get("still_moving_at_end"))),
        end_time=end_time,
        score=score_meta,
        labels={v: "%s@%s" % (by_veh[v]["line"],
                              by_veh[v]["departure"].rsplit("_", 1)[-1][:5])
                for v in by_veh},
        edges=[dict(train=v, blocker=next_of[v],
                    blocker_label=None) for v in sorted(next_of)
               if next_of[v] is not None],
        cycles=[dict(id=i + 1, trains=c,
                     min_stall_time=min(ev["stall"].get(v, 0) for v in c),
                     victims=victim_count.get(i, 0))
                for i, c in enumerate(cycles)],
        cascades=cascades,
        root_deadlocks=roots,
        insertion_incidents=ins_incidents,
        stalled_trains=[clean_rec(r) for r in sorted(
            stalled, key=lambda r: (r.get("stall_time") or 0))],
        stalled_trains_by_veh={r["vehicle"]: clean_rec(r) for r in stalled
                               if r["vehicle"]},
        insertion_trains=[clean_rec(r) for r in sorted(
            [r for r in insertion if r["status"] == "insertion_blocked"],
            key=lambda r: r["scheduled_start"])],
        after_sim_end_trains=[clean_rec(r) for r in after_end],
        areas_deadlocks=[dict(area=a, root_deadlocks=n)
                         for a, n in area_roots.most_common()],
        areas_stalled=[dict(area=a, stalled=n) for a, n in area_stalled.most_common()],
        stall_hours={str(h): n for h, n in sorted(hours.items())},
        first_10=first10,
        event_mismatches=ev["mismatches"],
    )
    return result, trains, net, facilities, ev, roots, ins_incidents, \
        stalled, insertion, after_end


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------
def fmt_coord(c):
    if not c:
        return "-"
    return "(%.0f, %.0f)" % (c[0], c[1])


def render(res, top, net, facilities):
    L = []
    A = L.append
    c = res["counts"]
    labels = res.get("labels", {})

    def short_veh(v):
        if not v:
            return "-"
        return labels.get(v, str(v)[-12:])

    def why(rec):
        r = rec.get("blocker_reason_stall")
        b = rec.get("blocker_at_stall")
        if r == "self U-turn":
            return "self U-turn"
        if r == "reserved":
            return "holds:" + short_veh(b)
        if r == "direction claim":
            return "claim:" + short_veh(b)
        if r is None:
            return "-"
        return r

    A("=" * 108)
    A("RAILSIM DEADLOCK DIAGNOSTICS  %s" % res["experiment"])
    A("=" * 108)
    A("events   %s" % res["inputs"]["events"])
    A("network  %s" % res["inputs"]["network"])
    A("schedule %s" % res["inputs"]["schedule"])
    A("network: %d nodes, %d links, %d resources (%d reverse pairs), railsim attrs=%s"
      % (res["network"]["nodes"], res["network"]["links"],
         res["network"]["resources"], res["network"]["reverse_pairs"],
         res["network"]["has_railsim_resource_attribute"]))
    A("")
    A("TRAINS: %d scheduled | %d got TransitDriverStarts | %d entered network | "
      "%d completed | %d unfinished"
      % (c["scheduled"], c["with_driver_start"], c["entered_network"],
         c["completed"], c["unfinished"]))
    A("  of the unfinished: %d stalled en route, %d blocked before entering, "
      "%d scheduled after sim end (%s)"
      % (c["stalled"], c["insertion_blocked"], c["after_sim_end"],
         hms(res["end_time"])))
    if res["event_mismatches"]:
        A("  event consistency warnings: %s" % res["event_mismatches"])
    A("")
    A("  blockers are taken from railsimLinkStateChangeEvent (EXHAUSTED=reserved,")
    A("  EMPTY=released) per railsimResourceId track, plus a replay of railsim's")
    A("  SimpleDeadlockAvoidance direction claims.  'stopped by' = the state when")
    A("  the train last moved, 'end' = who holds the wanted track at 24:00.")
    A("")

    # ---------------- 1. stalled trains ----------------
    A("-" * 108)
    A("1a. STALLED EN ROUTE - entered the network, never reached the last stop (%d)"
      % len(res["stalled_trains"]))
    A("-" * 108)
    A("%-8s %-5s %-5s %-22s %-7s %-7s %-18s %-11s %s"
      % ("stall", "line", "dep", "last stop reached", "head", "want",
         "stopped by", "end blocker", "vehicle id"))
    for rec in res["stalled_trains"]:
        laststop = rec.get("last_stop_name") or "-"
        if rec.get("last_stop_time"):
            laststop += " @%s" % hms(rec["last_stop_time"])
        A("%-8s %-5s %-5s %-22s %-7s %-7s %-18s %-11s %s"
          % (hms(rec.get("stall_time")), rec["line"],
             rec["departure"].rsplit("_", 1)[-1][:5], laststop[:22],
             rec.get("head_link") or "-", rec.get("wanted_link") or "-",
             why(rec)[:18], short_veh(rec.get("blocker")),
             rec.get("vehicle") or "-"))
    A("")
    A("  last stop reached = last scheduled stop actually served; 'stopped by' is")
    A("  the blocker at the start of the final standstill; 'end blocker' is the")
    A("  occupancy at 24:00.  Blocker labels are line@HH:MM; the full vehicle ids")
    A("  and complete blocker sets are in deadlocks.json (blocker_at_stall,")
    A("  blockers_stall, blockers_end).")
    A("")

    A("-" * 108)
    A("1b. BLOCKED BEFORE ENTERING THE NETWORK (%d)" % len(res["insertion_trains"]))
    A("-" * 108)
    for rec in res["insertion_trains"]:
        A("  %s %-6s %-5s  blocked at link %-7s (route idx %-3s) by %-10s %s  [%s]"
          % (hms(rec["scheduled_start"]), rec["line"],
             rec["departure"].rsplit("_", 1)[-1][:5],
             rec.get("blocked_at_departure_link") or "-",
             rec.get("blocked_at_departure_index"),
             rec.get("blocked_at_departure_reason") or "?",
             short_veh(rec.get("blocked_at_departure_by")),
             rec.get("vehicle") or "-"))
    A("")

    A("-" * 108)
    A("1c. SCHEDULED AFTER THE SIMULATION END (%d) - not simulated, not a deadlock"
      % len(res["after_sim_end_trains"]))
    A("-" * 108)
    for rec in res["after_sim_end_trains"][:top]:
        A("  %s  %-6s %s" % (hms(rec["scheduled_start"]), rec["line"],
                             rec["departure"]))
    if len(res["after_sim_end_trains"]) > top:
        A("  ... %d more" % (len(res["after_sim_end_trains"]) - top))
    A("")

    # ---------------- 2. wait-for graph ----------------
    A("-" * 108)
    A("2. WAIT-FOR GRAPH AT END OF RUN")
    A("-" * 108)
    A("  nodes (stalled trains):                       %d" % c["stalled"])
    A("  edges train -> blocker:                       %d" % len(res["edges"]))
    A("  trains in cycles (true deadlocks):            %d in %d cycles"
      % (c["in_cycles"], c["cycles"]))
    A("  trains queued behind a cycle (victims):       %d" % c["victims"])
    reasons = Counter(r.get("blocker_reason_end") for r in res["stalled_trains"])
    A("  wanted track at 24:00 is: reserved %d | direction-claimed %d | "
      "own U-turn %d | free %d"
      % (reasons.get("reserved", 0), reasons.get("direction claim", 0),
         reasons.get("self U-turn", 0), reasons.get("free", 0)))
    A("  stalled trains still moving when the sim ended:     %d"
      % c["still_moving_at_end"])
    A("  chains ending at a self-block (own U-turn):        %d" % c["chain_end_self"])
    A("  chains ending at a train that is not stalled:      %d" % c["chain_end_moving"])
    A("  chains ending at a genuinely free track:           %d" % c["chain_end_free"])
    A("  independent stall-time cascades:                    %d" % c["cascades"])
    A("  cascades with a real root cause (cycles/self-block):%d" % c["root_deadlocks"])
    A("")

    # ---------------- 3. cycles ----------------
    A("-" * 108)
    A("3. CYCLES AT END OF RUN (true deadlocks)")
    A("-" * 108)
    for cyc in res["cycles"]:
        A("  cycle #%-3d first stall %s  %d trains  victims %d"
          % (cyc["id"], hms(cyc["min_stall_time"]), len(cyc["trains"]),
             cyc["victims"]))
        for v in cyc["trains"]:
            r = res["stalled_trains_by_veh"].get(v, {})
            A("      %-12s head %-7s wants %-7s  %-26s %s"
              % (short_veh(v), r.get("head_link") or "-",
                 r.get("wanted_link") or "-", r.get("direction") or "", v))
    if not res["cycles"]:
        A("  (none)")
    A("")

    # ---------------- 4. root causes ----------------
    A("-" * 108)
    A("4. ROOT CAUSES / DEADLOCK CASCADES (ordered by first stalled train)")
    A("-" * 108)
    A("  A cascade = all stalled trains connected through 'who blocked me when I")
    A("  stopped'.  Root trains are the first cause at the end of the chain;")
    A("  'n trains' counts the whole cascade.  railsim's directional claim reaches")
    A("  ahead along the route, so a root train can block others BEFORE it stops")
    A("  itself: the cascade time is its first victim's stall.")
    A("")
    for r in res["root_deadlocks"]:
        A("[%d] %s  %s" % (r.get("root_id", r["id"]), hms(r["time"]),
                           r["classification"]))
        A("    cascade: %d stalled trains; first stalled: %s (%s dep %s)"
          % (r["n_trains"], short_veh(r["first_train"]),
             r["first_train_line"], r["first_train_departure"]))
        A("    root trains:")
        for m in r["root_members"]:
            A("      %-12s %-34s head %-7s %s pos %.0fm osm:way:%s  %s"
              % (short_veh(m["vehicle"]), m["direction"][:34],
                 m["head_link"] or "-", fmt_coord(m["head_coords"]),
                 m["head_pos"] or 0.0, m["osm_way"] or "-", m["vehicle"]))
            A("      %-12s wants %-7s %s osm:way:%s   nearest station: %s (%.0fm)"
              % ("", m["wanted_link"] or "-", fmt_coord(m["wanted_coords"]),
                 m["wanted_osm_way"] or "-", m["near_station"] or "-",
                 m["station_distance_m"] or -1))
        A("    contested link %s %s osm:way:%s; nearest station %s (%.0f m)"
          % (r["contested_link"] or "-", fmt_coord(r["contested_coords"]),
             r["contested_osm_way"] or "-", r["near_station"] or "-",
             r["station_distance_m"] or -1))
        A("")
    nonroot = [c for c in res["cascades"] if not c.get("is_root")]
    if nonroot:
        A("  NOT ROOT CAUSES (stalled, but the chain ends at a train that is still")
        A("  moving or at a free track - 'just slow / simulation ended'):")
        for casc in nonroot:
            A("    %s  %-26s %2d train(s)  first %s  chain end %s"
              % (hms(casc["time"]), casc["classification"], casc["n_trains"],
                 short_veh(casc["first_train"]),
                 short_veh(casc["root_trains"][0]) if casc["root_trains"] else "-"))
        A("")
    if res["insertion_incidents"]:
        A("  INSERTION-BLOCKED INCIDENTS (%d groups, %d trains)"
          % (c["insertion_incidents"], c["insertion_blocked"]))
        A("  %-9s %-5s %-9s %-8s %-15s %s"
          % ("first", "n", "lines", "link", "reason", "blocker at that time"))
        for inc in res["insertion_incidents"][:top]:
            A("  %-9s %-5d %-9s %-8s %-15s %s"
              % (hms(inc["time"]), inc["count"],
                 ",".join(inc.get("lines", []))[:9],
                 inc["contested_link"] or "-",
                 inc.get("blocker_reason") or "?",
                 short_veh(inc["blocker_at_departure"])))
        if len(res["insertion_incidents"]) > top:
            A("  ... %d more insertion incidents"
              % (len(res["insertion_incidents"]) - top))
        A("")

    # ---------------- 5. summary ----------------
    A("-" * 108)
    A("5. SUMMARY")
    A("-" * 108)
    A("  stalled trains (entered network, never reached last stop): %d" % c["stalled"])
    A("  of which in an end-of-run cycle:                          %d" % c["in_cycles"])
    A("  independent root deadlocks (cascades with a real root):   %d"
      % c["root_deadlocks"])
    A("  end-of-run cycles:                                        %d" % c["cycles"])
    A("  insertion-blocked trains / incidents:                     %d / %d"
      % (c["insertion_blocked"], c["insertion_incidents"]))
    A("  stalled trains queued behind an end-of-run cycle:         %d" % c["victims"])
    A("")
    A("  Root causes by classification:")
    kinds = Counter(r["classification"] for r in res["root_deadlocks"])
    for k, n in kinds.most_common():
        nt = sum(r["n_trains"] for r in res["root_deadlocks"]
                 if r["classification"] == k)
        A("    %-38s %2d cascades  %4d stalled trains" % (k, n, nt))
    A("  (terminus self-block = train wants the reverse of the link it stands on,")
    A("   same capacity-1 resource; head-on/station throat = two-train cycles)")
    A("")
    A("  Root causes per station/area:")
    for a in res["areas_deadlocks"][:top]:
        A("    %-46s %d" % (a["area"][:46], a["root_deadlocks"]))
    A("")
    A("  Stalled trains per nearest station/area (top %d):" % top)
    for a in res["areas_stalled"][:top]:
        A("    %-46s %d" % (a["area"][:46], a["stalled"]))
    A("")
    A("  Trains becoming stalled / failing to start, hour by hour:")
    for h in range(0, 27):
        n = res["stall_hours"].get(str(h))
        if n:
            A("    %02d:00-%02d:59  %4d  %s" % (h, h, n, "#" * min(60, n)))
    A("")

    # ---------------- 6. first 10 ----------------
    A("-" * 108)
    A("6. FIRST 10 THINGS THAT WENT WRONG (chronological)")
    A("-" * 108)
    for i, e in enumerate(res["first_10"]):
        if e["kind"] == "INSERTION BLOCKED":
            A("  %2d. %s  %d train%s could not enter at link %s (%s); blocked by %s"
              % (i + 1, hms(e["time"]), e.get("count", 0),
                 "" if e.get("count", 0) == 1 else "s",
                 e.get("contested_link") or "-", e.get("near_station") or "-",
                 short_veh(e.get("blocker_at_departure"))))
            continue
        if e.get("blocker_reason") == "self U-turn":
            by = "ITSELF (route reverses on the same track)"
        elif e.get("blocker"):
            by = "%s (%s)" % (short_veh(e["blocker"]), e.get("blocker_reason"))
        else:
            by = e.get("blocker_reason") or "-"
        A("  %2d. %s  %-11s %-34s stopped at link %s (%s), wants %s; blocked by %s"
          % (i + 1, hms(e["time"]), short_veh(e["train"]),
             (e.get("direction") or "")[:34], e.get("head_link") or "-",
             e.get("nearest_station") or "-", e.get("wanted_link") or "-", by))
    A("")
    A("Full data: %s/deadlocks.json" % res["experiment"])
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="railsim deadlock diagnostics")
    ap.add_argument("experiment", help="experiment directory with a finished run")
    ap.add_argument("--top", type=int, default=DEFAULT_TOP,
                    help="size of top-N lists (default %d)" % DEFAULT_TOP)
    ap.add_argument("--txt", default=None,
                    help="text output path (default <exp>/deadlocks.txt)")
    ap.add_argument("--cache", default=None,
                    help="pickle file for the parsed schedule/network and the "
                         "event scan (skips the ~2 min scan if it exists)")
    args = ap.parse_args()
    t0 = time.time()
    (res, trains, net, facilities, ev, roots, ins_incidents,
     stalled, insertion, after_end) = analyse(args.experiment, args.top, args.cache)
    text = render(res, args.top, net, facilities)
    json_path = os.path.join(args.experiment, "deadlocks.json")
    txt_path = args.txt or os.path.join(args.experiment, "deadlocks.txt")
    with open(json_path, "w") as f:
        json.dump(res, f, indent=1, default=str)
    with open(txt_path, "w") as f:
        f.write(text + "\n")
    print(text)
    log("wrote %s and %s in %.1fs" % (json_path, txt_path, time.time() - t0))


if __name__ == "__main__":
    main()
