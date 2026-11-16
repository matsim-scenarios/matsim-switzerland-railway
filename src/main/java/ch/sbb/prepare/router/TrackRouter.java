package ch.sbb.prepare.router;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.network.io.NetworkWriter;
import org.matsim.core.population.routes.NetworkRoute;
import org.matsim.core.population.routes.RouteUtils;
import org.matsim.pt.transitSchedule.TransitScheduleFactoryImpl;
import org.matsim.pt.transitSchedule.api.*;
import org.matsim.vehicles.MatsimVehicleWriter;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.VehicleType;
import org.matsim.vehicles.Vehicles;
import org.matsim.vehicles.VehiclesFactory;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * F1 router: RT4b + the "off-model track" model (see docs/assumptions.md and docs/pipeline.md).
 *
 * <p>Track that our OSM/standard-gauge infrastructure data does not contain (metre gauge, foreign
 * lines, dual-gauge-only corridors) is modelled as OFF-MODEL track: unlimited capacity links with a
 * realistic length (real distance on the OSM railway graph, else straight line x detour factor) and a
 * running time clearly below the scheduled one. A stop that has no real track gets its own off-model
 * stop node and is never snapped onto a standard-gauge running track. Everything on real Swiss
 * standard-gauge track is routed exactly as in RT4b.
 *
 * <p>Data decisions (no station list):
 * <ul>
 * <li>{@code --stopClass} CSV (matsim_input/tables/OFFMODEL/stop_track_gauge.csv) marks each stop real/off-model,
 * decided from A03a tags, the gauge of the nearest OSM track and the platform geometry.</li>
 * <li>{@code --legDist} CSV (matsim_input/tables/OFFMODEL/leg_distance.csv) gives the real distance of every
 * consecutive stop pair on the OSM railway graph.</li>
 * </ul>
 */
public final class TrackRouter {

    // ---------------------------------------------------------------- options
    static String scheduleFile = "matsim_input/olten/unmapped_schedule.xml.gz";
    static String networkFile = "matsim_input/tmp/network_osm.xml.gz";
    static String dataDir = "matsim_input/tables";
    static String platformFile = null;   // G1: default <dataDir>/A03a/platform_tracks.csv
    static String outDir = "matsim_input/olten/routed";
    /** G2: the off-model model (F1) is only active when it is configured (-–stopClass/-–legDist). */
    static boolean offModelOn = false;
    static String stopClassFile = null;
    static String legDistFile = null;
    static int threads = Math.max(1, Math.min(12, Runtime.getRuntime().availableProcessors() - 2));
    static double vmaxKmh = 160;
    static double relStrict = 0.8;
    static boolean relationsOn = true;
    static int limitRoutes = -1;
    static String onlyRoute = null;
    static int maxCandidates = 6;
    static final Router.Params baseParams = new Router.Params();
    static final ArrayList<String> ISSUES = new ArrayList<>();
    // rule switches (all rules default on except the expensive negotiation stage)
    static boolean r1On = true, r2On = true, r3On = true, r4On = true, r5On = true, r6On = true,
            r7On = true, r8On = true, r9On = false;
    // R7
    static double corridorRatio = 0.6, corridorMinKm = 20.0, corridorRadius = 2000.0, corridorFactor = 5.0;
    static double corridorGain = 0.0;
    // R9
    static boolean dubiousOn = true;
    static int maxIter = 8;
    static double margin = 60.0, histDelta = 60.0;
    static final List<String> CORRIDOR = Collections.synchronizedList(new ArrayList<String>());
    static final ArrayList<String> NEG_LOG = new ArrayList<>();
    // ---- G3 R7c: corridor choice by booked time, decided once and not re-negotiated by R9
    /** share of the alternative that must lie on a line (OSM track_ref) of the direct path */
    static double r7cSameLine = 0.60;
    /** R9 may not replan a route whose leg the timetable decided (R7c) */
    static boolean r9Pin = true;
    static boolean[] CORRIDOR_JOB = new boolean[0];
    static final List<String> R7C = Collections.synchronizedList(new ArrayList<String>());

    // ---- G2 passing loops
    static boolean loopsOn = true;
    static byte[] LOOP_DIR = null;
    static Loops LOOPS = null;

    // ---- off-model track model
    /** detour factor for a hop whose real distance is not in the leg table */
    static double offDetour = 1.25;
    /** freespeed cap of an off-model link [m/s] (no train is faster than this off-model) */
    static double offVmaxMs = 160 / 3.6;
    /** minimum running time of an off-model link = offRuntimeFrac x scheduled leg time */
    static double offRuntimeFrac = 0.85;
    /** if a real leg needs more than scheduled + infeasibleSlack it is replaced by off-model track */
    static double infeasibleSlack = 60.0;
    static final HashMap<String, Boolean> stopReal = new HashMap<>();
    static final HashMap<String, String> stopReason = new HashMap<>();
    static final HashMap<String, Double> legDist = new HashMap<>();
    static final HashMap<String, String> legDistSource = new HashMap<>();

    // stats
    static int statOffHops = 0, statOffHopsCapped = 0, statStubs = 0, statChainStops = 0;
    static double statOffKm = 0, statOffSpeedSum = 0, statOffSpeedMin = 1e9, statOffSpeedMax = 0;
    static int statChains = 0, statRoutesWithChain = 0, statRoutesFullyOff = 0;
    static int statInfeasibleSplit = 0, statInfeasibleKept = 0, statRealNoCand = 0;
    static final ArrayList<String> INFEASIBLE_EXAMPLES = new ArrayList<>();

    // ---------------------------------------------------------------- model

    static final class Standort {
        String id, name, area;
        double x, y;
    }

    static final class Job {
        Sched.Line line;
        Sched.Route route;
        int index;
        boolean[] real;          // per stop: routed on real track
        int[][] segAnchors;      // anchor stop indices per real segment
        Router.Plan[] plans;
        Router.Plan plan;        // unused for solving; kept for the analyser (segments 0..n)
        int[] links;             // >=0 network link, <0 -> -1-extIndex
        int[] stopPos;
        byte[] kind;             // per stop: 0 platform, 1 fallback, 2 off-model
        double cost;
        int nChain;
        int nInfeasibleSplit;
    }

    static final class ExtLink {
        String id;
        int fromRef, toRef;   // >=0 real node index, <0 -> ext node
        double length, speed;
        double x1, y1, x2, y2;
    }

    static final class ExtBuilder {
        final Net net;
        ExtBuilder(Net net) { this.net = net; }
        final ArrayList<ExtLink> links = new ArrayList<>();
        final ArrayList<String> nodeIds = new ArrayList<>();
        final ArrayList<double[]> nodeXY = new ArrayList<>();
        final HashMap<String, Integer> linkCache = new HashMap<>();
        final HashMap<String, Integer> nodeCache = new HashMap<>();
        int counter = 0;

        int node(String key, double x, double y) {
            Integer v = nodeCache.get(key);
            if (v != null) return v;
            if (key.startsWith("N:")) {
                int real = net.nodeIndex(Long.parseLong(key.substring(2)));
                nodeCache.put(key, real);
                return real;
            }
            int idx = nodeIds.size();
            nodeIds.add("extn_" + (idx));
            nodeXY.add(new double[]{x, y});
            int ref = -1 - idx;
            nodeCache.put(key, ref);
            return ref;
        }

        private int add(String fk, double fx, double fy, String tk, double tx, double ty,
                        double len, double speed, String tag) {
            String key = fk + ">" + tk;
            Integer v = linkCache.get(key);
            if (v != null) {
                ExtLink e = links.get(v);
                if (speed > e.speed) e.speed = speed;    // keep the fastest scheduled requirement
                if (Math.abs(len - e.length) > 0.05 * Math.max(1.0, len) && Math.abs(len - e.length) > 5) {
                    System.out.println("EXT LENGTH CONFLICT " + key + ": " + e.length + " vs " + len);
                }
                return v;
            }
            int from = node(fk, fx, fy), to = node(tk, tx, ty);
            ExtLink e = new ExtLink();
            e.id = "ext_" + (counter++);
            e.fromRef = from; e.toRef = to; e.length = Math.max(0.1, len); e.speed = speed;
            e.x1 = fx; e.y1 = fy; e.x2 = tx; e.y2 = ty;
            links.add(e);
            int idx = links.size() - 1;
            linkCache.put(key, idx);
            return idx;
        }

        /**
         * An off-model link: length = real distance (leg table) or straight line x detour, freespeed such
         * that its free running time is offRuntimeFrac x the scheduled time of the hop, capped to
         * [25 km/h, offVmaxMs]. Returns the link index.
         */
        int offLink(String fk, double fx, double fy, String tk, double tx, double ty, double len, double schedDt) {
            double dt = Math.max(schedDt, 20.0);
            double v = len / (offRuntimeFrac * dt);
            v = Math.max(25 / 3.6, Math.min(offVmaxMs, v));
            if (len / (offRuntimeFrac * dt) > offVmaxMs) statOffHopsCapped++;
            statOffHops++;
            statOffKm += len / 1000.0;
            statOffSpeedSum += v * 3.6;
            statOffSpeedMin = Math.min(statOffSpeedMin, v * 3.6);
            statOffSpeedMax = Math.max(statOffSpeedMax, v * 3.6);
            return add(fk, fx, fy, tk, tx, ty, len, v, "off");
        }

        /** 1 m departure stub of an off-model route start (the train is placed at the end of its start link). */
        int stub(String fk, double fx, double fy, String tk, double tx, double ty) {
            statStubs++;
            return add(fk, fx, fy, tk, tx, ty, 1.0, 100 / 3.6, "stub");
        }

        /** legacy straight extension link (kept for --legacyExt) */
        int link(String fk, double fx, double fy, String tk, double tx, double ty, double t) {
            double len = Math.hypot(tx - fx, ty - fy);
            double sp;
            if (t > 1) sp = Math.max(20.0, len / (0.8 * t));
            else sp = 55.0;
            if (sp > 100) sp = 100;
            return add(fk, fx, fy, tk, tx, ty, len, sp, "legacy");
        }
    }

    // ---------------------------------------------------------------- main

    public static void main(String[] args) throws Exception {
        parseArgs(args);
        new java.io.File(outDir).mkdirs();     // the diagnostics (loops.csv, r7c.csv, ...) go here first
        long t0 = System.currentTimeMillis();
        Router.Params prm = baseParams;
        prm.maxCandidates = maxCandidates;
        Net.Options no = new Net.Options();
        no.r2 = r2On;
        no.r3 = r3On;
        no.dirConfMin = baseParams.dirConfMin;
        Net net = Net.load(networkFile, dataDir, true, no);
        printDirectionAgreement(net);

        Sched sched = Sched.read(scheduleFile);
        Stops stops = new Stops(net, prm);
        stops.dubiousOn = dubiousOn;
        stops.loadPlatforms(platformFile != null ? platformFile : dataDir + "/A03a/platform_tracks.csv");
        loadStopClasses(stopClassFile);
        loadLegDistances(legDistFile);
        System.out.printf(Locale.ROOT, "schedule: %d lines, %d routes, %d stop facilities%n",
                sched.lines.size(), countRoutes(sched), sched.stops.size());
        System.out.printf(Locale.ROOT, "stop classes: %d real, %d off-model (of %d stops in the table)%n",
                stopReal.values().stream().filter(Boolean::booleanValue).count(),
                stopReal.values().stream().filter(b -> !b).count(), stopReal.size());
        System.out.printf(Locale.ROOT, "leg distances: %d stop pairs from %s%n", legDist.size(), legDistFile);

        RelData rel = RelData.load(dataDir + "/A06/match.csv", dataDir + "/A06/relation_ways.csv", net, relStrict);
        System.out.printf(Locale.ROOT, "relations: %d good route matches, %d relations loaded%n",
                rel.routeRel.size(), rel.relWays.size());

        // ---- flatten
        ArrayList<Job> jobs = new ArrayList<>();
        for (Sched.Line l : sched.lines)
            for (Sched.Route r : l.routes)
                if (onlyRoute == null || r.id.contains(onlyRoute)) {
                    Job j = new Job(); j.line = l; j.route = r; j.index = jobs.size();
                    jobs.add(j);
                }
        if (limitRoutes > 0 && jobs.size() > limitRoutes) jobs = new ArrayList<>(jobs.subList(0, limitRoutes));

        // ---- precompute candidates (serial)
        Map<String, Router.Cand> candCache = new HashMap<>();
        for (Job j : jobs) for (Sched.StopRef sr : j.route.stops) {
            Sched.StopF sf = sched.stops.get(sr.stopId);
            if (sf == null) throw new IllegalStateException("unknown stop " + sr.stopId);
            candCache.computeIfAbsent(sr.stopId, k -> isRealStop(k)
                    ? stops.candidates(sf) : new Router.Cand(new int[0], new double[0]));
        }
        System.out.printf(Locale.ROOT, "stop candidates ready (%d distinct stops)%n", candCache.size());
        String dbgStop = System.getenv("RT4B_DEBUG_STOP");
        if (dbgStop != null) {
            for (String id : dbgStop.split(",")) {
                Router.Cand c = candCache.get(id);
                if (c == null) { System.out.println("dbg stop " + id + ": no candidates"); continue; }
                System.out.println("dbg stop " + id + " method=" + stops.method(id) + " -> " + c.links.length + " candidates");
                for (int i = 0; i < c.links.length; i++) {
                    int l = c.links[i];
                    System.out.printf(Locale.ROOT, "   link %d way %d len %.1f dist %.2f stub %b extra %.1f%n",
                            net.linkId[l], net.wayId[net.lWay[l]], net.lLen[l],
                            net.distToLink(l, sched.stops.get(id).x, sched.stops.get(id).y), c.stub[i], c.extra[i]);
                }
            }
        }

        // ---- route (parallel)
        Job[] arr = jobs.toArray(new Job[0]);
        routeAll(arr, prm, net, candCache, rel, stops, sched);

        int[][] stop0 = new int[arr.length][];
        for (int i = 0; i < arr.length; i++) stop0[i] = stopLinksOf(arr[i]);

        // ---- G2 passing loops: detect them from the link usage of the first pass, then re-route
        //      with a per-direction track assignment (each direction gets its own loop track)
        if (loopsOn) {
            int[] use = new int[net.nLinks];
            boolean[] stopLink = new boolean[net.nLinks];
            for (Job j : arr) {
                int nDep = Math.max(1, j.route.deps.size());
                int[] core = coreOf(j);
                int off = 0;
                for (Sched.StopRef sr : j.route.stops) {
                    Router.Cand c = candCache.get(sr.stopId);
                    if (c == null) continue;
                    String m = stops.method(sr.stopId);
                    // a published platform track wins: never hand its whole candidate pool to a loop
                    if ("track_ref".equals(m) || "stop_node".equals(m))
                        for (int x : c.links) if (x >= 0) stopLink[x] = true;
                }
                for (Router.Plan p : j.plans) {
                    for (int x : p.core) use[x] += nDep;
                    for (int i = 0; i < p.stopCorePos.length; i++) {
                        int q = p.stopCorePos[i];
                        if (q >= 0 && q < p.core.length && p.core[q] >= 0) stopLink[p.core[q]] = true;
                    }
                    off += p.core.length;
                }
            }
            LOOPS = Loops.detect(net, use, stopLink, sched);
            LOOPS.write(outDir + "/loops.csv");
            LOOP_DIR = LOOPS.dir;
            resetCounters();
            System.out.println("G2: re-routing with the passing-loop track assignment");
            routeAll(arr, prm, net, candCache, rel, stops, sched);
            // the published platform track wins: a route whose stop link moved is re-routed without
            // the loop assignment (keeps platform_ok exactly at the level of the plain router)
            int nFix = 0;
            for (Job j : arr) {
                if (java.util.Arrays.equals(stopLinksOf(j), stop0[j.index])) continue;
                Router rr = new Router(net, prm.copy());
                rr.loopDir = null;
                routeOne(j, rr, rr.newSearcher(), net, candCache, rel, stops, sched);
                nFix++;
            }
            System.out.printf(Locale.ROOT, "G2: %d route(s) re-routed without the loop assignment (stop link changed)%n", nFix);
        }

        // ---- R7: timetable-fit corridor choice (per real segment)
        if (r7On) {
            CORRIDOR_JOB = new boolean[arr.length];
            R7C.clear();
            Router.Params lp = prm.copy();
            Router router = new Router(net, lp);
            router.loopDir = LOOP_DIR;
            Router.Sear se = router.newSearcher();
            for (Job j : arr) applyCorridor(j, router, se, sched, net, rel);
            System.out.println("R7 corridor fit fired on " + CORRIDOR.size() + " leg(s)");
            for (int i = 0; i < Math.min(200, CORRIDOR.size()); i++) System.out.println("   " + CORRIDOR.get(i));
            new File(outDir).mkdirs();
            try (java.io.PrintWriter pw = new java.io.PrintWriter(new java.io.OutputStreamWriter(
                    new java.io.FileOutputStream(outDir + "/r7c.csv"), StandardCharsets.UTF_8))) {
                pw.println("line,route,leg,from_stop,to_stop,from_name,to_name,sched_s,fast_s,fast_km,alt_s,alt_km,"
                        + "cov_fast,cov_alt,same_line,decision,refs_fast,refs_alt");
                for (String s : R7C) pw.println(s);
            }
            int nTake = CORRIDOR.size();
            System.out.println("R7c decisions: " + nTake + " taken, " + (R7C.size() - nTake) + " kept");
        }

        // ---- R9: negotiated re-routing on the R7 corridors
        if (r9On) negotiate(arr, sched, net, prm, candCache, rel, stops);

        // ---- assemble with off-model chains (serial, deterministic)
        ExtBuilder ext = new ExtBuilder(net);
        for (Job j : arr) assemble(j, sched, net, ext);
        System.out.printf(Locale.ROOT, "assembled; ext links: %d%n", ext.links.size());

        new File(outDir).mkdirs();
        Stats st = analyse(arr, sched, net, ext);
        writeAll(arr, sched, net, ext, st);
        System.out.printf(Locale.ROOT, "TOTAL %.1f s%n", (System.currentTimeMillis() - t0) / 1000.0);
    }

    /** route every job in parallel; loopDir (may be null) is the G2 passing-loop assignment */
    static void routeAll(Job[] arr, Router.Params prm, Net net, Map<String, Router.Cand> candCache,
                         RelData rel, Stops stops, Sched sched) {
        AtomicInteger next = new AtomicInteger();
        AtomicInteger done = new AtomicInteger();
        int nt = Math.min(threads, Math.max(1, arr.length));
        Thread[] th = new Thread[nt];
        List<String> errs = Collections.synchronizedList(new ArrayList<String>());
        for (int i = 0; i < nt; i++) {
            th[i] = new Thread(() -> {
                Router.Params lp = prm.copy();
                Router router = new Router(net, lp);
                router.loopDir = LOOP_DIR;
                Router.Sear se = router.newSearcher();
                while (true) {
                    int k = next.getAndIncrement();
                    if (k >= arr.length) break;
                    Job j = arr[k];
                    try {
                        routeOne(j, router, se, net, candCache, rel, stops, sched);
                    } catch (RuntimeException e) {
                        errs.add(j.route.id + ": " + e);
                        e.printStackTrace();
                    }
                    int d = done.incrementAndGet();
                    if (d % 50 == 0) System.out.printf(Locale.ROOT, "  routed %d/%d%n", d, arr.length);
                }
            });
            th[i].start();
        }
        for (Thread t : th) {
            try { t.join(); } catch (InterruptedException e) { throw new RuntimeException(e); }
        }
        if (!errs.isEmpty()) {
            System.out.println("ROUTING ERRORS: " + errs.size());
            for (int i = 0; i < Math.min(10, errs.size()); i++) System.out.println("  " + errs.get(i));
        }
    }

    /** the network link chosen for every stop of a route (pass comparison for the platform rule) */
    static int[] stopLinksOf(Job j) {
        int ns = j.route.stops.size();
        int[] out = new int[ns];
        Arrays.fill(out, -2);
        if (j.plans == null) return out;
        for (Router.Plan p : j.plans) {
            for (int i = 0; i < ns && i < p.stopCorePos.length; i++) {
                int q = p.stopCorePos[i];
                if (q >= 0 && q < p.core.length) out[i] = p.core[q];
            }
        }
        return out;
    }

    /** counters of one routing pass (the loop detection needs a second pass, so reset them) */
    static void resetCounters() {
        synchronized (ISSUES) { ISSUES.clear(); }
        synchronized (CORRIDOR) { CORRIDOR.clear(); }
        statOffHops = 0; statOffHopsCapped = 0; statStubs = 0; statChainStops = 0;
        statOffKm = 0; statOffSpeedSum = 0; statOffSpeedMin = 1e9; statOffSpeedMax = 0;
        statChains = 0; statRoutesWithChain = 0; statRoutesFullyOff = 0;
        statInfeasibleSplit = 0; statInfeasibleKept = 0; statRealNoCand = 0;
        INFEASIBLE_EXAMPLES.clear();
        NEG_LOG.clear();
    }

    static int countRoutes(Sched s) {
        int n = 0;
        for (Sched.Line l : s.lines) n += l.routes.size();
        return n;
    }

    // ---------------------------------------------------------------- stop classes / leg distances

    static void loadStopClasses(String file) throws IOException {
        if (file == null) return;
        try (BufferedReader br = new BufferedReader(new InputStreamReader(Net.open(file), StandardCharsets.UTF_8))) {
            String line = br.readLine();
            String[] h = Net.csv(line);
            int iId = col(h, "stop_id"), iReal = col(h, "real"), iReason = col(h, "reason");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = Net.splitSimple(line);
                stopReal.put(c[iId], "1".equals(c[iReal]));
                stopReason.put(c[iId], c[iReason]);
            }
        }
    }

    static void loadLegDistances(String file) throws IOException {
        if (file == null) return;
        try (BufferedReader br = new BufferedReader(new InputStreamReader(Net.open(file), StandardCharsets.UTF_8))) {
            String line = br.readLine();
            String[] h = Net.csv(line);
            int iA = col(h, "from_stop"), iB = col(h, "to_stop"), iD = col(h, "dist_m"),
                    iC = col(h, "connected"), iBl = col(h, "beeline_m");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = Net.splitSimple(line);
                String a = c[iA], b = c[iB];
                String k = a.compareTo(b) <= 0 ? a + "|" + b : b + "|" + a;
                legDist.put(k, Double.parseDouble(c[iD]));
                legDistSource.put(k, "1".equals(c[iC]) ? "osm-graph" : "beeline x" + offDetour
                        + " (beeline " + c[iBl] + " m)");
            }
        }
    }

    static int col(String[] h, String name) {
        for (int i = 0; i < h.length; i++) if (h[i].equals(name)) return i;
        throw new IllegalStateException("no column " + name);
    }

    static boolean isRealStop(String stopId) {
        Boolean b = stopReal.get(stopId);
        return b == null || b;              // unknown stop -> keep RT4b behaviour (real)
    }

    /** real distance of an off-model hop, or straight line x detour if unknown */
    static double offLength(String a, String b, double x1, double y1, double x2, double y2) {
        if (a.startsWith("S:") && b.startsWith("S:")) {
            String ida = a.substring(2), idb = b.substring(2);
            String k = ida.compareTo(idb) <= 0 ? ida + "|" + idb : idb + "|" + ida;
            Double d = legDist.get(k);
            if (d != null && d > 0) return d;
        }
        return Math.hypot(x2 - x1, y2 - y1) * offDetour;
    }

    // ---------------------------------------------------------------- args
    static void parseArgs(String[] a) {
        for (int i = 0; i < a.length; i++) {
            String k = a[i];
            switch (k) {
                case "--schedule": scheduleFile = a[++i]; break;
                case "--network": networkFile = a[++i]; break;
                case "--data": dataDir = a[++i]; break;
                case "--out": outDir = a[++i]; break;
                case "--stopClass": stopClassFile = a[++i]; break;
                case "--legDist": legDistFile = a[++i]; break;
                case "--loops": loopsOn = Boolean.parseBoolean(a[++i]); break;
                case "--loopPenalty": baseParams.loopPenalty = Double.parseDouble(a[++i]); break;
                case "--offDetour": offDetour = Double.parseDouble(a[++i]); break;
                case "--offVmax": offVmaxMs = Double.parseDouble(a[++i]) / 3.6; break;
                case "--offRuntimeFrac": offRuntimeFrac = Double.parseDouble(a[++i]); break;
                case "--infeasibleSlack": infeasibleSlack = Double.parseDouble(a[++i]); break;
                case "--threads": threads = Integer.parseInt(a[++i]); break;
                case "--vmax": vmaxKmh = Double.parseDouble(a[++i]); break;
                case "--relStrict": relStrict = Double.parseDouble(a[++i]); break;
                case "--relBonus": baseParams.relBonus = Double.parseDouble(a[++i]); break;
                case "--relationsOff": relationsOn = false; break;
                case "--r1": r1On = Boolean.parseBoolean(a[++i]); break;
                case "--r2": r2On = Boolean.parseBoolean(a[++i]); break;
                case "--r3": r3On = Boolean.parseBoolean(a[++i]); break;
                case "--r4": r4On = Boolean.parseBoolean(a[++i]); break;
                case "--r5": r5On = Boolean.parseBoolean(a[++i]); break;
                case "--r6": r6On = Boolean.parseBoolean(a[++i]); break;
                case "--r7": r7On = Boolean.parseBoolean(a[++i]); break;
                case "--r8": r8On = Boolean.parseBoolean(a[++i]); break;
                case "--r9": r9On = Boolean.parseBoolean(a[++i]); break;
                case "--dirPenalty": baseParams.dirPenalty = Double.parseDouble(a[++i]); break;
                case "--dirEntry": baseParams.dirEntryCost = Double.parseDouble(a[++i]); break;
                case "--unknownPenalty": baseParams.unknownPenalty = Double.parseDouble(a[++i]); break;
                case "--freightPenalty": baseParams.freightPenalty = Double.parseDouble(a[++i]); break;
                case "--crevCost": baseParams.crossoverCost = Double.parseDouble(a[++i]); break;
                case "--sidingPenalty": baseParams.sidingPenalty = Double.parseDouble(a[++i]); break;
                case "--crossoverPenalty": baseParams.crossoverPenalty = Double.parseDouble(a[++i]); break;
                case "--servicePenalty": baseParams.servicePenalty = Double.parseDouble(a[++i]); break;
                case "--dirConfMin": baseParams.dirConfMin = Double.parseDouble(a[++i]); break;
                case "--dubious": dubiousOn = Boolean.parseBoolean(a[++i]); break;
                case "--softTurn": baseParams.softTurnPenalty = Double.parseDouble(a[++i]); break;
                case "--corridorRatio": corridorRatio = Double.parseDouble(a[++i]); break;
                case "--corridorMinKm": corridorMinKm = Double.parseDouble(a[++i]); break;
                case "--corridorRadius": corridorRadius = Double.parseDouble(a[++i]); break;
                case "--corridorFactor": corridorFactor = Double.parseDouble(a[++i]); break;
                case "--corridorGain": corridorGain = Double.parseDouble(a[++i]); break;
                case "--r7cSameLine": r7cSameLine = Double.parseDouble(a[++i]); break;
                case "--r9pin": r9Pin = Boolean.parseBoolean(a[++i]); break;
                case "--maxIter": maxIter = Integer.parseInt(a[++i]); break;
                case "--margin": margin = Double.parseDouble(a[++i]); break;
                case "--histDelta": histDelta = Double.parseDouble(a[++i]); break;
                case "--revCost": baseParams.revCost = Double.parseDouble(a[++i]); break;
                case "--r5RevPenalty": baseParams.r5RevPenalty = Double.parseDouble(a[++i]); break;
                case "--platforms": platformFile = a[++i]; break;
                case "--unmatchedCostPerM": baseParams.unmatchedCostPerM = Double.parseDouble(a[++i]); break;
                case "--cutoffFactor": baseParams.cutoffFactor = Double.parseDouble(a[++i]); break;
                case "--cutoffAdd": baseParams.cutoffAdd = Double.parseDouble(a[++i]); break;
                case "--maxCandidates": maxCandidates = Integer.parseInt(a[++i]); break;
                case "--limitRoutes": limitRoutes = Integer.parseInt(a[++i]); break;
                case "--onlyRoute": onlyRoute = a[++i]; break;
                default: throw new IllegalArgumentException("unknown arg " + k);
            }
        }
        baseParams.vmaxMs = vmaxKmh / 3.6;
        baseParams.r1 = r1On; baseParams.r2 = r2On; baseParams.r4 = r4On; baseParams.r5 = r5On;
        baseParams.r6 = r6On; baseParams.r8 = r8On;
        // G2: the off-model model (F1) is a property of the Switzerland case; the Olten case must stay
        // bit-identical to G1/out/final, so it is off unless the two data files are given.
        offModelOn = stopClassFile != null || legDistFile != null;
    }

    // ---------------------------------------------------------------- direction stats
    static void printDirectionAgreement(Net net) {
        int nRel = 0, agree = 0, nConflict = 0;
        for (int w = 0; w < net.nWays; w++) {
            int r = net.wayRelDir[w];
            if (r != 1 && r != 2) continue;
            if (net.wayDirRaw[w] == 0 || net.wayDirConfRaw[w] < baseParams.dirConfMin) continue;
            nRel++;
            int rd = r == 1 ? 1 : -1;
            if (rd == net.wayDirRaw[w]) agree++; else nConflict++;
        }
        System.out.printf(Locale.ROOT, "direction prior: %d ways with DIR+relation evidence, agree %d (%.1f %%), disagree %d%n",
                nRel, agree, 100.0 * agree / Math.max(1, nRel), nConflict);
        int nF = 0, nB = 0, nBoth = 0, nUnk = 0;
        for (int w = 0; w < net.nWays; w++) {
            if (net.wayDir[w] == 1) nF++;
            else if (net.wayDir[w] == -1) nB++;
            else if (net.wayBoth[w]) nBoth++;
            else nUnk++;
        }
        System.out.printf(Locale.ROOT, "resolved direction: %d forward, %d backward, %d both, %d unknown%n",
                nF, nB, nBoth, nUnk);
    }

    // ---------------------------------------------------------------- per route
    static void routeOne(Job j, Router router, Router.Sear se, Net net,
                         Map<String, Router.Cand> candCache, RelData rel, Stops stops, Sched sched) {
        Sched.Route r = j.route;
        int nStops = r.stops.size();
        j.real = new boolean[nStops];
        j.kind = new byte[nStops];
        j.stopPos = new int[nStops];
        Arrays.fill(j.stopPos, -1);

        ArrayList<Integer> anchors = new ArrayList<>();
        for (int i = 0; i < nStops; i++) {
            Sched.StopRef sr = r.stops.get(i);
            Router.Cand c = candCache.get(sr.stopId);
            boolean classified = isRealStop(sr.stopId);
            if (classified && (c == null || c.links.length == 0)) {
                synchronized (TrackRouter.class) { statRealNoCand++; }
            }
            if (classified && c != null && c.links.length > 0) {
                anchors.add(i);
                boolean matched = stops.method(sr.stopId).equals("track_ref") || stops.method(sr.stopId).equals("stop_node");
                j.kind[i] = (byte) (matched ? 0 : 1);
                j.real[i] = true;
            } else {
                j.kind[i] = 2;
            }
        }
        int m = anchors.size();
        if (m == 0) {
            j.segAnchors = new int[0][];
            j.plans = new Router.Plan[0];
            j.plan = null;
            synchronized (TrackRouter.class) { statRoutesFullyOff++; }
            return;
        }

        // segment boundary: an off-model stop between two anchors, or an infeasible real leg
        boolean[] split = new boolean[m];
        for (int k = 1; k < m; k++) split[k] = offModelOn && anchors.get(k) - anchors.get(k - 1) > 1;
        boolean[] infSplit = new boolean[m];

        for (int pass = 0; pass < (offModelOn ? 3 : 1); pass++) {
            buildSegments(j, anchors, split);
            j.plans = new Router.Plan[j.segAnchors.length];
            for (int s = 0; s < j.segAnchors.length; s++) {
                int[] a = j.segAnchors[s];
                // from/to = the whole route: G1's router hands the full stop range to solve() so that
                // stops before the first anchor become a prefix chain (identical Olten output)
                Router.Plan p = solveAnchors(j, a, 0, nStops, router, se, net, candCache, rel, sched);
                if (p == null) throw new IllegalStateException("no plan");
                j.plans[s] = p;
            }
            boolean changed = false;
            for (int s = 0; s < j.segAnchors.length; s++) {
                int[] a = j.segAnchors[s];
                Router.Plan p = j.plans[s];
                for (int k = 0; k + 1 < a.length; k++) {
                    int i0 = p.stopCorePos[a[k]], i1 = p.stopCorePos[a[k + 1]];
                    if (i0 < 0 || i1 < 0) continue;
                    int lo = Math.min(i0, i1), hi = Math.max(i0, i1);
                    double rt = 0;
                    for (int q = lo + 1; q <= hi; q++)
                        rt += net.lLen[p.core[q]] / Math.max(Math.min(net.lRouteSpeed[p.core[q]], paramsVmax()), 0.5);
                    double schedT = r.stops.get(a[k + 1]).arr - r.stops.get(a[k]).dep;
                    if (rt <= schedT + infeasibleSlack) continue;
                    Sched.StopF s1 = sched.stops.get(r.stops.get(a[k]).stopId);
                    Sched.StopF s2 = sched.stops.get(r.stops.get(a[k + 1]).stopId);
                    double L = offLength("S:" + s1.id, "S:" + s2.id, s1.x, s1.y, s2.x, s2.y);
                    if (L / offVmaxMs < schedT) {
                        int ai = anchorIndex(anchors, a[k + 1]);
                        if (ai >= 0 && !infSplit[ai]) {
                            infSplit[ai] = true;
                            synchronized (TrackRouter.class) {
                                statInfeasibleSplit++;
                                if (INFEASIBLE_EXAMPLES.size() < 60)
                                    INFEASIBLE_EXAMPLES.add(String.format(Locale.ROOT,
                                            "%s %s: %s -> %s real %.1f km / %.0f s min vs %.0f s scheduled, off-model %.1f km",
                                            j.line.name, j.route.id, s1.name, s2.name, kmOf(p, net, lo, hi), rt, schedT, L / 1000));
                            }
                        }
                        split[ai >= 0 ? ai : k + 1] = true;
                        changed = true;
                    } else {
                        synchronized (TrackRouter.class) { statInfeasibleKept++; }
                    }
                }
            }
            if (!changed) break;
        }
        int nInf = 0;
        for (boolean b : infSplit) if (b) nInf++;
        j.nInfeasibleSplit = nInf;
        j.plan = j.plans[0];
        if (!j.plans[0].issues.isEmpty()) {
            synchronized (ISSUES) { ISSUES.add(r.id + " : " + j.plans[0].issues); }
        }
    }

    static int anchorIndex(ArrayList<Integer> anchors, int stop) {
        for (int i = 0; i < anchors.size(); i++) if (anchors.get(i) == stop) return i;
        return -1;
    }

    static double kmOf(Router.Plan p, Net net, int lo, int hi) {
        double km = 0;
        for (int q = lo + 1; q <= hi; q++) km += net.lLen[p.core[q]];
        return km / 1000;
    }

    static void buildSegments(Job j, ArrayList<Integer> anchors, boolean[] split) {
        ArrayList<int[]> segs = new ArrayList<>();
        ArrayList<Integer> cur = new ArrayList<>();
        for (int k = 0; k < anchors.size(); k++) {
            if (k > 0 && split[k] && !cur.isEmpty()) {
                segs.add(toIntArray(cur));
                cur = new ArrayList<>();
            }
            cur.add(anchors.get(k));
        }
        if (!cur.isEmpty()) segs.add(toIntArray(cur));
        j.segAnchors = segs.toArray(new int[0][]);
    }

    static int[] toIntArray(ArrayList<Integer> l) {
        int[] a = new int[l.size()];
        for (int i = 0; i < a.length; i++) a[i] = l.get(i);
        return a;
    }

    static int[] rangeInts(int from, int to) {
        if (to <= from) return new int[0];
        int[] a = new int[to - from];
        for (int i = from; i < to; i++) a[i - from] = i;
        return a;
    }

    /** Solve one real segment (anchor stop indices) between stop bounds from..to (exclusive). */
    static Router.Plan solveAnchors(Job j, int[] anchorIdx, int from, int to,
                                    Router router, Router.Sear se, Net net,
                                    Map<String, Router.Cand> candCache, RelData rel, Sched sched) {
        Sched.Route r = j.route;
        int nStops = r.stops.size();
        ArrayList<Router.Cand> cands = new ArrayList<>();
        for (int a : anchorIdx) cands.add(candCache.get(r.stops.get(a).stopId));
        int[] schedArr = new int[nStops], schedDep = new int[nStops];
        for (int i = 0; i < nStops; i++) {
            schedArr[i] = (int) Math.round(r.stops.get(i).arr);
            schedDep[i] = (int) Math.round(r.stops.get(i).dep);
        }
        int[] relF = null, relB = null;
        Integer relId = rel.routeRel.get(r.id);
        if (relId != null) {
            RelData.Ways w = rel.relWays.get(relId);
            if (w != null) { relF = w.forward; relB = w.backward; }
        }
        byte[] mask = router.relMask();
        boolean relSet = false;
        if (relationsOn && relF != null) {
            for (int x : relF) if (x >= 0) mask[x] |= 1;
            for (int x : relB) if (x >= 0) mask[x] |= 2;
            relSet = true;
        }
        String dl = System.getenv("RT4B_DEBUG_LEG");
        if (dl != null && dl.startsWith(r.id + ":")) {
            Router.DBG_ROUTE = r.id;
            Router.DBG_LEG = Integer.parseInt(dl.substring(r.id.length() + 1));
            Router.DBG_ON = true;
        }
        try {
            return router.solve(cands, anchorIdx, nStops, schedArr, schedDep, se, from, to);
        } finally {
            if (relSet) {
                for (int x : relF) if (x >= 0) mask[x] = 0;
                for (int x : relB) if (x >= 0) mask[x] = 0;
            }
        }
    }

    // ---------------------------------------------------------------- assembly

    static void assemble(Job j, Sched sched, Net net, ExtBuilder ext) {
        if (!offModelOn) { assembleG1(j, sched, net, ext); return; }
        Sched.Route r = j.route;
        int nStops = r.stops.size();
        j.stopPos = new int[nStops];
        Arrays.fill(j.stopPos, -1);
        ArrayList<Integer> links = new ArrayList<>();

        if (j.segAnchors == null || j.segAnchors.length == 0) {
            // whole route off-model: one chain through all stops
            chain(j, links, rangeInts(0, nStops), -1, -1,
                    r.stops.get(0).dep, r.stops.get(nStops - 1).arr, sched, net, ext);
            j.links = toInt(links);
            j.stopPos = fixPositions(j.stopPos, j.links.length);
            if (j.links.length == 0) throw new IllegalStateException("empty route " + r.id);
            statRoutesWithChain++;
            return;
        }

        int prevStop = -1;
        for (int s = 0; s < j.segAnchors.length; s++) {
            int[] a = j.segAnchors[s];
            int first = a[0], last = a[a.length - 1];
            Router.Plan p = j.plans[s];
            if (p.core.length == 0) throw new IllegalStateException("empty core " + r.id);

            if (s > 0 || first > 0) {
                int prevLink = prevStop >= 0 ? links.get(j.stopPos[prevStop]) : -1;
                double tStart = prevStop >= 0 ? r.stops.get(prevStop).dep : r.stops.get(first).dep;
                chain(j, links, rangeInts(prevStop + 1, first), prevLink, p.core[0],
                        tStart, r.stops.get(first).arr, sched, net, ext);
            }
            int offset = links.size();
            for (int x : p.core) links.add(x);
            for (int k = 0; k < a.length; k++) {
                j.stopPos[a[k]] = p.stopCorePos[a[k]] + offset;
                if (j.stopPos[a[k]] < 0) throw new IllegalStateException("stop without core position " + r.id + " " + a[k]);
            }
            prevStop = last;
        }
        if (prevStop + 1 < nStops) {
            int prevLink = links.get(j.stopPos[prevStop]);
            chain(j, links, rangeInts(prevStop + 1, nStops), prevLink, -1,
                    r.stops.get(prevStop).dep, r.stops.get(nStops - 1).arr, sched, net, ext);
        }
        if (j.nChain > 0) statRoutesWithChain++;
        j.links = toInt(links);
        j.stopPos = fixPositions(j.stopPos, j.links.length);
    }

    /**
     * Append an off-model chain connecting the link {@code prevLink} (the link the train is on, i.e. the
     * last real link before the chain; -1 at the route start) through the off-model stops
     * {@code stops} to the real link {@code nextCoreLink} (-1 at the route end).
     *
     * <p>Every off-model stop gets its own off-model node and the link that ARRIVES at it as stop link
     * (railsim stops at the end of the facility link). A route that starts off-model gets a 1 m
     * departure stub, because the train is placed at the end of its start link.
     */
    static void chain(Job j, ArrayList<Integer> links, int[] stops, int prevLink, int nextCoreLink,
                      double tStart, double tEnd, Sched sched, Net net, ExtBuilder ext) {
        Sched.Route r = j.route;
        int n = stops.length;
        boolean startHere = prevLink < 0;
        boolean endHere = nextCoreLink < 0;
        if (n == 0 && (startHere || endHere)) throw new IllegalStateException("empty chain at route end " + r.id);
        int nPts = n + 1 + (endHere ? 0 : 1);   // (stub | attachFrom) + stops + [attachTo]
        String[] key = new String[nPts];
        double[] px = new double[nPts], py = new double[nPts];
        double[] tIn = new double[nPts], tOut = new double[nPts];
        int[] stopIdx = new int[nPts];
        Arrays.fill(stopIdx, -1);

        int q = 0;
        if (startHere) {
            // 1 m departure stub: the train is placed at the END of its start link
            Sched.StopF sf = sched.stops.get(r.stops.get(stops[0]).stopId);
            key[q] = "D:" + sf.id; px[q] = sf.x; py[q] = sf.y;
            tIn[q] = tOut[q] = tStart;
            q++;
        } else {
            int node = net.lTo[prevLink];
            key[q] = "N:" + net.nodeId[node]; px[q] = net.nx[node]; py[q] = net.ny[node];
            tIn[q] = tOut[q] = tStart;
            q++;
        }
        for (int i = 0; i < n; i++) {
            int si = stops[i];
            Sched.StopF sf = sched.stops.get(r.stops.get(si).stopId);
            key[q] = "S:" + sf.id; px[q] = sf.x; py[q] = sf.y;
            tIn[q] = r.stops.get(si).arr;
            tOut[q] = r.stops.get(si).dep;
            stopIdx[q] = si;
            q++;
        }
        if (!endHere) {
            int node = net.lFrom[nextCoreLink];
            key[q] = "N:" + net.nodeId[node]; px[q] = net.nx[node]; py[q] = net.ny[node];
            tIn[q] = tOut[q] = tEnd;
            q++;
        }

        int base = links.size();
        int first = startHere ? 1 : 1;   // index of the first hop
        if (startHere) {
            int e = ext.stub(key[0], px[0], py[0], key[1], px[1], py[1]);
            links.add(-1 - e);
            first = 2;
        }
        for (int i = first; i < nPts; i++) {
            double dt = tIn[i] - tOut[i - 1];
            int f = ext.offLink(key[i - 1], px[i - 1], py[i - 1], key[i], px[i], py[i],
                    offLength(key[i - 1], key[i], px[i - 1], py[i - 1], px[i], py[i]), dt);
            links.add(-1 - f);
        }
        for (int i = 1; i < nPts; i++) {
            if (stopIdx[i] >= 0) j.stopPos[stopIdx[i]] = base + (i - 1);
        }
        synchronized (TrackRouter.class) {
            statChains++;
            statChainStops += n;
            j.nChain++;
        }
    }

    /**
     * G2: the assembly of the merged router when the off-model model is off (Olten case). This is
     * agent G1's assemble() verbatim: one plan, prefix/suffix chains, a mid-route stop without a
     * track candidate snapped to the nearest link after its predecessor. Keeping this code path
     * separate is what makes the merged router's Olten output identical to swarm/r2/G1/out/final.
     */
    static void assembleG1(Job j, Sched sched, Net net, ExtBuilder ext) {
        Sched.Route r = j.route;
        int nStops = r.stops.size();
        // G2: R9 replaces j.plans, so the assembly must read the plan from there (G1 updated j.plan
        // directly; reading the stale j.plan re-assembled the pre-R9 path of 27 routes)
        Router.Plan p = (j.plans != null && j.plans.length > 0) ? j.plans[0] : j.plan;
        j.stopPos = new int[nStops];
        Arrays.fill(j.stopPos, -1);

        ArrayList<Integer> links = new ArrayList<>();
        int[] pre = p.prefixStops, suf = p.suffixStops;

        if (p.core.length == 0) {
            int[] all = new int[nStops];
            for (int i = 0; i < nStops; i++) all[i] = i;
            chainG1(all, 0, nStops, -1, 0, 0, r, sched, net, ext, links, j.stopPos);
            j.links = toInt(links);
            j.stopPos = fixPositions(j.stopPos, j.links.length);
            return;
        }

        int coreOffset = 0;
        if (pre.length > 0) {
            int attach = net.lFrom[p.core[0]];
            double attachTime = r.stops.get(p.prefixStops.length).arr;
            chainG1(pre, 0, pre.length, attach, 0, attachTime, r, sched, net, ext, links, j.stopPos);
            coreOffset = links.size();
        }
        for (int x : p.core) links.add(x);
        for (int i = 0; i < nStops; i++) if (p.stopCorePos[i] >= 0) j.stopPos[i] = p.stopCorePos[i] + coreOffset;

        int prevPos = -1;
        for (int i = 0; i < nStops; i++) {
            if (p.stopCorePos[i] >= 0) { prevPos = j.stopPos[i]; continue; }
            if (i < p.prefixStops.length) continue;
            if (p.suffixStops.length > 0 && i >= p.suffixStops[0]) continue;
            Sched.StopF sf = sched.stops.get(r.stops.get(i).stopId);
            int start = Math.max(0, prevPos);
            int best = Math.min(start + 1, links.size() - 1);
            double bd = Double.POSITIVE_INFINITY;
            for (int q = start; q < links.size(); q++) {
                int code = links.get(q);
                if (code < 0) continue;
                double d = net.distToLink(code, sf.x, sf.y);
                if (d < bd) { bd = d; best = q; }
            }
            j.stopPos[i] = best;
            prevPos = best;
            j.kind[i] = 2;
        }

        if (suf.length > 0) {
            int attach = net.lTo[p.core[p.core.length - 1]];
            double attachTime = r.stops.get(p.suffixStops[0] - 1).dep;
            chainG1(suf, 0, suf.length, attach, 1, attachTime, r, sched, net, ext, links, j.stopPos);
        }
        j.links = toInt(links);
    }

    /** G1's chain() verbatim (off-model prefix/suffix chain of the Olten path). */
    static void chainG1(int[] stops, int from, int to, int attach, int mode, double attachTime,
                        Sched.Route r, Sched sched, Net net, ExtBuilder ext,
                        ArrayList<Integer> links, int[] stopPos) {
        int n = to - from;
        int base = links.size();
        if (attach < 0) {
            double[] t = new double[n];
            String[] keys = new String[n];
            double[] xs = new double[n], ys = new double[n];
            for (int i = 0; i < n; i++) {
                int si = stops[from + i];
                Sched.StopF sf = sched.stops.get(r.stops.get(si).stopId);
                keys[i] = "S:" + sf.id; xs[i] = sf.x; ys[i] = sf.y;
                t[i] = i == 0 ? r.stops.get(si).dep : r.stops.get(si).arr;
            }
            for (int i = 0; i + 1 < n; i++) {
                int e = ext.link(keys[i], xs[i], ys[i], keys[i + 1], xs[i + 1], ys[i + 1], t[i + 1] - t[i]);
                links.add(-1 - e);
                stopPos[stops[from + i]] = base + Math.min(i, Math.max(0, n - 2));
            }
            stopPos[stops[from + n - 1]] = base + Math.max(0, n - 2);
            return;
        }
        int m = n + 1;
        String[] keys = new String[m];
        double[] xs = new double[m], ys = new double[m], t = new double[m];
        if (mode == 0) {
            for (int i = 0; i < n; i++) {
                int si = stops[from + i];
                Sched.StopF sf = sched.stops.get(r.stops.get(si).stopId);
                keys[i] = "S:" + sf.id; xs[i] = sf.x; ys[i] = sf.y;
                t[i] = r.stops.get(si).dep;
            }
            keys[n] = "N:" + net.nodeId[attach];
            xs[n] = net.nx[attach]; ys[n] = net.ny[attach];
            t[n] = attachTime;
        } else {
            keys[0] = "N:" + net.nodeId[attach];
            xs[0] = net.nx[attach]; ys[0] = net.ny[attach];
            t[0] = attachTime;
            for (int i = 0; i < n; i++) {
                int si = stops[from + i];
                Sched.StopF sf = sched.stops.get(r.stops.get(si).stopId);
                keys[i + 1] = "S:" + sf.id; xs[i + 1] = sf.x; ys[i + 1] = sf.y;
                t[i + 1] = r.stops.get(si).arr;
            }
        }
        for (int i = 0; i < m - 1; i++) {
            int e = ext.link(keys[i], xs[i], ys[i], keys[i + 1], xs[i + 1], ys[i + 1], t[i + 1] - t[i]);
            links.add(-1 - e);
        }
        for (int i = 0; i < n; i++) {
            int si = stops[from + i];
            if (mode == 0) stopPos[si] = base + i;
            else stopPos[si] = base + Math.min(i + 1, n - 1);
        }
    }

    static int[] toInt(ArrayList<Integer> l) {
        int[] a = new int[l.size()];
        for (int i = 0; i < a.length; i++) a[i] = l.get(i);
        return a;
    }

    static int[] fixPositions(int[] pos, int n) {
        for (int i = 0; i < pos.length; i++) if (pos[i] < 0) pos[i] = 0;
        return pos;
    }


    // ---------------------------------------------------------------- R7

    /** R7: compare the leg's free-flow running time with the scheduled time; if the fastest path is
     *  far too fast, search corridor alternatives and take the slowest one that still fits. */
    static final boolean DebugR7 = System.getenv("RT4B_DEBUG_R7") != null;

    static boolean setRelMask(byte[] mask, RelData rel, String routeId) {
        Integer relId = rel.routeRel.get(routeId);
        if (!relationsOn || relId == null) return false;
        RelData.Ways w = rel.relWays.get(relId);
        if (w == null) return false;
        for (int x : w.forward) if (x >= 0) mask[x] |= 1;
        for (int x : w.backward) if (x >= 0) mask[x] |= 2;
        return true;
    }

    static void clearRelMask(byte[] mask, RelData rel, String routeId) {
        Integer relId = rel.routeRel.get(routeId);
        if (relId == null) return;
        RelData.Ways w = rel.relWays.get(relId);
        if (w == null) return;
        for (int x : w.forward) if (x >= 0) mask[x] = 0;
        for (int x : w.backward) if (x >= 0) mask[x] = 0;
    }

    /** share of the path length that lies on the route's own OSM relation, in travel direction */
    static double relCoverage(int[] path, Net net, byte[] mask) {
        if (mask == null || path == null) return 0;
        double on = 0, tot = 0;
        for (int l : path) {
            int w = net.lWay[l];
            if (w < 0) continue;
            tot += net.lLen[l];
            int bit = net.fwd[l] ? 1 : 2;
            if ((mask[w] & bit) != 0) on += net.lLen[l];
        }
        return tot > 0 ? on / tot : 0;
    }

    static void applyCorridor(Job j, Router router, Router.Sear se, Sched sched, Net net, RelData rel) {
        if (j.plans == null) return;
        for (Router.Plan p : j.plans) applyCorridorOne(j, p, router, se, sched, net, rel);
    }

    /** length per OSM line identity: the line number tag `ref` (600 = Gotthard, 290 = Bern-Thun),
     *  or the usage class when the way carries no line number */
    static HashMap<String, Double> refLengths(int[] path, Net net) {
        HashMap<String, Double> m = new HashMap<>();
        for (int l : path) {
            int w = net.lWay[l];
            if (w < 0) continue;
            String r = net.wayLineRef[w];
            String key = (r == null || r.isEmpty()) ? "u" + net.wayUsage[w] : r;
            m.merge(key, (double) net.lLen[l], Double::sum);
        }
        return m;
    }

    static String refString(HashMap<String, Double> m) {
        ArrayList<Map.Entry<String, Double>> e = new ArrayList<>(m.entrySet());
        e.sort((x, y) -> Double.compare(y.getValue(), x.getValue()));
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < e.size() && i < 5; i++) {
            if (i > 0) b.append('|');
            b.append(e.get(i).getKey()).append(':').append(Math.round(e.get(i).getValue()));
        }
        return b.toString();
    }

    /** share of the alternative that runs on a line number which carries at least {@code minShare}
     *  of the direct path (a line that only touches the leg at its throat does not make it "the same line") */
    static double sameLineShare(int[] alt, int[] prev, Net net, double minShare) {
        HashMap<String, Double> a = refLengths(alt, net), p = refLengths(prev, net);
        double ptot = 0, tot = 0, same = 0;
        for (double v : p.values()) ptot += v;
        for (Map.Entry<String, Double> en : a.entrySet()) {
            tot += en.getValue();
            Double pv = p.get(en.getKey());
            if (pv != null && ptot > 0 && pv / ptot >= minShare) same += en.getValue();
        }
        return tot > 0 ? same / tot : 0;
    }

    static void r7cRow(Job j, int k, Sched.StopF s1, Sched.StopF s2, double schedT, double rt, double km,
                       double altRt, double altKm, double covPrev, double covAlt, double same, String dec,
                       int[] prev, int[] alt, Net net) {
        R7C.add(String.format(Locale.ROOT,
                "%s,%s,%d,%s,%s,%s,%s,%.0f,%.0f,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%s,%s,%s",
                j.line.name, j.route.id, k, j.route.stops.get(k).stopId, j.route.stops.get(k + 1).stopId,
                s1.name.replace(',', ' '), s2.name.replace(',', ' '), schedT, rt, km / 1000, altRt, altKm / 1000,
                covPrev, covAlt, same, dec,
                prev == null ? "" : refString(refLengths(prev, net)),
                alt == null ? "" : refString(refLengths(alt, net))));
    }

    static void applyCorridorOne(Job j, Router.Plan p, Router router, Router.Sear se, Sched sched, Net net,
                                 RelData rel) {
        int[] core = p.core;
        int[] pos = p.stopCorePos;
        if (core.length == 0) return;
        int nStops = j.route.stops.size();
        for (int k = nStops - 2; k >= 0; k--) {
            int a = pos[k], b = pos[k + 1];
            if (a < 0 || b < 0 || b <= a) continue;
            double km = 0, rt = 0;
            for (int q = a + 1; q <= b; q++) {
                km += net.lLen[core[q]];
                double sp = Math.min(net.lRouteSpeed[core[q]], baseParams.vmaxMs);
                rt += net.lLen[core[q]] / Math.max(sp, 0.5);
            }
            double schedT = j.route.stops.get(k + 1).arr - j.route.stops.get(k).dep;
            if (km <= corridorMinKm * 1000 || rt >= corridorRatio * schedT) continue;
            Sched.StopF s1 = sched.stops.get(j.route.stops.get(k).stopId);
            Sched.StopF s2 = sched.stops.get(j.route.stops.get(k + 1).stopId);
            int[] prev = Arrays.copyOfRange(core, a, b + 1);
            ArrayList<String> log = new ArrayList<>();
            // G1: set the R8 relation mask for this route so the corridor search and the choice below
            // know which ways the route's own OSM relation uses.
            byte[] mask = router.relMask();
            boolean relSet = setRelMask(mask, rel, j.route.id);
            int[] alt;
            double covPrev, covAlt;
            try {
                covPrev = relCoverage(prev, net, mask);
                alt = router.bestCorridor(core[a], core[b], prev, s1.x, s1.y, s2.x, s2.y,
                        schedT, corridorRadius, corridorFactor, se, log);
                covAlt = alt == null ? 0 : relCoverage(alt, net, mask);
            } finally {
                if (relSet) clearRelMask(mask, rel, j.route.id);
            }
            if (alt == null || alt.length < 2) {
                r7cRow(j, k, s1, s2, schedT, rt, km, 0, 0, covPrev, 0, 0, "NO_ALT", prev, null, net);
                continue;
            }
            double altKmG = 0, prevKmG = 0;
            for (int q = 1; q < alt.length; q++) altKmG += net.lLen[alt[q]];
            for (int q = 1; q < prev.length; q++) prevKmG += net.lLen[prev[q]];
            double altRtG = 0;
            for (int q = 1; q < alt.length; q++) {
                double sp = Math.min(net.lRouteSpeed[alt[q]], baseParams.vmaxMs);
                altRtG += net.lLen[alt[q]] / Math.max(sp, 0.5);
            }
            // ---- R7c (G3): the alternative is only taken when it is the SAME railway line as the
            // direct path over most of its length (OSM track_ref). A different line that merely
            // connects the same two stations is a detour, not a corridor (Guerbetal 298 vs the
            // main line 290, Hauenstein mountain line 503 vs the base tunnel 500). The Gotthard
            // mountain line and the base tunnel are both line 600, so the booked mountain-line
            // time can be followed there.
            double same = sameLineShare(alt, prev, net, 0.10);
            String dec;
            if (altKmG > prevKmG * 1.7 + 2000) dec = "KEEP_ABSURD";
            else if (covAlt + 0.05 < covPrev && altKmG < prevKmG * 1.20) dec = "KEEP_RELATION";
            else if (same < r7cSameLine) dec = "KEEP_OTHER_LINE";
            else if (altRtG < rt * 1.20) dec = "KEEP_TOO_FAST";
            else if (altKmG < prevKmG * 1.08) dec = "KEEP_TOO_SHORT";
            else if (Math.abs(alt.length - (b - a + 1)) < 2 && altRtG <= rt * 1.02) dec = "KEEP_SAME";
            else dec = "TAKE";
            r7cRow(j, k, s1, s2, schedT, rt, km, altRtG, altKmG, covPrev, covAlt, same, dec, prev, alt, net);
            if (!"TAKE".equals(dec)) continue;
            if (alt[0] != core[a] || alt[alt.length - 1] != core[b]) {
                System.out.println("R7 WARNING: alternative endpoints do not match for " + j.route.id
                        + " leg " + k + " (" + alt[0] + "/" + core[a] + " .. " + alt[alt.length - 1] + "/" + core[b] + ") - skipped");
                continue;
            }
            double altKm = 0, altRt = 0;
            for (int q = 1; q < alt.length; q++) {
                altKm += net.lLen[alt[q]];
                double sp = Math.min(net.lRouteSpeed[alt[q]], baseParams.vmaxMs);
                altRt += net.lLen[alt[q]] / Math.max(sp, 0.5);
            }
            int[] ncore = new int[core.length - (b - a + 1) + alt.length];
            int w = 0;
            for (int q = 0; q < a; q++) ncore[w++] = core[q];
            for (int q = 0; q < alt.length; q++) ncore[w++] = alt[q];
            for (int q = b + 1; q < core.length; q++) ncore[w++] = core[q];
            int delta = alt.length - (b - a + 1);
            for (int q = 0; q < pos.length; q++) if (pos[q] >= b) pos[q] += delta;
            p.core = ncore;
            core = ncore;
            CORRIDOR_JOB[j.index] = true;
            CORRIDOR.add(String.format(Locale.ROOT,
                    "%s %s: %s -> %s  sched %.0f s, min %.0f -> %.0f s, %.1f -> %.1f km",
                    j.line.name, j.route.id, s1.name, s2.name, schedT, rt, altRt, km / 1000, altKm / 1000));
        }
    }

    // ---------------------------------------------------------------- R9

    static List<Conflicts.Train> trainList(Job[] arr) {
        ArrayList<Conflicts.Train> out = new ArrayList<>();
        for (Job j : arr) {
            if (j.plans == null || j.plans.length == 0) continue;
            int[] core = coreOf(j);
            if (core.length == 0) continue;
            int ns = j.route.stops.size();
            int[] stopPos = new int[ns];
            Arrays.fill(stopPos, -1);
            int off = 0;
            for (Router.Plan p : j.plans) {
                for (int i = 0; i < ns; i++) if (p.stopCorePos[i] >= 0) stopPos[i] = p.stopCorePos[i] + off;
                off += p.core.length;
            }
            for (Sched.Dep d : j.route.deps) {
                Conflicts.Train t = new Conflicts.Train();
                t.route = j.index;
                t.links = core;
                t.stopPos = stopPos;
                t.arr = new double[ns];
                t.dep = new double[ns];
                for (int i = 0; i < ns; i++) {
                    t.arr[i] = j.route.stops.get(i).arr;
                    t.dep[i] = j.route.stops.get(i).dep;
                }
                t.t0 = d.time;
                out.add(t);
            }
        }
        return out;
    }

    static int[] coreOf(Job j) {
        if (j.plans == null || j.plans.length == 0) return new int[0];
        int n = 0;
        for (Router.Plan p : j.plans) n += p.core.length;
        int[] a = new int[n];
        int q = 0;
        for (Router.Plan p : j.plans) {
            for (int x : p.core) a[q++] = x;
        }
        return a;
    }

    static double totalKm(Job[] arr, Net net) {
        double km = 0;
        for (Job j : arr) for (int l : coreOf(j)) km += net.lLen[l];
        return km / 1000;
    }

    /** reject a re-plan that grows too much or would traverse one directed link twice */
    static boolean acceptReplan(Job j, Router.Plan[] np, Net net) {
        if (np == null || np.length == 0) return false;
        HashSet<Integer> seen = new HashSet<>();
        for (Router.Plan p : np) {
            if (p == null || p.core.length == 0) return false;
            for (int l : p.core) if (!seen.add(l)) return false;
        }
        double oldKm = 0, newKm = 0;
        for (int l : coreOf(j)) oldKm += net.lLen[l];
        for (Router.Plan p : np) for (int l : p.core) newKm += net.lLen[l];
        if (newKm > oldKm * 1.02 + 2000) return false;
        // G1: no single leg may grow by more than 20 % + 1 km (stops a replan detouring on one leg)
        for (int s = 0; s < np.length && s < j.plans.length; s++) {
            int[] op = j.plans[s].stopCorePos, npp = np[s].stopCorePos;
            int[] oc = j.plans[s].core, nc = np[s].core;
            for (int k = 0; k + 1 < op.length && k + 1 < npp.length; k++) {
                if (op[k] < 0 || op[k + 1] < 0 || npp[k] < 0 || npp[k + 1] < 0) continue;
                double ok = 0, nk = 0;
                for (int q = op[k] + 1; q <= op[k + 1] && q < oc.length; q++) ok += net.lLen[oc[q]];
                for (int q = npp[k] + 1; q <= npp[k + 1] && q < nc.length; q++) nk += net.lLen[nc[q]];
                if (nk > ok * 1.2 + 1000) return false;
            }
        }
        return true;
    }

    static void negotiate(Job[] arr, Sched sched, Net net, Router.Params prm,
                          Map<String, Router.Cand> candCache, RelData rel, Stops stops) {
        Router.Params lp = prm.copy();
        Router router = new Router(net, lp);
        router.loopDir = LOOP_DIR;
        Router.Sear se = router.newSearcher();
        Conflicts conf = new Conflicts(net);
        String[] names = new String[arr.length];
        for (Job j : arr) names[j.index] = j.route.id;
        Router.Plan[][] best = new Router.Plan[arr.length][];
        for (int i = 0; i < arr.length; i++) best[i] = arr[i].plans;
        Conflicts.Result r0 = conf.analyze(trainList(arr), margin, true, 0, 0);
        int bestPairs = r0.pairsAll;
        double bestKm = totalKm(arr, net);
        int bestIter = 0;
        System.out.printf(Locale.ROOT, "R9 iter 0: headon_pairs %d (eligible %d, excl self %d), trains %d%n",
                r0.pairsAll, r0.pairsEligible, r0.pairsExclSelf, r0.trainsInvolved);
        NEG_LOG.add(String.format(Locale.ROOT, "0 %d %d %d", r0.pairsAll, r0.pairsEligible, r0.pairsExclSelf));
        for (int it = 1; it <= maxIter; it++) {
            Conflicts.Result res = conf.analyze(trainList(arr), margin, true, histDelta, it);
            for (int l = 0; l < net.nLinks; l++) {
                if (res.addPen[l] <= 0) continue;
                router.pen[l] = Math.min(400.0, router.pen[l] + res.addPen[l]);
            }
            System.out.printf(Locale.ROOT, "R9 iter %d: history on %d routes, conflicts %d (eligible %d)%n",
                    it, res.affectedRoutes.size(), res.pairsAll, res.pairsEligible);
            int nReplan = 0, nAccept = 0;
            for (Job j : arr) {
                if (!res.affectedRoutes.contains(j.index)) continue;
                if (r9Pin && CORRIDOR_JOB.length > j.index && CORRIDOR_JOB[j.index]) continue;
                nReplan++;
                Router.Plan[] np;
                try {
                    np = replanSegments(j, router, se, net, candCache, rel, stops, sched);
                } catch (RuntimeException e) {
                    continue;
                }
                if (acceptReplan(j, np, net)) { j.plans = np; nAccept++; }
            }
            Conflicts.Result r = conf.analyze(trainList(arr), margin, true, 0, it);
            double km = totalKm(arr, net);
            System.out.printf(Locale.ROOT, "R9 iter %d: replanned %d, accepted %d -> headon_pairs %d (eligible %d, excl self %d), trains %d, km %.1f%n",
                    it, nReplan, nAccept, r.pairsAll, r.pairsEligible, r.pairsExclSelf, r.trainsInvolved, km);
            NEG_LOG.add(String.format(Locale.ROOT, "%d %d %d %d", it, r.pairsAll, r.pairsEligible, r.pairsExclSelf));
            if (r.pairsAll < bestPairs || (r.pairsAll == bestPairs && km < bestKm)) {
                bestPairs = r.pairsAll;
                bestKm = km;
                bestIter = it;
                for (int i = 0; i < arr.length; i++) best[i] = arr[i].plans;
            }
        }
        for (int i = 0; i < arr.length; i++) arr[i].plans = best[i];
        System.out.printf(Locale.ROOT, "R9 kept iteration %d: headon_pairs %d, km %.1f%n", bestIter, bestPairs, bestKm);
    }

    static Router.Plan[] replanSegments(Job j, Router router, Router.Sear se, Net net,
                                        Map<String, Router.Cand> candCache, RelData rel, Stops stops, Sched sched) {
        if (j.segAnchors == null || j.segAnchors.length == 0) return null;
        Router.Plan[] out = new Router.Plan[j.segAnchors.length];
        for (int s = 0; s < j.segAnchors.length; s++) {
            int[] a = j.segAnchors[s];
            out[s] = solveAnchors(j, a, 0, j.route.stops.size(), router, se, net, candCache, rel, sched);
        }
        return out;
    }

    // ---------------------------------------------------------------- relations
    static final class RelData {
        HashMap<String, Integer> routeRel = new HashMap<>();
        HashMap<Integer, Ways> relWays = new HashMap<>();

        static final class Ways {
            int[] forward = new int[0], backward = new int[0];
        }

        static RelData load(String matchFile, String relWaysFile, Net net, double strict) throws IOException {
            RelData rd = new RelData();
            HashSet<Integer> need = new HashSet<>();
            try (BufferedReader br = new BufferedReader(new InputStreamReader(Net.open(matchFile), StandardCharsets.UTF_8))) {
                String line = br.readLine();
                String[] h = Net.csv(line);
                int iR = col(h, "transit_route_id"), iRel = col(h, "rel_id"), iScore = col(h, "match_score"),
                        iDir = col(h, "direction_ok"), iGood = col(h, "good");
                while ((line = br.readLine()) != null) {
                    if (line.isEmpty()) continue;
                    String[] c = Net.splitSimple(line);
                    if (!c[iGood].equals("1")) continue;
                    double sc = Double.parseDouble(c[iScore]);
                    if (sc < strict || !c[iDir].equals("1")) continue;
                    int rid = Integer.parseInt(c[iRel]);
                    rd.routeRel.put(c[iR], rid);
                    need.add(rid);
                }
            }
            HashMap<Integer, ArrayList<Integer>> fw = new HashMap<>(), bw = new HashMap<>();
            try (BufferedReader br = new BufferedReader(new InputStreamReader(Net.open(relWaysFile), StandardCharsets.UTF_8))) {
                String line = br.readLine();
                String[] h = Net.csv(line);
                int iRel = col(h, "rel_id"), iW = col(h, "way_id"), iD = col(h, "travel_dir"), iSeq = col(h, "seq");
                while ((line = br.readLine()) != null) {
                    if (line.isEmpty()) continue;
                    String[] c = Net.splitSimple(line);
                    int rid;
                    try { rid = Integer.parseInt(c[iRel]); } catch (NumberFormatException e) { continue; }
                    if (!need.contains(rid)) continue;
                    int w = net.wayIndexPublic(Long.parseLong(c[iW]));
                    if (w < 0) continue;
                    String d = c[iD];
                    if (d.equals("forward")) fw.computeIfAbsent(rid, k -> new ArrayList<>()).add(w);
                    else if (d.equals("backward")) bw.computeIfAbsent(rid, k -> new ArrayList<>()).add(w);
                }
            }
            for (int rid : need) {
                Ways w = new Ways();
                ArrayList<Integer> a = fw.get(rid), b = bw.get(rid);
                if (a != null) { int[] x = new int[a.size()]; for (int i = 0; i < x.length; i++) x[i] = a.get(i); w.forward = x; }
                if (b != null) { int[] x = new int[b.size()]; for (int i = 0; i < x.length; i++) x[i] = b.get(i); w.backward = x; }
                rd.relWays.put(rid, w);
            }
            return rd;
        }

        static int col(String[] h, String name) {
            for (int i = 0; i < h.length; i++) if (h[i].equals(name)) return i;
            throw new IllegalStateException("no column " + name);
        }
    }

    // ---------------------------------------------------------------- stats
    static final class RouteStat {
        String line, route;
        int nDep, nStops, nPlat, nOff, nRev, nRelaxed, nSeg, nChain, nOffStops;
        double pathKm, beelineKm, wrongKm, serviceKm, minRuntime, schedRuntime, offKm;
        double[] legMin = new double[0], legSched = new double[0], legKm = new double[0];
        int[] legFrom = new int[0], legTo = new int[0];
        String[] stopIds = new String[0];
    }

    static final class Stats {
        ArrayList<RouteStat> routes = new ArrayList<>();
    }

    static Stats analyse(Job[] arr, Sched sched, Net net, ExtBuilder ext) {
        Stats st = new Stats();
        for (Job j : arr) {
            RouteStat rs = new RouteStat();
            rs.line = j.line.name;
            rs.route = j.route.id;
            rs.nDep = j.route.deps.size();
            rs.nStops = j.route.stops.size();
            rs.nSeg = j.segAnchors == null ? 0 : j.segAnchors.length;
            rs.nChain = j.nChain;
            int[] L = j.links;
            double path = 0, wrong = 0, service = 0, minrt = 0, off = 0;
            for (int code : L) {
                if (code < 0) { double l = ext.links.get(-1 - code).length; path += l; off += l; continue; }
                path += net.lLen[code];
                minrt += net.lLen[code] / Math.min(net.lRouteSpeed[code], paramsVmax());
                int w = net.lWay[code];
                if (w < 0) continue;
                byte k = net.wayKlass[w];
                if (k == Net.K_YARD || k == Net.K_SPUR || k == Net.K_SIDING) service += net.lLen[code];
                if (net.wayDirRaw[w] != 0 && net.wayDirConfRaw[w] >= baseParams.dirConfMin) {
                    int dir = net.fwd[code] ? 1 : -1;
                    if (dir != net.wayDirRaw[w]) wrong += net.lLen[code];
                }
            }
            rs.pathKm = path / 1000; rs.wrongKm = wrong / 1000; rs.serviceKm = service / 1000;
            rs.offKm = off / 1000;
            rs.minRuntime = minrt;
            Sched.StopF a = sched.stops.get(j.route.stops.get(0).stopId);
            Sched.StopF b = sched.stops.get(j.route.stops.get(j.route.stops.size() - 1).stopId);
            rs.beelineKm = Math.hypot(a.x - b.x, a.y - b.y) / 1000;
            rs.schedRuntime = j.route.stops.get(j.route.stops.size() - 1).arr - j.route.stops.get(0).dep;
            rs.stopIds = new String[j.route.stops.size()];
            for (int i = 0; i < j.route.stops.size(); i++) rs.stopIds[i] = j.route.stops.get(i).stopId;
            for (int i = 0; i < j.stopPos.length; i++) {
                if (j.kind[i] == 0) rs.nPlat++;
                else if (j.kind[i] == 2) rs.nOff++;
                Sched.StopRef sr = j.route.stops.get(i);
                if (i > 0) {
                    int p0 = j.stopPos[i - 1], p1 = j.stopPos[i];
                    if (p1 < p0) { int t = p0; p0 = p1; p1 = t; }
                    double m = 0;
                    for (int q = p0 + 1; q <= p1 && q < L.length; q++) {
                        int code = L[q];
                        m += code < 0 ? ext.links.get(-1 - code).length / Math.min(ext.links.get(-1 - code).speed, paramsVmax())
                                : net.lLen[code] / Math.min(net.lRouteSpeed[code], paramsVmax());
                    }
                    double sch = sr.arr - j.route.stops.get(i - 1).dep;
                    double km = 0;
                    for (int q = p0 + 1; q <= p1 && q < L.length; q++) {
                        int code = L[q];
                        km += code < 0 ? ext.links.get(-1 - code).length : net.lLen[code];
                    }
                    rs.legKm = append(rs.legKm, km / 1000.0);
                    rs.legMin = append(rs.legMin, m);
                    rs.legSched = append(rs.legSched, sch);
                    rs.legFrom = append(rs.legFrom, i - 1);
                    rs.legTo = append(rs.legTo, i);
                }
            }
            for (int i = 0; i + 1 < L.length; i++) {
                int x = L[i], y = L[i + 1];
                if (x >= 0 && y >= 0 && net.rev[x] == y) {
                    boolean atStop = false;
                    for (int p : j.stopPos) if (p == i) { atStop = true; break; }
                    if (atStop) rs.nRev++;
                }
            }
            rs.nRelaxed = (j.plans == null || j.plans.length == 0) ? 0 : j.plans[0].nRelaxed;
            st.routes.add(rs);
        }
        return st;
    }

    static double paramsVmax() { return baseParams.vmaxMs; }

    static double[] append(double[] a, double v) {
        double[] b = Arrays.copyOf(a, a.length + 1); b[a.length] = v; return b;
    }

    static int[] append(int[] a, int v) {
        int[] b = Arrays.copyOf(a, a.length + 1); b[a.length] = v; return b;
    }

    // ---------------------------------------------------------------- output
    static void writeAll(Job[] arr, Sched sched, Net net, ExtBuilder ext, Stats st) throws IOException {
        new File(outDir).mkdirs();
        // -------- report csv
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/routing_report.csv"), StandardCharsets.UTF_8))) {
            pw.println("line,route,n_departures,n_stops,n_stops_platform_matched,n_offnetwork_stops,path_km,beeline_km,n_reversals,wrongdir_km,service_km,n_relaxed_legs,min_runtime_s,scheduled_runtime_s,n_segments,n_infeasible_split,offmodel_km");
            for (RouteStat r : st.routes) {
                pw.printf(Locale.ROOT, "%s,%s,%d,%d,%d,%d,%.3f,%.3f,%d,%.3f,%.3f,%d,%.1f,%.1f,%d,%d,%.3f%n",
                        r.line, r.route, r.nDep, r.nStops, r.nPlat, r.nOff, r.pathKm, r.beelineKm,
                        r.nRev, r.wrongKm, r.serviceKm, r.nRelaxed, r.minRuntime, r.schedRuntime,
                        r.nSeg, r.nChain, r.offKm);
            }
        }
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/legs.csv"), StandardCharsets.UTF_8))) {
            pw.println("line,route,from_stop,to_stop,min_runtime_s,scheduled_s");
            for (RouteStat r : st.routes)
                for (int i = 0; i < r.legMin.length; i++)
                    pw.printf(Locale.ROOT, "%s,%s,%d,%d,%.1f,%.1f%n", r.line, r.route, r.legFrom[i], r.legTo[i], r.legMin[i], r.legSched[i]);
        }
        // -------- leg table with stop ids and km (G1)
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/legs_ids.csv"), StandardCharsets.UTF_8))) {
            pw.println("line,route,from_stop,to_stop,min_runtime_s,scheduled_s,km");
            for (RouteStat r : st.routes)
                for (int i = 0; i < r.legMin.length; i++)
                    pw.printf(Locale.ROOT, "%s,%s,%s,%s,%.1f,%.1f,%.3f%n", r.line, r.route,
                            r.stopIds[r.legFrom[i]], r.stopIds[r.legTo[i]], r.legMin[i], r.legSched[i], r.legKm[i]);
        }
        // -------- route links (for the sanity checks)
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/route_links.csv"), StandardCharsets.UTF_8))) {
            pw.println("route,seq,link,way_id,length");
            for (Job j : arr) {
                for (int i = 0; i < j.links.length; i++) {
                    int c = j.links[i];
                    if (c >= 0) pw.printf(Locale.ROOT, "%s,%d,%d,%d,%.2f%n", j.route.id, i, net.linkId[c],
                            net.lWay[c] >= 0 ? net.wayId[net.lWay[c]] : -1, net.lLen[c]);
                    else pw.printf(Locale.ROOT, "%s,%d,%s,0,%.2f%n", j.route.id, i, ext.links.get(-1 - c).id, ext.links.get(-1 - c).length);
                }
            }
        }
        // -------- off-model stop / chain table (documentation)
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/offmodel.csv"), StandardCharsets.UTF_8))) {
            pw.println("line,route,n_stops,n_offmodel_stops,n_segments,n_infeasible_split,offmodel_km,path_km");
            for (RouteStat r : st.routes)
                pw.printf(Locale.ROOT, "%s,%s,%d,%d,%d,%d,%.3f,%.3f%n",
                        r.line, r.route, r.nStops, r.nOff, r.nSeg, r.nChain, r.offKm, r.pathKm);
        }
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(outDir + "/offmodel_legs.txt"), StandardCharsets.UTF_8))) {
            pw.println("# real legs replaced by off-model track because the real path cannot keep the timetable");
            for (String s : INFEASIBLE_EXAMPLES) pw.println(s);
        }

        // -------- network
        boolean[] used = new boolean[net.nLinks];
        for (Job j : arr) for (int c : j.links) if (c >= 0) { used[c] = true; if (net.rev[c] >= 0) used[net.rev[c]] = true; }
        ArrayList<Integer> all = new ArrayList<>();
        for (int l = 0; l < net.nLinks; l++) if (used[l]) all.add(l);
        LinkedHashSet<Integer> usedNodes = new LinkedHashSet<>();
        for (int l : all) { usedNodes.add(net.lFrom[l]); usedNodes.add(net.lTo[l]); }

        Network onet = NetworkUtils.createNetwork();
        HashMap<Integer, Node> nmap = new HashMap<>();
        for (int v : usedNodes)
            nmap.put(v, NetworkUtils.createAndAddNode(onet, Id.create(Long.toString(net.nodeId[v]), Node.class),
                    new Coord(net.nx[v], net.ny[v])));
        for (int l : all) {
            Link lk = NetworkUtils.createAndAddLink(onet, Id.create(Long.toString(net.linkId[l]), Link.class),
                    nmap.get(net.lFrom[l]), nmap.get(net.lTo[l]), net.lLen[l], net.lRouteSpeed[l], 9999.0, 1.0);
            lk.setAllowedModes(Collections.singleton("rail"));
            if (net.lWay[l] >= 0) lk.getAttributes().putAttribute("osm:way:id", net.wayId[net.lWay[l]]);
        }
        HashMap<Integer, Node> enodes = new HashMap<>();
        for (int i = 0; i < ext.nodeIds.size(); i++) {
            double[] xy = ext.nodeXY.get(i);
            enodes.put(-1 - i, NetworkUtils.createAndAddNode(onet, Id.create(ext.nodeIds.get(i), Node.class),
                    new Coord(xy[0], xy[1])));
        }
        for (ExtLink e : ext.links) {
            Node f = e.fromRef >= 0 ? nmap.get(e.fromRef) : enodes.get(e.fromRef);
            Node t = e.toRef >= 0 ? nmap.get(e.toRef) : enodes.get(e.toRef);
            if (f == null || t == null) continue;
            Link lk = NetworkUtils.createAndAddLink(onet, Id.create(e.id, Link.class), f, t,
                    Math.max(0.1, e.length), e.speed, 9999.0, 1.0);
            lk.setAllowedModes(Collections.singleton("rail"));
            lk.getAttributes().putAttribute("boundaryExtension", Boolean.TRUE);
            lk.getAttributes().putAttribute("offModel", Boolean.TRUE);
        }
        new NetworkWriter(onet).write(outDir + "/transitNetwork.xml.gz");

        // -------- schedule
        TransitScheduleFactory fac = new TransitScheduleFactoryImpl();
        TransitSchedule sch = fac.createTransitSchedule();
        HashMap<String, TransitStopFacility> facCache = new HashMap<>();
        for (Job j : arr) {
            Sched.Route r = j.route;
            for (int i = 0; i < r.stops.size(); i++) {
                Sched.StopRef sr = r.stops.get(i);
                Sched.StopF sf = sched.stops.get(sr.stopId);
                int code = j.links[Math.max(0, Math.min(j.links.length - 1, j.stopPos[i]))];
                String lid = code >= 0 ? Long.toString(net.linkId[code]) : ext.links.get(-1 - code).id;
                String fid = sr.stopId + ".link:" + lid;
                if (!facCache.containsKey(fid)) {
                    TransitStopFacility f = fac.createTransitStopFacility(
                            Id.create(fid, TransitStopFacility.class), new Coord(sf.x, sf.y), sf.blocking);
                    f.setName(sf.name);
                    f.setLinkId(Id.create(lid, Link.class));
                    if (sf.area != null) f.setStopAreaId(Id.create(sf.area, TransitStopArea.class));
                    facCache.put(fid, f);
                    sch.addStopFacility(f);
                }
            }
        }
        for (Job j : arr) {
            Sched.Route r = j.route;
            TransitLine tl = sch.getTransitLines().get(Id.create(j.line.id, TransitLine.class));
            if (tl == null) {
                tl = fac.createTransitLine(Id.create(j.line.id, TransitLine.class));
                tl.setName(j.line.name);
                sch.addTransitLine(tl);
            }
            ArrayList<TransitRouteStop> trs = new ArrayList<>();
            for (int i = 0; i < r.stops.size(); i++) {
                Sched.StopRef sr = r.stops.get(i);
                Sched.StopF sf = sched.stops.get(sr.stopId);
                int code = j.links[Math.max(0, Math.min(j.links.length - 1, j.stopPos[i]))];
                String lid = code >= 0 ? Long.toString(net.linkId[code]) : ext.links.get(-1 - code).id;
                TransitStopFacility f = facCache.get(sr.stopId + ".link:" + lid);
                TransitRouteStop ts = fac.createTransitRouteStop(f, sr.arr, sr.dep);
                ts.setAwaitDepartureTime(sr.await);
                trs.add(ts);
            }
            ArrayList<Id<Link>> mids = new ArrayList<>();
            for (int i = 1; i + 1 < j.links.length; i++) mids.add(Id.create(linkIdOf(j.links[i], net, ext), Link.class));
            NetworkRoute nr = RouteUtils.createLinkNetworkRouteImpl(
                    Id.create(linkIdOf(j.links[0], net, ext), Link.class),
                    mids,
                    Id.create(linkIdOf(j.links[j.links.length - 1], net, ext), Link.class));
            TransitRoute tr = fac.createTransitRoute(Id.create(r.id, TransitRoute.class), nr, trs, r.mode);
            for (Sched.Dep d : r.deps) {
                Departure dep = fac.createDeparture(Id.create(d.id, Departure.class), d.time);
                dep.setVehicleId(Id.createVehicleId(d.veh));
                tr.addDeparture(dep);
            }
            tl.addRoute(tr);
        }
        new TransitScheduleWriter(sch).writeFile(outDir + "/transitSchedule.xml.gz");

        // -------- vehicles
        Vehicles veh = org.matsim.vehicles.VehicleUtils.createVehiclesContainer();
        VehiclesFactory vf = veh.getFactory();
        VehicleType vt = vf.createVehicleType(Id.create("rail", VehicleType.class));
        vt.setMaximumVelocity(100);
        vt.setLength(250);
        vt.setNetworkMode("rail");
        veh.addVehicleType(vt);
        HashSet<String> seen = new HashSet<>();
        for (Job j : arr) for (Sched.Dep d : j.route.deps)
            if (seen.add(d.veh)) veh.addVehicle(vf.createVehicle(Id.createVehicleId(d.veh), vt));
        new MatsimVehicleWriter(veh).writeFile(outDir + "/transitVehicles.xml.gz");

        // -------- console summary
        summary(st, arr, ext);
    }

    static String linkIdOf(int code, Net net, ExtBuilder ext) {
        return code >= 0 ? Long.toString(net.linkId[code]) : ext.links.get(-1 - code).id;
    }

    static void summary(Stats st, Job[] arr, ExtBuilder ext) {
        int nRev = 0, nRelax = 0, nOff = 0, nStops = 0, nPlat = 0, nSeg = 0, nRouteSeg = 0;
        double km = 0, wrong = 0, service = 0, offKm = 0;
        for (RouteStat r : st.routes) {
            nRev += r.nRev; nRelax += r.nRelaxed; nOff += r.nOff;
            nStops += r.nStops; nPlat += r.nPlat;
            km += r.pathKm; wrong += r.wrongKm; service += r.serviceKm; offKm += r.offKm;
            nSeg += r.nSeg;
            if (r.nSeg > 1) nRouteSeg++;
        }
        System.out.printf(Locale.ROOT, "routes %d, stops %d, platform matched %d (%.2f%%), off-model %d (%.2f%%)%n",
                st.routes.size(), nStops, nPlat, 100.0 * nPlat / Math.max(1, nStops), nOff, 100.0 * nOff / Math.max(1, nStops));
        System.out.printf(Locale.ROOT, "path %.1f km, off-model %.1f km (%.2f%% of path), wrongdir %.2f km (%.2f%%), service %.2f km, reversals %d, relaxed legs %d%n",
                km, offKm, 100 * offKm / Math.max(1e-9, km), wrong, 100 * wrong / Math.max(1e-9, km), service, nRev, nRelax);
        System.out.printf(Locale.ROOT, "off-model model: %d routes fully off-model, %d routes with a chain, %d segments (%d routes split), %d chains, %d off-model stops in chains, %d infeasible real legs -> off-model (%d kept), %d real stops without candidate%n",
                statRoutesFullyOff, statRoutesWithChain, nSeg, nRouteSeg, statChains, statChainStops,
                statInfeasibleSplit, statInfeasibleKept, statRealNoCand);
        System.out.printf(Locale.ROOT, "off-model links: %d hops (+%d departure stubs), %.1f km, mean freespeed %.1f km/h (min %.1f, max %.1f), %d hops limited by the %.0f km/h cap%n",
                statOffHops, statStubs, statOffKm, statOffSpeedSum / Math.max(1, statOffHops),
                statOffSpeedMin, statOffSpeedMax, statOffHopsCapped, offVmaxMs * 3.6);
        System.out.printf(Locale.ROOT, "network: %d nodes, %d links written; ext links %d%n",
                ext.nodeIds.size(), 0, ext.links.size());
    }
}
