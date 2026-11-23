package ch.sbb.prepare.router;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;

/**
 * G2 — passing loops on single-track lines.
 *
 * <p>On a single-track line the timetable makes opposing trains meet inside a passing loop (two
 * parallel tracks between two switches). The RT4b router sends every train over the same track of
 * such a loop and only writes the links a route uses into the network, so the second track does not
 * exist in the simulation and opposing trains cannot pass.
 *
 * <p>This class detects the loops from the OSM graph itself (no station list):
 * <ol>
 * <li>a <b>seed</b> is a pair of parallel links: same travel sense (cos &gt; 0.94), perpendicular
 *     distance of the midpoints &le; {@link #MAX_PAR} m, length ratio in [0.3, 3]. The seed's first
 *     link is used by a route, the partner may be used or not;</li>
 * <li>the partner is grown into a <b>track chain</b> B by walking along links that stay parallel to
 *     the used corridor (so a loop track that OSM models as several ways is kept whole);</li>
 * <li>track A = the used, non-stop links that have a parallel partner in B</li>
 * <li>the loop is only accepted when the line next to it is used in <b>both</b> travel directions
 *     (that is what makes it a passing loop on a single-track line, not one track of a double-track
 *     line) and when the direction assignment does not contradict the OSM/DIR direction tags.</li>
 * </ol>
 *
 * <p>Each direction then gets its own track: the track that the timetable already uses in a
 * direction keeps it; otherwise left-hand running decides (the train keeps to the left track); a
 * published platform track always wins (stop links are never reassigned). The router charges
 * {@link Router.Params#loopPenalty} for running against the assignment and drops the siding/service
 * penalty on the assigned track, so the path search uses the loop the way a railway does.
 */
public final class Loops {

    /** maximum perpendicular distance between the midpoints of two tracks of one loop */
    static final double MAX_PAR = 15.0;
    /** minimum cosine between the travel directions of two tracks of one loop (~20 deg) */
    static final double MIN_COS = 0.94;
    /** minimum perpendicular distance below which two links are the same track, not two tracks */
    static final double MIN_OFFSET = 1.5;
    /** maximum track spacing of a loop (two neighbouring lines are not a passing loop) */
    static final double MAX_SPACING = 12.0;
    /** maximum length of a passing loop / multi-track stretch [m] */
    static final double MAX_LOOP_LEN = 1600.0;

    public static final class Loop {
        public int[] a = new int[0];        // directed links of track A (travel sense = A)
        public int[] b = new int[0];        // directed links of track B (same travel sense)
        public boolean bLeft;               // B lies to the left of A in A's travel sense
        public double offsetM;              // median signed offset of B from A (+ = left)
        public double lenA, lenB;           // track lengths [m]
        public int useAFwd, useABwd, useBFwd, useBBwd;
        public boolean active;              // assignment written into loopDir
        public String why = "";             // why not active / how the direction was decided
        public long nodeA0, nodeA1;         // switch nodes of track A
        public String waysA = "", waysB = "";
        public String place = "";           // nearest stop name (report only)
        public double placeDist = -1;
        public int fwdIsB = 0;              // 1: forward trains use B, 0: A, -1: undecided
        public int idx;
    }

    final Net net;
    final int[] use;
    final boolean[] stopLink;
    public final ArrayList<Loop> loops = new ArrayList<>();
    /** per directed link: +1 = this direction is the assigned one, -1 = against it, 0 = not a loop */
    public byte[] dir;

    private final HashMap<Long, int[]> grid = new HashMap<>();
    private static final double CELL = 40.0;
    private final HashMap<Long, int[]> pairGrid = new HashMap<>();

    private Loops(Net net, int[] use, boolean[] stopLink) {
        this.net = net;
        this.use = use;
        this.stopLink = stopLink;
    }

    // ------------------------------------------------------------------ grid

    private static long key(int cx, int cy) {
        return ((long) cx << 32) ^ (cy & 0xffffffffL);
    }

    private void buildGrid() {
        HashMap<Long, ArrayList<Integer>> tmp = new HashMap<>();
        for (int l = 0; l < net.nLinks; l++) {
            long k = key(cell(net, l, 0), cell(net, l, 1));
            ArrayList<Integer> a = tmp.get(k);
            if (a == null) tmp.put(k, a = new ArrayList<>());
            a.add(l);
        }
        for (Map.Entry<Long, ArrayList<Integer>> e : tmp.entrySet()) {
            int[] v = new int[e.getValue().size()];
            for (int i = 0; i < v.length; i++) v[i] = e.getValue().get(i);
            grid.put(e.getKey(), v);
        }
    }

    private static int cell(Net n, int l, int i) {
        double x = (n.nx[n.lFrom[l]] + n.nx[n.lTo[l]]) * 0.5;
        double y = (n.ny[n.lFrom[l]] + n.ny[n.lTo[l]]) * 0.5;
        return (int) Math.floor((i == 0 ? x : y) / CELL);
    }

    /** unit direction of a link */
    private double[] unit(int l) {
        double dx = net.nx[net.lTo[l]] - net.nx[net.lFrom[l]];
        double dy = net.ny[net.lTo[l]] - net.ny[net.lFrom[l]];
        double d = Math.hypot(dx, dy);
        return d > 1e-9 ? new double[]{dx / d, dy / d} : new double[]{0, 0};
    }

    private double len(int l) { return net.lLen[l]; }

    /**
     * perpendicular distance of m's midpoint from the line through l, and the along-line offset
     * relative to l's midpoint. Returns {perp, along, cos}.
     */
    private double[] rel(int l, int m) {
        double[] u = unit(l);
        double mx = (net.nx[net.lFrom[m]] + net.nx[net.lTo[m]]) * 0.5 - (net.nx[net.lFrom[l]] + net.nx[net.lTo[l]]) * 0.5;
        double my = (net.ny[net.lFrom[m]] + net.ny[net.lTo[m]]) * 0.5 - (net.ny[net.lFrom[l]] + net.ny[net.lTo[l]]) * 0.5;
        double along = mx * u[0] + my * u[1];
        double perp = mx * -u[1] + my * u[0];        // signed: positive = to the left of u
        double[] v = unit(m);
        double c = u[0] * v[0] + u[1] * v[1];
        return new double[]{perp, along, c};
    }

    /**
     * Is m a parallel track of l (same travel sense, close, comparable length)? A link that is the
     * COLLINEAR CONTINUATION of l on the same track (offset ~0, along ~half a link) is not a parallel
     * track, which is why the along-offset is bounded by half the shorter link and the perpendicular
     * offset has a lower bound (two tracks of one loop lie side by side, 2-15 m apart).
     */
    private boolean parallel(int l, int m, double maxPar) {
        if (m == l || m == net.rev[l] || net.rev[l] < 0) return false;
        double ll = len(l), lm = len(m);
        if (lm < 0.3 * ll || lm > 3.0 * ll) return false;
        double[] r = rel(l, m);
        if (r[2] < MIN_COS) return false;
        double ap = Math.abs(r[0]);
        if (ap < MIN_OFFSET || ap > maxPar) return false;
        if (Math.abs(r[1]) > 0.6 * Math.min(ll, lm) + 3.0) return false;
        return true;
    }

    /** nearest parallel partner of l, or -1 */
    private int partner(int l) {
        int cx = cell(net, l, 0), cy = cell(net, l, 1);
        int best = -1;
        double bd = Double.POSITIVE_INFINITY;
        for (int ix = -1; ix <= 1; ix++)
            for (int iy = -1; iy <= 1; iy++) {
                int[] v = grid.get(key(cx + ix, cy + iy));
                if (v == null) continue;
                for (int m : v) {
                    if (!parallel(l, m, MAX_PAR)) continue;
                    double[] r = rel(l, m);
                    double d = Math.abs(r[0]);
                    if (d < bd) { bd = d; best = m; }
                }
            }
        return best;
    }

    /** is any used link (not l/rev[l]) parallel to x? (x is a candidate loop-track link) */
    private boolean nearUsed(int x, int l) {
        int cx = cell(net, x, 0), cy = cell(net, x, 1);
        for (int ix = -1; ix <= 1; ix++)
            for (int iy = -1; iy <= 1; iy++) {
                int[] v = grid.get(key(cx + ix, cy + iy));
                if (v == null) continue;
                for (int u : v) {
                    if (u == x || u == net.rev[x]) continue;
                    if (use[u] <= 0) continue;
                    if (parallel(u, x, MAX_PAR)) return true;
                }
            }
        return false;
    }

    // ------------------------------------------------------------------ detection

    public static Loops detect(Net net, int[] use, boolean[] stopLink, Sched sched) {
        Loops L = new Loops(net, use, stopLink);
        L.run(sched);
        return L;
    }

    private int nNotSingle = 0;

    private void run(Sched sched) {
        buildGrid();
        int nSeeds = 0;
        boolean[] done = new boolean[net.nLinks];      // links already part of an accepted loop
        for (int l = 0; l < net.nLinks; l++) {
            if (use[l] <= 0 || stopLink[l]) continue;
            if (done[l]) continue;
            int m = partner(l);
            if (m < 0) continue;
            if (stopLink[m]) continue;                  // a platform track always wins
            nSeeds++;
            Loop lp = build(l, m);
            if (lp == null) continue;
            // accept only if it is a loop on a SINGLE-track line: track A (the track the timetable
            // uses today) must itself be used in BOTH travel directions. On a double-track line each
            // track carries one direction only, and swapping them would send a train against traffic.
            if (!sharedTrack(lp)) { nNotSingle++; continue; }
            // direction assignment
            decide(lp);
            if (lp.active) {
                for (int x : lp.a) { done[x] = true; if (net.rev[x] >= 0) done[net.rev[x]] = true; }
                for (int x : lp.b) { done[x] = true; if (net.rev[x] >= 0) done[net.rev[x]] = true; }
            }
            loops.add(lp);
        }
        // write the assignment
        dir = new byte[net.nLinks];
        int nActive = 0;
        for (Loop lp : loops) {
            lp.idx = loops.indexOf(lp);
            if (!lp.active) continue;
            nActive++;
            boolean fwdB = lp.fwdIsB == 1;
            for (int a : lp.a) {
                dir[a] = (byte) (fwdB ? -1 : 1);
                if (net.rev[a] >= 0) dir[net.rev[a]] = (byte) (fwdB ? 1 : -1);
            }
            for (int b : lp.b) {
                dir[b] = (byte) (fwdB ? 1 : -1);
                if (net.rev[b] >= 0) dir[net.rev[b]] = (byte) (fwdB ? -1 : 1);
            }
        }
        System.out.printf(Locale.ROOT,
                "loops: %d parallel seeds, %d not on a single-track line (dropped), %d loops on single track, %d used for the assignment%n",
                nSeeds, nNotSingle, loops.size(), nActive);
        place(sched);
    }

    /** grow the seed into two tracks and validate */
    private Loop build(int l, int m) {
        ArrayList<Integer> b = new ArrayList<>();
        b.add(m);
        // ---- grow forward from m
        int cur = m;
        HashSet<Integer> seen = new HashSet<>();
        seen.add(m);
        while (true) {
            int nxt = step(cur, seen, l);
            if (nxt < 0) break;
            b.add(nxt);
            seen.add(nxt);
            cur = nxt;
        }
        // ---- grow backward from m
        cur = m;
        while (true) {
            int nxt = stepBack(cur, seen, l);
            if (nxt < 0) break;
            b.add(nxt);
            seen.add(nxt);
            cur = nxt;
        }
        // ---- track A: used, non-stop links parallel to some link of B
        ArrayList<Integer> a = new ArrayList<>();
        double[] u = unit(l);
        for (int x : b) {
            int cx = cell(net, x, 0), cy = cell(net, x, 1);
            for (int ix = -1; ix <= 1; ix++)
                for (int iy = -1; iy <= 1; iy++) {
                    int[] v = grid.get(key(cx + ix, cy + iy));
                    if (v == null) continue;
                    for (int y : v) {
                        if (use[y] <= 0 || stopLink[y] || a.contains(y)) continue;
                        if (!parallel(y, x, MAX_PAR)) continue;
                        double[] uy = unit(y);
                        if (uy[0] * u[0] + uy[1] * u[1] < 0.9) continue;
                        a.add(y);
                    }
                }
        }
        if (a.isEmpty() || b.isEmpty()) return null;
        Loop lp = new Loop();
        lp.a = toArray(a);
        lp.b = toArray(b);
        // mutual parallelism: every link of one track must have a partner on the other track
        for (int x : lp.a) {
            boolean ok = false;
            for (int y : lp.b) if (parallel(x, y, MAX_PAR)) { ok = true; break; }
            if (!ok) return null;
        }
        for (int y : lp.b) {
            boolean ok = false;
            for (int x : lp.a) if (parallel(x, y, MAX_PAR)) { ok = true; break; }
            if (!ok) return null;
        }
        for (int x : lp.a) lp.lenA += len(x);
        for (int x : lp.b) lp.lenB += len(x);
        if (lp.lenB > 1.35 * lp.lenA || lp.lenA > 1.35 * lp.lenB) return null;
        if (Math.max(lp.lenA, lp.lenB) > MAX_LOOP_LEN) return null;
        if (lp.a.length > 160 || lp.b.length > 160) return null;
        // signed offset (B from A) and usage votes
        double[] offs = new double[lp.b.length];
        for (int i = 0; i < lp.b.length; i++) {
            double best = Double.POSITIVE_INFINITY, s = 0;
            for (int x : lp.a) {
                double[] r = rel(x, lp.b[i]);
                if (Math.abs(r[0]) < best) { best = Math.abs(r[0]); s = r[0]; }
            }
            offs[i] = s;
        }
        Arrays.sort(offs);
        lp.offsetM = offs[offs.length / 2];
        if (Math.abs(lp.offsetM) < MIN_OFFSET || Math.abs(lp.offsetM) > MAX_SPACING) return null;
        lp.bLeft = lp.offsetM > 0;
        for (int x : lp.a) { lp.useAFwd += use[x]; if (net.rev[x] >= 0) lp.useABwd += use[net.rev[x]]; }
        for (int x : lp.b) { lp.useBFwd += use[x]; if (net.rev[x] >= 0) lp.useBBwd += use[net.rev[x]]; }
        lp.nodeA0 = net.nodeId[net.lFrom[lp.a[0]]];
        lp.nodeA1 = net.nodeId[net.lTo[lp.a[lp.a.length - 1]]];
        lp.waysA = ways(lp.a);
        lp.waysB = ways(lp.b);
        return lp;
    }

    private String ways(int[] ls) {
        HashSet<Long> s = new HashSet<>();
        for (int x : ls) if (net.lWay[x] >= 0) s.add(net.wayId[net.lWay[x]]);
        StringBuilder sb = new StringBuilder();
        for (long w : s) { if (sb.length() > 0) sb.append(';'); sb.append(w); }
        return sb.toString();
    }

    /** next link of the loop-track chain after {@code cur} (or -1) */
    private int step(int cur, HashSet<Integer> seen, int seed) {
        return stepGen(cur, seen, seed, true);
    }

    private int stepBack(int cur, HashSet<Integer> seen, int seed) {
        return stepGen(cur, seen, seed, false);
    }

    private int stepGen(int cur, HashSet<Integer> seen, int seed, boolean forward) {
        int node = forward ? net.lTo[cur] : net.lFrom[cur];
        int s = forward ? net.outStart[node] : net.inStart[node];
        int e = forward ? net.outStart[node + 1] : net.inStart[node + 1];
        int[] arr = forward ? net.outLinks : net.inLinks;
        int found = -1, nCand = 0;
        for (int q = s; q < e; q++) {
            int x = arr[q];
            if (x == net.rev[cur]) continue;
            if (seen.contains(x)) continue;
            if (stopLink[x]) continue;
            if (!nearUsed(x, seed)) continue;
            double[] r = rel(cur, x);
            if (r[2] < 0.8 && r[2] > -0.8) continue;          // must keep the travel sense
            nCand++;
            found = x;
        }
        return nCand == 1 ? found : -1;
    }

    /** is track A used in both travel directions (i.e. is this a single-track section)? */
    private boolean sharedTrack(Loop lp) {
        int n = 0, both = 0;
        for (int x : lp.a) {
            n++;
            if (use[x] > 0 && net.rev[x] >= 0 && use[net.rev[x]] > 0) both++;
        }
        return n > 0 && both * 2 >= n;
    }

    /** is a section next to the loop used in both travel directions? */
    private boolean adjacentBidirectional(Loop lp) {
        HashSet<Integer> in = new HashSet<>();
        for (int x : lp.a) in.add(x);
        for (int x : lp.b) in.add(x);
        for (int x : lp.a) {
            for (int side = 0; side < 2; side++) {
                int node = side == 0 ? net.lFrom[x] : net.lTo[x];
                for (int q = net.outStart[node]; q < net.outStart[node + 1]; q++) {
                    int y = net.outLinks[q];
                    if (in.contains(y) || net.rev[y] < 0) continue;
                    if (use[y] > 0 && use[net.rev[y]] > 0) return true;
                }
                for (int q = net.inStart[node]; q < net.inStart[node + 1]; q++) {
                    int y = net.inLinks[q];
                    if (in.contains(y) || net.rev[y] < 0) continue;
                    if (use[y] > 0 && use[net.rev[y]] > 0) return true;
                }
            }
        }
        return false;
    }

    /**
     * Which track serves which direction. The timetable wins (a track already used in one direction
     * keeps that direction), otherwise left-hand running decides; a stop link is never reassigned.
     */
    private void decide(Loop lp) {
        boolean aFwd = lp.useAFwd > 0, aBwd = lp.useABwd > 0;
        boolean bFwd = lp.useBFwd > 0, bBwd = lp.useBBwd > 0;
        if (aFwd && !aBwd && bBwd && !bFwd) { lp.active = false; lp.why = "already one track per direction"; return; }
        if ((aFwd && !aBwd && bFwd && !bBwd) || (aBwd && !aFwd && bBwd && !bFwd)) {
            lp.active = false; lp.why = "both tracks are used in the same direction"; return;
        }
        boolean fwdB;
        if (bFwd && !bBwd) { fwdB = true; lp.why = "track B already used in this direction"; }
        else if (aFwd && !aBwd) { fwdB = false; lp.why = "track A already used in this direction"; }
        else fwdB = lp.bLeft ? true : false;   // left-hand running: the forward train keeps to the left track
        if (!lp.why.isEmpty() && lp.why.startsWith("track")) { /* keep */ }
        else lp.why = lp.bLeft ? "left-hand running (B is the left track)" : "left-hand running (A is the left track)";
        // the assigned direction of a track must not contradict a direction the timetable already
        // established for it: forward trains may not be sent onto a backward-only track (and v.v.).
        // (G3: the old pair of tests rejected the CONSISTENT case "B is used backward only" while
        // keeping B as the backward track — at Grenchen Nord that kept the only passing loop of the
        // Moutier-Grenchen single track inactive, 59 % of the Switzerland late-seconds.)
        if (fwdB && aFwd && !aBwd) { lp.active = false; lp.why = "contradiction: A is used forward only"; return; }
        if (fwdB && bBwd && !bFwd) { lp.active = false; lp.why = "contradiction: B is used backward only"; return; }
        if (!fwdB && aBwd && !aFwd) { lp.active = false; lp.why = "contradiction: A is used backward only"; return; }
        if (!fwdB && bFwd && !bBwd) { lp.active = false; lp.why = "contradiction: B is used forward only"; return; }
        // no NEW wrong-direction kilometres against the OSM/DIR direction tags: a direction the
        // timetable already runs (use[directed link] > 0) is not made worse by formalising it here
        // (G3: at Grenchen Nord the timetable already runs 144 trains backward over B's 4 links, so
        // the old test refused the only passing loop of the Moutier-Grenchen single track)
        int nWrong = 0;
        for (int x : lp.a) nWrong += cwNew(fwdB ? net.rev[x] : x);
        for (int x : lp.b) nWrong += cwNew(fwdB ? x : net.rev[x]);
        if (nWrong > 0) { lp.active = false; lp.why = nWrong + " link(s) would run against the OSM direction tag"; return; }
        lp.fwdIsB = fwdB ? 1 : 0;
        lp.active = true;
    }

    /** 1 if running this directed link contradicts the OSM/DIR direction tag */
    /** 1 if this directed link is against the OSM/DIR tag and the timetable does not use it already */
    private int cwNew(int l) {
        if (cw(l) == 0) return 0;
        if (l >= 0 && l < use.length && use[l] > 0) return 0;
        return 1;
    }

    private int cw(int l) {
        if (l < 0) return 0;
        int w = net.lWay[l];
        if (w < 0) return 0;
        if (net.wayDir[w] == 0 || net.wayDirConf[w] < 0.6) return 0;
        int d = net.fwd[l] ? 1 : -1;
        return d == net.wayDir[w] ? 0 : 1;
    }

    // ------------------------------------------------------------------ report

    /** nearest stop name for every loop (report only; the detection itself uses no station list) */
    private void place(Sched sched) {
        if (sched == null) return;
        for (Loop lp : loops) {
            if (lp.a.length == 0) continue;
            double x = net.nx[net.lFrom[lp.a[0]]], y = net.ny[net.lFrom[lp.a[0]]];
            double bd = Double.POSITIVE_INFINITY;
            String name = "?";
            for (Sched.StopF sf : sched.stops.values()) {
                double d = Math.hypot(sf.x - x, sf.y - y);
                if (d < bd) { bd = d; name = sf.name; }
            }
            lp.place = name;
            lp.placeDist = bd;
        }
    }

    public void write(String file) throws Exception {
        try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
                new FileOutputStream(file), StandardCharsets.UTF_8))) {
            pw.println("nearest_stop,dist_m,active,tracks_A,tracks_B,len_A_m,len_B_m,offset_B_m,B_is_left,"
                    + "use_A_fwd,use_A_bwd,use_B_fwd,use_B_bwd,fwd_track,why,ways_A,ways_B,links_A,links_B,nodeA0,nodeA1");
            for (Loop lp : loops) {
                pw.printf(Locale.ROOT, "%s,%.0f,%b,%d,%d,%.0f,%.0f,%.1f,%b,%d,%d,%d,%d,%s,%s,%s,%s,%s,%s,%d,%d%n",
                        lp.place == null ? "?" : lp.place.replace(',', ' '), lp.placeDist, lp.active,
                        lp.a.length, lp.b.length, lp.lenA, lp.lenB, lp.offsetM, lp.bLeft,
                        lp.useAFwd, lp.useABwd, lp.useBFwd, lp.useBBwd,
                        lp.active ? (lp.fwdIsB == 1 ? "B" : "A") : "-", lp.why == null ? "" : lp.why.replace(',', ' '),
                        lp.waysA, lp.waysB, ids(lp.a), ids(lp.b), lp.nodeA0, lp.nodeA1);
            }
        }
    }

    private String ids(int[] ls) {
        StringBuilder sb = new StringBuilder();
        for (int x : ls) { if (sb.length() > 0) sb.append(';'); sb.append(net.linkId[x]); }
        return sb.toString();
    }

    private static int[] toArray(ArrayList<Integer> l) {
        int[] a = new int[l.size()];
        for (int i = 0; i < a.length; i++) a[i] = l.get(i);
        return a;
    }
}
