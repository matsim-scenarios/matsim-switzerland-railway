package ch.sbb.prepare.router;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Edge-based (directed link = state) A* search plus the per-route Viterbi over stop candidate
 * links. v2: direction dominates (R1), unknown is not free (R2), realistic speeds (R3),
 * crossover discipline (R4), no dead-end platforms (R5), freight-only penalty (R6),
 * timetable-fit corridors (R7), relation prior as tie-breaker (R8).
 */
public final class Router {

    public static final class Params {
        public double vmaxMs = 160.0 / 3.6;
        // R1
        public boolean r1 = true;
        public double dirPenalty = 8.0;
        public double dirEntryCost = 120.0;
        public double dirConfMin = 0.6;
        // R2
        public boolean r2 = true;
        public double unknownPenalty = 1.5;
        // R4
        public boolean r4 = true;
        public double crossoverCost = 30.0;
        public boolean crossChord = true;
        public double crossoverPenalty = 1.3;
        // R5
        public boolean r5 = true;
        /** extra cost for a reversal at a mid-route stop where a timetable-feasible through path
         *  leaves the same arrival track. G1: 300 s = a driver change of ends / brake test; the
         *  RT4b value 3000 s is 50 min and priced real reversals (Bern) out of existence. */
        public double r5RevPenalty = 300.0;
        // R6
        public boolean r6 = true;
        public double freightPenalty = 4.0;
        // R8
        public boolean r8 = true;
        public double relBonus = 0.85;
        // klass
        public double sidingPenalty = 1.5;
        public double servicePenalty = 6.0;
        // G2 passing loops: one-off cost [s] for entering a loop on the track of the other direction.
        // A one-off entry cost keeps the decision local: a per-link factor would also re-price whole
        // corridors and can push a route onto a completely different line (measured: IR17 1 149 ->
        // 3 337 links, 4 590 s late at Olten).
        public double loopPenalty = 120.0;
        // route model
        public double revCost = 300.0;
        public double unmatchedCostPerM = 2.0;
        public double cutoffFactor = 2.0;
        public double cutoffAdd = 600.0;
        public boolean turnsOn = true;
        public boolean filterOn = true;
        public int maxCandidates = 6;
        public int maxSettled = 600_000;
        public double relaxPenalty = 7200.0;
        public double turnPenalty = 0.0;
        public double softTurnPenalty = 0.0;   // v2: no soft (illegal) turn solution by default
        public double absurdFactor = 3.0;
        public double absurdAdd = 1800.0;

        public Params copy() {
            Params q = new Params();
            q.vmaxMs = vmaxMs; q.r1 = r1; q.dirPenalty = dirPenalty; q.dirEntryCost = dirEntryCost;
            q.dirConfMin = dirConfMin; q.r2 = r2; q.unknownPenalty = unknownPenalty;
            q.r4 = r4; q.crossoverCost = crossoverCost; q.crossChord = crossChord; q.crossoverPenalty = crossoverPenalty;
            q.r5 = r5; q.r5RevPenalty = r5RevPenalty;
            q.r6 = r6; q.freightPenalty = freightPenalty; q.r8 = r8; q.relBonus = relBonus;
            q.sidingPenalty = sidingPenalty; q.servicePenalty = servicePenalty;
            q.loopPenalty = loopPenalty;
            q.revCost = revCost; q.unmatchedCostPerM = unmatchedCostPerM; q.cutoffFactor = cutoffFactor;
            q.cutoffAdd = cutoffAdd; q.turnsOn = turnsOn; q.filterOn = filterOn;
            q.maxCandidates = maxCandidates; q.maxSettled = maxSettled; q.relaxPenalty = relaxPenalty;
            q.turnPenalty = turnPenalty; q.softTurnPenalty = softTurnPenalty;
            q.absurdFactor = absurdFactor; q.absurdAdd = absurdAdd;
            return q;
        }
    }

    public static final class Plan {
        public int[] core = new int[0];
        public int[] stopCorePos = new int[0];
        public int[] prefixStops = new int[0];
        public int prefixAttachNode = -1;
        public int[] suffixStops = new int[0];
        public int suffixAttachNode = -1;
        public int nRelaxed = 0;
        public int nSoftTurns = 0;
        public double cost = 0;
        public final java.util.ArrayList<String> issues = new java.util.ArrayList<>();
    }

    private final Net n;
    private final Params p;
    private final byte[] relMask;
    /** additive penalty per link (negotiation history) */
    public double[] pen;
    /** multiplicative cost factor per link (R7 corridor search), null = 1 */
    public double[] mult;
    /** G2: per directed link passing-loop assignment (+1 = assigned direction, -1 = against, 0 = none) */
    public byte[] loopDir;
    /** number of times the R4 crossover chord rule actually blocked a transition */
    public long nChordBlocked = 0;
    /** R5 (G1): reversals left unpenalised because no timetable-feasible through path exists */
    public long nRevForced = 0;
    /** R5 (G1): reversals penalised because a through path fits the booked leg time */
    public long nRevPenalised = 0;
    /** debugging: route id + leg index whose cost matrix is dumped (RT4B_DEBUG_LEG=route:leg) */
    public static volatile String DBG_ROUTE = null;
    public static volatile int DBG_LEG = -1;
    public static volatile boolean DBG_ON = false;

    public Router(Net net, Params params) {
        this.n = net;
        this.p = params;
        this.relMask = new byte[net.nWays];
        this.pen = new double[net.nLinks];
    }

    public Params params() { return p; }
    public byte[] relMask() { return relMask; }

    // ------------------------------------------------------------------ cost

    public boolean dirKnown(int w) {
        return n.wayDir[w] != 0 && n.wayDirConf[w] >= p.dirConfMin;
    }

    public boolean wrongDir(int l) {
        int w = n.lWay[l];
        if (w < 0 || !dirKnown(w)) return false;
        int dir = n.fwd[l] ? 1 : -1;
        return dir != n.wayDir[w];
    }

    /** R1/R2/R3/R6/R8 base cost of one link, in seconds (>= free-flow time). */
    public double cost(int l) {
        int w = n.lWay[l];
        double sp = n.lRouteSpeed[l];
        if (sp > p.vmaxMs) sp = p.vmaxMs;
        if (sp < 0.5) sp = 0.5;
        double t = n.lLen[l] / sp;
        if (w < 0) return t;
        double f = 1.0;
        boolean loopOk = loopDir != null && loopDir[l] > 0;
        if (loopOk) {
            // G2: an assigned loop track is a running track of this direction, not a siding
            f = 1.0;
        } else {
            switch (n.wayKlass[w]) {
                case Net.K_SIDING: f = p.sidingPenalty; break;
                case Net.K_CROSS: f = p.crossoverPenalty; break;
                case Net.K_YARD: case Net.K_SPUR: f = p.servicePenalty; break;
                default: break;
            }
        }
        if (p.r6 && n.wayFreightOnly[w]) f *= p.freightPenalty;
        double c = t * f;
        if (p.r8 && relMask != null) {
            int bit = n.fwd[l] ? 1 : 2;
            if ((relMask[w] & bit) != 0) c *= p.relBonus;   // R8: tie-breaker, applied first
        }
        if (p.r1 && c < t) c = t;                            // R1: clamp before the direction factor
        if (wrongDir(l)) c *= p.dirPenalty;                  // R1 factor (also the v1 x3 when r1=false)
        else if (p.r2 && n.wayUnknownPenalty[w]) c *= p.unknownPenalty;
        double m = mult == null ? 1.0 : mult[l];
        return c * m + pen[l];
    }

    /** fixed cost of entering link b from link prev (R1 wrong-direction entry, R4 crossover). */
    public double entryCost(int prev, int b) {
        double e = 0;
        if (loopDir != null && loopDir[b] < 0 && (prev < 0 || loopDir[prev] >= 0)) {
            // G2: entering a passing loop on the track that belongs to the other direction
            e += p.loopPenalty;
        }
        if (p.r1 && wrongDir(b)) {
            if (prev < 0 || !wrongDir(prev) || n.lWay[prev] != n.lWay[b]) e += p.dirEntryCost;
        }
        if (p.r4) {
            int wb = n.lWay[b];
            if (wb >= 0 && n.wayKlass[wb] == Net.K_CROSS) {
                if (prev < 0 || n.lWay[prev] < 0 || n.wayKlass[n.lWay[prev]] != Net.K_CROSS) e += p.crossoverCost;
            }
        }
        return e;
    }

    private static boolean isCrossoverLink(Net n, int l) {
        int w = n.lWay[l];
        return w >= 0 && n.wayKlass[w] == Net.K_CROSS;
    }

    // ------------------------------------------------------------------ search

    public static final class Sear {
        final Net n; final Router r; final Params p;
        final double[] dist; final int[] prev; final int[] stamp; final int[] done;
        int cur;
        int[] hs = new int[1 << 12];
        double[] hk = new double[1 << 12];
        int hn;
        int[] tgt = new int[0];
        double[] tdist = new double[0];
        boolean[] reach = new boolean[0];
        boolean[] isTgt;
        double[] tx = new double[0], ty = new double[0];
        double cut;
        int nSettled;

        Sear(Net n, Router r, Params p) {
            this.n = n; this.r = r; this.p = p;
            int m = n.nLinks;
            dist = new double[m]; prev = new int[m]; stamp = new int[m]; done = new int[m];
            isTgt = new boolean[m];
            Arrays.fill(prev, -1);
        }

        void push(int s, double k) {
            if (hn == hs.length) {
                hs = Arrays.copyOf(hs, hn * 2);
                hk = Arrays.copyOf(hk, hn * 2);
            }
            int i = hn++;
            while (i > 0) {
                int par = (i - 1) >> 1;
                if (hk[par] <= k) break;
                hs[i] = hs[par]; hk[i] = hk[par]; i = par;
            }
            hs[i] = s; hk[i] = k;
        }

        int pop() {
            int top = hs[0];
            int lastS = hs[--hn];
            double lastK = hk[hn];
            int i = 0;
            while (true) {
                int c = 2 * i + 1;
                if (c >= hn) break;
                if (c + 1 < hn && hk[c + 1] < hk[c]) c++;
                if (hk[c] >= lastK) break;
                hs[i] = hs[c]; hk[i] = hk[c]; i = c;
            }
            hs[i] = lastS; hk[i] = lastK;
            return top;
        }

        double h(int l) {
            if (isTgt[l]) return 0;
            double x = n.nx[n.lTo[l]], y = n.ny[n.lTo[l]];
            double best = Double.POSITIVE_INFINITY;
            for (int i = 0; i < tgt.length; i++) {
                double dx = x - tx[i], dy = y - ty[i];
                double d = dx * dx + dy * dy;
                if (d < best) best = d;
            }
            return Math.sqrt(best) / p.vmaxMs;
        }

        void run(int[] sources, double[] sCost, int[] targets, byte[] mask) {
            cur++;
            hn = 0;
            nSettled = 0;
            if (tdist.length < targets.length) { tdist = new double[targets.length]; reach = new boolean[targets.length]; }
            Arrays.fill(tdist, Double.POSITIVE_INFINITY);
            Arrays.fill(reach, false);
            for (int i = 0; i < tgt.length; i++) isTgt[tgt[i]] = false;
            tgt = targets;
            if (tx.length < targets.length) { tx = new double[targets.length]; ty = new double[targets.length]; }
            for (int i = 0; i < targets.length; i++) {
                isTgt[targets[i]] = true;
                tx[i] = n.nx[n.lFrom[targets[i]]];
                ty[i] = n.ny[n.lFrom[targets[i]]];
            }
            cut = Double.POSITIVE_INFINITY;
            for (int i = 0; i < sources.length; i++) {
                int s = sources[i];
                if (s < 0) continue;
                if (stamp[s] != cur || sCost[i] < dist[s]) {
                    stamp[s] = cur; dist[s] = sCost[i]; prev[s] = -1;
                    push(s, sCost[i] + h(s));
                }
            }
            int remaining = targets.length;
            double bestT = Double.POSITIVE_INFINITY;
            while (hn > 0) {
                if (hk[0] > cut) break;
                int s = pop();
                if (stamp[s] != cur || done[s] == cur) continue;
                done[s] = cur;
                nSettled++;
                if (nSettled > p.maxSettled) break;
                if (isTgt[s]) {
                    for (int i = 0; i < targets.length; i++) {
                        if (targets[i] == s && Double.isInfinite(tdist[i])) {
                            tdist[i] = dist[s];
                            reach[i] = true;
                            remaining--;
                            if (dist[s] < bestT) {
                                bestT = dist[s];
                                cut = bestT * p.cutoffFactor + p.cutoffAdd;
                            }
                        }
                    }
                    if (remaining == 0) break;
                }
                int from = n.lTo[s];
                double ds = dist[s];
                for (int k = n.outStart[from]; k < n.outStart[from + 1]; k++) {
                    int b = n.outLinks[k];
                    if (b == n.rev[s]) continue;
                    if (n.lFrom[b] == n.lTo[s] && n.lTo[b] == n.lFrom[s]) continue;
                    double tp = 0;
                    if (p.turnsOn && !n.legal(from, n.lFrom[s], n.lTo[b])) {
                        if (p.turnPenalty <= 0) continue;
                        tp = p.turnPenalty;
                    }
                    if (p.filterOn && !n.inGraph[b]) continue;
                    // R4: crossover -> (<=25 m) -> crossover back onto the original track
                    if (p.r4 && p.crossChord) {
                        int pv = prev[s];
                        if (pv >= 0 && isCrossoverLink(n, pv) && isCrossoverLink(n, b) && n.lLen[s] <= 25.0
                                && n.lFrom[pv] == n.lTo[b]) {
                            r.nChordBlocked++;
                            continue;
                        }
                    }
                    double nd = ds + r.cost(b) + r.entryCost(s, b) + tp;
                    if (stamp[b] != cur || nd < dist[b]) {
                        stamp[b] = cur; dist[b] = nd; prev[b] = s;
                        push(b, nd + h(b));
                    }
                }
            }
            for (int i = 0; i < targets.length; i++) {
                if (!reach[i] && !Double.isInfinite(cut)) tdist[i] = cut;
            }
        }

        int[] pathTo(int target) {
            if (stamp[target] != cur || done[target] != cur) return null;
            int len = 0;
            int c = target;
            while (true) {
                len++;
                if (prev[c] < 0) break;
                c = prev[c];
                if (len > 3_000_000) return null;
            }
            int[] out = new int[len];
            c = target;
            for (int i = len - 1; i >= 0; i--) {
                out[i] = c;
                if (prev[c] < 0) break;
                c = prev[c];
            }
            return out;
        }
    }

    public Sear newSearcher() { return new Sear(n, this, p); }

    // ------------------------------------------------------------------ route solve

    public static final class Cand {
        public final int[] links;
        public final double[] extra;
        /** R5: the only way onward is reversing (stub / dead-end track) */
        public final boolean[] stub;
        public Cand(int[] links, double[] extra) { this(links, extra, new boolean[links.length]); }
        public Cand(int[] links, double[] extra, boolean[] stub) {
            this.links = links; this.extra = extra; this.stub = stub;
        }
    }

    private int[] range(int a, int b) {
        if (b <= a) return new int[0];
        int[] r = new int[b - a];
        for (int i = a; i < b; i++) r[i - a] = i;
        return r;
    }

    /**
     * @param cands     candidate list per anchor (in stop order)
     * @param anchors   stop index of each anchor
     * @param nStops    total number of stops of the route
     */
    public Plan solve(List<Cand> candsIn, int[] anchors, int nStops, int[] schedArr, int[] schedDep,
                      Sear se) {
        return solve(candsIn, anchors, nStops, schedArr, schedDep, se, 0, nStops);
    }

    /**
     * @param from first stop index that belongs to this plan (prefix stops before it are ignored)
     * @param to   exclusive end of the stop range of this plan (suffix stops after it are ignored)
     */
    public Plan solve(List<Cand> candsIn, int[] anchors, int nStops, int[] schedArr, int[] schedDep,
                      Sear se, int from, int to) {
        int m = anchors.length;
        Plan plan = new Plan();
        plan.stopCorePos = new int[nStops];
        Arrays.fill(plan.stopCorePos, -1);
        if (m == 0) {
            plan.prefixStops = range(from, to);
            return plan;
        }
        // ---------- R5: drop dead-end stop candidates unless first/last or all are dead ends
        boolean[] allStub = new boolean[m];
        ArrayList<Cand> cands = new ArrayList<>(m);
        for (int k = 0; k < m; k++) {
            Cand c = candsIn.get(k);
            boolean anyThrough0 = false;
            for (int i = 0; i < c.links.length; i++) if (!c.stub[i]) { anyThrough0 = true; break; }
            allStub[k] = !anyThrough0;
            if (!p.r5 || k == 0 || k == m - 1) { cands.add(c); continue; }
            if (!anyThrough0) { cands.add(c); continue; }
            int cnt = 0;
            for (int i = 0; i < c.links.length; i++) if (!c.stub[i]) cnt++;
            int[] la = new int[cnt]; double[] ea = new double[cnt];
            int q = 0;
            for (int i = 0; i < c.links.length; i++)
                if (!c.stub[i]) { la[q] = c.links[i]; ea[q] = c.extra[i]; q++; }
            cands.add(new Cand(la, ea));
        }
        plan.prefixStops = range(from, anchors[0]);
        plan.suffixStops = range(anchors[m - 1] + 1, to);
        if (m == 1) {
            Cand c = cands.get(0);
            int best = 0;
            for (int i = 1; i < c.links.length; i++) if (c.extra[i] < c.extra[best]) best = i;
            plan.core = new int[]{c.links[best]};
            plan.stopCorePos[anchors[0]] = 0;
            return plan;
        }

        double[][] dp = new double[m][];
        int[][] back = new int[m][];
        for (int k = 0; k < m; k++) {
            Cand c = cands.get(k);
            dp[k] = new double[c.links.length];
            back[k] = new int[c.links.length];
            if (k == 0) {
                for (int i = 0; i < c.links.length; i++) dp[0][i] = cost(c.links[i]) + c.extra[i];
            } else Arrays.fill(dp[k], Double.POSITIVE_INFINITY);
        }

        int nRelaxed = 0;
        int nSoft = 0;
        double[][][] dleg = new double[m - 1][][];
        int[][][][] pleg = new int[m - 1][][][];
        for (int k = 0; k + 1 < m; k++) {
            Cand a = cands.get(k);
            Cand b = cands.get(k + 1);
            double[][] dd = new double[a.links.length][b.links.length];
            int[][][] pp = new int[a.links.length][b.links.length][];
            boolean[] relaxedSrc = new boolean[a.links.length];
            for (int i = 0; i < a.links.length; i++) {
                if (Double.isInfinite(dp[k][i])) { Arrays.fill(dd[i], Double.POSITIVE_INFINITY); continue; }
                boolean ok = legSearch(a.links[i], dp[k][i], b.links, se, dd[i], pp[i], k > 0);
                if (!ok) {
                    p.turnsOn = false;
                    ok = legSearch(a.links[i], dp[k][i], b.links, se, dd[i], pp[i], k > 0);
                    p.turnsOn = true;
                    if (ok) { nRelaxed++; relaxedSrc[i] = true; plan.issues.add("leg " + k + " src " + n.linkId[a.links[i]] + " TURNS_OFF"); }
                }
                if (!ok) {
                    p.filterOn = false;
                    ok = legSearch(a.links[i], dp[k][i], b.links, se, dd[i], pp[i], k > 0);
                    p.filterOn = true;
                    if (ok) { nRelaxed++; relaxedSrc[i] = true; plan.issues.add("leg " + k + " src " + n.linkId[a.links[i]] + " NOFILTER"); }
                }
                if (!ok) {
                    Arrays.fill(dd[i], Double.POSITIVE_INFINITY);
                }
            }
            // R5 (G1): a reversal at a mid-route stop costs extra ONLY when the same arrival track also
            // offers a non-reversing continuation that fits the booked running time of this leg.
            // If the reversal is geometrically forced (no through path leaves this link) or the through
            // path cannot make the timetable, the reversal is the real operation (Bern, Zürich HB, ...)
            // and must not be priced out of existence.
            if (p.r5 && k > 0 && !allStub[k]) {
                double schedLeg = Math.max(1.0, schedArr[anchors[k + 1]] - schedDep[anchors[k]]);
                for (int i = 0; i < a.links.length; i++) {
                    if (Double.isInfinite(dp[k][i])) continue;
                    boolean anyRev = false;
                    for (int j = 0; j < b.links.length; j++) {
                        if (pp[i][j] != null && pp[i][j][0] != a.links[i]) { anyRev = true; break; }
                    }
                    if (!anyRev) continue;
                    double[] d2 = new double[b.links.length];
                    int[][] p2 = new int[b.links.length][];
                    boolean ok = legSearch(a.links[i], dp[k][i], b.links, se, d2, p2, false);
                    boolean feasible = false;
                    if (ok) {
                        for (int j = 0; j < b.links.length; j++) {
                            if (p2[j] != null && legRuntime(p2[j]) <= schedLeg) { feasible = true; break; }
                        }
                    }
                    if (!feasible) { nRevForced++; continue; }
                    for (int j = 0; j < b.links.length; j++) {
                        if (pp[i][j] != null && pp[i][j][0] != a.links[i]) dd[i][j] += p.r5RevPenalty;
                    }
                    nRevPenalised++;
                }
            }
            // optional soft-turn solution (off in v2: illegal turns must be 0)
            double bestRt = Double.POSITIVE_INFINITY;
            for (int i = 0; i < a.links.length; i++)
                for (int j = 0; j < b.links.length; j++)
                    if (pp[i][j] != null) bestRt = Math.min(bestRt, legRuntime(pp[i][j]));
            if (Double.isFinite(bestRt) && p.softTurnPenalty > 0) {
                double[][] dd2 = new double[a.links.length][b.links.length];
                int[][][] pp2 = new int[a.links.length][b.links.length][];
                boolean[] relaxed2 = new boolean[a.links.length];
                p.turnPenalty = p.softTurnPenalty;
                for (int i = 0; i < a.links.length; i++) {
                    if (Double.isInfinite(dp[k][i])) { Arrays.fill(dd2[i], Double.POSITIVE_INFINITY); continue; }
                    boolean ok2 = legSearch(a.links[i], dp[k][i], b.links, se, dd2[i], pp2[i], k > 0);
                    if (!ok2) { relaxed2[i] = true; Arrays.fill(dd2[i], Double.POSITIVE_INFINITY); }
                }
                p.turnPenalty = 0.0;
                double bestRt2 = Double.POSITIVE_INFINITY;
                for (int i = 0; i < a.links.length; i++)
                    for (int j = 0; j < b.links.length; j++)
                        if (pp2[i][j] != null) bestRt2 = Math.min(bestRt2, legRuntime(pp2[i][j]));
                if (bestRt2 < 0.75 * bestRt) {
                    dd = dd2; pp = pp2; relaxedSrc = relaxed2;
                    nSoft++;
                }
            }
            if (DBG_ON && k == DBG_LEG) {
                System.out.println("=== DBG leg " + k + " of " + DBG_ROUTE
                        + " sched " + (schedArr[anchors[k + 1]] - schedDep[anchors[k]]) + " s");
                for (int i = 0; i < a.links.length; i++) {
                    System.out.printf(java.util.Locale.ROOT, " src %d id %d way %d dp %.1f%n",
                            i, n.linkId[a.links[i]], n.lWay[a.links[i]] < 0 ? -1 : n.wayId[n.lWay[a.links[i]]], dp[k][i]);
                    for (int j = 0; j < b.links.length; j++) {
                        if (pp[i][j] == null) continue;
                        double km = 0;
                        for (int q = 0; q < pp[i][j].length; q++) km += n.lLen[pp[i][j][q]];
                        System.out.printf(java.util.Locale.ROOT,
                                "   -> tgt %d id %d way %d  cost %8.1f  rt %7.1f s  km %7.3f  rev %b  nlinks %d%n",
                                j, n.linkId[b.links[j]], n.lWay[b.links[j]] < 0 ? -1 : n.wayId[n.lWay[b.links[j]]],
                                dd[i][j], legRuntime(pp[i][j]), km / 1000,
                                pp[i][j][0] != a.links[i], pp[i][j].length);
                    }
                }
            }
            dleg[k] = dd;
            pleg[k] = pp;
            boolean any = false;
            for (int j = 0; j < b.links.length; j++) {
                double best = Double.POSITIVE_INFINITY; int bi = -1;
                for (int i = 0; i < a.links.length; i++) {
                    double v = dp[k][i] + dd[i][j] + (relaxedSrc[i] ? p.relaxPenalty : 0);
                    if (v < best) { best = v; bi = i; }
                }
                dp[k + 1][j] = best + b.extra[j];
                back[k + 1][j] = bi;
                if (!Double.isInfinite(dp[k + 1][j])) any = true;
            }
            if (!any) return null;
        }

        int lastC = -1;
        double bestC = Double.POSITIVE_INFINITY;
        for (int j = 0; j < dp[m - 1].length; j++) {
            if (dp[m - 1][j] < bestC) { bestC = dp[m - 1][j]; lastC = j; }
        }
        if (lastC < 0 || Double.isInfinite(bestC)) return null;

        int[] chosen = new int[m];
        chosen[m - 1] = lastC;
        for (int k = m - 2; k >= 0; k--) chosen[k] = back[k + 1][chosen[k + 1]];
        for (int k = 0; k < m; k++) if (chosen[k] < 0) return null;

        ArrayList<Integer> core = new ArrayList<>();
        core.add(cands.get(0).links[chosen[0]]);
        plan.stopCorePos[anchors[0]] = 0;
        for (int k = 0; k + 1 < m; k++) {
            int[] path = pleg[k][chosen[k]][chosen[k + 1]];
            if (path == null) return null;
            int src = cands.get(k).links[chosen[k]];
            int q0 = (path[0] == src) ? 1 : 0;   // q0 == 0 -> the leg reverses at this stop
            for (int q = q0; q < path.length; q++) core.add(path[q]);
            plan.stopCorePos[anchors[k + 1]] = core.size() - 1;
        }
        plan.core = new int[core.size()];
        for (int i = 0; i < core.size(); i++) plan.core[i] = core.get(i);
        plan.nRelaxed = nRelaxed;
        plan.nSoftTurns = nSoft;
        plan.cost = bestC;
        return plan;
    }

    public double legRuntime(int[] path) {
        if (path == null) return Double.POSITIVE_INFINITY;
        double t = 0;
        for (int l : path) {
            double sp = n.lRouteSpeed[l];
            if (sp > p.vmaxMs) sp = p.vmaxMs;
            t += n.lLen[l] / Math.max(sp, 0.5);
        }
        return t;
    }

    public void legSearchPublic(int src, double base, int[] targets, Sear se,
                                double[] dist, int[][] paths, boolean allowRev) {
        legSearch(src, base, targets, se, dist, paths, allowRev);
    }

    private boolean legSearch(int src, double base, int[] targets, Sear se,
                              double[] dist, int[][] paths, boolean allowRev) {
        int r = n.rev[src];
        int[] sources;
        double[] scost;
        if (allowRev && r >= 0 && r != src) {
            sources = new int[]{src, r};
            scost = new double[]{base, base + p.revCost + cost(r)};
        } else {
            sources = new int[]{src};
            scost = new double[]{base};
        }
        se.run(sources, scost, targets, relMask);
        boolean any = false;
        for (int j = 0; j < targets.length; j++) {
            if (se.reach[j]) { dist[j] = se.tdist[j]; any = true; }
            else dist[j] = Double.POSITIVE_INFINITY;
        }
        if (!any) return false;
        if (paths != null) {
            for (int j = 0; j < targets.length; j++) {
                paths[j] = se.reach[j] ? se.pathTo(targets[j]) : null;
            }
        }
        return true;
    }

    // ------------------------------------------------------------------ R7 corridors

    /**
     * Search up to 3 corridor alternatives for one leg. The previous path's links are penalised x5
     * outside a radius around the two stops; take the alternative with the largest pure running
     * time that still fits the scheduled leg time.
     *
     * @return the chosen path (srcLink ... tgtLink) or null if the original should be kept
     */
    public int[] bestCorridor(int srcLink, int tgtLink, int[] prevPath, double x1, double y1,
                              double x2, double y2, double schedRuntime, double radius, double factor,
                              Sear se, ArrayList<String> log) {
        if (prevPath == null || prevPath.length < 2) return null;
        double[] baseMult = mult;
        double[] m = new double[n.nLinks];
        Arrays.fill(m, 1.0);
        mult = m;
        int[] best = null;
        double bestRt = -1;
        for (int it = 0; it < 5; it++) {
            // penalise the previous path outside the stop radii
            for (int l : prevPath) {
                double mx = (n.nx[n.lFrom[l]] + n.nx[n.lTo[l]]) * 0.5;
                double my = (n.ny[n.lFrom[l]] + n.ny[n.lTo[l]]) * 0.5;
                boolean nearStop = Math.hypot(mx - x1, my - y1) <= radius || Math.hypot(mx - x2, my - y2) <= radius;
                if (!nearStop) m[l] = factor;
            }
            double[][] dd = new double[1][1];
            int[][][] pp = new int[1][1][];
            boolean ok = legSearch(srcLink, 0.0, new int[]{tgtLink}, se, dd[0], pp[0], false);
            int[] path = ok ? pp[0][0] : null;
            if (path == null) break;
            double rt = legRuntime(path);
            int idx = 0;
            for (int i = 0; i < path.length; i++) if (path[i] == tgtLink) { idx = i; break; }
            path = Arrays.copyOf(path, idx + 1);
            if (rt <= schedRuntime && rt > bestRt) { bestRt = rt; best = path; }
            log.add(String.format(java.util.Locale.ROOT, "alt %d: %.0f s (budget %.0f s, %d links)", it + 1, rt, schedRuntime, path.length));
            prevPath = path;
        }
        mult = baseMult;
        return best;
    }

    /** pure running time of a link list */
    public double runtimeOf(List<Integer> links) {
        double t = 0;
        for (int l : links) {
            double sp = n.lRouteSpeed[l];
            if (sp > p.vmaxMs) sp = p.vmaxMs;
            t += n.lLen[l] / Math.max(sp, 0.5);
        }
        return t;
    }

    /** R5 helper: a link is a dead end when the only way onward is its own reverse. */
    public boolean stub(int l) { return n.stub(l); }
}
