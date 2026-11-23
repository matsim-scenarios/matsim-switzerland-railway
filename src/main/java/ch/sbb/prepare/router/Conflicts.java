package ch.sbb.prepare.router;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Head-on conflict detection by timetable interpolation with the free-flow speed profile scaled
 * to the scheduled leg time (the same definition as swarm/bench/check_mapping.py), plus R9
 * history-cost suggestions.
 */
public final class Conflicts {

    public static final class Train {
        public int route;            // index into the route list
        public int links[];
        public int stopPos[];        // per stop, -1 = not an anchor
        public double arr[], dep[];  // per stop, seconds
        public double t0;            // departure time of this run
    }

    public static final class Result {
        public int pairsAll;            // checker metric (includes self pairs)
        public int pairsExclSelf;
        public int pairsEligible;       // R9-relevant pairs (no 'both' track, no stop link)
        public int trainsInvolved;
        public int nIntervals;
        public int[] segConflicts;
        public int[] segCross;
        public int[] segCountFw, segCountBw;
        public Set<Integer> affectedRoutes = new HashSet<>();
        /** suggested additive history cost per directed link (seconds) */
        public double[] addPen;
        public int[] segN;
    }

    private final Net net;
    private final int[] segId;
    private final int nSeg;
    private final int[] segLinkA, segLinkB;

    public Conflicts(Net net) {
        this.net = net;
        segId = new int[net.nLinks];
        Arrays.fill(segId, -1);
        int n = 0;
        int[] la = new int[net.nLinks], lb = new int[net.nLinks];
        for (int i = 0; i < net.nLinks; i++) {
            if (segId[i] >= 0) continue;
            int r = net.rev[i];
            // canonical link: the one running from the smaller node index
            int a = i, b = r;
            if (b >= 0 && net.lFrom[b] < net.lTo[b] && net.lFrom[a] >= net.lTo[a]) { int t = a; a = b; b = t; }
            segId[i] = n;
            la[n] = a; lb[n] = b;
            if (r >= 0 && r != i) segId[r] = n;
            n++;
        }
        nSeg = n;
        segLinkA = Arrays.copyOf(la, n);
        segLinkB = Arrays.copyOf(lb, n);
    }

    public int nSeg() { return nSeg; }
    public int segOf(int link) { return segId[link]; }
    public int segLinkA(int s) { return segLinkA[s]; }

    /** occupancy of every link of one run; NaN = no timing (outside a timed leg) */
    private double[][] times(Train t) {
        int[] links = t.links;
        int n = links.length;
        double[] cum = new double[n + 1];
        double cap = 55.6;   // the checker caps freespeed at 200 km/h
        for (int i = 0; i < n; i++) cum[i + 1] = cum[i] + net.lLen[links[i]] / Math.min(net.lRouteSpeed[links[i]], cap);
        double[] tin = new double[n], tout = new double[n];
        Arrays.fill(tin, Double.NaN);
        for (int k = 0; k + 1 < t.stopPos.length; k++) {
            int i0 = t.stopPos[k], i1 = t.stopPos[k + 1];
            if (i0 < 0 || i1 < 0 || i1 <= i0) continue;
            double t0 = t.dep[k], t1 = t.arr[k + 1];
            if (t1 < t0) t1 += 24 * 3600;
            double d0 = cum[i0 + 1], d1 = cum[i1 + 1];
            double span = Math.max(d1 - d0, 1e-6);
            for (int i = i0 + 1; i <= i1; i++) {
                tin[i] = t0 + (cum[i] - d0) / span * (t1 - t0);
                tout[i] = t0 + (cum[i + 1] - d0) / span * (t1 - t0);
            }
        }
        return new double[][]{tin, tout};
    }

    public Result analyze(List<Train> trains, double margin, boolean excludeBothAndStops, double histBase,
                          int iter) {
        Result out = new Result();
        out.addPen = new double[net.nLinks];
        out.segConflicts = new int[nSeg];
        out.segCross = new int[nSeg];
        out.segCountFw = new int[nSeg];
        out.segCountBw = new int[nSeg];
        out.segN = new int[nSeg];

        ArrayList<double[][]> tt = new ArrayList<>(trains.size());
        int total = 0;
        for (Train t : trains) {
            double[][] x = times(t);
            tt.add(x);
            for (int i = 0; i < t.links.length; i++) if (!Double.isNaN(x[0][i])) total++;
        }
        out.nIntervals = total;

        int[] trTrain = new int[total], trSeg = new int[total];
        byte[] trDir = new byte[total];
        double[] trStart = new double[total], trEnd = new double[total];
        int ptr = 0;
        for (int ti = 0; ti < trains.size(); ti++) {
            Train t = trains.get(ti);
            double[][] x = tt.get(ti);
            for (int i = 0; i < t.links.length; i++) {
                double a = x[0][i];
                if (Double.isNaN(a)) continue;
                trTrain[ptr] = ti;
                trSeg[ptr] = segId[t.links[i]];
                trDir[ptr] = (byte) (net.lFrom[t.links[i]] < net.lTo[t.links[i]] ? 1 : 0);
                trStart[ptr] = t.t0 + a - margin;
                trEnd[ptr] = t.t0 + x[1][i] + margin;
                ptr++;
            }
        }
        int[] start = new int[nSeg + 1];
        for (int i = 0; i < total; i++) start[trSeg[i] + 1]++;
        for (int i = 0; i < nSeg; i++) start[i + 1] += start[i];
        int[] fill = Arrays.copyOf(start, nSeg);
        int[] idx = new int[total];
        for (int i = 0; i < total; i++) idx[fill[trSeg[i]]++] = i;

        HashSet<Long> setAll = new HashSet<>();
        HashSet<Long> setSelf = new HashSet<>();
        HashSet<Long> setElig = new HashSet<>();
        for (int sg = 0; sg < nSeg; sg++) {
            int from = start[sg], to = start[sg + 1];
            if (to - from < 2) continue;
            int nA = 0, nB = 0;
            for (int q = from; q < to; q++) { if (trDir[idx[q]] == 1) nA++; else nB++; }
            if (nA == 0 || nB == 0) continue;
            int[] aIdx = new int[nA], bIdx = new int[nB];
            int ia = 0, ib = 0;
            for (int q = from; q < to; q++) { int ii = idx[q]; if (trDir[ii] == 1) aIdx[ia++] = ii; else bIdx[ib++] = ii; }
            sortByStart(aIdx, trStart);
            sortByStart(bIdx, trStart);
            int la = segLinkA[sg], lb = segLinkB[sg];
            boolean segBoth = excludeBothAndStops && (isBoth(la) || (lb >= 0 && isBoth(lb)));
            int nAconf = 0, nBconf = 0;
            for (int qa = 0; qa < nA; qa++) {
                int iaa = aIdx[qa];
                double sa = trStart[iaa], ea = trEnd[iaa];
                for (int qb = 0; qb < nB; qb++) {
                    int ibb = bIdx[qb];
                    if (trStart[ibb] >= ea) break;
                    if (trEnd[ibb] <= sa) continue;
                    int t1 = trTrain[iaa], t2 = trTrain[ibb];
                    long k = t1 < t2 ? ((long) t1 << 32) | t2 : ((long) t2 << 32) | t1;
                    setAll.add(k);
                    if (t1 == t2) setSelf.add(k);
                    else {
                        out.segCross[sg]++;
                        if (!segBoth && !hasStop(at(trains, t1), la) && !hasStop(at(trains, t1), lb)
                                && !hasStop(at(trains, t2), la) && !hasStop(at(trains, t2), lb)) {
                            setElig.add(k);
                            out.affectedRoutes.add(trains.get(t1).route);
                            out.affectedRoutes.add(trains.get(t2).route);
                        }
                    }
                    out.segConflicts[sg]++;
                    if (trDir[iaa] == 1) nAconf++; else nBconf++;
                }
            }
            out.segCountFw[sg] = nA;
            out.segCountBw[sg] = nB;
            out.segN[sg] = nAconf + nBconf;
            // R9 history: the minority direction of a segment that two different trains share
            if (histBase > 0 && !segBoth && out.segCross[sg] > 0) {
                int conf = out.segCross[sg];
                double add = histBase * (1 + 0.4 * (iter - 1)) * Math.min(2.0, 1.0 + conf / 8.0);
                if (add > 400) add = 400;
                if (nAconf < nBconf) out.addPen[la] += add;
                else if (nBconf < nAconf && lb >= 0) out.addPen[lb] += add;
            }
        }
        out.pairsAll = setAll.size();
        out.pairsExclSelf = setAll.size() - setSelf.size();
        out.pairsEligible = setElig.size();
        HashSet<Integer> involved = new HashSet<>();
        for (long k : setAll) { involved.add((int) (k >>> 32)); involved.add((int) k); }
        out.trainsInvolved = involved.size();
        return out;
    }

    private static Train at(List<Train> l, int i) { return l.get(i); }

    private boolean isBoth(int l) {
        int w = net.lWay[l];
        return w >= 0 && (net.wayBoth[w] || net.wayDirSrc[w] == 4);
    }

    private boolean hasStop(Train t, int link) {
        for (int p : t.stopPos) if (p >= 0 && p < t.links.length && t.links[p] == link) return true;
        return false;
    }

    private static void sortByStart(int[] a, double[] start) {
        for (int i = 1; i < a.length; i++) {
            int v = a[i];
            double sv = start[v];
            int j = i - 1;
            while (j >= 0 && start[a[j]] > sv) { a[j + 1] = a[j]; j--; }
            a[j + 1] = v;
        }
    }
}
