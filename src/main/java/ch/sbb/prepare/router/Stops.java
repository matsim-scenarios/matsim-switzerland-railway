package ch.sbb.prepare.router;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.*;

/** stop -> candidate directed links, per ROUTER_SPEC section 4 (+ R5 stub flags, RT3 dubious-platform fallback). */
public final class Stops {

    private final Net n;
    private final Router.Params p;
    private final Map<String, int[]> platformWays = new HashMap<>();
    private final Map<String, String> platformMethod = new HashMap<>();
    private final Map<String, String> platformCode = new HashMap<>();
    private final Map<String, Router.Cand> cache = new HashMap<>();
    private final Map<String, Double> nearestDist = new HashMap<>();
    /** RT3 idea: the A03a platform match is dubious (far away or a yard/spur ladder) */
    public int nDoubtfulPlatforms = 0;
    /** switch: offer through tracks when the platform match is dubious (R5 companion) */
    public boolean dubiousOn = true;

    public Stops(Net net, Router.Params params) {
        this.n = net;
        this.p = params;
    }

    public void loadPlatforms(String file) throws IOException {
        try (BufferedReader br = new BufferedReader(new java.io.InputStreamReader(
                Net.open(file), java.nio.charset.StandardCharsets.UTF_8), 1 << 20)) {
            String line = br.readLine();
            String[] h = Net.csv(line);
            int iId = col(h, "stop_id"), iW = col(h, "way_ids"), iM = col(h, "method"), iC = col(h, "platform_code");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = Net.splitSimple(line);
                String id = c[iId];
                platformMethod.put(id, c[iM]);
                platformCode.put(id, c[iC]);
                if (c[iW].isEmpty()) continue;
                String[] ws = c[iW].split(";");
                int[] idx = new int[ws.length];
                int k = 0;
                for (String w : ws) {
                    int wi = n.wayIndexPublic(Long.parseLong(w.trim()));
                    if (wi >= 0) idx[k++] = wi;
                }
                platformWays.put(id, Arrays.copyOf(idx, k));
            }
        }
    }

    private static int col(String[] h, String name) {
        for (int i = 0; i < h.length; i++) if (h[i].equals(name)) return i;
        throw new IllegalStateException("no column " + name);
    }

    public String method(String stopId) { return platformMethod.getOrDefault(stopId, "none"); }

    // ------------------------------------------------------------------

    public Router.Cand candidates(Sched.StopF s) {
        Router.Cand c = cache.get(s.id);
        if (c == null) {
            c = compute(s);
            cache.put(s.id, c);
        }
        return c;
    }

    /** true when there is no routable link at all within 300 m */
    public boolean offNetwork(Sched.StopF s) {
        return candidates(s).links.length == 0;
    }

    public double nearestLinkDist(Sched.StopF s) {
        Double d = nearestDist.get(s.id);
        if (d == null) { candidates(s); d = nearestDist.get(s.id); }
        return d == null ? 1e9 : d;
    }

    private Router.Cand compute(Sched.StopF s) {
        int[] ways = platformWays.get(s.id);
        if (ways != null && ways.length > 0) {
            Router.Cand c = fromPlatformWays(s, ways);
            if (c != null && c.links.length > 0) return c;
        }
        Router.Cand c = fromNearestWays(s);
        if (c != null && c.links.length > 0) return c;
        return new Router.Cand(new int[0], new double[0]);
    }

    // ---- platform matched ------------------------------------------------

    private Router.Cand fromPlatformWays(Sched.StopF s, int[] ways) {
        ArrayList<Integer> seed = new ArrayList<>();
        for (int w : ways) {
            for (int k = n.wayHead[w]; k < n.wayHead[w + 1]; k++) {
                int l = n.wayNext[k];
                if (n.inGraph[l]) seed.add(l);
            }
        }
        if (seed.isEmpty()) return null;
        double best = Double.POSITIVE_INFINITY;
        int bestLink = -1;
        for (int l : seed) {
            double d = n.distToLink(l, s.x, s.y);
            if (d < best) { best = d; bestLink = l; }
        }
        List<Integer> pool;
        if (best <= 200) {
            // G1: one OSM platform way can be several km long (e.g. way 196318918 at Sempach-Neuenkirch,
            // 4.1 km).  Only the links at the published platform coordinate are the platform; a link
            // kilometres away on the same way is not.  Restrict the pool to links within 150 m
            // (or 100 m beyond the nearest) of the platform coordinate.
            double lim = Math.max(150.0, best + 100.0);
            ArrayList<Integer> near = new ArrayList<>(seed.size());
            for (int l : seed) if (n.distToLink(l, s.x, s.y) <= lim) near.add(l);
            pool = near.isEmpty() ? seed : near;
        } else {
            pool = extend(s, seed);
            if (pool.isEmpty()) return null;
        }
        Router.Cand c = cluster(s, pool, 0.0);
        // RT3: A03a compares platform codes against track_ref by digits, so platform 1 can be
        // matched to a yard ladder "B1" 78 m away, or the platform way can be a dead-end stub.
        // Offer through tracks as well, so R5 can avoid the false reversal.
        boolean dubious = best > 30
                || (bestLink >= 0 && (n.wayKlass[n.lWay[bestLink]] == Net.K_YARD
                || n.wayKlass[n.lWay[bestLink]] == Net.K_SPUR));
        if (dubious && dubiousOn) {
            nDoubtfulPlatforms++;
            Router.Cand alt = nearestThrough(s.x, s.y, 250, platformCode.get(s.id));
            if (alt != null && alt.links.length > 0) {
                int[] la = Arrays.copyOf(c.links, c.links.length + alt.links.length);
                double[] ea = Arrays.copyOf(c.extra, c.extra.length + alt.links.length);
                boolean[] st = Arrays.copyOf(c.stub, c.stub.length + alt.links.length);
                for (int i = 0; i < alt.links.length; i++) {
                    la[c.links.length + i] = alt.links[i];
                    ea[c.extra.length + i] = 60.0 + 2.0 * alt.extra[i];
                    st[c.stub.length + i] = n.stub(alt.links[i]);
                }
                return new Router.Cand(la, ea, st);
            }
        }
        return c;
    }

    /** extend a platform track along degree-2 nodes; keeps only links within 200 m of the stop */
    private List<Integer> extend(Sched.StopF s, List<Integer> seed) {
        LinkedHashSet<Integer> seen = new LinkedHashSet<>(seed);
        ArrayList<Integer> frontier = new ArrayList<>(seed);
        ArrayList<Integer> out = new ArrayList<>();
        for (int l : seed) if (n.distToLink(l, s.x, s.y) <= 200) out.add(l);
        for (int depth = 0; depth < 12; depth++) {
            ArrayList<Integer> next = new ArrayList<>();
            for (int l : frontier) {
                for (int node : new int[]{n.lFrom[l], n.lTo[l]}) {
                    if (n.nbrCount[node] != 2) continue;
                    for (int k = n.outStart[node]; k < n.outStart[node + 1]; k++) {
                        int b = n.outLinks[k];
                        if (!n.inGraph[b] || !seen.add(b)) continue;
                        next.add(b);
                        if (n.distToLink(b, s.x, s.y) <= 200) out.add(b);
                    }
                    for (int k = n.inStart[node]; k < n.inStart[node + 1]; k++) {
                        int b = n.inLinks[k];
                        if (!n.inGraph[b] || !seen.add(b)) continue;
                        next.add(b);
                        if (n.distToLink(b, s.x, s.y) <= 200) out.add(b);
                    }
                }
            }
            if (next.isEmpty()) break;
            frontier = next;
        }
        return out;
    }

    // ---- unmatched -------------------------------------------------------

    private Router.Cand fromNearestWays(Sched.StopF s) {
        HashMap<Integer, Double> bestPerWay = new HashMap<>();
        double minAll = Double.POSITIVE_INFINITY;
        int minLink = -1;
        for (int l = 0; l < n.nLinks; l++) {
            if (!n.inGraph[l]) continue;
            double d = n.distToLink(l, s.x, s.y);
            if (d < minAll) { minAll = d; minLink = l; }
            if (d > 100) continue;
            int w = n.lWay[l];
            if (w < 0) continue;
            byte k = n.wayKlass[w];
            if (k == Net.K_YARD || k == Net.K_SPUR) continue;
            Double cur = bestPerWay.get(w);
            if (cur == null || d < cur) bestPerWay.put(w, d);
        }
        nearestDist.put(s.id, minAll);
        if (bestPerWay.isEmpty()) {
            if (minAll > 300 || minLink < 0) return new Router.Cand(new int[0], new double[0]);
            double extra = p.unmatchedCostPerM * minAll;
            int[] links = new int[]{minLink};
            if (n.rev[minLink] >= 0 && n.rev[minLink] != minLink) links = new int[]{minLink, n.rev[minLink]};
            return new Router.Cand(links, new double[]{extra, extra});
        }
        List<Map.Entry<Integer, Double>> es = new ArrayList<>(bestPerWay.entrySet());
        es.sort((a, b) -> {
            int c = Double.compare(a.getValue(), b.getValue());
            return c != 0 ? c : Integer.compare(a.getKey(), b.getKey());
        });
        ArrayList<Integer> links = new ArrayList<>();
        ArrayList<Double> extra = new ArrayList<>();
        int nWays = Math.min(es.size(), 8);
        for (int i = 0; i < nWays; i++) {
            int w = es.get(i).getKey();
            double d = es.get(i).getValue();
            int bestL = -1; double bd = Double.POSITIVE_INFINITY;
            for (int k = n.wayHead[w]; k < n.wayHead[w + 1]; k++) {
                int l = n.wayNext[k];
                if (!n.inGraph[l]) continue;
                double dd = n.distToLink(l, s.x, s.y);
                if (dd < bd) { bd = dd; bestL = l; }
            }
            if (bestL < 0) continue;
            double e = p.unmatchedCostPerM * d;
            links.add(bestL); extra.add(e);
            int r = n.rev[bestL];
            if (r >= 0 && r != bestL) { links.add(r); extra.add(e); }
        }
        int[] la = new int[links.size()];
        double[] ea = new double[extra.size()];
        boolean[] st = new boolean[la.length];
        for (int i = 0; i < la.length; i++) { la[i] = links.get(i); ea[i] = extra.get(i); st[i] = n.stub(la[i]); }
        return new Router.Cand(la, ea, st);
    }

    /** nearest through-track candidates around a coordinate, one per (way,direction).
     *  A way whose track_ref matches the published platform code is offered at zero extra cost:
     *  the platform CODE is authoritative even when A03a's way match is a yard ladder. */
    private Router.Cand nearestThrough(double x, double y, double maxDist, String code) {
        ArrayList<int[]> near = new ArrayList<>();   // {link, dist*1000}
        HashSet<String> codes = normCodes(code);
        for (int i = 0; i < n.nLinks; i++) {
            if (!n.inGraph[i]) continue;
            int k = n.wayKlass[n.lWay[i]];
            if (k == Net.K_YARD || k == Net.K_SPUR) continue;
            double d = n.distToLink(i, x, y);
            if (d <= maxDist) near.add(new int[]{i, (int) Math.round(d * 1000), matches(n.lWay[i], codes) ? 0 : 1});
        }
        near.sort((a, b) -> a[2] != b[2] ? Integer.compare(a[2], b[2])
                : (a[1] != b[1] ? Integer.compare(a[1], b[1]) : Integer.compare(a[0], b[0])));
        HashSet<Integer> seenWay = new HashSet<>();
        ArrayList<Integer> links = new ArrayList<>();
        ArrayList<Double> extra = new ArrayList<>();
        for (int[] e : near) {
            int w = n.lWay[e[0]];
            if (!seenWay.add(w)) continue;
            links.add(e[0]); extra.add(e[2] == 0 ? 0.0 : e[1] / 1000.0);
            if (links.size() >= 10) break;
        }
        int[] la = new int[links.size()];
        double[] ea = new double[la.length];
        boolean[] st = new boolean[la.length];
        for (int i = 0; i < la.length; i++) { la[i] = links.get(i); ea[i] = extra.get(i); st[i] = n.stub(la[i]); }
        return new Router.Cand(la, ea, st);
    }

    // ---- direction clustering -------------------------------------------

    private Router.Cand cluster(Sched.StopF s, List<Integer> pool, double baseExtra) {
        ArrayList<int[]> order = new ArrayList<>();
        ArrayList<Double> dist = new ArrayList<>();
        for (int l : pool) {
            order.add(new int[]{l});
            dist.add(n.distToLink(l, s.x, s.y));
        }
        Integer[] idx = new Integer[order.size()];
        for (int i = 0; i < idx.length; i++) idx[i] = i;
        Arrays.sort(idx, (a, b) -> {
            int c = Double.compare(dist.get(a), dist.get(b));
            return c != 0 ? c : Integer.compare(order.get(a)[0], order.get(b)[0]);
        });
        ArrayList<Integer> centres = new ArrayList<>();
        ArrayList<Integer> chosen = new ArrayList<>();
        for (int ii : idx) {
            int l = order.get(ii)[0];
            double b = n.bearing(l);
            boolean found = false;
            for (int c = 0; c < centres.size(); c++) {
                if (circDist(b, centres.get(c)) < 45) { found = true; break; }
            }
            if (found) continue;
            centres.add((int) Math.round(b));
            chosen.add(l);
            if (chosen.size() >= p.maxCandidates) break;
        }
        int[] la = new int[chosen.size()];
        double[] ea = new double[chosen.size()];
        boolean[] st = new boolean[chosen.size()];
        for (int i = 0; i < la.length; i++) { la[i] = chosen.get(i); ea[i] = baseExtra; st[i] = n.stub(la[i]); }
        return new Router.Cand(la, ea, st);
    }

    private static HashSet<String> normCodes(String c) {
        HashSet<String> out = new HashSet<>();
        if (c == null) return out;
        for (String part : c.replace(';', '/').split("/")) {
            StringBuilder d = new StringBuilder();
            for (int i = 0; i < part.length(); i++) if (Character.isDigit(part.charAt(i))) d.append(part.charAt(i));
            String x = d.toString().replaceFirst("^0+(?!$)", "");
            if (!x.isEmpty()) out.add(x);
        }
        return out;
    }

    private boolean matches(int w, HashSet<String> codes) {
        if (codes.isEmpty()) return false;
        String tr = n.wayTrackRef[w];
        if (tr == null || tr.trim().isEmpty()) return false;
        for (String c : normCodes(tr)) if (codes.contains(c)) return true;
        return false;
    }

    private static double circDist(double a, double b) {
        double d = Math.abs(a - b) % 360;
        return d > 180 ? 360 - d : d;
    }
}
