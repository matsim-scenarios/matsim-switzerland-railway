package ch.sbb.prepare.router;

import it.unimi.dsi.fastutil.longs.Long2IntOpenHashMap;
import it.unimi.dsi.fastutil.longs.LongOpenHashSet;

import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamReader;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.zip.GZIPInputStream;

/**
 * Static track graph + all per-way data products.
 *
 * Link ids of the pt2matsim network are decimal numbers that fit in a long; they are kept as long
 * and re-stringified on output so that no 665k-element String table is needed.
 */
public final class Net {

    public static final byte K_MAIN = 0, K_BRANCH = 1, K_OTHER = 2, K_SIDING = 3, K_YARD = 4,
            K_SPUR = 5, K_CROSS = 6, K_UNKNOWN = 7;

    public int nLinks, nNodes, nWays;
    public long[] linkId;
    public int[] lFrom, lTo, lWay;
    public float[] lLen, lSpeed;
    /** R3-corrected routing / output speed (== lSpeed when R3 is off or the way is speed-tagged) */
    public float[] lRouteSpeed;
    /** routable heavy-rail link in component 0 -> usable by the router */
    public boolean[] inGraph;
    /** link runs in the node order of its OSM way */
    public boolean[] fwd;
    public boolean[] fwdKnown;
    /** reverse link (same way, opposite direction) or -1 */
    public int[] rev;

    public int[] outStart, outLinks, inStart, inLinks;
    public int[] nbrCount;              // number of distinct neighbour nodes in the routing graph

    public long[] nodeId;
    public double[] nx, ny;

    public long[] wayId;
    public byte[] wayKlass;
    public boolean[] wayRoutable;
    /** resolved direction preference: -1 backward, 0 both/unknown, +1 forward */
    public byte[] wayDir;
    public float[] wayDirConf;
    /** where wayDir came from: 0 none, 1 DIR table, 2 parallel partner (R2), 3 chain (R2), 4 DIR 'both' */
    public byte[] wayDirSrc;
    /** DIR table raw */
    public byte[] wayDirRaw;
    public float[] wayDirConfRaw;
    public boolean[] wayBoth;
    /** R2: no direction evidence at all -> both directions cost x1.5 */
    public boolean[] wayUnknownPenalty;
    /** A06 way_usage_direction alone: bit1 = used forward only, bit2 = used backward only */
    public byte[] wayRelDir;
    /** A01 raw tags */
    public boolean[] wayTagged;        // has any OSM maxspeed tag
    public String[] wayTrackRef;
    /** G3: OSM line number (tag ref, e.g. 600 = Gotthard), empty when absent */
    public String[] wayLineRef;
    public byte[] wayUsage;            // 1 main, 2 branch, 3 industrial, 4 military, 5 freight, 6 tourism, 7 siding, 0 other
    public int[] wayPassengerLines;    // -1 if tag absent
    public boolean[] wayFreightOnly;

    /** CSR: links of each way */
    public int[] wayHead, wayNext;

    public LongOpenHashSet legalTurns;
    public boolean[] isJunction;

    // ------------------------------------------------------------------ io

    public static InputStream open(String f) throws IOException {
        InputStream in = new FileInputStream(f);
        if (f.endsWith(".gz")) in = new GZIPInputStream(in, 1 << 16);
        return new BufferedInputStream(in, 1 << 20);
    }

    // ------------------------------------------------------------------ csv

    /** quote-aware csv line split (ways.csv contains quoted names with commas) */
    public static String[] csv(String line) {
        ArrayList<String> out = new ArrayList<>(32);
        StringBuilder sb = new StringBuilder(64);
        boolean q = false;
        for (int i = 0, n = line.length(); i < n; i++) {
            char c = line.charAt(i);
            if (q) {
                if (c == '"') {
                    if (i + 1 < n && line.charAt(i + 1) == '"') { sb.append('"'); i++; }
                    else q = false;
                } else sb.append(c);
            } else if (c == '"') q = true;
            else if (c == ',') { out.add(sb.toString()); sb.setLength(0); }
            else sb.append(c);
        }
        out.add(sb.toString());
        return out.toArray(new String[0]);
    }

    private static BufferedReader reader(String f) throws IOException {
        return new BufferedReader(new InputStreamReader(open(f), StandardCharsets.UTF_8), 1 << 20);
    }

    // ------------------------------------------------------------------ load

    public static final class Options {
        public double dirConfMin = 0.6;
        public boolean r2 = true;
        public boolean r3 = true;
    }

    public static Net load(String networkFile, String dataDir, boolean verbose) throws Exception {
        return load(networkFile, dataDir, verbose, new Options());
    }

    public static Net load(String networkFile, String dataDir, boolean verbose, Options o) throws Exception {
        Net n = new Net();
        n.readNetwork(networkFile);
        if (verbose) System.out.printf("network: %d nodes, %d links%n", n.nNodes, n.nLinks);
        n.readWays(dataDir + "/A01/ways.csv");
        n.readComponents(dataDir + "/A01/components.csv");
        if (verbose) System.out.printf("ways: %d, routable+comp0 links: %d%n", n.nWays, n.countGraphLinks());
        n.readSegments(dataDir + "/A05/segment_block.csv");
        n.readTurns(dataDir + "/A04/turns.csv");
        n.buildIndexes();
        if (verbose) System.out.printf("graph built: out-links %d, junctions %d%n", n.outLinks.length, n.countJunctions());
        n.readDirectionFinal(dataDir + "/DIR/direction_final.csv");
        n.readRelationUsage(dataDir + "/A06/way_usage_direction.csv");
        n.resolveDirections(o.dirConfMin, o.r2);
        n.prepareSpeeds(o.r3);
        return n;
    }

    private void readNetwork(String file) throws Exception {
        int capL = 1 << 20, capN = 1 << 19;
        linkId = new long[capL]; lFrom = new int[capL]; lTo = new int[capL];
        lLen = new float[capL]; lSpeed = new float[capL];
        nodeId = new long[capN]; nx = new double[capN]; ny = new double[capN];
        nLinks = 0; nNodes = 0;
        Long2IntOpenHashMap nodeMap = new Long2IntOpenHashMap(1 << 20);
        nodeMap.defaultReturnValue(-1);

        XMLInputFactory f = XMLInputFactory.newInstance();
        f.setProperty(XMLInputFactory.IS_COALESCING, Boolean.TRUE);
        f.setProperty(XMLInputFactory.SUPPORT_DTD, Boolean.FALSE);
        f.setProperty(XMLInputFactory.IS_SUPPORTING_EXTERNAL_ENTITIES, Boolean.FALSE);

        long[] curWay = new long[1];
        boolean inLink = false;
        int curFrom = -1, curTo = -1;
        float curLen = 0, curSpeed = 0;
        long curId = 0;
        try (InputStream in = open(file)) {
            XMLStreamReader r = f.createXMLStreamReader(in);
            while (r.hasNext()) {
                int ev = r.next();
                if (ev == XMLStreamConstants.START_ELEMENT) {
                    String tag = r.getLocalName();
                    if (tag.equals("node")) {
                        long id = Long.parseLong(r.getAttributeValue(null, "id"));
                        double x = Double.parseDouble(r.getAttributeValue(null, "x"));
                        double y = Double.parseDouble(r.getAttributeValue(null, "y"));
                        if (nNodes == nodeId.length) {
                            nodeId = java.util.Arrays.copyOf(nodeId, nNodes * 2);
                            nx = java.util.Arrays.copyOf(nx, nNodes * 2);
                            ny = java.util.Arrays.copyOf(ny, nNodes * 2);
                        }
                        nodeId[nNodes] = id; nx[nNodes] = x; ny[nNodes] = y;
                        nodeMap.put(id, nNodes);
                        nNodes++;
                    } else if (tag.equals("link")) {
                        inLink = true;
                        curId = Long.parseLong(r.getAttributeValue(null, "id"));
                        curFrom = nodeMap.get(Long.parseLong(r.getAttributeValue(null, "from")));
                        curTo = nodeMap.get(Long.parseLong(r.getAttributeValue(null, "to")));
                        curLen = Float.parseFloat(r.getAttributeValue(null, "length"));
                        curSpeed = Float.parseFloat(r.getAttributeValue(null, "freespeed"));
                        curWay[0] = -1;
                    } else if (tag.equals("attribute") && inLink) {
                        String nm = r.getAttributeValue(null, "name");
                        if ("osm:way:id".equals(nm)) {
                            String txt = r.getElementText().trim();
                            try { curWay[0] = Long.parseLong(txt); } catch (NumberFormatException e) { curWay[0] = -1; }
                        }
                    }
                } else if (ev == XMLStreamConstants.END_ELEMENT) {
                    if (r.getLocalName().equals("link") && inLink) {
                        inLink = false;
                        if (nLinks == linkId.length) {
                            int c = nLinks * 2;
                            linkId = java.util.Arrays.copyOf(linkId, c);
                            lFrom = java.util.Arrays.copyOf(lFrom, c);
                            lTo = java.util.Arrays.copyOf(lTo, c);
                            lLen = java.util.Arrays.copyOf(lLen, c);
                            lSpeed = java.util.Arrays.copyOf(lSpeed, c);
                        }
                        linkId[nLinks] = curId; lFrom[nLinks] = curFrom; lTo[nLinks] = curTo;
                        lLen[nLinks] = curLen; lSpeed[nLinks] = curSpeed;
                        // stash the way id in lWay temporarily (reallocated later)
                        if (nLinks >= wayTmp.length) wayTmp = java.util.Arrays.copyOf(wayTmp, Math.max(1024, nLinks * 2));
                        wayTmp[nLinks] = curWay[0];
                        nLinks++;
                    }
                }
            }
            r.close();
        }
        wayIdOfLink = java.util.Arrays.copyOf(wayTmp, nLinks);
        wayTmp = null;
        linkId = java.util.Arrays.copyOf(linkId, nLinks);
        lFrom = java.util.Arrays.copyOf(lFrom, nLinks);
        lTo = java.util.Arrays.copyOf(lTo, nLinks);
        lLen = java.util.Arrays.copyOf(lLen, nLinks);
        lSpeed = java.util.Arrays.copyOf(lSpeed, nLinks);
        nodeId = java.util.Arrays.copyOf(nodeId, nNodes);
        nx = java.util.Arrays.copyOf(nx, nNodes);
        ny = java.util.Arrays.copyOf(ny, nNodes);
    }

    private long[] wayTmp = new long[0];
    private long[] wayIdOfLink;

    private void readWays(String file) throws IOException {
        int cap = 1 << 17;
        wayId = new long[cap]; wayKlass = new byte[cap]; wayRoutable = new boolean[cap];
        wayTagged = new boolean[cap]; wayTrackRef = new String[cap]; wayUsage = new byte[cap];
        wayLineRef = new String[cap];
        wayPassengerLines = new int[cap]; wayFreightOnly = new boolean[cap];
        nWays = 0;
        Long2IntOpenHashMap idx = new Long2IntOpenHashMap(cap);
        idx.defaultReturnValue(-1);
        try (BufferedReader br = reader(file)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iId = col(h, "way_id"), iKlass = col(h, "klass"), iRout = col(h, "routable");
            int iMax = col(h, "maxspeed"), iMaxF = col(h, "maxspeed_forward"), iMaxB = col(h, "maxspeed_backward");
            int iRef = col(h, "track_ref"), iUsage = col(h, "usage"), iPl = col(h, "passenger_lines");
            int iLine = col(h, "ref");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = csv(line);
                long id = Long.parseLong(c[iId]);
                if (nWays == wayId.length) {
                    int cc = nWays * 2;
                    wayId = java.util.Arrays.copyOf(wayId, cc);
                    wayKlass = java.util.Arrays.copyOf(wayKlass, cc);
                    wayRoutable = java.util.Arrays.copyOf(wayRoutable, cc);
                    wayTagged = java.util.Arrays.copyOf(wayTagged, cc);
                    wayTrackRef = java.util.Arrays.copyOf(wayTrackRef, cc);
                    wayLineRef = java.util.Arrays.copyOf(wayLineRef, cc);
                    wayUsage = java.util.Arrays.copyOf(wayUsage, cc);
                    wayPassengerLines = java.util.Arrays.copyOf(wayPassengerLines, cc);
                    wayFreightOnly = java.util.Arrays.copyOf(wayFreightOnly, cc);
                }
                wayId[nWays] = id;
                wayKlass[nWays] = klassOf(c[iKlass]);
                wayRoutable[nWays] = c[iRout].equals("1");
                wayTagged[nWays] = !c[iMax].isEmpty() || !c[iMaxF].isEmpty() || !c[iMaxB].isEmpty();
                wayTrackRef[nWays] = c[iRef];
                wayLineRef[nWays] = c[iLine].trim();
                wayUsage[nWays] = usageOf(c[iUsage]);
                wayPassengerLines[nWays] = c[iPl].isEmpty() ? -1 : Integer.parseInt(c[iPl]);
                wayFreightOnly[nWays] = (c[iPl].equals("0") || wayUsage[nWays] == 3 || wayUsage[nWays] == 4
                        || wayUsage[nWays] == 5);
                idx.put(id, nWays);
                nWays++;
            }
        }
        wayId = java.util.Arrays.copyOf(wayId, nWays);
        wayKlass = java.util.Arrays.copyOf(wayKlass, nWays);
        wayRoutable = java.util.Arrays.copyOf(wayRoutable, nWays);
        wayTagged = java.util.Arrays.copyOf(wayTagged, nWays);
        wayTrackRef = java.util.Arrays.copyOf(wayTrackRef, nWays);
        wayLineRef = java.util.Arrays.copyOf(wayLineRef, nWays);
        wayUsage = java.util.Arrays.copyOf(wayUsage, nWays);
        wayPassengerLines = java.util.Arrays.copyOf(wayPassengerLines, nWays);
        wayFreightOnly = java.util.Arrays.copyOf(wayFreightOnly, nWays);

        int[] w = new int[nLinks];
        for (int i = 0; i < nLinks; i++) w[i] = idx.get(wayIdOfLink[i]);
        lWay = w;
    }

    private static int col(String[] h, String name) {
        for (int i = 0; i < h.length; i++) if (h[i].equals(name)) return i;
        throw new IllegalStateException("no column " + name);
    }

    private static byte klassOf(String k) {
        switch (k) {
            case "main": return K_MAIN;
            case "branch": return K_BRANCH;
            case "siding": return K_SIDING;
            case "yard": return K_YARD;
            case "spur": return K_SPUR;
            case "crossover": return K_CROSS;
            case "rail_other": return K_OTHER;
            default: return K_UNKNOWN;
        }
    }

    private static byte usageOf(String u) {
        switch (u) {
            case "main": return 1;
            case "branch": return 2;
            case "industrial": return 3;
            case "military": return 4;
            case "freight": return 5;
            case "tourism": return 6;
            case "siding": return 7;
            default: return 0;
        }
    }

    private boolean[] comp0;

    private void readComponents(String file) throws IOException {
        comp0 = new boolean[nWays];
        try (BufferedReader br = reader(file)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iW = col(h, "way_id"), iC = col(h, "component_id");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = splitSimple(line);
                long w = Long.parseLong(c[iW]);
                int wi = wayIndex(w);
                if (wi >= 0 && c[iC].equals("0")) comp0[wi] = true;
            }
        }
        inGraph = new boolean[nLinks];
        for (int i = 0; i < nLinks; i++) {
            int w = lWay[i];
            inGraph[i] = w >= 0 && wayRoutable[w] && comp0[w];
        }
    }

    private Long2IntOpenHashMap wayIdxMap;

    private int wayIndex(long id) {
        if (wayIdxMap == null) {
            wayIdxMap = new Long2IntOpenHashMap(nWays);
            wayIdxMap.defaultReturnValue(-1);
            for (int i = 0; i < nWays; i++) wayIdxMap.put(wayId[i], i);
        }
        return wayIdxMap.get(id);
    }

    public int wayIndexPublic(long id) { return wayIndex(id); }

    public static String[] splitSimple(String line) {
        int n = 1;
        for (int i = 0; i < line.length(); i++) if (line.charAt(i) == ',') n++;
        String[] out = new String[n];
        int s = 0, k = 0;
        for (int i = 0; i < line.length(); i++) {
            if (line.charAt(i) == ',') { out[k++] = line.substring(s, i); s = i + 1; }
        }
        out[k] = line.substring(s);
        return out;
    }

    /** A05 segment table: one row per OSM segment in way order -> forward = (from,to) listed. */
    private void readSegments(String file) throws IOException {
        fwd = new boolean[nLinks];
        fwdKnown = new boolean[nLinks];
        LongOpenHashSet fwdSet = new LongOpenHashSet(1 << 20);
        try (BufferedReader br = reader(file)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iW = col(h, "way_id"), iF = col(h, "from_node"), iT = col(h, "to_node");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = splitSimple(line);
                int w = wayIndex(Long.parseLong(c[iW]));
                if (w < 0) continue;
                int f = nodeIndex(Long.parseLong(c[iF]));
                int t = nodeIndex(Long.parseLong(c[iT]));
                if (f < 0 || t < 0) continue;
                fwdSet.add(key3(w, f, t));
            }
        }
        for (int i = 0; i < nLinks; i++) {
            int w = lWay[i];
            if (w < 0) continue;
            if (fwdSet.contains(key3(w, lFrom[i], lTo[i]))) { fwd[i] = true; fwdKnown[i] = true; }
            else if (fwdSet.contains(key3(w, lTo[i], lFrom[i]))) { fwd[i] = false; fwdKnown[i] = true; }
            else { fwd[i] = true; fwdKnown[i] = false; }
        }
    }

    private Long2IntOpenHashMap nodeMap2;

    public int nodeIndex(long id) {
        if (nodeMap2 == null) {
            nodeMap2 = new Long2IntOpenHashMap(nNodes);
            nodeMap2.defaultReturnValue(-1);
            for (int i = 0; i < nNodes; i++) nodeMap2.put(nodeId[i], i);
        }
        return nodeMap2.get(id);
    }

    private static long key3(int a, int b, int c) {
        return ((long) a << 38) | ((long) b << 19) | c;
    }

    private static long key3n(int a, int b, int c) {
        return ((long) a << 40) | ((long) b << 20) | c;
    }

    // ------------------------------------------------------------------ direction

    private void readDirectionFinal(String file) throws IOException {
        wayDirRaw = new byte[nWays];
        wayDirConfRaw = new float[nWays];
        wayBoth = new boolean[nWays];
        try (BufferedReader br = reader(file)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iW = col(h, "way_id"), iD = col(h, "direction"), iC = col(h, "confidence");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = splitSimple(line);
                int w = wayIndex(Long.parseLong(c[iW]));
                if (w < 0) continue;
                String d = c[iD];
                wayDirRaw[w] = d.equals("forward") ? (byte) 1 : d.equals("backward") ? (byte) -1 : (byte) 0;
                wayBoth[w] = d.equals("both");
                wayDirConfRaw[w] = Float.parseFloat(c[iC]);
            }
        }
    }

    private void readRelationUsage(String relFile) throws IOException {
        wayRelDir = new byte[nWays];
        try (BufferedReader br = reader(relFile)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iW = col(h, "way_id"), iF = col(h, "n_rel_forward"), iB = col(h, "n_rel_backward");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = splitSimple(line);
                int w = wayIndex(Long.parseLong(c[iW]));
                if (w < 0) continue;
                int f = Integer.parseInt(c[iF]), b = Integer.parseInt(c[iB]);
                wayRelDir[w] = (byte) ((f > 0 && b == 0 ? 1 : 0) | (b > 0 && f == 0 ? 2 : 0));
            }
        }
    }

    /**
     * Resolve the direction of every way. R2: running-line ways with unknown/low-confidence direction
     * inherit from a parallel partner (2.5-8 m, opposite direction) or from the neighbouring way of the
     * same track chain; if still unknown they get a mild x1.5 in both directions.
     */
    public void resolveDirections(double cut, boolean r2) {
        wayDir = new byte[nWays];
        wayDirConf = new float[nWays];
        wayDirSrc = new byte[nWays];
        wayUnknownPenalty = new boolean[nWays];
        int nKnown = 0, nBoth = 0, nPar = 0, nChain = 0, nMild = 0;
        for (int w = 0; w < nWays; w++) {
            if (wayBoth[w]) { wayDirSrc[w] = 4; nBoth++; continue; }
            if (wayDirRaw[w] != 0 && wayDirConfRaw[w] >= cut) {
                wayDir[w] = wayDirRaw[w];
                wayDirConf[w] = wayDirConfRaw[w];
                wayDirSrc[w] = 1;
                nKnown++;
            }
        }
        if (r2) {
            // ---- pass 1: parallel partner
            for (int w = 0; w < nWays; w++) {
                if (wayDirSrc[w] != 0 || !isRunningLine(w)) continue;
                int[] inh = parallelPartner(w, cut);
                if (inh != null) {
                    wayDir[w] = (byte) inh[0];
                    wayDirConf[w] = (float) Math.min(1.0, inh[1] / 100.0 * 0.9);
                    wayDirSrc[w] = 2;
                    nPar++;
                }
            }
            // ---- pass 2: chain propagation (repeat so chains resolve over several hops)
            for (int round = 0; round < 5; round++) {
                int changed = 0;
                for (int w = 0; w < nWays; w++) {
                    if (wayDirSrc[w] != 0 || !isRunningLine(w)) continue;
                    int[] inh = chainNeighbour(w, cut);
                    if (inh != null) {
                        wayDir[w] = (byte) inh[0];
                        wayDirConf[w] = (float) Math.min(1.0, inh[1] / 100.0 * 0.85);
                        wayDirSrc[w] = 3;
                        changed++;
                    }
                }
                nChain += changed;
                if (changed == 0) break;
            }
            for (int w = 0; w < nWays; w++) {
                if (wayDirSrc[w] == 0 && isRunningLine(w)) { wayUnknownPenalty[w] = true; nMild++; }
            }
        }
        System.out.printf(Locale.ROOT,
                "direction: DIR known %d, both %d; R2 parallel %d, chain %d, still unknown (x1.5) %d%n",
                nKnown, nBoth, nPar, nChain, nMild);
    }

    private boolean isRunningLine(int w) {
        byte k = wayKlass[w];
        return k == K_MAIN || k == K_BRANCH || k == K_OTHER;
    }

    private static double angDiff(double a, double b) {
        double d = Math.abs(a - b) % 360;
        return d > 180 ? 360 - d : d;
    }

    public double bearing(int l) {
        double dx = nx[lTo[l]] - nx[lFrom[l]];
        double dy = ny[lTo[l]] - ny[lFrom[l]];
        double a = Math.toDegrees(Math.atan2(dx, dy));
        return a < 0 ? a + 360 : a;
    }

    /** distance from point to link segment */
    public double distToLink(int l, double px, double py) {
        double ax = nx[lFrom[l]], ay = ny[lFrom[l]], bx = nx[lTo[l]], by = ny[lTo[l]];
        double dx = bx - ax, dy = by - ay;
        double dd = dx * dx + dy * dy;
        double t = dd > 1e-9 ? ((px - ax) * dx + (py - ay) * dy) / dd : 0;
        if (t < 0) t = 0; else if (t > 1) t = 1;
        double qx = ax + t * dx, qy = ay + t * dy;
        return Math.hypot(px - qx, py - qy);
    }

    private HashMap<Long, int[]> grid;
    private static final double CELL = 40.0;

    private static long cellKey(int cx, int cy) {
        return ((long) cx << 32) ^ (cy & 0xffffffffL);
    }

    private void buildGrid() {
        HashMap<Long, ArrayList<Integer>> tmp = new HashMap<>();
        for (int l = 0; l < nLinks; l++) {
            if (!inGraph[l]) continue;
            int w = lWay[l];
            if (w < 0 || wayDirSrc[w] != 1) continue;
            double mx = (nx[lFrom[l]] + nx[lTo[l]]) * 0.5;
            double my = (ny[lFrom[l]] + ny[lTo[l]]) * 0.5;
            int cx = (int) Math.floor(mx / CELL), cy = (int) Math.floor(my / CELL);
            long k = cellKey(cx, cy);
            tmp.computeIfAbsent(k, x -> new ArrayList<>()).add(l);
        }
        grid = new HashMap<>(tmp.size() * 2);
        for (HashMap.Entry<Long, ArrayList<Integer>> e : tmp.entrySet()) {
            int[] a = new int[e.getValue().size()];
            for (int i = 0; i < a.length; i++) a[i] = e.getValue().get(i);
            grid.put(e.getKey(), a);
        }
    }

    /** R2 pass 1: nearest parallel known-direction link 2.5-8 m away -> {dir, conf} of this way or null. */
    private int[] parallelPartner(int w, double cut) {
        if (grid == null) buildGrid();
        double bestD = 1e9;
        int bestL = -1;
        for (int k = wayHead[w]; k < wayHead[w + 1]; k++) {
            int l = wayNext[k];
            if (!inGraph[l]) continue;
            double mx = (nx[lFrom[l]] + nx[lTo[l]]) * 0.5;
            double my = (ny[lFrom[l]] + ny[lTo[l]]) * 0.5;
            int cx = (int) Math.floor(mx / CELL), cy = (int) Math.floor(my / CELL);
            double bl = bearing(l);
            for (int ix = -1; ix <= 1; ix++)
                for (int iy = -1; iy <= 1; iy++) {
                    int[] lst = grid.get(cellKey(cx + ix, cy + iy));
                    if (lst == null) continue;
                    for (int m : lst) {
                        int wm = lWay[m];
                        if (wm == w || wayDirSrc[wm] != 1) continue;
                        double bm = bearing(m);
                        double ad = angDiff(bl, bm);
                        if (ad > 25 && ad < 155) continue;      // not parallel
                        double d = distToLink(m, mx, my);
                        if (d < 2.5 || d > 8.0) continue;
                        if (d < bestD) { bestD = d; bestL = m; }
                    }
                }
        }
        if (bestL < 0) return null;
        int wm = lWay[bestL];
        boolean sameOrient = angDiff(bearing(bestPickLinkFor(w, bestL)), bearing(bestL)) < 90;
        int dir = sameOrient ? -wayDir[wm] : wayDir[wm];
        return new int[]{dir, Math.round(wayDirConf[wm] * 100)};
    }

    private int bestPickLinkFor(int w, int partnerLink) {
        // the link of way w closest to the partner link's midpoint (for orientation)
        double mx = (nx[lFrom[partnerLink]] + nx[lTo[partnerLink]]) * 0.5;
        double my = (ny[lFrom[partnerLink]] + ny[lTo[partnerLink]]) * 0.5;
        int best = -1; double bd = 1e9;
        for (int k = wayHead[w]; k < wayHead[w + 1]; k++) {
            int l = wayNext[k];
            double d = distToLink(l, mx, my);
            if (d < bd) { bd = d; best = l; }
        }
        return best >= 0 ? best : partnerLink;
    }

    /** R2 pass 2: continuation of the same track chain at a degree-2 end node -> {dir, conf} or null. */
    private int[] chainNeighbour(int w, double cut) {
        for (int k = wayHead[w]; k < wayHead[w + 1]; k++) {
            int l = wayNext[k];
            if (!inGraph[l]) continue;
            for (int side = 0; side < 2; side++) {
                int node = side == 0 ? lFrom[l] : lTo[l];
                // only an end of the way (single link of this way at that node)
                int cnt = 0;
                for (int q = inStart[node]; q < inStart[node + 1]; q++) if (lWay[inLinks[q]] == w) cnt++;
                for (int q = outStart[node]; q < outStart[node + 1]; q++) if (lWay[outLinks[q]] == w) cnt++;
                if (cnt != 2) continue;   // each link counts once in and once out at its ends
                for (int q = outStart[node]; q < outStart[node + 1]; q++) {
                    int m = outLinks[q];
                    if (lWay[m] == w || !inGraph[m]) continue;
                    int wm = lWay[m];
                    if (wm < 0 || wayDirSrc[wm] != 1) continue;
                    if (angDiff(bearing(l), bearing(m)) > 45) continue;
                    int dir = travelDot(l, m, node) > 0 ? wayDir[wm] : -wayDir[wm];
                    return new int[]{dir, Math.round(wayDirConf[wm] * 100)};
                }
                for (int q = inStart[node]; q < inStart[node + 1]; q++) {
                    int m = inLinks[q];
                    if (lWay[m] == w || !inGraph[m]) continue;
                    int wm = lWay[m];
                    if (wm < 0 || wayDirSrc[wm] != 1) continue;
                    if (angDiff(bearing(l), bearing(m)) > 45) continue;
                    int dir = travelDot(l, m, node) > 0 ? wayDir[wm] : -wayDir[wm];
                    return new int[]{dir, Math.round(wayDirConf[wm] * 100)};
                }
            }
        }
        return null;
    }

    /** scalar product of the two forward travel directions at a shared node. */
    private double travelDot(int l, int m, int node) {
        double[] a = forwardTravel(l, node);
        double[] b = forwardTravel(m, node);
        return a[0] * b[0] + a[1] * b[1];
    }

    /** unit-ish vector of the forward travel direction of link l through the shared node. */
    private double[] forwardTravel(int l, int node) {
        double ox, oy;
        if (node == lFrom[l]) { ox = nx[lTo[l]] - nx[node]; oy = ny[lTo[l]] - ny[node]; }
        else { ox = nx[lFrom[l]] - nx[node]; oy = ny[lFrom[l]] - ny[node]; }
        // forward travel leaves the node on this link iff (node==from)==fwd
        boolean leaves = (node == lFrom[l]) == fwd[l];
        double s = leaves ? 1 : -1;
        return new double[]{s * ox, s * oy};
    }

    // ------------------------------------------------------------------ speeds

    /** R3: realistic default speeds for ways without an OSM maxspeed tag. */
    public void prepareSpeeds(boolean on) {
        lRouteSpeed = lSpeed.clone();
        if (!on) return;
        double[] km = new double[8];
        int[] n = new int[8];
        for (int l = 0; l < nLinks; l++) {
            int w = lWay[l];
            if (w < 0 || wayTagged[w]) continue;
            double def = defaultSpeed(w);
            if (def > 0 && lSpeed[l] > def + 1e-6) {
                lRouteSpeed[l] = (float) def;
                km[wayKlass[w]] += lLen[l] / 1000.0;
                n[wayKlass[w]]++;
            }
        }
        String[] names = {"main", "branch", "rail_other", "siding", "yard", "spur", "crossover", "unknown"};
        System.out.println("R3 speed correction (untagged ways, lowered km per klass):");
        for (int k = 0; k < 8; k++)
            if (n[k] > 0) System.out.printf(Locale.ROOT, "   %-10s %6d links %8.1f km%n", names[k], n[k], km[k]);
    }

    private double defaultSpeed(int w) {
        switch (wayKlass[w]) {
            case K_YARD: case K_SPUR: return 40.0 / 3.6;
            case K_SIDING: return 60.0 / 3.6;
            case K_CROSS: return 60.0 / 3.6;
            case K_MAIN: case K_BRANCH: case K_OTHER:
                if (wayTrackRef[w] != null && !wayTrackRef[w].trim().isEmpty()
                        && !wayTrackRef[w].trim().equals("0")) return 80.0 / 3.6;
                return 120.0 / 3.6;
            default: return -1;
        }
    }

    // ------------------------------------------------------------------ turns / indexes

    private void readTurns(String file) throws IOException {
        legalTurns = new LongOpenHashSet(1 << 18);
        isJunction = new boolean[nNodes];
        try (BufferedReader br = reader(file)) {
            String line = br.readLine();
            String[] h = csv(line);
            int iN = col(h, "node_id"), iF = col(h, "from_node"), iT = col(h, "to_node"), iL = col(h, "legal");
            while ((line = br.readLine()) != null) {
                if (line.isEmpty()) continue;
                String[] c = splitSimple(line);
                int nd = nodeIndex(Long.parseLong(c[iN]));
                if (nd < 0) continue;
                isJunction[nd] = true;
                if (!c[iL].equals("1")) continue;
                int f = nodeIndex(Long.parseLong(c[iF]));
                int t = nodeIndex(Long.parseLong(c[iT]));
                if (f < 0 || t < 0) continue;
                legalTurns.add(key3n(nd, f, t));
                legalTurns.add(key3n(nd, t, f));
            }
        }
    }

    private void buildIndexes() {
        outStart = new int[nNodes + 1];
        inStart = new int[nNodes + 1];
        for (int i = 0; i < nLinks; i++) {
            outStart[lFrom[i] + 1]++;
            inStart[lTo[i] + 1]++;
        }
        for (int i = 0; i < nNodes; i++) { outStart[i + 1] += outStart[i]; inStart[i + 1] += inStart[i]; }
        outLinks = new int[outStart[nNodes]];
        inLinks = new int[inStart[nNodes]];
        int[] po = outStart.clone(), pi = inStart.clone();
        for (int i = 0; i < nLinks; i++) {
            outLinks[po[lFrom[i]]++] = i;
            inLinks[pi[lTo[i]]++] = i;
        }

        nbrCount = new int[nNodes];
        int[] mark = new int[nNodes];
        int stamp = 1;
        for (int v = 0; v < nNodes; v++) {
            stamp++;
            int c = 0;
            for (int k = inStart[v]; k < inStart[v + 1]; k++) {
                int u = lFrom[inLinks[k]];
                if (mark[u] != stamp) { mark[u] = stamp; c++; }
            }
            for (int k = outStart[v]; k < outStart[v + 1]; k++) {
                int u = lTo[outLinks[k]];
                if (mark[u] != stamp) { mark[u] = stamp; c++; }
            }
            nbrCount[v] = c;
        }

        rev = new int[nLinks];
        java.util.Arrays.fill(rev, -1);
        for (int i = 0; i < nLinks; i++) {
            int best = -1;
            for (int k = outStart[lTo[i]]; k < outStart[lTo[i] + 1]; k++) {
                int j = outLinks[k];
                if (lTo[j] != lFrom[i]) continue;
                if (j == i) continue;
                if (lWay[j] == lWay[i]) { best = j; break; }
                if (best < 0) best = j;
            }
            rev[i] = best;
        }

        wayHead = new int[nWays + 1];
        for (int i = 0; i < nLinks; i++) if (lWay[i] >= 0) wayHead[lWay[i] + 1]++;
        for (int i = 0; i < nWays; i++) wayHead[i + 1] += wayHead[i];
        wayNext = new int[nLinks];
        int[] pw = wayHead.clone();
        for (int i = 0; i < nLinks; i++) if (lWay[i] >= 0) wayNext[pw[lWay[i]]++] = i;
    }

    private int countGraphLinks() { return inGraph == null ? 0 : (int) countTrue(inGraph); }

    private int countJunctions() { return (int) countTrue(isJunction); }

    private static long countTrue(boolean[] a) {
        long c = 0;
        for (boolean b : a) if (b) c++;
        return c;
    }

    public boolean legal(int node, int from, int to) {
        if (!isJunction[node]) return true;
        return legalTurns.contains(key3n(node, from, to));
    }

    /** R5: the only way onward is the link's own reverse (stub / bay / buffer stop). */
    public boolean stub(int l) {
        int node = lTo[l];
        int r = rev[l];
        for (int k = outStart[node]; k < outStart[node + 1]; k++) {
            int b = outLinks[k];
            if (b == r) continue;
            if (!inGraph[b]) continue;
            if (!legal(node, lFrom[l], lTo[b])) continue;
            return false;
        }
        return true;
    }
}
