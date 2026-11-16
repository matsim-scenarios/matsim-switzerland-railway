package ch.sbb.prepare.router;

/** small debugging helper: java rt2.Diag <nodeId> <fromNodeId> <toNodeId> ... */
public final class Diag {
    public static void main(String[] a) throws Exception {
        Net n = Net.load("matsim_input/tmp/network_osm.xml.gz", "matsim_input/tables", false);
        long node = Long.parseLong(a[0]);
        int ni = n.nodeIndex(node);
        System.out.println("node " + node + " -> idx " + ni + " isJunction=" + (ni >= 0 && n.isJunction[ni])
                + " nbrs=" + (ni >= 0 ? n.nbrCount[ni] : -1));
        if (ni >= 0) {
            System.out.println("  out links: " + (n.outStart[ni + 1] - n.outStart[ni]));
            for (int k = n.outStart[ni]; k < n.outStart[ni + 1]; k++) {
                int l = n.outLinks[k];
                System.out.println("   link " + n.linkId[l] + " " + n.nodeId[n.lFrom[l]] + "->" + n.nodeId[n.lTo[l]]
                        + " way=" + (n.lWay[l] >= 0 ? n.wayId[n.lWay[l]] : -1) + " inGraph=" + n.inGraph[l]);
            }
            System.out.println("  in links: " + (n.inStart[ni + 1] - n.inStart[ni]));
            for (int k = n.inStart[ni]; k < n.inStart[ni + 1]; k++) {
                int l = n.inLinks[k];
                System.out.println("   link " + n.linkId[l] + " " + n.nodeId[n.lFrom[l]] + "->" + n.nodeId[n.lTo[l]]
                        + " way=" + (n.lWay[l] >= 0 ? n.wayId[n.lWay[l]] : -1));
            }
        }
        for (int i = 1; i + 1 < a.length; i += 2) {
            long from = Long.parseLong(a[i]), to = Long.parseLong(a[i + 1]);
            int fi = n.nodeIndex(from), ti = n.nodeIndex(to);
            System.out.println("legal(" + node + "," + from + "," + to + ") = " + n.legal(ni, fi, ti)
                    + "  keyFrom=" + fi + " keyTo=" + ti);
        }
    }
}
