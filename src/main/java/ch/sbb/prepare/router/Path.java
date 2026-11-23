package ch.sbb.prepare.router;

/** debug helper: shortest legal path between two directed link ids (output network ids) */
public final class Path {
    public static void main(String[] a) throws Exception {
        Net net = Net.load("matsim_input/tmp/network_osm.xml.gz", "matsim_input/tables", false);
        Router.Params p = new Router.Params();
        Router r = new Router(net, p);
        Router.Sear se = r.newSearcher();
        for (int i = 0; i + 1 < a.length; i += 2) {
            int s = link(net, Long.parseLong(a[i]));
            int t = link(net, Long.parseLong(a[i + 1]));
            if (s < 0 || t < 0) { System.out.println("unknown link " + a[i] + " " + a[i + 1]); continue; }
            double[][] dd = new double[1][1];
            int[][][] pp = new int[1][1][];
            r.legSearchPublic(s, 0, new int[]{t}, se, dd[0], pp[0], false);
            int[] path = pp[0][0];
            System.out.printf("path %s -> %s: %s%n", a[i], a[i + 1], path == null ? "NONE" : path.length + " links, " +
                    String.format(java.util.Locale.ROOT, "%.1f s", r.legRuntime(path)));
            if (path != null) {
                for (int l : path)
                    System.out.println("   " + net.linkId[l] + " way=" + (net.lWay[l] >= 0 ? net.wayId[net.lWay[l]] : -1)
                            + " len=" + net.lLen[l] + " fwd=" + net.fwd[l]
                            + " dir=" + (net.lWay[l] >= 0 ? net.wayDir[net.lWay[l]] : 0) + "/" + (net.lWay[l] >= 0 ? net.wayDirConf[net.lWay[l]] : 0));
            }
        }
    }

    static int link(Net n, long id) {
        for (int i = 0; i < n.nLinks; i++) if (n.linkId[i] == id) return i;
        return -1;
    }
}
