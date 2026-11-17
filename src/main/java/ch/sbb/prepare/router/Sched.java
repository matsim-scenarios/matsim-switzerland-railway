package ch.sbb.prepare.router;

import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamReader;
import java.io.InputStream;
import java.util.*;

/** minimal streaming reader of an unmapped MATSim transit schedule */
public final class Sched {

    public static final class StopF {
        public String id, name, area;
        public double x, y;
        public boolean blocking;
    }

    public static final class StopRef {
        public String stopId;
        public double arr, dep;
        public boolean await;
    }

    public static final class Dep {
        public String id, veh;
        public double time;
    }

    public static final class Route {
        public String id, mode = "rail";
        public ArrayList<StopRef> stops = new ArrayList<>();
        public ArrayList<Dep> deps = new ArrayList<>();
    }

    public static final class Line {
        public String id, name;
        public ArrayList<Route> routes = new ArrayList<>();
    }

    public final ArrayList<Line> lines = new ArrayList<>();
    public final LinkedHashMap<String, StopF> stops = new LinkedHashMap<>();

    public static double hms(String s) {
        if (s == null) return Double.NaN;
        int a = s.indexOf(':');
        if (a < 0) return Double.parseDouble(s);
        int b = s.indexOf(':', a + 1);
        int h = Integer.parseInt(s.substring(0, a));
        int m = Integer.parseInt(s.substring(a + 1, b));
        double sec = Double.parseDouble(s.substring(b + 1));
        return h * 3600.0 + m * 60.0 + sec;
    }

    public static Sched read(String file) throws Exception {
        Sched s = new Sched();
        XMLInputFactory f = XMLInputFactory.newInstance();
        f.setProperty(XMLInputFactory.IS_COALESCING, Boolean.TRUE);
        f.setProperty(XMLInputFactory.SUPPORT_DTD, Boolean.FALSE);
        f.setProperty(XMLInputFactory.IS_SUPPORTING_EXTERNAL_ENTITIES, Boolean.FALSE);
        Line line = null;
        Route route = null;
        try (InputStream in = Net.open(file)) {
            XMLStreamReader r = f.createXMLStreamReader(in);
            while (r.hasNext()) {
                int ev = r.next();
                if (ev == XMLStreamConstants.START_ELEMENT) {
                    String tag = r.getLocalName();
                    switch (tag) {
                        case "stopFacility": {
                            StopF sf = new StopF();
                            sf.id = r.getAttributeValue(null, "id");
                            sf.name = r.getAttributeValue(null, "name");
                            sf.area = r.getAttributeValue(null, "stopAreaId");
                            sf.x = Double.parseDouble(r.getAttributeValue(null, "x"));
                            sf.y = Double.parseDouble(r.getAttributeValue(null, "y"));
                            String bl = r.getAttributeValue(null, "isBlocking");
                            sf.blocking = bl != null && bl.equals("true");
                            s.stops.put(sf.id, sf);
                            break;
                        }
                        case "transitLine": {
                            line = new Line();
                            line.id = r.getAttributeValue(null, "id");
                            line.name = r.getAttributeValue(null, "name");
                            s.lines.add(line);
                            break;
                        }
                        case "transitRoute": {
                            route = new Route();
                            route.id = r.getAttributeValue(null, "id");
                            line.routes.add(route);
                            break;
                        }
                        case "transportMode":
                            if (route != null) route.mode = r.getElementText().trim();
                            break;
                        case "stop": {
                            if (route == null) break;
                            StopRef sr = new StopRef();
                            sr.stopId = r.getAttributeValue(null, "refId");
                            sr.arr = hms(r.getAttributeValue(null, "arrivalOffset"));
                            sr.dep = hms(r.getAttributeValue(null, "departureOffset"));
                            if (Double.isNaN(sr.arr)) sr.arr = sr.dep;
                            if (Double.isNaN(sr.dep)) sr.dep = sr.arr;
                            String aw = r.getAttributeValue(null, "awaitDeparture");
                            sr.await = aw != null && aw.equals("true");
                            route.stops.add(sr);
                            break;
                        }
                        case "departure": {
                            if (route == null) break;
                            Dep d = new Dep();
                            d.id = r.getAttributeValue(null, "id");
                            d.veh = r.getAttributeValue(null, "vehicleRefId");
                            d.time = hms(r.getAttributeValue(null, "departureTime"));
                            route.deps.add(d);
                            break;
                        }
                        default:
                            break;
                    }
                } else if (ev == XMLStreamConstants.END_ELEMENT) {
                    String tag = r.getLocalName();
                    if (tag.equals("transitRoute")) route = null;
                    else if (tag.equals("transitLine")) line = null;
                }
            }
            r.close();
        }
        return s;
    }
}
