package ch.sbb.matsim.contrib.railsim.qsimengine;

import java.util.List;

import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;
import org.junit.Test;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.NetworkFactory;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;

import static org.junit.Assert.assertEquals;

public class ReversalGeometryTest {
    @Test
    public void bodyUsesEachLinksOwnLength() {
        NetworkFactory factory = NetworkUtils.createNetwork().getFactory();
        Node from = factory.createNode(Id.createNodeId("from"), new Coord(0, 0));
        Node to = factory.createNode(Id.createNodeId("to"), new Coord(1, 0));
        List<RailLink> route = List.of(30, 100, 20).stream().map(length -> {
            Link link = factory.createLink(Id.createLinkId("link-" + length), from, to);
            link.setLength(length);
            return new RailLink(link, null);
        }).toList();

        assertEquals(List.of(route.get(2), route.get(1)), RailsimEngine.bodyLinks(route, 3, 10, 110));
        assertEquals(List.of(route.get(2), route.get(1), route.get(0)), RailsimEngine.bodyLinks(route, 3, 10, 111));
    }
}
