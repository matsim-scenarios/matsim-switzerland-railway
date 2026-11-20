package ch.sbb.prepare.platform;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.stream.Collectors;

import org.junit.Test;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.NetworkFactory;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.network.NetworkUtils;
import org.matsim.core.population.routes.RouteUtils;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt.transitSchedule.api.TransitScheduleFactory;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidate;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateCreator;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateImpl;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.PublicTransitStopImpl;
import org.matsim.pt2matsim.tools.ScheduleTools;

public class RailPlatformLinkCandidateCreatorTest {

	private static final double LINK_LENGTH = 20;
	private static final double TRAIN_LENGTH = 200;
	private static final String STOP_A = "8500218:0:3";
	private static final String STOP_B = "8500219:0:1";

	/**
	 * Straight double-direction track along x from node 0 to node {@code n}, links of 20 m, ids "i_j". Stop position
	 * with local_ref 3 at node {@code stopIndexA}, with local_ref 1 at node {@code stopIndexB} (if >= 0).
	 */
	private static Network track(int n, int stopIndexA, int stopIndexB) {
		Network network = NetworkUtils.createNetwork();
		NetworkFactory f = network.getFactory();
		for (int i = 0; i <= n; i++) {
			network.addNode(f.createNode(Id.createNodeId(i), new Coord(i * LINK_LENGTH, 0)));
		}
		for (int i = 0; i < n; i++) {
			addLink(network, i, i + 1);
			addLink(network, i + 1, i);
		}
		node(network, stopIndexA).getAttributes().putAttribute(OsmStopPositionNetworkEnricher.ATTR_LOCAL_REF, "3");
		if (stopIndexB >= 0) {
			node(network, stopIndexB).getAttributes().putAttribute(OsmStopPositionNetworkEnricher.ATTR_LOCAL_REF, "1");
		}
		return network;
	}

	private static Node node(Network network, int i) {
		return network.getNodes().get(Id.createNodeId(i));
	}

	private static void addLink(Network network, int from, int to) {
		Link l = network.getFactory().createLink(Id.createLinkId(from + "_" + to), node(network, from), node(network, to));
		l.setLength(LINK_LENGTH);
		l.setAllowedModes(Set.of("rail"));
		l.getAttributes().putAttribute("osm:way:id", 1L);
		network.addLink(l);
	}

	/** Unmapped schedule with one route over the given facilities (id, coord). */
	private static TransitSchedule schedule(String mode, Object... idsAndCoords) {
		TransitSchedule schedule = ScenarioUtils.createScenario(ConfigUtils.createConfig()).getTransitSchedule();
		TransitScheduleFactory f = schedule.getFactory();
		List<TransitRouteStop> stops = new ArrayList<>();
		for (int i = 0; i < idsAndCoords.length; i += 2) {
			TransitStopFacility facility = f.createTransitStopFacility(Id.create((String) idsAndCoords[i], TransitStopFacility.class),
					(Coord) idsAndCoords[i + 1], false);
			schedule.addStopFacility(facility);
			stops.add(f.createTransitRouteStop(facility, 0, 0));
		}
		TransitLine line = f.createTransitLine(Id.create("line", TransitLine.class));
		line.addRoute(f.createTransitRoute(Id.create("route", TransitRoute.class), null, stops, mode));
		schedule.addTransitLine(line);
		return schedule;
	}

	private static RailPlatformLinkCandidateCreator creator(TransitSchedule schedule, Network network) {
		return new RailPlatformLinkCandidateCreator(standardStub(network), schedule, network, Map.of("rail", Set.of("rail"), "bus", Set.of("rail")),
				Set.of("rail"), RailPlatformLinkCandidateCreator::platformCodeFromStopId, 500);
	}

	/** Delegate returning the link 0_1 as only candidate, marking the "standard" behaviour. */
	private static LinkCandidateCreator standardStub(Network network) {
		return (routeStop, line, route) -> {
			SortedSet<LinkCandidate> set = new TreeSet<>();
			set.add(new LinkCandidateImpl(network.getLinks().get(Id.createLinkId("0_1")), new PublicTransitStopImpl(line, route, routeStop)));
			return set;
		};
	}

	private static TransitRoute route(TransitSchedule schedule) {
		return schedule.getTransitLines().values().iterator().next().getRoutes().values().iterator().next();
	}

	private static Set<String> candidateLinks(LinkCandidateCreator c, TransitSchedule schedule) {
		TransitLine line = schedule.getTransitLines().values().iterator().next();
		TransitRoute route = route(schedule);
		return c.getLinkCandidates(route.getStops().get(0), line, route).stream()
				.map(lc -> lc.getLink().getId().toString()).collect(Collectors.toSet());
	}

	/** Simulates the mapping result: child facilities on the given links and the given network route. */
	private static void map(TransitSchedule schedule, List<String> routeLinks, String... stopLinks) {
		TransitRoute route = route(schedule);
		for (int k = 0; k < stopLinks.length; k++) {
			TransitStopFacility parent = route.getStops().get(k).getStopFacility();
			TransitStopFacility child = schedule.getFactory().createTransitStopFacility(
					Id.create(parent.getId() + ".link:" + stopLinks[k], TransitStopFacility.class), parent.getCoord(), false);
			child.setLinkId(Id.createLinkId(stopLinks[k]));
			schedule.addStopFacility(child);
			route.getStops().get(k).setStopFacility(child);
		}
		route.setRoute(RouteUtils.createNetworkRoute(routeLinks.stream().map(Id::createLinkId).toList()));
	}

	private static List<String> path(int from, int to) {
		List<String> links = new ArrayList<>();
		int step = from < to ? 1 : -1;
		for (int i = from; i != to; i += step) {
			links.add(i + "_" + (i + step));
		}
		return links;
	}

	private static List<String> stopLinks(TransitSchedule schedule) {
		return route(schedule).getStops().stream().map(s -> s.getStopFacility().getLinkId().toString()).toList();
	}

	private static List<String> routeLinks(TransitSchedule schedule) {
		return ScheduleTools.getTransitRouteLinkIds(route(schedule)).stream().map(Object::toString).toList();
	}

	@Test
	public void candidatesAreLinksEndingAtStopPosition() {
		Network network = track(40, 20, -1);
		TransitSchedule schedule = schedule("rail", STOP_A, new Coord(390, 50));
		assertThat(candidateLinks(creator(schedule, network), schedule)).containsExactlyInAnyOrder("19_20", "21_20");
	}

	@Test
	public void nonRailRoutesUseStandardCandidates() {
		Network network = track(40, 20, -1);
		TransitSchedule schedule = schedule("bus", STOP_A, new Coord(400, 0));
		assertThat(candidateLinks(creator(schedule, network), schedule)).containsExactly("0_1");
	}

	@Test
	public void unknownPlatformUsesStandardCandidates() {
		Network network = track(40, 20, -1);
		TransitSchedule schedule = schedule("rail", "8500218:0:7", new Coord(400, 0));
		assertThat(candidateLinks(creator(schedule, network), schedule)).containsExactly("0_1");
	}

	@Test
	public void stopPositionOutsideSearchRadiusUsesStandardCandidates() {
		Network network = track(40, 20, -1);
		TransitSchedule schedule = schedule("rail", STOP_A, new Coord(400, 2000));
		assertThat(candidateLinks(creator(schedule, network), schedule)).containsExactly("0_1");
	}

	@Test
	public void intermediateStopIsCentredOnStopPosition() {
		// stops at node 10 (A) and 30 (B), route 0 -> 40 with a third stop at the end
		Network network = track(60, 10, 30);
		TransitSchedule schedule = schedule("rail", STOP_A, new Coord(200, 0), STOP_B, new Coord(600, 0), "x:0:9", new Coord(1200, 0));
		RailPlatformLinkCandidateCreator creator = creator(schedule, network);
		map(schedule, path(0, 60), "9_10", "29_30", "59_60");

		RailPlatformStopShifter.run(schedule, network, creator, r -> TRAIN_LENGTH);

		// head 100 m after the stop position, i.e. the train is centred on it
		assertThat(stopLinks(schedule)).containsExactly("14_15", "34_35", "59_60");
		// first stop: route starts at the new stop link
		assertThat(routeLinks(schedule)).isEqualTo(path(14, 60));
	}

	@Test
	public void lastStopExtendsRouteAndStopsAtBufferStop() {
		// buffer stop 40 m after the stop position at node 20
		Network network = track(22, 20, 2);
		TransitSchedule schedule = schedule("rail", STOP_B, new Coord(40, 0), STOP_A, new Coord(400, 0));
		RailPlatformLinkCandidateCreator creator = creator(schedule, network);
		map(schedule, path(0, 20), "1_2", "19_20");

		RailPlatformStopShifter.run(schedule, network, creator, r -> TRAIN_LENGTH);

		assertThat(stopLinks(schedule).get(1)).isEqualTo("21_22");
		assertThat(routeLinks(schedule)).endsWith("19_20", "20_21", "21_22");
	}

	@Test
	public void departureFromStubTrackKeepsTailInsideBufferStop() {
		// stop position at node 20, buffer stop at node 22, train departs towards x=0
		Network network = track(22, 20, 2);
		TransitSchedule schedule = schedule("rail", STOP_A, new Coord(400, 0), STOP_B, new Coord(40, 0));
		RailPlatformLinkCandidateCreator creator = creator(schedule, network);
		map(schedule, path(22, 1), "21_20", "2_1");

		RailPlatformStopShifter.run(schedule, network, creator, r -> TRAIN_LENGTH);

		// only 40 m behind the stop position: head at 400 - (200 - 40) = 240 (node 12)
		assertThat(stopLinks(schedule).get(0)).isEqualTo("13_12");
	}

	@Test
	public void stopIsNotMovedBeyondNextStop() {
		Network network = track(40, 10, 13);
		TransitSchedule schedule = schedule("rail", "x:0:9", new Coord(0, 0), STOP_A, new Coord(200, 0), STOP_B, new Coord(260, 0));
		RailPlatformLinkCandidateCreator creator = creator(schedule, network);
		map(schedule, path(0, 40), "0_1", "9_10", "12_13");

		RailPlatformStopShifter.run(schedule, network, creator, r -> TRAIN_LENGTH);

		assertThat(stopLinks(schedule).get(1)).isEqualTo("11_12");
	}

	@Test
	public void longLinkAfterPlatformIsNotEntered() {
		Network network = track(40, 10, -1);
		network.getLinks().get(Id.createLinkId("12_13")).setLength(600);
		TransitSchedule schedule = schedule("rail", "x:0:9", new Coord(0, 0), STOP_A, new Coord(200, 0), "y:0:9", new Coord(800, 0));
		RailPlatformLinkCandidateCreator creator = creator(schedule, network);
		map(schedule, path(0, 40), "0_1", "9_10", "39_40");

		RailPlatformStopShifter.run(schedule, network, creator, r -> TRAIN_LENGTH);

		assertThat(stopLinks(schedule).get(1)).isEqualTo("11_12");
	}

	@Test
	public void platformCode() {
		TransitScheduleFactory f = ScenarioUtils.createScenario(ConfigUtils.createConfig()).getTransitSchedule().getFactory();
		assertThat(RailPlatformLinkCandidateCreator.platformCodeFromStopId(
				f.createTransitStopFacility(Id.create("8500218:0:10", TransitStopFacility.class), new Coord(0, 0), false))).isEqualTo("10");
		assertThat(RailPlatformLinkCandidateCreator.platformCodeFromStopId(
				f.createTransitStopFacility(Id.create("8500218", TransitStopFacility.class), new Coord(0, 0), false))).isNull();
		assertThat(RailPlatformLinkCandidateCreator.platformMatches("3", "2;3")).isTrue();
		assertThat(RailPlatformLinkCandidateCreator.platformMatches("3", "13")).isFalse();
	}
}
