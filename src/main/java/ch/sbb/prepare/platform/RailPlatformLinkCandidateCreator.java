package ch.sbb.prepare.platform;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Function;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.utils.geometry.CoordUtils;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidate;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateCreator;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateCreatorStandard;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateImpl;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.PublicTransitStop;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.PublicTransitStopImpl;

/**
 * Link candidate creator that maps rail stops onto the OSM stop position of their platform track.
 * <p>
 * For routes whose schedule transport mode is in {@code platformModes} (e.g. {@code rail}): the platform code of the
 * stop facility (e.g. {@code 10} for {@code 8500218:0:10}) is matched against OSM stop positions ({@code local_ref})
 * within {@code searchRadius}, see {@link OsmStopPositionNetworkEnricher}. The link candidates are the links ending at
 * that stop position node, which forces the route over the platform track. Afterwards,
 * {@link RailPlatformStopShifter} moves the halting position forward so that the train is centred on the platform.
 * <p>
 * For all other routes, or if no matching stop position is found, the candidates of the delegate (usually
 * {@link LinkCandidateCreatorStandard}) are returned unchanged.
 */
public class RailPlatformLinkCandidateCreator implements LinkCandidateCreator {

	private static final Logger log = LogManager.getLogger(RailPlatformLinkCandidateCreator.class);

	public static final double DEFAULT_SEARCH_RADIUS = 500.0;

	private final LinkCandidateCreator delegate;
	private final Set<String> platformModes;
	private final Map<String, Set<String>> modeAssignments;
	private final Function<TransitStopFacility, String> platformCodeExtractor;
	private final double searchRadius;

	private final List<Node> stopNodes = new ArrayList<>();
	private final Map<Id<PublicTransitStop>, SortedSet<LinkCandidate>> candidates = new HashMap<>();
	private final Map<Id<TransitStopFacility>, Node> stopNodeByFacility = new HashMap<>();
	private final Map<String, List<Link>> linksCache = new HashMap<>();

	public RailPlatformLinkCandidateCreator(LinkCandidateCreator delegate, TransitSchedule schedule, Network network,
			Map<String, Set<String>> modeAssignments, Set<String> platformModes,
			Function<TransitStopFacility, String> platformCodeExtractor, double searchRadius) {
		this.delegate = delegate;
		this.modeAssignments = modeAssignments;
		this.platformModes = platformModes;
		this.platformCodeExtractor = platformCodeExtractor;
		this.searchRadius = searchRadius;

		for (Node node : network.getNodes().values()) {
			if (node.getAttributes().getAttribute(OsmStopPositionNetworkEnricher.ATTR_LOCAL_REF) != null) {
				stopNodes.add(node);
			}
		}
		load(schedule);
	}

	/**
	 * Swiss GTFS: {@code <UIC>:0:<platform>}. Returns {@code null} if the stop id has no platform part.
	 */
	public static String platformCodeFromStopId(TransitStopFacility facility) {
		String[] parts = facility.getId().toString().split(":");
		if (parts.length < 3) {
			return null;
		}
		String code = parts[parts.length - 1].trim();
		return code.isEmpty() ? null : code;
	}

	private void load(TransitSchedule schedule) {
		int nPlatform = 0;
		int nFallback = 0;
		// pre-compute, because PTMapper queries candidates from multiple threads
		for (TransitLine line : schedule.getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {
				if (!platformModes.contains(route.getTransportMode())) {
					continue;
				}
				Set<String> networkModes = networkModes(route);
				for (TransitRouteStop routeStop : route.getStops()) {
					List<Link> links = stopLinks(routeStop.getStopFacility(), networkModes);
					if (links.isEmpty()) {
						nFallback++;
						continue;
					}
					nPlatform++;
					PublicTransitStop stop = new PublicTransitStopImpl(line, route, routeStop);
					SortedSet<LinkCandidate> set = new TreeSet<>();
					for (Link link : links) {
						set.add(new LinkCandidateImpl(link, stop));
					}
					candidates.put(stop.getId(), set);
				}
			}
		}
		log.info("Rail platform link candidates: {} route stops mapped to OSM stop positions, {} use standard candidates.", nPlatform, nFallback);
	}

	Set<String> networkModes(TransitRoute route) {
		return modeAssignments.getOrDefault(route.getTransportMode(), Set.of(route.getTransportMode()));
	}

	@Override
	public SortedSet<LinkCandidate> getLinkCandidates(TransitRouteStop routeStop, TransitLine line, TransitRoute route) {
		SortedSet<LinkCandidate> set = candidates.get(PublicTransitStop.createId(line, route, routeStop));
		return set != null ? set : delegate.getLinkCandidates(routeStop, line, route);
	}

	/**
	 * OSM stop position node of a (parent) stop facility, or {@code null} if none was found.
	 */
	public Node getStopNode(Id<TransitStopFacility> facilityId) {
		return stopNodeByFacility.get(facilityId);
	}

	public Set<String> getPlatformModes() {
		return platformModes;
	}

	private List<Link> stopLinks(TransitStopFacility facility, Set<String> networkModes) {
		return linksCache.computeIfAbsent(facility.getId() + "|" + networkModes, k -> {
			Node stopNode = findStopNode(facility);
			if (stopNode == null) {
				return Collections.emptyList();
			}
			List<Link> links = new ArrayList<>();
			for (Link l : stopNode.getInLinks().values()) {
				if (PlatformTrack.allowed(l, networkModes)) {
					links.add(l);
				}
			}
			if (!links.isEmpty()) {
				stopNodeByFacility.put(facility.getId(), stopNode);
			}
			return links;
		});
	}

	Node findStopNode(TransitStopFacility facility) {
		String code = platformCodeExtractor.apply(facility);
		if (code == null) {
			return null;
		}
		Node best = null;
		double bestDist = searchRadius;
		for (Node node : stopNodes) {
			String localRef = (String) node.getAttributes().getAttribute(OsmStopPositionNetworkEnricher.ATTR_LOCAL_REF);
			if (!platformMatches(code, localRef)) {
				continue;
			}
			double d = CoordUtils.calcEuclideanDistance(node.getCoord(), facility.getCoord());
			if (d <= bestDist) {
				best = node;
				bestDist = d;
			}
		}
		return best;
	}

	static boolean platformMatches(String code, String localRef) {
		if (localRef == null) {
			return false;
		}
		for (String ref : localRef.split(";")) {
			if (ref.trim().equalsIgnoreCase(code)) {
				return true;
			}
		}
		return false;
	}
}
