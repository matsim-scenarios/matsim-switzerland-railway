package ch.sbb.prepare.platform;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.ToDoubleFunction;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.population.routes.RouteUtils;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;
import org.matsim.pt2matsim.config.PublicTransitMappingStrings;
import org.matsim.pt2matsim.tools.ScheduleTools;

/**
 * Post-processing after mapping with {@link RailPlatformLinkCandidateCreator}: stops whose link ends at the OSM stop
 * position are moved forward along the route, so that the halting train (head at the end of the stop link, body
 * behind it) is centred on the stop position.
 * <ul>
 * <li>If there is a dead end (buffer stop) less than half a train length behind the stop position, the train is
 * shifted forward so that its tail does not stick out behind the buffer stop.</li>
 * <li>At the last stop, the route is extended along the platform track; it ends at a buffer stop if there is one.</li>
 * <li>At the first stop, the route is trimmed so that it starts at the new stop link.</li>
 * <li>A stop is never moved onto or beyond the link of the next stop.</li>
 * </ul>
 */
public final class RailPlatformStopShifter {

	private static final Logger log = LogManager.getLogger(RailPlatformStopShifter.class);

	private RailPlatformStopShifter() {
	}

	public static void run(TransitSchedule schedule, Network network, RailPlatformLinkCandidateCreator creator,
			ToDoubleFunction<TransitRoute> trainLength) {
		int nShifted = 0;
		for (TransitLine line : schedule.getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {
				if (creator.getPlatformModes().contains(route.getTransportMode()) && route.getRoute() != null) {
					nShifted += shift(schedule, network, route, creator, creator.networkModes(route), trainLength.applyAsDouble(route));
				}
			}
		}
		log.info("Rail platform stops: {} route stops moved to centre the train on the OSM stop position.", nShifted);
	}

	static int shift(TransitSchedule schedule, Network network, TransitRoute route, RailPlatformLinkCandidateCreator creator,
			Set<String> networkModes, double trainLength) {
		List<Link> links = new ArrayList<>();
		for (Id<Link> id : ScheduleTools.getTransitRouteLinkIds(route)) {
			links.add(network.getLinks().get(id));
		}
		List<TransitRouteStop> stops = route.getStops();

		int[] stopIdx = new int[stops.size()];
		int searchFrom = 0;
		for (int k = 0; k < stops.size(); k++) {
			Id<Link> linkId = stops.get(k).getStopFacility().getLinkId();
			int i = searchFrom;
			while (i < links.size() && !links.get(i).getId().equals(linkId)) {
				i++;
			}
			if (i == links.size()) {
				log.warn("Stop {} not found on route {}, route not shifted.", stops.get(k).getStopFacility().getId(), route.getId());
				return 0;
			}
			stopIdx[k] = i;
			searchFrom = i;
		}

		int nShifted = 0;
		for (int k = 0; k < stops.size(); k++) {
			int i = stopIdx[k];
			Link stopLink = links.get(i);
			Node stopNode = creator.getStopNode(parentId(stops.get(k).getStopFacility()));
			if (stopNode == null || stopLink.getToNode() != stopNode) {
				continue;
			}
			double behind = PlatformTrack.spaceBehind(stopLink, networkModes, trainLength / 2);
			double headOffset = trainLength - behind;
			boolean isLast = k == stops.size() - 1;
			int limit = isLast ? Integer.MAX_VALUE : stopIdx[k + 1] - 1;

			// the head halts at a link end: choose the one closest to the desired head position (long links after
			// the platform must not push the train far beyond it)
			int j = i;
			double travelled = 0;
			while (travelled < headOffset && j < limit) {
				if (j + 1 < links.size()) {
					double next = travelled + links.get(j + 1).getLength();
					if (next - headOffset > headOffset - travelled) {
						break;
					}
					j++;
					travelled = next;
				} else if (isLast) {
					List<Link> ext = PlatformTrack.forward(links.get(j), networkModes, headOffset - travelled);
					if (ext.isEmpty()) {
						break; // buffer stop
					}
					links.addAll(ext);
				} else {
					break;
				}
			}
			if (j != i) {
				stopIdx[k] = j;
				stops.get(k).setStopFacility(childFacility(schedule, stops.get(k).getStopFacility(), links.get(j)));
				nShifted++;
			}
		}

		// the route ends at the last stop and starts at the first one
		List<Id<Link>> linkIds = new ArrayList<>();
		for (int i = stopIdx[0]; i <= stopIdx[stops.size() - 1]; i++) {
			linkIds.add(links.get(i).getId());
		}
		if (stopIdx[0] != 0 || stopIdx[stops.size() - 1] != links.size() - 1 || nShifted > 0) {
			route.setRoute(RouteUtils.createNetworkRoute(linkIds));
		}
		return nShifted;
	}

	static Id<TransitStopFacility> parentId(TransitStopFacility facility) {
		String id = facility.getId().toString();
		int idx = id.lastIndexOf(PublicTransitMappingStrings.SUFFIX_CHILD_STOP_FACILITIES);
		return Id.create(idx < 0 ? id : id.substring(0, idx), TransitStopFacility.class);
	}

	private static TransitStopFacility childFacility(TransitSchedule schedule, TransitStopFacility template, Link link) {
		Id<TransitStopFacility> id = Id.create(parentId(template) + PublicTransitMappingStrings.SUFFIX_CHILD_STOP_FACILITIES + link.getId(),
				TransitStopFacility.class);
		TransitStopFacility facility = schedule.getFacilities().get(id);
		if (facility == null) {
			facility = schedule.getFactory().createTransitStopFacility(id, template.getCoord(), template.getIsBlockingLane());
			facility.setLinkId(link.getId());
			facility.setName(template.getName());
			facility.setStopAreaId(template.getStopAreaId());
			for (var e : template.getAttributes().getAsMap().entrySet()) {
				facility.getAttributes().putAttribute(e.getKey(), e.getValue());
			}
			schedule.addStopFacility(facility);
		}
		return facility;
	}
}
