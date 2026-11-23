/* *********************************************************************** *
 * project: org.matsim.*
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2023 by the members listed in the COPYING,        *
 *                   LICENSE and WARRANTY file.                            *
 * email           : info at matsim dot org                                *
 *                                                                         *
 * *********************************************************************** *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *   See also COPYING, LICENSE and WARRANTY file                           *
 *                                                                         *
 * *********************************************************************** */

package ch.sbb.run;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.mobsim.framework.MobsimDriverAgent;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;

import com.google.inject.Inject;

import ch.sbb.matsim.contrib.railsim.qsimengine.TrainPosition;
import ch.sbb.matsim.contrib.railsim.qsimengine.deadlocks.DeadlockAvoidance;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailResource;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailResourceManager;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.ResourceState;

/**
 * Interlocking-style <b>route locking</b> for station throats and junctions, on top of the section direction locks of
 * {@link SectionLockDeadlockAvoidance}. railsim (2027.0-2026w45) reserves track only over the braking distance and lets a train stop
 * anywhere, so a station throat fills with trains that each wait for the next throat link held by another train of a cycle. A real
 * interlocking instead holds a train at the home signal until its complete route to the platform (or through the junction) can be locked,
 * and holds a departing train at the platform until its route out is locked.
 * <p>
 * This class implements that as follows (all analysis from the {@link Scenario}, done once in
 * {@link #initResources(RailResourceManager)}):
 * <ol>
 * <li><b>Junction nodes</b>: a node is a junction if the links used by any transit route connect it to at least 3 distinct neighbour
 * nodes.</li>
 * <li><b>Cluster links</b>: a used link is a cluster link if its from- or to-node is a junction, or if the link carries a scheduled stop of
 * any route (every platform is part of a station area). In addition every link whose route distance to a junction node is less than
 * {@code clusterGap} metres is a cluster link; this closes the short plain gaps inside a throat (measured: 11-23 m links in the middle
 * of the Luzern and Bern throats that a pure endpoint rule leaves unprotected). Stops are cluster links themselves but do not grow an
 * area, otherwise a line with stops every few hundred metres becomes one multi-kilometre cluster (measured with stop neighbourhoods:
 * Olten 2.3 to 19.5 km). Physical clusters are the connected components of all cluster links (through shared nodes).</li>
 * <li><b>Protected areas</b>: every cluster link is protected. A whole section of {@link SectionLockDeadlockAvoidance} (maximal run of
 * resources used in both directions) is also protected if it touches a cluster at either end or contains a cluster. This is the
 * boundary rule of the task: a train may only enter a single-track section if the route into the station at its far end can be locked
 * too (and, symmetrically, a departing train may only leave the platform if the section and the next station route are lockable). A
 * protected area is a maximal run of protected links and is split at every scheduled stop into <b>arrival</b> (area entry .. stop) and
 * <b>departure</b> (after the stop .. area exit) routes; an area without a stop of this train is one <b>through</b> route. A route that
 * ends before the train's next stop is extended to it when that stop is at most 2 km away, so the platform (the place where the train can
 * stand without blocking anybody) is always part of the locked route. Unlike the section lock, a reversing stop does split the area here:
 * the all-or-nothing departure route check replaces the "extend the arrival lock over the departure stretch" deviation D1 of the section
 * lock and avoids holding a kilometre-long departure route during the whole station dwell.</li>
 * <li><b>Locking rule</b>: a train may reserve the first link of an arrival/departure/through route only if <em>every</em> resource of
 * that route is currently neither claimed/locked by another train nor physically reserved by another train; then it claims all of
 * them at once (all-or-nothing, exclusive in any direction). Its own claims and its own physical reservations are ignored, so it can
 * advance inside a route it already holds. Every reservation request, also for a plain unidirectional link outside the requesting train's
 * own area/section, is checked against exclusive route claims of other trains; otherwise a train could reserve a link that a route holder
 * already claimed (measured: a train stopped inside its own area because a following train on a plain track reserved the next link).
 * Claims are released link by link when the train releases the link (tail passed).</li>
 * <li>Outside protected areas the section direction locks of {@link SectionLockDeadlockAvoidance} are used unchanged; protected
 * sections are covered by the exclusive route lock instead, so a section claim is never created on a cluster link (that is what
 * makes the "both conditions" requirement consistent rather than contradictory).</li>
 * <li>Output: {@code clusters.csv} (physical cluster id, centre, links, km, nearest stop facility) and
 * {@code interlocking_denials.csv}; start-up statistics in the log.</li>
 * </ol>
 * <p>
 * <b>Verified railsim 2027.0-2026w45 facts</b> (jar inspected with {@code javap}, same method as EN1):
 * <ul>
 * <li>{@code RailResource.getState(RailLink)} is the global resource state: {@code EMPTY} if the resource has no reservation,
 * {@code IN_USE} if reservations &lt; capacity, {@code EXHAUSTED} otherwise; with capacity 1 it is {@code EMPTY} or
 * {@code EXHAUSTED}. It is the authoritative "is this resource reserved" query.</li>
 * <li>{@code RailResourceManager.isBlockedBy(link, position)} is <b>not</b> "blocked for this train": it is
 * {@code getReservedDist(link, position) > -1}, i.e. true when the link is reserved <em>by this position's own driver</em>. It is only
 * used (as in EN1) to locate the link that was just reserved.</li>
 * <li>{@code FixedBlockResource} keeps one {@code reservations} entry per driver (capacity 1) and a per-link driver array; a driver
 * already holding the resource passes {@code hasCapacity} again, which is how a reversal on its own track is allowed. Therefore a
 * resource is exclusively held by at most one driver and {@code onReserve}/{@code onRelease} give an exact physical-holder map.</li>
 * <li>{@code checkLink} is called from {@code RailResourceManagerImpl.tryBlockLink} after {@code hasCapacity} succeeded (or the link
 * is held by a related train), and is never called speculatively. {@code onReserve} fires after the reservation was stored.
 * {@code onRelease} fires only when the driver has no reservation left on any link of the resource; {@code onReleaseLink} fires for
 * every single link release. {@code checkLink} here only reads state (plus a per-driver route cache) and writes the denial CSV.</li>
 * </ul>
 * <p>
 * <b>Deviations from the plain task text</b> (all measured/documented in {@code swarm/dla2/REPORT.md}):
 * <ol>
 * <li>A reversing stop <b>does</b> split an area (in contrast to deviation D1 of the section lock): the all-or-nothing departure route
 * check at the platform already prevents a train from leaving the platform into a blocked throat, and splitting avoids claiming a
 * multi-kilometre departure route during the whole dwell (measured at Bern: 5.3 km area, of which 2.5 km would be held while the train
 * stands at the platform).</li>
 * <li>Cluster membership is extended from "endpoint is a junction" to "within {@code clusterGap} metres along the route of a junction
 * node", and stop links are cluster links themselves. Without the former, 11-23 m plain links in the middle of the Luzern and Bern
 * throats stayed unprotected and trains stopped in exactly those links (first EN3-diagnosed Luzern cascade); without the latter a
 * platform could lie in a plain gap between two throat clusters.</li>
 * <li>Whole sections adjacent to a cluster are protected, not only the cluster: this is how the section+arrival-route extension is
 * implemented, and it is applied symmetrically (departure) because otherwise a train leaving a platform into a single-track section can
 * meet a train that holds the section and waits for that platform (the mirror deadlock).</li>
 * <li>Route claims are exclusive in both directions, also where the section lock would only need a direction lock. This can cost
 * capacity for same-direction following inside a protected single-track section; it is measured.</li>
 * <li>A protected route is extended to the next scheduled stop when that stop is at most 2 km beyond the area end. This is a bounded
 * version of "the route must end at a safe point" (without it a train could be held between the home signal and the platform and block
 * the platform's occupant's departure; measured at Luzern). Through routes whose next stop is farther keep the area end as their end.</li>
 * <li>No cap on cluster or route length is applied; over-long clusters/routes are logged and listed in the report as a capacity risk
 * (a cap would need a defined safe point, which the network does not provide).</li>
 * </ol>
 */
public final class InterlockingDeadlockAvoidance implements DeadlockAvoidance {

	private static final Logger log = LogManager.getLogger(InterlockingDeadlockAvoidance.class);

	public static final double DEFAULT_CLUSTER_GAP_M = 300.;

	/** Maximum distance from the end of a protected area to the train's next stop for which the route is extended to the stop. */
	private static final double MAX_ROUTE_EXTENSION_M = 2000.;

	private static final String CSV_RESOURCES = "bidirectional_resources.csv";
	private static final String CSV_CLUSTERS = "clusters.csv";
	private static final String CSV_DENIALS = "interlocking_denials.csv";
	private static final int MAX_DENIAL_ROWS = 20000;

	private final Scenario scenario;
	private final Network network;
	private final double clusterGap;

	/** Resource -> true if the transit schedule uses two opposite links of it (the section-lock set). */
	private final Set<Id<RailResource>> bidirectional = new HashSet<>();

	/** All rail links by link id (from the resource manager). */
	private final Map<Id<Link>, RailLink> railLinks = new HashMap<>();

	/** Stop links / protection arrays per transit route: lineId|routeId. */
	private final Map<String, RouteInfo> routeInfos = new HashMap<>();

	/** Per resource: the (driver, link) claims currently held on it. A driver may hold both directions of a resource. */
	private final Map<Id<RailResource>, List<Claim>> claims = new HashMap<>();

	/** Per driver: its claims in route order (used to drop claims the train has passed). */
	private final Map<MobsimDriverAgent, List<Claim>> claimsByDriver = new HashMap<>();

	/** Cached route info per driver, only used to avoid rebuilding the "line|route" key on every call. */
	private final Map<MobsimDriverAgent, RouteInfo> driverRouteInfo = new HashMap<>();

	/** Exact physical reservation holder per resource (capacity 1), maintained from onReserve/onRelease. */
	private final Map<Id<RailResource>, MobsimDriverAgent> physicalHolder = new HashMap<>();

	private RailResourceManager rrm;

	private BufferedWriter denialOut;
	private int denialRows;

	private long checkLinkCalls;
	private long deniedExclusive;
	private long deniedAreaClaim;
	private long deniedAreaOccupied;
	private long deniedSection;
	private long notOnRoute;
	private long unknownRouteCalls;
	private int routeSizeMismatch;
	private double lastLogTime = Double.NEGATIVE_INFINITY;

	@Inject
	public InterlockingDeadlockAvoidance(Scenario scenario) {
		this(scenario, DEFAULT_CLUSTER_GAP_M);
	}

	public InterlockingDeadlockAvoidance(Scenario scenario, double clusterGap) {
		this.scenario = scenario;
		this.network = scenario.getNetwork();
		this.clusterGap = clusterGap;
	}

	// ------------------------------------------------------------------ init

	@Override
	public void initResources(RailResourceManager rrm) {

		this.rrm = rrm;

		for (RailResource r : rrm.getResources()) {
			for (RailLink l : r.getLinks()) {
				railLinks.put(l.getLinkId(), l);
			}
		}

		// --- 1. route sequences, used links, stop flags, bidirectional resources ---
		Map<Id<RailResource>, Set<Id<Link>>> used = new HashMap<>();
		Map<Id<RailResource>, Double> length = new HashMap<>();
		Set<Id<Link>> usedLinks = new HashSet<>();
		int routes = 0;
		int departures = 0;
		int linksWithoutResource = 0;
		int stopLinksNotOnRoute = 0;
		int reversingStops = 0;

		for (TransitLine line : scenario.getTransitSchedule().getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {

				routes++;
				departures += route.getDepartures().size();

				List<Id<Link>> sequence = routeSequence(route);
				RouteInfo info = new RouteInfo(routeKey(line, route), sequence);

				for (Id<Link> linkId : sequence) {
					usedLinks.add(linkId);
					RailLink rl = railLinks.get(linkId);
					if (rl == null) {
						linksWithoutResource++;
						continue;
					}
					Id<RailResource> rid = rl.getResource().getId();
					used.computeIfAbsent(rid, k -> new HashSet<>()).add(linkId);
					length.merge(rid, rl.getLength(), Math::max);
				}

				for (TransitRouteStop stop : route.getStops()) {
					Id<Link> stopLink = stop.getStopFacility().getLinkId();
					int idx = indexOf(sequence, stopLink);
					if (idx >= 0) {
						info.stop[idx] = true;
					} else {
						stopLinksNotOnRoute++;
					}
				}

				for (int i = 0; i + 1 < sequence.size(); i++) {
					if (info.stop[i]) {
						RailLink l = railLinks.get(sequence.get(i));
						if (l != null && l.isOppositeLink(sequence.get(i + 1))) {
							reversingStops++;
						}
					}
				}

				routeInfos.put(routeKey(line, route), info);
			}
		}

		// --- 2. bidirectional = schedule uses a link and its opposite link ---
		double bidirectionalKm = 0;
		int bidirectionalLinks = 0;
		List<Id<RailResource>> sorted = new ArrayList<>(used.keySet());
		sorted.sort((a, b) -> a.toString().compareTo(b.toString()));

		List<Id<RailResource>> bidirectionalResources = new ArrayList<>();
		for (Id<RailResource> rid : sorted) {
			Set<Id<Link>> links = used.get(rid);
			boolean both = false;
			for (Id<Link> linkId : links) {
				RailLink l = railLinks.get(linkId);
				if (l == null) {
					continue;
				}
				for (Id<Link> other : links) {
					if (!other.equals(linkId) && l.isOppositeLink(other)) {
						both = true;
						break;
					}
				}
				if (both) {
					break;
				}
			}
			if (both) {
				bidirectional.add(rid);
				bidirectionalResources.add(rid);
				bidirectionalLinks += links.size();
				bidirectionalKm += length.getOrDefault(rid, 0.) / 1000.;
			}
		}

		// --- 3. junction nodes from the used link graph ---
		Map<Id<Node>, Set<Id<Node>>> neighbours = new HashMap<>();
		for (Id<Link> linkId : usedLinks) {
			Link link = network.getLinks().get(linkId);
			if (link == null) {
				continue;
			}
			Id<Node> from = link.getFromNode().getId();
			Id<Node> to = link.getToNode().getId();
			if (from.equals(to)) {
				continue;
			}
			neighbours.computeIfAbsent(from, k -> new HashSet<>()).add(to);
			neighbours.computeIfAbsent(to, k -> new HashSet<>()).add(from);
		}
		Set<Id<Node>> junctions = new HashSet<>();
		for (Map.Entry<Id<Node>, Set<Id<Node>>> e : neighbours.entrySet()) {
			if (e.getValue().size() >= 3) {
				junctions.add(e.getKey());
			}
		}

		// --- 4. cluster links: junction nodes, stops, and everything within clusterGap of them along a route ---
		Set<Id<Link>> clusterLinks = new HashSet<>();
		for (RouteInfo info : routeInfos.values()) {
			int n = info.size();
			if (n == 0) {
				continue;
			}
			Id<Node>[] nodeAt = nodesOf(info);

			for (int i = 0; i < n; i++) {
				if (junctions.contains(nodeAt[i]) || junctions.contains(nodeAt[i + 1])) {
					clusterLinks.add(info.links.get(i));
				}
				if (info.stop[i]) {
					clusterLinks.add(info.links.get(i));
				}
			}

			double[] prefix = new double[n + 1];
			for (int i = 0; i < n; i++) {
				prefix[i + 1] = prefix[i] + linkLength(info.links.get(i));
			}

			// every link whose route distance to a junction node is below clusterGap is part of the station/junction area; this closes
			// the small plain gaps inside a throat that a pure "endpoint is a junction" rule leaves (measured: 11-23 m plain links
			// inside the Luzern and Bern throats). Stops are marked themselves but do not grow an area: otherwise a line with stops
			// every few hundred metres becomes one multi-kilometre cluster (measured: Olten 2.3 -> 19.5 km). The route extension to
			// the next stop below bridges the remaining plain links to the platform.
			for (int j = 0; j <= n; j++) {
				if (!junctions.contains(nodeAt[j])) {
					continue;
				}
				for (int i = j - 1; i >= 0 && prefix[j] - prefix[i + 1] < clusterGap; i--) {
					clusterLinks.add(info.links.get(i));
				}
				for (int i = j; i < n && prefix[i] - prefix[j] < clusterGap; i++) {
					clusterLinks.add(info.links.get(i));
				}
			}
		}

		// --- 5. physical clusters = connected components of cluster links via shared nodes ---
		List<PhysicalCluster> clusters = buildPhysicalClusters(clusterLinks);

		// --- 6. per route: sections, protected areas and their arrival/departure/through routes ---
		for (RouteInfo info : routeInfos.values()) {
			markSectionsAndAreas(info, clusterLinks);
		}

		double clusterTrackKm = clusters.stream().mapToDouble(c -> c.km).sum();
		int clusterSegments = 0;
		int routesWithAreas = 0;
		Map<SegmentType, List<Double>> lengthsByType = new HashMap<>();
		List<AreaRoute> longest = new ArrayList<>();
		for (Map.Entry<String, RouteInfo> e : routeInfos.entrySet()) {
			if (e.getValue().routes.isEmpty()) {
				continue;
			}
			routesWithAreas++;
			for (AreaRoute r : e.getValue().routes) {
				clusterSegments++;
				lengthsByType.computeIfAbsent(r.type, k -> new ArrayList<>()).add(r.metres);
				longest.add(r);
			}
		}
		longest.sort((a, b) -> Double.compare(b.metres, a.metres));

		log.info("interlocking static analysis: {} transit routes, {} departures, {} links used by the schedule ({} links not in the railsim "
						+ "resource map), {} junction nodes, {} cluster links ({} km of track counted once per physical track), {} physical clusters "
						+ "({} km); {} protected routes (arrival/departure/through) on {} of {} routes; {} stop links of the schedule are not part of "
						+ "their own route; {} stops where a route reverses; clusterGap={} m",
				routes, departures, usedLinks.size(), linksWithoutResource, junctions.size(), clusterLinks.size(),
				Math.round(clusterTrackKm * 10.) / 10., clusters.size(), Math.round(clusterTrackKm * 10.) / 10.,
				clusterSegments, routesWithAreas, routeInfos.size(), stopLinksNotOnRoute, reversingStops,
				Math.round(clusterGap));

		log.info("section lock kept unchanged outside protected areas: {} of {} resources are bidirectional ({} used links, {} km of track), "
						+ "as in SectionLockDeadlockAvoidance",
				bidirectional.size(), used.size(), bidirectionalLinks, Math.round(bidirectionalKm * 10.) / 10.);

		for (SegmentType t : SegmentType.values()) {
			logLengthStats(t.name().toLowerCase(Locale.ROOT) + " route", lengthsByType.getOrDefault(t, List.of()));
		}

		log.info("interlocking: {} protected routes longer than 2 km (no cap applied; an exclusive route that long costs capacity)", countAbove(longest, 2000));
		for (int i = 0; i < Math.min(10, longest.size()); i++) {
			AreaRoute r = longest.get(i);
			log.info(String.format(Locale.ROOT,
					"interlocking longest route #%d: %s %s %.0f m %d links start=%s end=%s",
					i + 1, r.type, r.routeKey, r.metres, r.end - r.start, r.startLink, r.endLink));
		}

		List<PhysicalCluster> bySize = new ArrayList<>(clusters);
		bySize.sort((a, b) -> Double.compare(b.km, a.km));
		for (int i = 0; i < Math.min(10, bySize.size()); i++) {
			PhysicalCluster c = bySize.get(i);
			log.info(String.format(Locale.ROOT,
					"interlocking cluster #%d: id=%d %.2f km %d links centre=(%.0f,%.0f) nearest stop=%s",
					i + 1, c.id, c.km, c.links.size(), c.x, c.y, c.nearestStop));
		}
		log.info("interlocking: {} physical clusters are larger than 2 km", countClustersAbove(bySize, 2.0));

		writeBidirectionalCsv(bidirectionalResources, used);
		writeClusterCsv(clusters);
	}

	private static int countAbove(List<AreaRoute> routes, double metres) {
		int c = 0;
		for (AreaRoute r : routes) {
			if (r.metres > metres) {
				c++;
			}
		}
		return c;
	}

	private static int countClustersAbove(List<PhysicalCluster> clusters, double km) {
		int c = 0;
		for (PhysicalCluster p : clusters) {
			if (p.km > km) {
				c++;
			}
		}
		return c;
	}

	private void logLengthStats(String label, List<Double> values) {
		if (values.isEmpty()) {
			log.info("interlocking route lengths ({}): none", label);
			return;
		}
		double[] m = values.stream().mapToDouble(Double::doubleValue).sorted().toArray();
		double sum = 0;
		for (double v : m) {
			sum += v;
		}
		log.info("interlocking route lengths ({}): n={}, mean={} m, p50={} m, p95={} m, max={} m",
				label, m.length, Math.round(sum / m.length), Math.round(m[m.length / 2]),
				Math.round(m[Math.min(m.length - 1, (int) (m.length * 0.95))]), Math.round(m[m.length - 1]));	}

	/**
	 * Computes the section-lock sections of this route (same rule as SectionLockDeadlockAvoidance: maximal run of bidirectional
	 * resources, ending after a non-reversing stop, before the first unidirectional resource, or at the route end) and marks every
	 * link of a section as protected if the section touches or contains a cluster. Then splits maximal protected runs at stops into
	 * the arrival/departure/through routes used by the locking rule.
	 */
	private void markSectionsAndAreas(RouteInfo info, Set<Id<Link>> clusterLinks) {

		int n = info.size();
		info.sectionEnd = new int[n];
		java.util.Arrays.fill(info.sectionEnd, -1);

		// every cluster link is protected, also the unidirectional platform/connection tracks that are in no section
		for (int i = 0; i < n; i++) {
			if (clusterLinks.contains(info.links.get(i))) {
				info.danger[i] = true;
			}
		}

		for (int k = 0; k < n; ) {
			RailLink l = railLinks.get(info.links.get(k));
			if (l == null || !bidirectional.contains(l.getResource().getId())) {
				k++;
				continue;
			}
			int e = k;
			while (e < n) {
				RailLink le = railLinks.get(info.links.get(e));
				if (le == null || !bidirectional.contains(le.getResource().getId())) {
					break;
				}
				boolean stop = info.stop[e];
				boolean reversal = stop && e + 1 < n && le.isOppositeLink(info.links.get(e + 1));
				e++;
				if (stop && !reversal) {
					break;
				}
			}

			boolean adjacent = (k > 0 && clusterLinks.contains(info.links.get(k - 1)))
					|| (e < n && clusterLinks.contains(info.links.get(e)));
			if (!adjacent) {
				for (int j = k; j < e; j++) {
					if (clusterLinks.contains(info.links.get(j))) {
						adjacent = true;
						break;
					}
				}
			}
			if (adjacent) {
				for (int j = k; j < e; j++) {
					info.danger[j] = true;
				}
			}
			for (int j = k; j < e; j++) {
				info.sectionEnd[j] = e;
			}
			k = e;
		}

		// maximal protected runs, split at non-reversing stops
		for (int i = 0; i < n; ) {
			if (!info.danger[i]) {
				i++;
				continue;
			}
			int s = i;
			while (i < n && info.danger[i]) {
				i++;
			}
			int e = i;

			List<Integer> splits = new ArrayList<>();
			for (int p = s; p < e; p++) {
				if (info.stop[p]) {
					// Unlike the section lock (deviation D1 there), a reversing stop DOES split an area here: the all-or-nothing
					// departure route check at the platform replaces the old "extend the arrival lock over the departure stretch"
					// and avoids holding a multi-kilometre departure route during the whole station dwell.
					splits.add(p);
				}
			}

			List<int[]> segments = new ArrayList<>();
			int cur = s;
			for (int p : splits) {
				if (p + 1 > cur) {
					segments.add(new int[]{cur, p + 1});
					cur = p + 1;
				}
			}
			if (cur < e) {
				segments.add(new int[]{cur, e});
			}

			// A protected route must end at a safe point. If the area ends before the train's next scheduled stop and that stop is
			// close, extend the last route to the stop: otherwise a train could be held at the area exit by an occupied platform and
			// block that platform's occupant (measured at Luzern: an 8 m junction area 1.2 km before the platform, held by IC21@05:07,
			// while the platform's IC21@05:01 needed exactly that junction area to leave).
			if (!segments.isEmpty() && e < n && !info.stop[e - 1]) {
				int p = -1;
				for (int x = e; x < n; x++) {
					if (info.stop[x]) {
						p = x;
						break;
					}
				}
				if (p >= 0) {
					double dist = 0;
					for (int x = e; x <= p; x++) {
						dist += linkLength(info.links.get(x));
					}
					if (dist <= MAX_ROUTE_EXTENSION_M) {
						segments.get(segments.size() - 1)[1] = p + 1;
						i = p + 1;
					}
				}
			}

			for (int j = 0; j < segments.size(); j++) {
				int[] seg = segments.get(j);
				boolean afterStop = seg[0] > 0 && info.stop[seg[0] - 1];
				boolean atStop = info.stop[seg[1] - 1];
				SegmentType type;
				if (atStop && afterStop) {
					type = SegmentType.MIDDLE;
				} else if (atStop) {
					type = SegmentType.ARRIVAL;
				} else if (afterStop) {
					type = SegmentType.DEPARTURE;
				} else {
					type = SegmentType.THROUGH;
				}
				for (int x = seg[0]; x < seg[1]; x++) {
					info.areaStart[x] = seg[0];
					info.areaEnd[x] = seg[1];
				}
				double metres = 0;
				for (int x = seg[0]; x < seg[1]; x++) {
					metres += linkLength(info.links.get(x));
				}
				info.routes.add(new AreaRoute(type, info.key, seg[0], seg[1], metres,
						info.links.get(seg[0]), info.links.get(seg[1] - 1)));
			}
		}
	}

	private Id<Node>[] nodesOf(RouteInfo info) {
		int n = info.size();
		@SuppressWarnings("unchecked")
		Id<Node>[] nodes = new Id[n + 1];
		for (int i = 0; i < n; i++) {
			Link l = network.getLinks().get(info.links.get(i));
			if (l != null) {
				nodes[i] = l.getFromNode().getId();
				if (i == n - 1) {
					nodes[n] = l.getToNode().getId();
				}
			}
		}
		return nodes;
	}

	private List<PhysicalCluster> buildPhysicalClusters(Set<Id<Link>> clusterLinks) {

		List<Id<Link>> links = new ArrayList<>(clusterLinks);
		links.sort((a, b) -> a.toString().compareTo(b.toString()));
		Map<Id<Link>, Integer> index = new HashMap<>();
		for (int i = 0; i < links.size(); i++) {
			index.put(links.get(i), i);
		}
		int[] parent = new int[links.size()];
		for (int i = 0; i < parent.length; i++) {
			parent[i] = i;
		}
		Map<Id<Node>, Integer> firstAtNode = new HashMap<>();
		for (int i = 0; i < links.size(); i++) {
			Link l = network.getLinks().get(links.get(i));
			if (l == null) {
				continue;
			}
			union(parent, i, firstAtNode.putIfAbsent(l.getFromNode().getId(), i));
			union(parent, i, firstAtNode.putIfAbsent(l.getToNode().getId(), i));
		}

		Map<Integer, List<Id<Link>>> components = new HashMap<>();
		for (int i = 0; i < links.size(); i++) {
			components.computeIfAbsent(find(parent, i), k -> new ArrayList<>()).add(links.get(i));
		}

		List<PhysicalCluster> result = new ArrayList<>();
		int id = 0;
		for (List<Id<Link>> component : components.values()) {
			PhysicalCluster cluster = new PhysicalCluster(id++, component);
			double sx = 0;
			double sy = 0;
			int n = 0;
			Set<String> countedPhysical = new HashSet<>();
			for (Id<Link> linkId : component) {
				Link l = network.getLinks().get(linkId);
				if (l == null) {
					continue;
				}
				sx += (l.getFromNode().getCoord().getX() + l.getToNode().getCoord().getX()) / 2.;
				sy += (l.getFromNode().getCoord().getY() + l.getToNode().getCoord().getY()) / 2.;
				n++;
				RailLink rl = railLinks.get(linkId);
				String key = rl == null ? "link:" + linkId : physicalKey(linkId, rl);
				if (countedPhysical.add(key)) {
					cluster.km += linkLength(linkId) / 1000.;
				}
			}
			cluster.x = n == 0 ? 0 : sx / n;
			cluster.y = n == 0 ? 0 : sy / n;
			result.add(cluster);
		}

		for (PhysicalCluster c : result) {
			double best = Double.POSITIVE_INFINITY;
			for (TransitStopFacility f : scenario.getTransitSchedule().getFacilities().values()) {
				double dx = f.getCoord().getX() - c.x;
				double dy = f.getCoord().getY() - c.y;
				double d = dx * dx + dy * dy;
				if (d < best) {
					best = d;
					c.nearestStop = f.getName() == null ? f.getId().toString() : f.getName();
					c.nearestStopDistance = Math.sqrt(d);
				}
			}
		}
		return result;
	}

	private static String physicalKey(Id<Link> linkId, RailLink link) {
		// count each physical track segment once: keep the smaller id of a link/opposite-link pair
		Id<Link> opposite = null;
		for (RailLink other : link.getResource().getLinks()) {
			if (!other.getLinkId().equals(linkId) && link.isOppositeLink(other.getLinkId())) {
				opposite = other.getLinkId();
				break;
			}
		}
		if (opposite == null) {
			return "link:" + linkId;
		}
		return linkId.toString().compareTo(opposite.toString()) <= 0 ? "link:" + linkId : "link:" + opposite;
	}

	private static int find(int[] parent, int i) {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	}

	private static void union(int[] parent, int a, Integer b) {
		if (b == null) {
			return;
		}
		int ra = find(parent, a);
		int rb = find(parent, b);
		if (ra != rb) {
			parent[ra] = rb;
		}
	}

	private List<Id<Link>> routeSequence(TransitRoute route) {
		List<Id<Link>> sequence = new ArrayList<>();
		sequence.add(route.getRoute().getStartLinkId());
		sequence.addAll(route.getRoute().getLinkIds());
		sequence.add(route.getRoute().getEndLinkId());
		return sequence;
	}

	private static String routeKey(TransitLine line, TransitRoute route) {
		return line.getId() + "|" + route.getId();
	}

	private static int indexOf(List<Id<Link>> list, Id<Link> value) {
		for (int i = 0; i < list.size(); i++) {
			if (list.get(i).equals(value)) {
				return i;
			}
		}
		return -1;
	}

	private double linkLength(Id<Link> linkId) {
		RailLink rl = railLinks.get(linkId);
		if (rl != null) {
			return rl.getLength();
		}
		Link l = network.getLinks().get(linkId);
		return l == null ? 0 : l.getLength();
	}

	// ---------------------------------------------------------------- output

	/** Writes every bidirectional resource with one representative used link and its coordinates. */
	private void writeBidirectionalCsv(List<Id<RailResource>> resources, Map<Id<RailResource>, Set<Id<Link>>> used) {

		Path file = outputDir().resolve(CSV_RESOURCES);
		try {
			if (file.getParent() != null) {
				Files.createDirectories(file.getParent());
			}
			try (BufferedWriter out = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
				out.write("resource_id,link_id,from_x,from_y,to_x,to_y,length_m,n_used_links,used_links\n");
				for (Id<RailResource> rid : resources) {
					Set<Id<Link>> links = used.get(rid);
					Id<Link> representative = null;
					for (Id<Link> linkId : links) {
						if (railLinks.containsKey(linkId)) {
							representative = linkId;
							break;
						}
					}
					if (representative == null) {
						continue;
					}
					Link link = network.getLinks().get(representative);
					List<String> ids = new ArrayList<>();
					for (Id<Link> linkId : links) {
						ids.add(linkId.toString());
					}
					out.write(String.format(Locale.ROOT, "%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s%n",
							rid, representative,
							link.getFromNode().getCoord().getX(), link.getFromNode().getCoord().getY(),
							link.getToNode().getCoord().getX(), link.getToNode().getCoord().getY(),
							link.getLength(), ids.size(), String.join(" ", ids)));
				}
			}
			log.info("interlocking: wrote {} bidirectional resources to {}", resources.size(), file);
		} catch (IOException e) {
			log.error("interlocking: could not write {}", file, e);
		}
	}

	private void writeClusterCsv(List<PhysicalCluster> clusters) {

		Path file = outputDir().resolve(CSV_CLUSTERS);
		try {
			if (file.getParent() != null) {
				Files.createDirectories(file.getParent());
			}
			try (BufferedWriter out = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
				out.write("cluster_id,centre_x,centre_y,n_links,km,nearest_stop_facility,nearest_stop_distance_m\n");
				for (PhysicalCluster c : clusters) {
					out.write(String.format(Locale.ROOT, "%d,%.1f,%.1f,%d,%.3f,%s,%.0f%n",
							c.id, c.x, c.y, c.links.size(), c.km,
							c.nearestStop == null ? "" : c.nearestStop.replace(',', ' '), c.nearestStopDistance));
				}
			}
			log.info("interlocking: wrote {} physical clusters to {}", clusters.size(), file);
		} catch (IOException e) {
			log.error("interlocking: could not write {}", file, e);
		}
	}

	private Path outputDir() {
		return Paths.get(scenario.getConfig().controller().getOutputDirectory());
	}

	// ------------------------------------------------------------- callbacks

	@Override
	public boolean checkLink(double time, RailLink link, TrainPosition position) {

		checkLinkCalls++;

		int idx = indexOf(position, link.getLinkId());
		if (idx < 0) {
			notOnRoute++;
			return true;
		}

		MobsimDriverAgent driver = position.getDriver();
		RouteInfo info = routeInfo(position);
		boolean sizeMatch = info != null && info.size() == position.getRouteSize();
		if (info != null && !sizeMatch) {
			routeSizeMismatch++;
		}

		// A route claim is authoritative for every link, also one that neither this train's area nor its section covers: an area
		// route may extend over plain/unidirectional links (and over another cluster), and another train must not reserve them
		// while the route holder still needs them. Own physical reservations are allowed (reversal on one's own track).
		Id<RailResource> rid = link.getResource().getId();
		Claim exclusive = exclusiveClaimByOther(rid, driver);
		if (exclusive != null && physicalHolder.get(rid) != driver) {
			deniedExclusive++;
			logDenial(time, driver, link, "exclusive", link.getLinkId(), link.getLinkId(), 0, 1, "claim",
					exclusive.driver().getId().toString(), exclusive.linkId().toString(), time - exclusive.time(),
					exclusive.resource().toString());
			return false;
		}

		if (sizeMatch && info.areaStart[idx] >= 0) {
			return checkArea(time, link, position, info, idx);
		}

		if (!bidirectional.contains(rid)) {
			return true;
		}

		int end = sectionEndAt(info, sizeMatch, position, idx);
		for (int i = idx; i < end; i++) {
			Claim conflict = conflictingClaim(position.getRoute(i), driver);
			if (conflict != null) {
				deniedSection++;
				logDenial(time, driver, link, "section", position.getRoute(idx).getLinkId(),
						position.getRoute(end - 1).getLinkId(), sectionLength(position, idx, end), end - idx,
						"claim", conflict.driver().getId().toString(), conflict.linkId().toString(),
						time - conflict.time(), conflict.resource().toString());
				return false;
			}
		}
		return true;
	}

	/**
	 * All-or-nothing check of the protected route containing {@code idx}: no resource of the route may be claimed or physically
	 * reserved by another train. Own claims and own reservations are ignored (the train may be inside the route).
	 */
	private boolean checkArea(double time, RailLink link, TrainPosition position, RouteInfo info, int idx) {

		MobsimDriverAgent driver = position.getDriver();
		int from = Math.max(idx, info.areaStart[idx]);
		int to = info.areaEnd[idx];

		for (int i = from; i < to; i++) {
			RailLink l = position.getRoute(i);
			Id<RailResource> rid = l.getResource().getId();

			Claim conflict = claimByOther(rid, driver);
			if (conflict != null) {
				deniedAreaClaim++;
				logDenial(time, driver, link, "area", position.getRoute(info.areaStart[idx]).getLinkId(),
						position.getRoute(to - 1).getLinkId(), sectionLength(position, from, to), to - from,
						"claim", conflict.driver().getId().toString(), conflict.linkId().toString(),
						time - conflict.time(), conflict.resource().toString());
				return false;
			}

			if (l.getResource().getState(l) != ResourceState.EMPTY) {
				MobsimDriverAgent holder = physicalHolder.get(rid);
				if (holder != driver) {
					deniedAreaOccupied++;
					logDenial(time, driver, link, "area", position.getRoute(info.areaStart[idx]).getLinkId(),
							position.getRoute(to - 1).getLinkId(), sectionLength(position, from, to), to - from,
							"occupied", holder == null ? "unknown" : holder.getId().toString(), l.getLinkId().toString(),
							-1, rid.toString());
					return false;
				}
			}
		}
		return true;
	}

	@Override
	public void onReserve(double time, RailResource resource, TrainPosition position) {

		MobsimDriverAgent driver = position.getDriver();
		physicalHolder.put(resource.getId(), driver);

		int idx = indexOfReserved(position, resource);
		if (idx < 0) {
			return;
		}

		List<Claim> mine = claimsByDriver.computeIfAbsent(driver, d -> new ArrayList<>());
		for (Claim c : mine) {
			removeClaim(c);
		}
		mine.clear();

		RouteInfo info = routeInfo(position);
		boolean sizeMatch = info != null && info.size() == position.getRouteSize();

		if (sizeMatch && info.areaStart[idx] >= 0) {
			int from = Math.max(idx, info.areaStart[idx]);
			int to = info.areaEnd[idx];
			for (int i = from; i < to; i++) {
				addClaim(mine, driver, position.getRoute(i), time, true);
			}
		} else if (bidirectional.contains(resource.getId())) {
			int end = sectionEndAt(info, sizeMatch, position, idx);
			for (int i = idx; i < end; i++) {
				addClaim(mine, driver, position.getRoute(i), time, false);
			}
		}

		if (mine.isEmpty()) {
			claimsByDriver.remove(driver);
		}
	}

	private void addClaim(List<Claim> mine, MobsimDriverAgent driver, RailLink link, double time, boolean exclusive) {
		for (Claim c : mine) {
			if (c.linkId().equals(link.getLinkId())) {
				return;
			}
		}
		Claim claim = new Claim(driver, link.getResource().getId(), link.getLinkId(), time, exclusive);
		mine.add(claim);
		claims.computeIfAbsent(claim.resource(), k -> new ArrayList<>()).add(claim);
	}

	@Override
	public void onRelease(double time, RailResource resource, MobsimDriverAgent driver) {

		if (physicalHolder.get(resource.getId()) == driver) {
			physicalHolder.remove(resource.getId());
		}

		Id<RailResource> rid = resource.getId();
		List<Claim> mine = claimsByDriver.get(driver);
		if (mine == null) {
			return;
		}
		for (int i = mine.size() - 1; i >= 0; i--) {
			Claim c = mine.get(i);
			if (c.resource().equals(rid)) {
				mine.remove(i);
				removeClaim(c);
			}
		}
		if (mine.isEmpty()) {
			claimsByDriver.remove(driver);
		}
	}

	@Override
	public void onReleaseLink(double time, RailLink link, MobsimDriverAgent driver) {

		List<Claim> mine = claimsByDriver.get(driver);
		if (mine == null) {
			return;
		}
		int end = -1;
		for (int i = 0; i < mine.size(); i++) {
			if (mine.get(i).linkId().equals(link.getLinkId())) {
				end = i;
				break;
			}
		}
		if (end < 0) {
			return;
		}
		for (int i = end; i >= 0; i--) {
			Claim c = mine.remove(i);
			removeClaim(c);
		}
		if (mine.isEmpty()) {
			claimsByDriver.remove(driver);
		}
	}

	@Override
	public boolean isReserved(RailResource resource) {
		List<Claim> list = claims.get(resource.getId());
		return list != null && !list.isEmpty();
	}

	@Override
	public boolean checkReroute(double time, RailLink start, RailLink end, List<RailLink> subRoute,
								List<RailLink> detour, TrainPosition position) {
		return true;
	}

	// --------------------------------------------------------------- helpers

	private void removeClaim(Claim claim) {
		List<Claim> list = claims.get(claim.resource());
		if (list == null) {
			return;
		}
		list.remove(claim);
		if (list.isEmpty()) {
			claims.remove(claim.resource());
		}
	}

	/** Index of the given link on the remaining route, starting at the current route index. */
	private static int indexOf(TrainPosition position, Id<Link> linkId) {

		for (int i = Math.max(0, position.getRouteIndex()); i < position.getRouteSize(); i++) {
			if (position.getRoute(i).getLinkId().equals(linkId)) {
				return i;
			}
		}
		int prev = position.getRouteIndex() - 1;
		if (prev >= 0 && prev < position.getRouteSize() && position.getRoute(prev).getLinkId().equals(linkId)) {
			return prev;
		}
		return -1;
	}

	/**
	 * Index of the link of {@code resource} the train has just reserved: the last link of that resource at or after
	 * {@code routeIndex - 1} that is currently reserved by the driver (reservations are made in route order).
	 */
	private int indexOfReserved(TrainPosition position, RailResource resource) {

		int first = -1;
		int lastHeld = -1;

		for (int i = Math.max(0, position.getRouteIndex() - 1); i < position.getRouteSize(); i++) {
			RailLink l = position.getRoute(i);
			if (!l.getResource().getId().equals(resource.getId())) {
				continue;
			}
			if (first < 0) {
				first = i;
			}
			if (rrm != null && rrm.isBlockedBy(l, position)) {
				lastHeld = i;
			}
		}
		return lastHeld >= 0 ? lastHeld : first;
	}

	private int sectionEndAt(RouteInfo info, boolean sizeMatch, TrainPosition position, int idx) {
		if (sizeMatch && idx >= 0 && idx < info.sectionEnd.length && info.sectionEnd[idx] >= 0) {
			return info.sectionEnd[idx];
		}
		return dynamicSectionEnd(position, idx);
	}

	/**
	 * Fallback for routes that are not in the schedule map (should not happen): the section end of SectionLockDeadlockAvoidance.
	 */
	private int dynamicSectionEnd(TrainPosition position, int startIdx) {

		int size = position.getRouteSize();
		RouteInfo info = routeInfo(position);
		boolean[] stop = null;
		if (info != null && info.size() == size) {
			stop = info.stop;
		}

		for (int i = startIdx; i < size; i++) {
			RailLink l = position.getRoute(i);
			if (!bidirectional.contains(l.getResource().getId())) {
				return i;
			}
			boolean isStop = stop != null ? stop[i] : position.isStop(l.getLinkId());
			if (!isStop) {
				continue;
			}
			boolean reversal = i + 1 < size && position.getRoute(i + 1).isOppositeLink(l.getLinkId());
			if (!reversal) {
				return i + 1;
			}
		}
		return size;
	}

	private double sectionLength(TrainPosition position, int from, int to) {
		double m = 0;
		for (int i = from; i < to; i++) {
			m += position.getRoute(i).getLength();
		}
		return m;
	}

	private RouteInfo routeInfo(TrainPosition position) {

		if (position.getPt() == null) {
			return null;
		}
		RouteInfo cached = driverRouteInfo.get(position.getDriver());
		if (cached != null && cached.size() == position.getRouteSize()) {
			return cached;
		}

		TransitRoute route = position.getPt().getTransitRoute();
		TransitLine line = position.getPt().getTransitLine();
		if (route == null || line == null) {
			return null;
		}
		RouteInfo info = routeInfos.get(routeKey(line, route));
		if (info == null) {
			unknownRouteCalls++;
			return null;
		}
		driverRouteInfo.put(position.getDriver(), info);
		return info;
	}

	/** Direction-conflicting claim of another driver (exclusive claims conflict in any direction). */
	private Claim conflictingClaim(RailLink wanted, MobsimDriverAgent driver) {

		List<Claim> list = claims.get(wanted.getResource().getId());
		if (list == null) {
			return null;
		}
		for (int i = 0; i < list.size(); i++) {
			Claim other = list.get(i);
			if (other.driver() == driver) {
				continue;
			}
			if (other.exclusive()) {
				return other;
			}
			if (wanted.isOppositeLink(other.linkId())) {
				return other;
			}
			RailLink claimed = railLinks.get(other.linkId());
			if (claimed != null && claimed.isOppositeLink(wanted.getLinkId())) {
				return other;
			}
		}
		return null;
	}

	/** Any claim of another driver on the resource (used by the exclusive route lock). */
	private Claim claimByOther(Id<RailResource> rid, MobsimDriverAgent driver) {
		List<Claim> list = claims.get(rid);
		if (list == null) {
			return null;
		}
		for (Claim c : list) {
			if (c.driver() != driver) {
				return c;
			}
		}
		return null;
	}

	/** An exclusive route claim of another driver on the resource, independent of direction. */
	private Claim exclusiveClaimByOther(Id<RailResource> rid, MobsimDriverAgent driver) {
		List<Claim> list = claims.get(rid);
		if (list == null) {
			return null;
		}
		for (Claim c : list) {
			if (c.driver() != driver && c.exclusive()) {
				return c;
			}
		}
		return null;
	}

	// ----------------------------------------------------------- diagnostics

	private void logDenial(double time, MobsimDriverAgent driver, RailLink link, String kind, Id<Link> startLink,
						   Id<Link> endLink, double metres, int links, String conflictKind, String conflictDriver,
						   String conflictLink, double conflictAge, String conflictResource) {

		if (denialRows < MAX_DENIAL_ROWS) {
			try {
				if (denialOut == null) {
					Path file = outputDir().resolve(CSV_DENIALS);
					Files.createDirectories(file.toAbsolutePath().getParent());
					denialOut = Files.newBufferedWriter(file, StandardCharsets.UTF_8);
					denialOut.write("time,driver,requested_link,kind,route_start_link,route_end_link,route_km,route_links,"
							+ "conflict_kind,conflict_driver,conflict_link,conflict_age_s,conflict_resource\n");
				}
				denialOut.write(String.format(Locale.ROOT, "%.0f,%s,%s,%s,%s,%s,%.3f,%d,%s,%s,%s,%.0f,%s%n",
						time, driver.getId(), link.getLinkId(), kind, startLink, endLink, metres / 1000., links,
						conflictKind, conflictDriver, conflictLink, conflictAge, conflictResource));
				denialRows++;
				if (denialRows % 500 == 0) {
					denialOut.flush();
				}
			} catch (IOException e) {
				log.error("interlocking: cannot write denial log", e);
				denialRows = MAX_DENIAL_ROWS;
			}
		}

		if (time - lastLogTime >= 3600) {
			lastLogTime = time;
			log.info("interlocking t={}: {} checkLink calls, {} denied (exclusive {}, area claim {}, area occupied {}, section {}), {} not on route, "
							+ "{} drivers with claims, {} claims on {} resources, {} routes without stop info, {} route size mismatches, "
							+ "{} physical reservations, {} denial rows logged",
					Math.round(time), checkLinkCalls, deniedExclusive + deniedAreaClaim + deniedAreaOccupied + deniedSection,
					deniedExclusive, deniedAreaClaim, deniedAreaOccupied, deniedSection, notOnRoute, claimsByDriver.size(),
					claims.values().stream().mapToInt(List::size).sum(), claims.size(), unknownRouteCalls,
					routeSizeMismatch, physicalHolder.size(), denialRows);
		}
	}

	/** Convenience for tests / debugging. */
	public Collection<Id<RailResource>> getBidirectionalResources() {
		return bidirectional;
	}

	// ------------------------------------------------------------------ types

	private enum SegmentType { ARRIVAL, DEPARTURE, THROUGH, MIDDLE }

	/** Per transit route: link sequence, stop flags, section ends and protected-route arrays. */
	private static final class RouteInfo {
		final String key;
		final List<Id<Link>> links;
		final boolean[] stop;
		/** Protected link (cluster, or a section of bidirectional track attached to a cluster). */
		final boolean[] danger;
		/** Index of the protected route containing this link, or -1. */
		final int[] areaStart;
		final int[] areaEnd;
		/** Static section end of the section lock for this link, or -1. */
		int[] sectionEnd;
		/** Protected routes of this route (for statistics, duplicates across departures on purpose). */
		final List<AreaRoute> routes = new ArrayList<>();

		RouteInfo(String key, List<Id<Link>> sequence) {
			this.key = key;
			this.links = sequence;
			this.stop = new boolean[sequence.size()];
			this.danger = new boolean[sequence.size()];
			this.areaStart = new int[sequence.size()];
			this.areaEnd = new int[sequence.size()];
			java.util.Arrays.fill(this.areaStart, -1);
			java.util.Arrays.fill(this.areaEnd, -1);
		}

		int size() {
			return links.size();
		}
	}

	private record AreaRoute(SegmentType type, String routeKey, int start, int end, double metres,
							 Id<Link> startLink, Id<Link> endLink) {
	}

	private static final class PhysicalCluster {
		final int id;
		final List<Id<Link>> links;
		double x;
		double y;
		double km;
		String nearestStop;
		double nearestStopDistance;

		PhysicalCluster(int id, List<Id<Link>> links) {
			this.id = id;
			this.links = links;
		}
	}

	/** One direction claim of a train on one resource; exclusive claims conflict in any direction. */
	private record Claim(MobsimDriverAgent driver, Id<RailResource> resource, Id<Link> linkId, double time,
						 boolean exclusive) {
	}
}
