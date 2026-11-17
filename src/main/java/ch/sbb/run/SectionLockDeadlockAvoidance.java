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
import java.io.InputStream;
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
import java.util.Properties;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import com.google.inject.Inject;
import org.matsim.core.mobsim.framework.MobsimDriverAgent;
import org.matsim.core.utils.misc.OptionalTime;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;

import ch.sbb.matsim.contrib.railsim.qsimengine.TrainPosition;
import ch.sbb.matsim.contrib.railsim.qsimengine.deadlocks.DeadlockAvoidance;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailResource;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailResourceManager;

/**
 * Section-lock deadlock avoidance for railsim {@code 2027.0-2026w45}.
 * <p>
 * railsim's default {@link ch.sbb.matsim.contrib.railsim.qsimengine.deadlocks.SimpleDeadlockAvoidance} claims the running
 * direction of <em>every</em> capacity-1 resource of a train's whole remaining route, because our network has no resource
 * with capacity &gt; 1 that could terminate that claim. On a double-track line that is used in one direction per track no
 * direction claim is needed at all, so the default makes trains block each other hundreds of kilometres away from any real
 * conflict.
 * <p>
 * This implementation only protects resources that the transit schedule really uses in <b>both</b> directions
 * (single-track lines, station tracks that are entered from both ends, terminus tracks where a train reverses). For those
 * it applies an all-or-nothing "section lock": a train may only reserve the first link of a section (= maximal run of
 * consecutive bidirectional resources of its own route) if no other train holds a claim in the opposite direction anywhere
 * in that section.
 * <p>
 * A section ends only at a <b>safe meeting point</b> ({@code sectionLock.stopRule=safe}, the default; see
 * {@link #analyseStop}): a stop whose own physical track is used in one direction only. At any other stop the section
 * continues to the next safe stop. {@code sectionLock.stopRule=any} restores the old behaviour ("every scheduled stop ends
 * the section") for comparison; it is <em>not</em> deadlock free - two opposing trains may then both enter a single-track
 * stretch up to their own platform track and refuse each other permanently (measured: 77 of 2 541 trains in the Switzerland
 * case, at Laufen and Courtemaîche, and 77 of 82 trains in the H1 Laufen sub-scenario).
 * <p>
 * <b>Invariant (three sentences).</b> (i) A train holds a claim on every link of the section ahead of it, and a section ends
 * only where an opposing train can pass: at a stop whose own track is used in one direction only, before the first
 * unidirectional resource, or at the end of the route. (ii) A train may start a section only if no other train holds a claim
 * in the opposite direction on any resource of that section, so of two opposing trains only the first one can ever be
 * inside a bidirectional stretch, and it keeps holding the whole stretch until it leaves it. (iii) Therefore the first
 * train always has a claim-free path to the far safe meeting point, an opposing body cannot stand in its way, a train that
 * follows in the same direction is gated only by the leader's block reservation (which the leader releases as it advances),
 * and a refusal always ends: it creates no claim, so the denied train blocks nothing, and the holder finishes its section.
 * <p>
 * Deviations from the plain description (all justified in {@code swarm/dla/REPORT.md}):
 * <ol>
 * <li>A section is <b>not</b> ended at a stop where the train reverses ({@code route[i+1].isOppositeLink(route[i])}). Such a
 * train occupies the platform track until it can leave on the opposite link, so the section is extended over the departure
 * stretch. Without this, a reversing train enters a platform that it cannot leave and blocks the station (measured: the
 * largest Bern cascade).</li>
 * <li>A train that traverses the same resource in both directions inside one section holds a claim for <b>both</b>
 * directions (a list of claims per resource, not a single one). <b>Changed by E2 (merged, {@code sectionLock.dedup=on},
 * default):</b> only <b>one</b> claim per physical track per section, in the direction the train meets it first - the second
 * traversal is the reversal retrace over rails the train already stands on and holds as a capacity-1 reservation. * {@code sectionLock.dedup=off} restores the base4 behaviour.</li>
 * <li>Claims are replaced wholesale on every reserve and trimmed on every link release, so no claim survives the train that
 * made it.</li>
 * <li>Bidirectional resources with capacity &gt; 1 (off-model track: metre gauge, abroad) are excluded from the lock: a
 * second train may enter them, so they can never deadlock and must never hold a section.</li>
 * </ol>
 * <p>
 * <b>BN2: the standing-area rule at reversing stops ({@code sectionLock.reversalStop=standingArea}).</b> A train that
 * reverses at a stop (Bern: IC6/IC61/IC81/ICE reverse on their platforms) occupies a body of 5-20 links behind the end of
 * its stop link, but its <em>departure</em> path is only needed at its published departure minute. The plain section rule
 * cannot end a section at a reversing stop (deviation 1) and therefore claims the whole departure path - at Bern a
 * 0.9-1.3 km section through the shared throat - for the whole 8-14 min dwell; punctual trains are refused at that claim
 * although the track is physically free and unused (measured: 37 155 of the 73 272 deviation-seconds of the Olten case).
 * <p>
 * With the rule on, a section may end at a stop where the train reverses if, at the moment the section is requested,
 * <ol>
 * <li>the <b>standing area</b> (the links under the train body when its head is at the end of the stop link) is free of
 * other trains' reservations and of other trains' claims (their way in and standing area),</li>
 * <li>the section does not cross another admitted train's standing area, and</li>
 * <li>neither the standing area lies on another admitted train's departure path nor another admitted train's standing area
 * lies on this train's departure path.</li>
 * </ol>
 * (3) is the deadlock-freedom condition: it makes the "standing area is on my way out" relation empty for every pair of
 * admitted trains, so no two standing trains can hold each other in. The claim then covers exactly the way in, the
 * standing area (claimed in <em>both</em> directions: a standing body blocks a track in both directions) and, for a
 * reversing train, the retraced departure lead in the departure direction (ZH1's rule, kept: a follower must never enter
 * the lead the train has to come back over). Nothing beyond the stop is claimed while the train stands; its departure
 * section is requested at departure time, when the normal test applies. If the test fails, the section continues past the
 * stop exactly as before, i.e. the old (safe but coarse) behaviour is the fallback.
 * <p>
 * Verified call sequence (railsim {@code 2027.0-2026w45}, source + {@code javap}):
 * <ul>
 * <li>{@code checkLink} is called only from {@code RailResourceManagerImpl.tryBlockLink} (line 240), <em>after</em>
 * {@code RailResource.hasCapacity} succeeded. It is never called speculatively by {@code hasCapacity} or by
 * {@code TrainRouter}.</li>
 * <li>{@code position.getRouteIndex()} is the index of the first link the head has <em>not</em> entered yet (nextLink); the
 * current head link is {@code route.get(routeIndex - 1)}. The reservation window starts at {@code getRouteIndex()}
 * (RailsimCalc.calcLinksToBlock).</li>
 * <li>{@code position.isStop(linkId)} is true only for the <em>next</em> stop ({@code TrainState.isStop}:
 * {@code nextStop.getLinkId().equals}), so the stop links of a whole route are derived from the transit schedule here.</li>
 * <li>{@code onReserve} is called for every link that was actually reserved (and for every link of a non-blocking area);
 * never for a link that is already fully reserved by the same driver.</li>
 * <li>{@code onRelease(resource, driver)} is only called when the driver has no more reservation on <em>any</em> link of
 * the resource ({@code FixedBlockResource.release} returns {@code allFree}); {@code onReleaseLink} is called for every
 * single link release.</li>
 * <li>At the end of a route all links of the route that are still blocked by the train are released, so every
 * <em>reserved</em> link produces release callbacks. Links that were only claimed (never reserved) do not, which is why
 * claims are re-computed on every reserve and trimmed on every link release.</li>
 * </ul>
 */
public final class SectionLockDeadlockAvoidance implements DeadlockAvoidance {

	private static final Logger log = LogManager.getLogger(SectionLockDeadlockAvoidance.class);

	private static final String CSV_RESOURCES = "bidirectional_resources.csv";
	private static final String CSV_DENIALS = "sectionlock_denials.csv";
	private static final String CSV_STOPS = "sectionlock_stops.csv";
	private static final int MAX_DENIAL_ROWS = 20000;

	/** Stop rule: {@code any} = old behaviour (every scheduled stop ends a section); {@code safe} = only a safe meeting point. */
	public static final String STOP_RULE_ANY = "any";
	public static final String STOP_RULE_SAFE = "safe";

	/** BN2: standing-area rule off (= base, a section never ends at a reversing stop). */
	public static final String REVERSAL_STOP_OFF = "off";
	/** BN2: standing-area rule at reversing stops only. */
	public static final String REVERSAL_STOP_STANDING = "standingArea";
	/** BN2: standing-area rule at reversing stops and at unsafe through stops (experimental, see the report). */
	public static final String REVERSAL_STOP_ANY = "standingAreaAny";

	private final Scenario scenario;
	private final Network network;

	/** Stop rule, see {@link #analyseStop}. */
	private final String stopRule;

	/** SWARM-FIX (I2, merge): E2's one-claim-per-resource de-duplication (default on; off = base4 behaviour). */
	private final boolean dedup;

	/**
	 * ZH1: retrace rule (default on; {@code sectionLock.retrace=off} restores the base7d behaviour).
	 * <p>
	 * A train's claim is a claim on the <b>section</b>, not on the few links its body happens to cover. A physical
	 * track that the train traverses twice in opposite directions - the platform track and the single-track lead of
	 * a terminus, where the train runs in, reverses and runs back out over the same rails - must therefore stay
	 * claimed in the <b>departure</b> direction until the train has really left it. Two things are needed:
	 * <ol>
	 * <li>{@link #onRelease} must not drop the claims of a resource when the train's <em>body</em> has left it: the
	 * body leaving is exactly what a reversal does on the arrival direction, and the departure-direction claim is
	 * part of the same section. Without this, a follower could enter the lead that the reversing train still needs
	 * (measured: Zürich HB, IR35 775.1131.R at platform 15 vs IC3 634.162.R);</li>
	 * <li>{@link #checkLink} must not refuse a train a track it already physically occupies: the other train cannot
	 * be there (capacity 1), and the refusal would stop a train from leaving the platform it stands on.</li>
	 * </ol>
	 */
	private final boolean retraceRule;

	/** BN2: standing-area rule mode ({@code off|standingArea|standingAreaAny}). */
	private final String reversalStopRule;

	/** BN2: the standing-area rule is active (reversing stops; with {@link #standingAreaAny} also unsafe through stops). */
	private final boolean standingAreaRule;

	/** BN2: also end sections at unsafe through stops when the standing-area admission test passes. */
	private final boolean standingAreaAny;

	/** BN2: per admitted train, the stop its current section ends at (standing area + departure path), if any. */
	private final Map<MobsimDriverAgent, Standing> driverStanding = new HashMap<>();

	private long standingSections;
	private long standingDenials;

	/**
	 * BN3: time-bounded retrace claim ({@code sectionLock.retracePass=on|off}, default on; off = BN2 behaviour).
	 * <p>
	 * ZH1 keeps the reversing train's claim on the <em>retraced departure lead</em> for its whole dwell so that no
	 * follower enters the single-track lead it has to come back over. That is structurally right but not
	 * time-bounded: a <b>through</b> train that only crosses the lead (its stop lies beyond it) is refused for the
	 * whole dwell even though it would have cleared the lead long before the reversing train departs (measured:
	 * EC 234 at Bern holds the lead 716 s and five IC1 lose 706 s each at Fribourg, BN2 §4).
	 * <p>
	 * With this rule a retrace claim does not conflict with a train B whose section ends at a scheduled stop
	 * <b>beyond</b> the lead and which is scheduled to have cleared the lead {@code retracePassMargin} seconds
	 * before the reversing train's published departure. A train that would stand on the lead (ZH1's follower into
	 * the same platform) is always refused: its body must be fully clear of the lead when its head stops.
	 */
	private final boolean retracePassRule;

	/** BN3: safety margin in seconds between the passing train's clearance of the lead and the reversing departure. */
	private final double retracePassMargin;

	/**
	 * BN3 diagnostic switch ({@code sectionLock.retracePassBody=on|off}, default on). {@code off} skips the
	 * "the standing body of the reversing train must not be on the passing train's way" test. It exists only to
	 * measure what the claim rule could buy if the body were not in the way; a run with it off must be audited
	 * for body overlaps.
	 */
	private final boolean retracePassBody;

	/**
	 * P4: the same excuse when our section ends <b>before</b> our next scheduled stop
	 * ({@code sectionLock.retracePassAhead=on|off}, <b>default off</b>; off = BN3 behaviour exactly).
	 * <p>
	 * A section is the maximal run of bidirectional resources, so it can be cut at a link the schedule uses in one
	 * direction only - in the Bern throat the arrival chain of platform 2 is cut 73 m after the platform-4 lead.
	 * BN3 then refuses the crossing because the train does not stand where its section ends. In reality it does not
	 * stand there either: it drives on to its next scheduled stop (platform 2, 650 m further), where its body is
	 * clear of the lead by a wide margin. The rule therefore substitutes our <em>next scheduled stop</em> for the end
	 * of the section in all of BN3's body and time tests and adds two tests that make the substitution safe:
	 * <ul>
	 * <li>(f) <b>way ahead free</b>: no other train holds a reservation ({@code RailResourceManager.isBlockedBy}) or a
	 * non-retrace claim on any link of our way from the section start to that stop, and the way is at most
	 * {@code sectionLock.retracePassAheadMaxM} metres (2 000 m). So we cannot be stopped with our body on the lead:
	 * everything we have to drive over is free, and any train that could take a link of our way later is <em>ahead</em>
	 * of us (it never needs the lead, which lies behind it) and therefore clears it;</li>
	 * <li>(g) the time bound is computed for the whole way to that stop, not only over the lead, so we are predicted
	 * to stand clear of the lead {@code retracePassMargin} s before the reversing train's published departure.</li>
	 * </ul>
	 * Invariant: after the crossing we hold our ordinary claim, we stand at a scheduled stop whose body is completely
	 * clear of the lead, and the reversing train's standing body is neither on our way in nor ours on its departure
	 * path - exactly BN3's invariant, with "the place where we really stand" instead of "the place where our section
	 * ends". A refused crossing creates no claim, so a refusal can never block anybody.
	 */
	private final boolean retracePassAheadRule;

	/** P4: longest way ahead (m) that the ahead rule may look at; longer ways are refused. */
	private final double retracePassAheadMaxM;

	private long retracePassAllowed;
	private long retracePassRefused;
	private long retracePassAheadAllowed;
	/** BN3: why a candidate crossing of a retrace claim was refused (diagnostics). */
	private final long[] retraceRefusalReason = new long[9];
	private static final String[] RETRACE_REFUSAL_NAMES = {
			"holder-not-standing", "lead-not-on-section", "section-no-stop", "stop-on-way-to-lead",
			"body-on-lead", "time-bound", "body-of-holder-on-way", "path-interference", "next-stop-way-not-free"};

	private void refuseRetrace(int reason) {
		retracePassRefused++;
		if (reason >= 0 && reason < retraceRefusalReason.length) {
			retraceRefusalReason[reason]++;
		}
	}

	/** Stop link -> true if an opposing train does not need this stop's own track (only used with stopRule=safe). */
	private final Map<Id<Link>, Boolean> stopSafe = new HashMap<>();

	/** Stop link -> human readable reason for the safety decision. */
	private final Map<Id<Link>, String> stopSafeReason = new HashMap<>();

	// ---------------------------------------------------------------- V1 pairwise meet rule
	/**
	 * V1: pairwise meeting rule. H1's rule ends a section at a stop only if the stop's own physical track is used in
	 * one direction by the <em>whole</em> timetable. At a two-track station of a single-track line each platform track
	 * is normally used in both directions at different times of the day, so the rule refuses to end the section there,
	 * the stretches on both sides of the station become one long exclusive section and a scheduled meet cannot happen.
	 * <p>
	 * A stop is therefore also a meeting point when the station has a <b>second platform track</b> that the opposite
	 * direction really uses, i.e. another stop link with
	 * <ol>
	 * <li>the same station (same UIC number of the stop facility),</li>
	 * <li>a <b>different physical track</b> (different railsim resource), and</li>
	 * <li>a <b>different OSM way</b> - two links of one OSM way are the same continuous rail, however far apart they
	 * are (that is the Laufen artefact: both directions of platform 2 are two links of way 177304964, 550 m apart),</li>
	 * </ol>
	 * and at least one transit route stops on that partner link.
	 * <p>
	 * The rule is applied <b>pairwise</b> at claim time as well ({@link #pairwiseMeet}): a claim conflict with train B
	 * is excused only if B's own claim leads to a stop link that is a partner of ours by exactly the three tests
	 * above. So two trains that stand on the <em>same</em> track of a two-track station still refuse each other.
	 */
	private final boolean meetRule;

	/** Stop link -> partner stop links at the same station on a different track and a different OSM way. */
	private final Map<Id<Link>, Set<Id<Link>>> meetPartners = new HashMap<>();

	/** Stop link -> true if it has at least one partner (i.e. the station really has a second platform track). */
	private final Set<Id<Link>> meetSafe = new HashSet<>();

	/** Link -> OSM way id (network attribute {@code osm:way:id}), used to tell parallel tracks from one rail. */
	private final Map<Id<Link>, String> linkWay = new HashMap<>();

	/** Link -> station key (UIC number of the stop facility), for "same station" tests. */
	private final Map<Id<Link>, String> linkStation = new HashMap<>();

	/** Routes stopping on a link, used to require that a partner track is really served. */
	private final Set<Id<Link>> servicedStopLinks = new HashSet<>();

	/** Link -> its physical reverse link (same length, swapped endpoints), from the network (diagnostics only). */
	private final Map<Id<Link>, Id<Link>> reverseLink = new HashMap<>();

	/** Resource -> true if the transit schedule uses two opposite links of it. */
	private final Set<Id<RailResource>> bidirectional = new HashSet<>();

	/** All rail links by link id (from the resource manager). */
	private final Map<Id<Link>, RailLink> railLinks = new HashMap<>();

	/** Stop links per transit route: lineId|routeId -> per-route-index stop flags. */
	private final Map<String, RouteInfo> routeInfos = new HashMap<>();

	/** Per resource: the (driver, link) claims currently held on it. A driver may hold both directions of a resource. */
	private final Map<Id<RailResource>, List<Claim>> claims = new HashMap<>();

	/** Station key -> claims whose section ends at a stop of that station (pairwise meet test). */
	private final Map<String, List<Claim>> stopClaims = new HashMap<>();

	/** Per driver: its claims in route order (used to drop claims the train has passed). */
	private final Map<MobsimDriverAgent, List<Claim>> claimsByDriver = new HashMap<>();

	/** Cached route info per driver, only used to avoid rebuilding the "line|route" key on every call. */
	private final Map<MobsimDriverAgent, RouteInfo> driverRouteInfo = new HashMap<>();

	private RailResourceManager rrm;

	private BufferedWriter denialOut;
	private int denialRows;

	private long checkLinkCalls;
	private long denied;
	private long notOnRoute;
	private long unknownRouteCalls;
	private double lastLogTime = Double.NEGATIVE_INFINITY;

	@Inject
	public SectionLockDeadlockAvoidance(Scenario scenario) {
		this(scenario, STOP_RULE_SAFE, true);
	}

	public SectionLockDeadlockAvoidance(Scenario scenario, String stopRule) {
		this(scenario, stopRule, true);
	}

	public SectionLockDeadlockAvoidance(Scenario scenario, String stopRule, boolean dedup) {
		this.scenario = scenario;
		this.network = scenario.getNetwork();
		this.stopRule = STOP_RULE_ANY.equalsIgnoreCase(stopRule) ? STOP_RULE_ANY : STOP_RULE_SAFE;
		this.dedup = dedup;
		this.meetRule = isOn(runProperty("sectionLock.meet", "on"));
		this.retraceRule = isOn(runProperty("sectionLock.retrace", "on"));
		this.reversalStopRule = runProperty("sectionLock.reversalStop", REVERSAL_STOP_OFF).trim();
		this.standingAreaRule = REVERSAL_STOP_STANDING.equalsIgnoreCase(reversalStopRule)
				|| REVERSAL_STOP_ANY.equalsIgnoreCase(reversalStopRule);
		this.standingAreaAny = REVERSAL_STOP_ANY.equalsIgnoreCase(reversalStopRule);
		this.retracePassRule = isOn(runProperty("sectionLock.retracePass", "on"));
		this.retracePassMargin = parseDouble(runProperty("sectionLock.retracePassMargin", "120"), 120.);
		this.retracePassBody = isOn(runProperty("sectionLock.retracePassBody", "on"));
		this.retracePassAheadRule = isOn(runProperty("sectionLock.retracePassAhead", "off"));
		this.retracePassAheadMaxM = parseDouble(runProperty("sectionLock.retracePassAheadMaxM", "2000"), 2000.);
	}

	/** BN3: numeric local property, {@code def} if missing or invalid. */
	private static double parseDouble(String v, double def) {
		try {
			return v == null || v.isEmpty() ? def : Double.parseDouble(v);
		} catch (NumberFormatException e) {
			return def;
		}
	}

	private static boolean isOn(String v) {
		return v == null || v.isEmpty() || "on".equalsIgnoreCase(v) || "true".equalsIgnoreCase(v);
	}

	/** V1: local property switch (run.properties in the working directory), default on. */
	private static String runProperty(String key, String def) {
		Path p = Paths.get("run.properties");
		if (!Files.exists(p)) {
			return def;
		}
		Properties pr = new Properties();
		try (InputStream in = Files.newInputStream(p)) {
			pr.load(in);
		} catch (IOException e) {
			return def;
		}
		return pr.getProperty(key, def).trim();
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

		// --- 1. static analysis: which links does the schedule use, per resource ---
		Map<Id<RailResource>, Set<Id<Link>>> used = new HashMap<>();
		Map<Id<RailResource>, Double> length = new HashMap<>();
		Set<Id<Link>> stopLinks = new HashSet<>();
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
				RouteInfo info = new RouteInfo(sequence);
				analyseRetrace(info);

				for (Id<Link> linkId : sequence) {
					RailLink rl = railLinks.get(linkId);
					if (rl == null) {
						linksWithoutResource++;
						continue;
					}
					Id<RailResource> rid = rl.getResource().getId();
					used.computeIfAbsent(rid, k -> new HashSet<>()).add(linkId);
					length.merge(rid, rl.getLength(), Math::max);
				}

				// stop links, expressed as index into the railsim route sequence
				List<TransitRouteStop> routeStops = route.getStops();
				for (int j = 0; j < routeStops.size(); j++) {
					TransitRouteStop stop = routeStops.get(j);
					Id<Link> stopLink = stop.getStopFacility().getLinkId();
					stopLinks.add(stopLink);
					int idx = indexOf(sequence, stopLink);
					if (idx >= 0) {
						info.stop[idx] = true;
						if (info.stopNumber[idx] < 0) {
							info.stopNumber[idx] = j;
						}
					} else {
						stopLinksNotOnRoute++;
					}
				}

				// count stops where the route continues on the opposite link (= the train reverses there)
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

		buildReverseLinks();

		// --- 2. bidirectional = schedule uses a link and its opposite link ---
		double bidirectionalKm = 0;
		int bidirectionalLinks = 0;
		List<Id<RailResource>> capacityResources = new ArrayList<>();
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
				// Off-model track (boundaryExtension, capacity > 1) can never deadlock: a second train may enter.
				// Such a resource must not take part in the section lock at all.
				int capacity = 1;
				for (Id<Link> linkId : links) {
					RailLink l = railLinks.get(linkId);
					if (l != null) {
						capacity = l.getResource().getTotalCapacity();
						break;
					}
				}
				if (capacity > 1) {
					capacityResources.add(rid);
					continue;
				}
				bidirectional.add(rid);
				bidirectionalResources.add(rid);
				bidirectionalLinks += links.size();
				bidirectionalKm += length.getOrDefault(rid, 0.) / 1000.;
			}
		}

		// --- 2b. BN2: static route geometry for the standing-area rule (way-out ranges, resource index) ---
		analyseRouteGeometry();
		if (standingAreaRule) {
			log.info("sectionLock standing-area rule: ON (sectionLock.reversalStop={}); a section may end at a reversing stop"
					+ " (and, with standingAreaAny, at an unsafe through stop) when the standing area is free and no admitted"
					+ " train's departure path crosses it", reversalStopRule);
		}
		if (retracePassRule) {
			log.info("sectionLock time-bounded retrace claim: ON (sectionLock.retracePassMargin={} s); a train whose stop lies"
					+ " beyond a reversing train's departure lead may cross it when it is scheduled to be clear that long"
					+ " before the reversing train's published departure", retracePassMargin);
			log.info("sectionLock retracePassAhead={}: a crossing is also excused when our section ends before our next"
					+ " scheduled stop, if the whole way to that stop is free (max {} m)",
					retracePassAheadRule ? "on" : "off", Math.round(retracePassAheadMaxM));
		}

		// --- 3. stop rule: which stops are safe meeting points ---
		for (Id<Link> linkId : stopLinks) {
			if (STOP_RULE_ANY.equals(stopRule)) {
				stopSafe.put(linkId, true);
				stopSafeReason.put(linkId, "stopRule=any");
			} else {
				analyseStop(linkId);
			}
		}

		// --- 3b. V1 pairwise meet rule: stations with a real second platform track ---
		analyseMeetPoints(stopLinks);

		log.info("sectionLock static analysis: {} transit routes, {} departures, {} links used by the schedule ({} links not in the railsim "
						+ "resource map), {} resources used by the schedule; {} of them are BIDIRECTIONAL ({} used links, {} km of track, measured as "
						+ "one link length per resource); {} stop links of the schedule are not part of their own route; {} stops where a route "
						+ "reverses; {} bidirectional resources with capacity > 1 (off-model track) are NOT locked",
				routes, departures, used.values().stream().mapToInt(Set::size).sum(), linksWithoutResource, used.size(),
				bidirectional.size(), bidirectionalLinks, Math.round(bidirectionalKm * 10.) / 10., stopLinksNotOnRoute, reversingStops,
				capacityResources.size());

		logSectionStatistics();

		writeBidirectionalCsv(bidirectionalResources, used);
		writeStopCsv(stopLinks);

		long safe = stopLinks.stream().filter(l -> stopSafe.getOrDefault(l, true)).count();
		log.info("sectionLock stopRule={}: {} stop links, {} safe meeting points, {} unsafe (section continues past them)",
				stopRule, stopLinks.size(), safe, stopLinks.size() - safe);
	}

	/**
	 * ZH1 retrace rule: mark every link whose opposite link already appeared earlier on the same route. The train
	 * passes such a track twice in opposite directions - the terminus turn-around (arrival over the platform link and
	 * the single-track lead, departure back over the opposite links of the same resources).
	 */
	private void analyseRetrace(RouteInfo info) {

		Set<Id<Link>> seen = new HashSet<>();
		for (int i = 0; i < info.size(); i++) {
			Id<Link> id = info.links.get(i);
			if (id == null) {
				continue;
			}
			Id<Link> opp = oppositeLink(id);
			info.retrace[i] = opp != null && seen.contains(opp);
			seen.add(id);
		}
	}

	/**
	 * BN2 static geometry per route: {@code nextNonBi[i]} = the first index at or after {@code i} whose resource is not
	 * bidirectional, and {@code resourceIndices} = the route indices of every resource. The <b>departure path</b>
	 * (way out) of a stop at index {@code i} is the maximal run of consecutive bidirectional resources after it,
	 * {@code [i + 1, nextNonBi[i + 1])} - the track the train has to traverse when it leaves the stop.
	 */
	private void analyseRouteGeometry() {

		for (RouteInfo info : routeInfos.values()) {
			int size = info.size();
			info.nextNonBi = new int[size + 1];
			info.nextNonBi[size] = size;
			for (int i = size - 1; i >= 0; i--) {
				RailLink l = railLinks.get(info.links.get(i));
				boolean bi = l != null && bidirectional.contains(l.getResource().getId());
				info.nextNonBi[i] = bi ? info.nextNonBi[i + 1] : i;
			}
			Map<Id<RailResource>, List<Integer>> indices = new HashMap<>();
			for (int i = 0; i < size; i++) {
				RailLink l = railLinks.get(info.links.get(i));
				if (l != null) {
					indices.computeIfAbsent(l.getResource().getId(), k -> new ArrayList<>()).add(i);
				}
			}
			info.resourceIndices = new HashMap<>();
			for (Map.Entry<Id<RailResource>, List<Integer>> e : indices.entrySet()) {
				List<Integer> list = e.getValue();
				int[] a = new int[list.size()];
				for (int i = 0; i < a.length; i++) {
					a[i] = list.get(i);
				}
				info.resourceIndices.put(e.getKey(), a);
			}
		}
	}

	/** The opposite link of {@code id} in its railsim resource (same physical track, other direction), or null. */
	private Id<Link> oppositeLink(Id<Link> id) {

		RailLink l = railLinks.get(id);
		if (l == null) {
			return null;
		}
		for (RailLink other : l.getResource().getLinks()) {
			if (!other.getLinkId().equals(id) && l.isOppositeLink(other.getLinkId())) {
				return other.getLinkId();
			}
		}
		return null;
	}

	private List<Id<Link>> routeSequence(TransitRoute route) {		List<Id<Link>> sequence = new ArrayList<>();
		sequence.add(route.getRoute().getStartLinkId());
		sequence.addAll(route.getRoute().getLinkIds());
		sequence.add(route.getRoute().getEndLinkId());
		return sequence;
	}

	// ------------------------------------------------------------- stop rule

	/** Maps every network link to its physical reverse link (same length +- 0.1 m, swapped from/to nodes). Diagnostics only. */
	private void buildReverseLinks() {
		Map<String, Id<Link>> byKey = new HashMap<>();
		for (Link l : network.getLinks().values()) {
			byKey.put(key(l.getFromNode().getId(), l.getToNode().getId(), l.getLength()), l.getId());
		}
		for (Link l : network.getLinks().values()) {
			Id<Link> rev = byKey.get(key(l.getToNode().getId(), l.getFromNode().getId(), l.getLength()));
			if (rev != null && !rev.equals(l.getId())) {
				reverseLink.put(l.getId(), rev);
			}
		}
	}

	private static String key(Id<?> from, Id<?> to, double length) {
		return from + "|" + to + "|" + Math.round(length * 10.);
	}

	/**
	 * A stop is a <b>safe meeting point</b> if an opposing train does not need the stop's own track:
	 * <ol>
	 * <li>the physical track of the stop link (the railsim resource = all links between the same two nodes, including their
	 * reverse links) is used in <b>one direction only</b> - a double-track line, a one-way station track, or</li>
	 * <li>the track is off-model (capacity &gt; 1), so a second train may stand on it - then the resource is not part of
	 * {@link #bidirectional} either.</li>
	 * </ol>
	 * Otherwise the section must not end at this stop: two opposing trains could each enter the single-track stretch up to
	 * their own platform track, each claim the section beyond it and then need the other train's standing place - a mutual
	 * refusal that never clears. That is the defect measured at Laufen (IC51) and Courtemaîche (RE56) in the Switzerland
	 * case and at Interlaken West in round 1.
	 * <p>
	 * A parallel second track <em>elsewhere</em> at the station is deliberately <b>not</b> accepted as an exception (round 1
	 * used a geometric 150 m test): a second track does not help two trains that both use the same track, and the geometric
	 * test was measured to be wrong at exactly the two failing places - at Courtemaîche it accepted links 92-113 m away along
	 * the same curving single track, at Laufen a partner 1.5 m beside the same track (H1/REPORT.md §3).
	 */
	private void analyseStop(Id<Link> stopLink) {

		Link l = network.getLinks().get(stopLink);
		if (l == null) {
			stopSafe.put(stopLink, true);
			stopSafeReason.put(stopLink, "link not in network");
			return;
		}
		RailLink rl = railLinks.get(stopLink);
		Id<RailResource> rid = rl == null ? null : rl.getResource().getId();
		if (rid == null) {
			stopSafe.put(stopLink, true);
			stopSafeReason.put(stopLink, "link has no railsim resource");
			return;
		}
		if (!bidirectional.contains(rid)) {
			stopSafe.put(stopLink, true);
			stopSafeReason.put(stopLink, "stop track used in one direction only (one-way resource)");
			return;
		}
		stopSafe.put(stopLink, false);
		Id<Link> rev = reverseLink.get(stopLink);
		stopSafeReason.put(stopLink, "stop track used in BOTH directions"
				+ (rev == null ? "" : " (reverse link " + rev + " is used as well)"));
	}

	/**
	 * V1: pairwise meet rule preparation. For every scheduled stop link, finds the <b>partner stop links</b> at the
	 * same station: another stop link that (i) belongs to the same station (same UIC number of the stop facility),
	 * (ii) is a different physical track (different railsim resource) and (iii) is a different OSM way. Condition (iii)
	 * is what separates a real second platform track from two links of one continuous rail (at Laufen both directions
	 * of platform 2 are links of way 177304964, 550 m apart - H1 measured the deadlock that a geometric test caused
	 * there). Only partner links on which at least one transit route really stops count.
	 */
	private void analyseMeetPoints(Set<Id<Link>> stopLinks) {

		// station key and OSM way of every link
		for (Link l : network.getLinks().values()) {
			Object way = l.getAttributes().getAttribute("osm:way:id");
			if (way != null) {
				linkWay.put(l.getId(), way.toString());
			}
		}
		for (TransitLine line : scenario.getTransitSchedule().getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {
				for (TransitRouteStop stop : route.getStops()) {
					Id<Link> lid = stop.getStopFacility().getLinkId();
					if (lid == null) {
						continue;
					}
					servicedStopLinks.add(lid);
					linkStation.putIfAbsent(lid, stationKey(stop.getStopFacility().getId()));
				}
			}
		}

		for (Id<Link> linkId : stopLinks) {
			String station = linkStation.get(linkId);
			Id<RailResource> own = resourceOf(linkId);
			String ownWay = linkWay.get(linkId);
			if (station == null || own == null) {
				continue;
			}
			Set<Id<Link>> partners = new HashSet<>();
			for (Id<Link> other : stopLinks) {
				if (other.equals(linkId) || !servicedStopLinks.contains(other)) {
					continue;
				}
				if (!station.equals(linkStation.get(other))) {
					continue;
				}
				Id<RailResource> rid = resourceOf(other);
				if (rid == null || rid.equals(own)) {
					continue;
				}
				String otherWay = linkWay.get(other);
				if (ownWay == null || otherWay == null || ownWay.equals(otherWay)) {
					continue;
				}
				partners.add(other);
			}
			if (!partners.isEmpty()) {
				meetPartners.put(linkId, partners);
				meetSafe.add(linkId);
			}
		}

		log.info("sectionLock meet rule ({}): {} of {} stop links have a real second platform track at their station "
						+ "(partner stop links on a different physical track and a different OSM way); {} such stops",
				meetRule ? "on" : "off", meetSafe.size(), stopLinks.size(), meetSafe.size());
	}

	/** UIC number of a stop facility id ({@code 8500218:0:7.link:42} -> {@code 8500218}). */
	private static String stationKey(Id<TransitStopFacility> facilityId) {
		String s = facilityId.toString();
		int i = s.indexOf(".link:");
		if (i >= 0) {
			s = s.substring(0, i);
		}
		s = s.replace("Parent", "");
		int c = s.indexOf(':');
		return c > 0 ? s.substring(0, c) : s;
	}

	private Id<RailResource> resourceOf(Id<Link> linkId) {
		RailLink rl = railLinks.get(linkId);
		return rl == null ? null : rl.getResource().getId();
	}

	/** True if the two links are the two tracks of one station (V1 pairwise meet rule). */
	private boolean isMeetingPair(Id<Link> mine, Id<Link> theirs) {
		if (mine == null || theirs == null || mine.equals(theirs)) {
			return false;
		}
		String station = linkStation.get(mine);
		if (station == null || !station.equals(linkStation.get(theirs))) {
			return false;
		}
		Id<RailResource> a = resourceOf(mine);
		Id<RailResource> b = resourceOf(theirs);
		if (a == null || b == null || a.equals(b)) {
			return false;
		}
		String wa = linkWay.get(mine);
		String wb = linkWay.get(theirs);
		return wa != null && wb != null && !wa.equals(wb);
	}

	/** Writes every scheduled stop link with its safety verdict (evidence for the report). */
	private void writeStopCsv(Set<Id<Link>> stopLinks) {
		Path file = outputDir().resolve(CSV_STOPS);
		try {
			Files.createDirectories(file.toAbsolutePath().getParent());
			try (BufferedWriter out = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
				out.write("link_id,safe,rule,reason,resource_id,resource_bidirectional,reverse_link,length_m\n");
				List<Id<Link>> ids = new ArrayList<>(stopLinks);
				ids.sort((a, b) -> a.toString().compareTo(b.toString()));
				for (Id<Link> id : ids) {
					Link l = network.getLinks().get(id);
					RailLink rl = railLinks.get(id);
					Id<RailResource> rid = rl == null ? null : rl.getResource().getId();
					Id<Link> rev = reverseLink.get(id);
					out.write(String.format(Locale.ROOT, "%s,%s,%s,\"%s\",%s,%s,%s,%.1f%n", id,
							stopSafe.getOrDefault(id, true), stopRule, stopSafeReason.getOrDefault(id, ""),
							rid == null ? "" : rid, rid != null && bidirectional.contains(rid),
							rev == null ? "" : rev.toString(), l == null ? 0. : l.getLength()));
				}
			}
			log.info("sectionLock: wrote {} stop links to {}", stopLinks.size(), file);
		} catch (IOException e) {
			log.error("sectionLock: could not write {}", file, e);
		}
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

	/**
	 * Logs how long the sections that this strategy will lock actually are. A section that spans a whole single-track line
	 * is exactly the protection we want; a section that is only short means the bidirectional resources are station tracks.
	 */
	private void logSectionStatistics() {

		List<double[]> lengths = new ArrayList<>(); // {metres, links}
		List<String> longest = new ArrayList<>();
		int routesWithSections = 0;
		int sections = 0;

		for (Map.Entry<String, RouteInfo> e : routeInfos.entrySet()) {
			RouteInfo info = e.getValue();
			boolean any = false;
			int i = 0;
			while (i < info.size()) {
				RailLink l = railLinks.get(info.links.get(i));
				if (l == null || !bidirectional.contains(l.getResource().getId())) {
					i++;
					continue;
				}
				int start = i;
				double metres = 0;
				while (i < info.size()) {
					RailLink li = railLinks.get(info.links.get(i));
					if (li == null || !bidirectional.contains(li.getResource().getId())) {
						break;
					}
					metres += li.getLength();
					boolean stop = info.stop[i];
					boolean reversal = stop && i + 1 < info.size()
							&& li.isOppositeLink(info.links.get(i + 1));
					boolean safe = STOP_RULE_ANY.equals(stopRule) || stopSafe.getOrDefault(li.getLinkId(), true)
							|| (meetRule && meetSafe.contains(li.getLinkId()));
					boolean standing = standingAreaRule && (reversal
							|| (standingAreaAny && !stopSafe.getOrDefault(li.getLinkId(), true)));
					i++;
					if (stop && ((!reversal && safe) || standing)) {
						break;
					}
				}
				any = true;
				sections++;
				lengths.add(new double[]{metres, i - start});
				if (metres > 20000) {
					longest.add(String.format(Locale.ROOT, "%s idx=%d %.1fkm %d links start=%s", e.getKey(), start, metres / 1000.,
							i - start, info.links.get(start)));
				}
			}
			if (any) {
				routesWithSections++;
			}
		}

		double[] metres = lengths.stream().mapToDouble(a -> a[0]).sorted().toArray();
		double sum = 0;
		for (double m : metres) {
			sum += m;
		}
		log.info("sectionLock sections: {} sections on {} of {} routes, total {} km, mean {} km, p50 {} km, p95 {} km, max {} km ({} links)",
				sections, routesWithSections, routeInfos.size(), Math.round(sum / 1000.), Math.round(sum / Math.max(1, metres.length) / 100.) / 10.,
				metres.length == 0 ? 0 : Math.round(metres[metres.length / 2] / 100.) / 10.,
				metres.length == 0 ? 0 : Math.round(metres[(int) (metres.length * 0.95)] / 100.) / 10.,
				metres.length == 0 ? 0 : Math.round(metres[metres.length - 1] / 100.) / 10.,
				(int) lengths.stream().mapToDouble(a -> a[1]).max().orElse(0));

		longest.sort(String::compareTo);
		for (int i = 0; i < Math.min(10, longest.size()); i++) {
			log.info("sectionLock longest section: {}", longest.get(i));
		}
	}

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
			log.info("sectionLock: wrote {} bidirectional resources to {}", resources.size(), file);
		} catch (IOException e) {
			log.error("sectionLock: could not write {}", file, e);
		}
	}

	private Path outputDir() {
		return Paths.get(scenario.getConfig().controller().getOutputDirectory());
	}

	// ------------------------------------------------------------- callbacks

	@Override
	public boolean checkLink(double time, RailLink link, TrainPosition position) {

		checkLinkCalls++;

		RailResource resource = link.getResource();
		Id<RailResource> rid = resource.getId();

		// No deadlock avoidance at all for resources that are used in one direction only.
		if (!bidirectional.contains(rid)) {
			return true;
		}

		MobsimDriverAgent driver = position.getDriver();

		// This train already holds a claim on exactly this link: its section is already locked.
		// BN2: a retrace claim (the departure direction of the lead a reversing train stands on) does not count:
		// it was placed while standing and must not bypass the section test of the real departure section.
		List<Claim> mine = claimsByDriver.get(driver);
		if (mine != null) {
			for (Claim c : mine) {
				if (c.linkId().equals(link.getLinkId()) && !c.retrace()) {
					return true;
				}
			}
		}

		int idx = indexOf(position, link.getLinkId());
		if (idx < 0) {
			notOnRoute++;
			return true;
		}

		int end = sectionEnd(position, idx);

		// SWARM-FIX (E2): one check per physical track, in the direction the train meets it first. A
		// reversing train traverses the platform track twice (arrival direction, then back out over the
		// same rails, which is the opposite link of the same resource). Claiming/checking both directions
		// of that one resource refuses an arriving train that wants the *same* platform track: measured at
		// Luzern, "IR26@05:31 wants 71487, refused by the departure-direction claim of IC21 on 71486".
		// The duplicate direction is the track the reversing train physically stands on and holds as a
		// capacity-1 reservation, so no claim is needed for it.
		// SWARM-FIX (I2, merge): switchable via sectionLock.dedup=on|off so the merge can be regression
		// tested against both parents; off = base4 behaviour.
		Set<Id<RailResource>> seen = dedup ? new HashSet<>() : null;
		for (int i = idx; i < end; i++) {
			RailLink wanted = position.getRoute(i);
			if (seen != null && !seen.add(wanted.getResource().getId())) {
				continue;
			}
			// ZH1 retrace rule (2): a train may always traverse a physical track it already occupies. The other
			// train cannot be on it (capacity 1), and the refusal would stop a train from leaving the platform it
			// stands on - the second half of the terminus deadlock (see the field comment of retraceRule).
			if (retraceRule && holdsTrack(position, wanted)) {
				continue;
			}
			Claim conflict = conflictingClaim(wanted, driver);
			if (conflict != null && retracePass(time, position, idx, end, conflict)) {
				continue;
			}
			if (conflict != null && !(meetRule && pairwiseMeet(position, end, conflict))) {
				denied++;
				logDenial(time, driver, link, position, idx, end, conflict);
				return false;
			}
		}
		return true;
	}

	@Override
	public void onReserve(double time, RailResource resource, TrainPosition position) {

		Id<RailResource> rid = resource.getId();
		if (!bidirectional.contains(rid)) {
			return;
		}

		MobsimDriverAgent driver = position.getDriver();
		int idx = indexOfReserved(position, resource);
		if (idx < 0) {
			return;
		}

		List<Claim> mine = claimsByDriver.computeIfAbsent(driver, d -> new ArrayList<>());

		// The section ahead of the train changed (the train advanced), so replace all of its claims.
		for (Claim c : mine) {
			removeClaim(c);
		}
		mine.clear();

		int end = sectionEnd(position, idx);
		updateStanding(position, end);

		RouteInfo info = routeInfo(position);
		int stopIdx = -1;
		boolean standingClaims = false;
		boolean reversal = false;
		if (standingAreaRule && end > 0 && end <= position.getRouteSize() && info != null
				&& info.size() == position.getRouteSize() && info.stop[end - 1] && standingStop(info, end - 1)) {
			stopIdx = end - 1;
			reversal = isReversalStop(info, stopIdx);
			standingClaims = true;
		}
		int from = standingClaims ? Math.min(idx, bodyStart(position, stopIdx)) : idx;
		boolean stopIsStopLink = stopIdx >= 0;
		Id<Link> stopLink = stopIsStopLink ? position.getRoute(stopIdx).getLinkId() : sectionStopLink(position, end);
		String station = stopLink == null ? null : linkStation.get(stopLink);

		Set<Id<RailResource>> claimed = dedup ? new HashSet<>() : null;
		for (int i = from; i < end; i++) {
			RailLink l = position.getRoute(i);
			Id<RailResource> r = l.getResource().getId();
			if (!bidirectional.contains(r)) {
				continue;
			}
			// SWARM-FIX (E2): one claim per physical track per section, in the direction the train meets
			// it first (see checkLink). The second traversal of the same resource is the reversal
			// retrace: the same rails under the train, which its capacity-1 reservation already protects.
			if (claimed != null && !claimed.add(r)) {
				continue;
			}
			boolean duplicate = false;
			for (Claim c : mine) {
				if (c.linkId().equals(l.getLinkId())) {
					duplicate = true;
					break;
				}
			}
			if (duplicate) {
				continue;
			}
			boolean isStanding = standingClaims && i <= stopIdx;
			boolean retrace = info != null && info.size() == position.getRouteSize() && info.retrace[i];
			Claim claim = new Claim(driver, r, l.getLinkId(), stopLink, time, retrace, isStanding);
			mine.add(claim);
			claims.computeIfAbsent(r, k -> new ArrayList<>()).add(claim);
			if (station != null) {
				stopClaims.computeIfAbsent(station, k -> new ArrayList<>()).add(claim);
			}
		}

		// BN2: a reversing train keeps the retraced part of its departure lead claimed in the departure
		// direction while it stands (ZH1's rule). Only the <em>retraced</em> links are claimed; the rest of
		// the departure path stays free until the train really departs - that is the Bern fix. These claims
		// are the opposite direction of links already claimed (or standing-claimed) by the same train, so they
		// are deliberately not de-duplicated by resource.
		if (standingClaims && reversal) {
			int outTo = wayOutEnd(info, stopIdx);
			Set<Id<Link>> retraceLinks = new HashSet<>();
			for (int i = end; i < outTo; i++) {
				if (!info.retrace[i]) {
					continue;
				}
				RailLink l = position.getRoute(i);
				Id<RailResource> r = l.getResource().getId();
				if (!bidirectional.contains(r) || !retraceLinks.add(l.getLinkId())) {
					continue;
				}
				Claim claim = new Claim(driver, r, l.getLinkId(), null, time, true, false);
				mine.add(claim);
				claims.computeIfAbsent(r, k -> new ArrayList<>()).add(claim);
			}
		}

		if (mine.isEmpty()) {
			claimsByDriver.remove(driver);
		}
	}

	@Override
	public void onRelease(double time, RailResource resource, MobsimDriverAgent driver) {

		Id<RailResource> rid = resource.getId();
		List<Claim> mine = claimsByDriver.get(driver);
		if (mine == null) {
			return;
		}
		if (retraceRule) {
			// ZH1 retrace rule: the resource is physically free (the train's body has left it), but a claim on the
			// OPPOSITE direction of the same track can still be needed: a terminus reversal runs in over the platform
			// and the lead and back out over the same rails (Claim.retrace). Only such retrace claims are kept;
			// every other claim of the resource is dropped as before - it belongs to a reservation the train has
			// given back (a look-ahead window it does not need now) and would otherwise block other trains for ever.
			for (int i = mine.size() - 1; i >= 0; i--) {
				Claim c = mine.get(i);
				if (c.resource().equals(rid) && !c.retrace() && !c.standing()) {
					mine.remove(i);
					removeClaim(c);
				}
			}
			if (mine.isEmpty()) {
				claimsByDriver.remove(driver);
			}
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

		// Drop every claim of this driver up to and including the link that just left the train's tail.
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
		// Re-routing is not active in this network (no railsimEntry / railsimExit links), and a detour does not
		// change the direction lock of the sections the train is going to use.
		return true;
	}

	// --------------------------------------------------------------- helpers

	private void removeClaim(Claim claim) {
		String station = claim.stopLink() == null ? null : linkStation.get(claim.stopLink());
		if (station != null) {
			List<Claim> st = stopClaims.get(station);
			if (st != null) {
				st.remove(claim);
				if (st.isEmpty()) {
					stopClaims.remove(station);
				}
			}
		}
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
		// a train standing at a stop may request its current head link (routeIndex - 1)
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

	/**
	 * Exclusive end index of the section starting at {@code startIdx}: maximal run of consecutive bidirectional resources,
	 * ending before the first unidirectional resource, after a scheduled stop, or at the end of the route.
	 * <p>
	 * A stop where the train reverses does not end the section by the <em>old</em> rule, because the train keeps
	 * occupying the platform track until it can leave on the opposite link. With {@code sectionLock.reversalStop=standingArea}
	 * (BN2) it may end there when the standing-area admission test passes; otherwise the section continues over the
	 * departure path (the safe but coarse fallback). With {@code sectionLock.stopRule=safe} only a stop that is a safe
	 * meeting point ends a section; at an unsafe stop (its own track is used in both directions) the section continues to
	 * the next safe stop, so a train never stops inside a single-track stretch where it cannot be passed. With
	 * {@code sectionLock.reversalStop=standingAreaAny} the admission test may also end the section at an unsafe through
	 * stop.
	 */
	private int sectionEnd(TrainPosition position, int startIdx) {

		int size = position.getRouteSize();
		boolean[] stop = null;
		RouteInfo info = routeInfo(position);
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
			if (reversal) {
				// BN2: a section may end at a reversing stop when the standing-area admission test passes. Otherwise
				// the section continues over the departure path (the safe but coarse old behaviour that made a
				// dwelling reversing train hold the whole Bern station box).
				if (standingStop(info, i) && admitSection(position, startIdx, i)) {
					standingSections++;
					return i + 1;
				}
				continue;
			}
			if (!STOP_RULE_ANY.equals(stopRule) && !stopSafe.getOrDefault(l.getLinkId(), true)) {
				// V1 pairwise meet rule: end the section here only if the trains that are around use a
				// different track of this station (see meetPossibleNow); otherwise the section continues.
				if (!(meetRule && meetSafe.contains(l.getLinkId()) && meetPossibleNow(position, l))) {
					if (standingStop(info, i) && admitSection(position, startIdx, i)) {
						standingSections++;
						return i + 1;
					}
					continue;
				}
			}
			return i + 1;
		}
		return size;
	}

	/**
	 * BN2: true if the standing-area rule applies at the stop at route index {@code i}: every reversing stop (the
	 * default), and with {@code standingAreaAny} also an unsafe through stop (its own track is used in both directions).
	 * Safe meeting points are left to the existing rules - they end a section anyway.
	 */
	private boolean standingStop(RouteInfo info, int i) {
		if (!standingAreaRule || info == null || i < 0 || i >= info.size()) {
			return false;
		}
		if (isReversalStop(info, i)) {
			return true;
		}
		return standingAreaAny && !stopSafe.getOrDefault(info.links.get(i), true);
	}

	/** BN2: true if the route continues on the opposite link at index {@code i} - the train reverses there. */
	private boolean isReversalStop(RouteInfo info, int i) {
		if (info == null || i < 0 || i + 1 >= info.size()) {
			return false;
		}
		RailLink l = railLinks.get(info.links.get(i));
		return l != null && l.isOppositeLink(info.links.get(i + 1));
	}

	/** BN2: first route index of the body of a train whose head is at the end of the stop link at {@code stopIdx}. */
	private int bodyStart(TrainPosition position, int stopIdx) {
		double remaining = position.getTrain().length();
		int i = stopIdx;
		while (i > 0 && remaining > position.getRoute(i).getLength()) {
			remaining -= position.getRoute(i).getLength();
			i--;
		}
		return i;
	}

	/** BN2: exclusive end of the departure path (maximal run of bidirectional resources) after the stop at {@code stopIdx}. */
	private int wayOutEnd(RouteInfo info, int stopIdx) {
		if (info == null || info.nextNonBi == null) {
			return stopIdx + 1;
		}
		int from = Math.max(0, Math.min(stopIdx + 1, info.size()));
		return info.nextNonBi[from];
	}

	/** BN2: does {@code resource} occur on the route in the index range {@code [from, to)}? */
	private boolean resourceInRange(RouteInfo info, Id<RailResource> resource, int from, int to) {
		if (info == null || info.resourceIndices == null || from >= to) {
			return false;
		}
		int[] indices = info.resourceIndices.get(resource);
		if (indices == null) {
			return false;
		}
		for (int i : indices) {
			if (i >= from && i < to) {
				return true;
			}
		}
		return false;
	}

	/** BN2: the published departure second of the train at the stop at route index {@code stopIdx}. */
	private double publishedDeparture(TrainPosition position, RouteInfo info, int stopIdx) {
		if (position.getPt() == null || info == null || info.stopNumber == null || stopIdx < 0 || stopIdx >= info.size()) {
			return Double.POSITIVE_INFINITY;
		}
		List<TransitRouteStop> stops = position.getPt().getTransitRoute().getStops();
		int j = info.stopNumber[stopIdx];
		double offset = Double.NaN;
		if (j >= 0 && j < stops.size()) {
			TransitRouteStop stop = stops.get(j);
			OptionalTime off = stop.getDepartureOffset();
			if (!off.isDefined()) {
				off = stop.getArrivalOffset();
			}
			if (off.isDefined()) {
				offset = off.seconds();
			}
		}
		double base = position.getPt().getDeparture() == null ? Double.NaN
				: position.getPt().getDeparture().getDepartureTime();
		if (Double.isNaN(offset) || Double.isNaN(base)) {
			return Double.POSITIVE_INFINITY;
		}
		return base + offset;
	}

	/**
	 * BN2 standing-area admission test for a section {@code [start, stopIdx]} that would end at a stop.
	 * <p>
	 * <b>(1)</b> The standing area (body links, head at the end of the stop link) must not be reserved by another train.
	 * <b>(2)</b> It must not be the claimed way or the standing area of another train - a train that is already admitted
	 * into this station would otherwise drive into the standing body or the body would block its way in. <b>(3)</b> The
	 * way in must not cross another train's standing area (its body is physically in the way). <b>(4)</b> Neither may
	 * this train's standing area lie on another admitted train's departure path, nor another train's standing area on
	 * this train's departure path. (4) is what makes the rule deadlock free: if no admitted train stands on another's
	 * way out, a standing train can always leave. The departure path is the maximal run of bidirectional resources after
	 * the stop (the track the train must traverse to leave); it is a static route property.
	 */
	private boolean admitSection(TrainPosition position, int start, int stopIdx) {

		RouteInfo info = routeInfo(position);
		if (info == null || info.size() != position.getRouteSize() || info.nextNonBi == null) {
			return false;
		}
		MobsimDriverAgent driver = position.getDriver();
		int bodyStart = bodyStart(position, stopIdx);
		Set<Id<RailResource>> body = new HashSet<>();
		for (int i = bodyStart; i <= stopIdx; i++) {
			RailLink l = position.getRoute(i);
			// (1) not occupied by another train
			if (rrm != null && !rrm.hasCapacity(0, l.getLinkId(), RailResourceManager.ANY_TRACK, position)) {
				standingDenials++;
				return false;
			}
			Id<RailResource> r = l.getResource().getId();
			body.add(r);
			// (2) not the claimed way or standing area of another train
			List<Claim> list = claims.get(r);
			if (list != null) {
				for (Claim c : list) {
					if (c.driver() != driver) {
						standingDenials++;
						return false;
					}
				}
			}
		}
		// (3) the way in does not cross another train's standing area
		for (int i = start; i <= stopIdx; i++) {
			Id<RailResource> r = position.getRoute(i).getResource().getId();
			List<Claim> list = claims.get(r);
			if (list != null) {
				for (Claim c : list) {
					if (c.driver() != driver && c.standing()) {
						standingDenials++;
						return false;
					}
				}
			}
		}
		// (4) departure-path interference with the other admitted trains, ordered by the published departure time:
		// the train that leaves first is the one that can go; a train whose standing body would still be on a way
		// out when its user leaves is refused. This keeps the "body on another train's way out" relation acyclic
		// (it follows the departure times), which is what makes the rule deadlock free.
		int outFrom = stopIdx + 1;
		int outTo = wayOutEnd(info, stopIdx);
		double myDeparture = publishedDeparture(position, info, stopIdx);
		for (Map.Entry<MobsimDriverAgent, Standing> e : driverStanding.entrySet()) {
			if (e.getKey() == driver) {
				continue;
			}
			Standing other = e.getValue();
			if (other.info() == null || other.resources() == null) {
				continue;
			}
			// (4a) our standing area is on the other train's departure path and the other train leaves first
			if (myDeparture >= other.departure()) {
				for (Id<RailResource> r : body) {
					if (resourceInRange(other.info(), r, other.stopIdx() + 1, other.outEnd())) {
						standingDenials++;
						return false;
					}
				}
			}
			// (4b) the other train's standing area is on our departure path and it is still standing when we leave
			if (other.departure() >= myDeparture && outFrom < outTo) {
				for (Id<RailResource> r : other.resources()) {
					if (resourceInRange(info, r, outFrom, outTo)) {
						standingDenials++;
						return false;
					}
				}
			}
		}
		return true;
	}

	/** BN2: remember (or clear) the stop an admitted train is currently heading for (standing area + departure path). */
	private void updateStanding(TrainPosition position, int end) {

		MobsimDriverAgent driver = position.getDriver();
		RouteInfo info = routeInfo(position);
		if (info == null || info.size() != position.getRouteSize() || info.nextNonBi == null
				|| end <= 0 || end > position.getRouteSize() || !info.stop[end - 1]
				|| !standingStop(info, end - 1)) {
			driverStanding.remove(driver);
			return;
		}
		int stopIdx = end - 1;
		int bodyStart = bodyStart(position, stopIdx);
		Set<Id<RailResource>> resources = new HashSet<>();
		for (int i = bodyStart; i <= stopIdx; i++) {
			resources.add(position.getRoute(i).getResource().getId());
		}
		driverStanding.put(driver, new Standing(info, stopIdx, bodyStart, resources, wayOutEnd(info, stopIdx),
				publishedDeparture(position, info, stopIdx)));
	}

	/**
	 * BN3: may this train traverse a resource on which another train holds a <b>retrace claim</b>?
	 * <p>
	 * A retrace claim is the departure-direction claim on the rails a reversing train has already used on its way in
	 * ({@link Claim#retrace()}), placed by {@link #onReserve} for the links between the end of its stop and the end of
	 * its departure path. Those links are <b>physically free</b>: the train's body stands in front of its stop link, and
	 * the claim only reserves the future departure. Keeping it for the whole dwell is ZH1's protection against a
	 * follower entering the single-track lead the reversing train must come back over. It is not needed when the other
	 * train only <em>crosses</em> the lead and is gone before the reversing train departs.
	 * <p>
	 * The claim therefore does not conflict with a train whose section (a) ends at a scheduled stop, (b) has no stop
	 * between its start and the lead, (c) is long enough beyond the lead that its own standing body is completely clear
	 * of the lead, and (d) is scheduled to have its head at that stop {@code retracePassMargin} seconds before the
	 * reversing train's published departure - the head being at the stop means the whole body is past the lead. (e)
	 * Neither train's standing body may lie on the other's departure path, so no new "standing body blocks a way out"
	 * pair can be created. If any test fails the claim stays in force (BN2 behaviour).
	 * <p>
	 * Deadlock freedom. A train that is allowed through keeps its ordinary section claim up to its stop, and its
	 * standing body is clear of the lead and of the reversing train's departure path. So the reversing train can always
	 * leave over the lead as soon as the passing train has cleared it, and the passing train can always reach its stop:
	 * the reversing train stands on its own platform body, which is neither on the passing train's way in nor on its
	 * departure path. No claim is removed and no capacity is added; only the time extent of the retrace claim is
	 * bounded.
	 */
	private boolean retracePass(double time, TrainPosition position, int start, int end, Claim conflict) {

		if (!retracePassRule || conflict == null || !conflict.retrace() || conflict.standing()) {
			return false;
		}
		RouteInfo info = routeInfo(position);
		if (info == null || info.size() != position.getRouteSize() || info.nextNonBi == null) {
			return false;
		}
		Standing other = driverStanding.get(conflict.driver());
		if (other == null || other.info() == null || other.resources() == null
				|| !Double.isFinite(other.departure())) {
			// the holder is not standing at a known stop (already moving) - keep ZH1's protection
			refuseRetrace(0);
			return false;
		}

		// (a) the lead must be on our section; first index of that resource from the section start on
		Id<RailResource> lead = conflict.resource();
		int leadIdx = -1;
		for (int i = start; i < end; i++) {
			if (position.getRoute(i).getResource().getId().equals(lead)) {
				leadIdx = i;
				break;
			}
		}
		if (leadIdx < 0) {
			refuseRetrace(1);
			return false;
		}
		// our section has to end at a scheduled stop: that is where we stand and where our body clears the lead.
		// P4: the run of bidirectional resources can be cut before our next stop (a link that the schedule uses in
		// one direction only). With sectionLock.retracePassAhead=on the stop we really stand at is our next
		// scheduled stop, and the way to it has to be free (tests (f)/(g) below).
		int stand = end - 1;
		boolean ahead = false;
		if (!info.stop[stand]) {
			if (!retracePassAheadRule) {
				refuseRetrace(2);
				return false;
			}
			stand = nextStopIndex(info, start);
			if (stand < end) {
				refuseRetrace(2);
				return false;
			}
			ahead = true;
		}
		// no scheduled stop at or before the lead: the train must not stand on the lead, and the passage over it has to
		// be one uninterrupted movement
		for (int i = start; i <= leadIdx; i++) {
			if (info.stop[i]) {
				refuseRetrace(3);
				return false;
			}
		}
		// our standing body must be completely clear of the lead, at every stop inside the section and at the stop we
		// really stand at
		int bodyFrom = bodyStart(position, stand);
		if (bodyFrom <= leadIdx) {
			refuseRetrace(4);
			return false;
		}
		for (int i = leadIdx + 1; i <= stand; i++) {
			if (info.stop[i] && bodyStart(position, i) <= leadIdx) {
				refuseRetrace(4);
				return false;
			}
		}
		// (f) P4: everything between here and the stop we stand at must be free of other trains, otherwise we could be
		// stopped with our body on the lead.
		if (ahead) {
			double way = wayLength(position, start, stand);
			if (way > retracePassAheadMaxM || !wayAheadFree(position, start, stand)) {
				refuseRetrace(8);
				return false;
			}
		}
		// (d) time bound: our head is at the end of our stop link (= body clear of the lead) at the published arrival,
		// but never earlier than what we can physically drive from here
		double arrival = publishedArrival(position, info, stand);
		double run = ahead ? minPassageToStop(position, start, stand) : minPassageSeconds(position, start, leadIdx);
		double clear = Math.max(arrival, time + run);
		if (!(clear + retracePassMargin <= other.departure())) {
			refuseRetrace(5);
			return false;
		}
		// (e) no standing body on the other's departure path / no other standing body on ours, and the other's
		// standing body must not lie on our way in at all (a train cannot drive through a standing train)
		if (retracePassBody) {
			for (int i = start; i <= stand; i++) {
				Id<RailResource> r = position.getRoute(i).getResource().getId();
				if (other.resources().contains(r)) {
					refuseRetrace(6);
					return false;
				}
			}
		}
		for (int i = bodyFrom; i <= stand; i++) {
			Id<RailResource> r = position.getRoute(i).getResource().getId();
			if (resourceInRange(other.info(), r, other.stopIdx() + 1, other.outEnd())) {
				refuseRetrace(7);
				return false;
			}
		}
		int outTo = wayOutEnd(info, stand);
		if (stand + 1 < outTo) {
			for (Id<RailResource> r : other.resources()) {
				if (resourceInRange(info, r, stand + 1, outTo)) {
					refuseRetrace(7);
					return false;
				}
			}
		}
		if (ahead) {
			retracePassAheadAllowed++;
		}
		retracePassAllowed++;
		return true;
	}

	/** P4: route index of the first scheduled stop at or after {@code from}, or -1. */
	private int nextStopIndex(RouteInfo info, int from) {

		if (info == null || info.stop == null) {
			return -1;
		}
		for (int i = Math.max(0, from); i < info.size(); i++) {
			if (info.stop[i]) {
				return i;
			}
		}
		return -1;
	}

	/** P4: length in metres of the route from index {@code from} to {@code to} (inclusive). */
	private double wayLength(TrainPosition position, int from, int to) {

		double metres = 0;
		for (int i = Math.max(0, from); i <= to && i < position.getRouteSize(); i++) {
			metres += position.getRoute(i).getLength();
		}
		return metres;
	}

	/**
	 * P4: no other train holds a reservation or a claim (other than a retrace claim, which reserves only a future
	 * departure over rails that are free now) on our route in {@code [from, to]}.
	 */
	private boolean wayAheadFree(TrainPosition position, int from, int to) {

		if (rrm == null) {
			return false;
		}
		MobsimDriverAgent driver = position.getDriver();
		for (int i = Math.max(0, from); i <= to && i < position.getRouteSize(); i++) {
			RailLink l = position.getRoute(i);
			Claim conflict = conflictingClaim(l, driver);
			if (conflict != null && !conflict.retrace()) {
				return false;
			}
			if (rrm.isBlockedBy(l, position)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * P4: lower bound on the seconds needed to drive from the section start {@code start} to the end of the stop link
	 * at {@code stand} and to pull the whole train body past it (slowest link speed on that stretch).
	 */
	private double minPassageToStop(TrainPosition position, int start, int stand) {

		double metres = position.getTrain().length();
		double speed = Double.MAX_VALUE;
		for (int i = Math.max(0, start); i <= stand && i < position.getRouteSize(); i++) {
			RailLink l = position.getRoute(i);
			metres += l.getLength();
			speed = Math.min(speed, Math.max(1., l.getAllowedFreespeed(position.getDriver())));
		}
		return metres / Math.max(1., speed);
	}

	/** BN3: the published <b>arrival</b> second of the train at the stop at route index {@code stopIdx}. */
	private double publishedArrival(TrainPosition position, RouteInfo info, int stopIdx) {

		if (position.getPt() == null || info == null || info.stopNumber == null || stopIdx < 0 || stopIdx >= info.size()) {
			return Double.POSITIVE_INFINITY;
		}
		List<TransitRouteStop> stops = position.getPt().getTransitRoute().getStops();
		int j = info.stopNumber[stopIdx];
		double offset = Double.NaN;
		if (j >= 0 && j < stops.size()) {
			OptionalTime off = stops.get(j).getArrivalOffset();
			if (!off.isDefined()) {
				off = stops.get(j).getDepartureOffset();
			}
			if (off.isDefined()) {
				offset = off.seconds();
			}
		}
		double base = position.getPt().getDeparture() == null ? Double.NaN
				: position.getPt().getDeparture().getDepartureTime();
		if (Double.isNaN(offset) || Double.isNaN(base)) {
			return Double.POSITIVE_INFINITY;
		}
		return base + offset;
	}

	/**
	 * BN3: lower bound on the seconds needed to drive from the section start {@code start} to the end of the lead at
	 * {@code leadIdx} and to pull the whole train body past it. Uses the slowest link speed on that stretch, so it is a
	 * pessimistic estimate of the clearance time.
	 */
	private double minPassageSeconds(TrainPosition position, int start, int leadIdx) {

		double metres = position.getTrain().length();
		double speed = Double.MAX_VALUE;
		for (int i = start; i <= leadIdx; i++) {
			RailLink l = position.getRoute(i);
			metres += l.getLength();
			speed = Math.min(speed, Math.max(1., l.getAllowedFreespeed(position.getDriver())));
		}
		return metres / Math.max(1., speed);
	}

	/**
	 * own claim was made with a stop link as its section end. The two trains may be in the same stretch only if they
	 * meet at one station on two different physical tracks - which is exactly what {@link #isMeetingPair} checks.
	 * A conflict with a train that stops on <em>our</em> track (or that has no stop at the end of its claim) is
	 * always refused, so two trains on one rail still exclude each other and no head-on deadlock can be created.
	 * <p>
	 * BN2: a <b>standing</b> claim (the body of a train that is admitted / standing at a stop) and a <b>retrace</b>
	 * claim (the departure lead of a reversing train) are never excused by a meet: they are the same rails, not two
	 * platform tracks. A section that ends at a reversing stop of our own is not a meet either.
	 */
	private boolean pairwiseMeet(TrainPosition position, int end, Claim conflict) {
		if (conflict.retrace() || conflict.standing()) {
			return false;
		}
		Id<Link> mine = sectionStopLink(position, end);
		if (mine == null || conflict.stopLink() == null) {
			return false;
		}
		RouteInfo info = routeInfo(position);
		if (info != null && info.size() == position.getRouteSize() && isReversalStop(info, end - 1)) {
			return false;
		}
		return isMeetingPair(mine, conflict.stopLink());
	}

	/**
	 * V1 pairwise test used when a section would end at a station with a second platform track: may this
	 * train meet the trains that are currently heading to the same station? It may, if every other train
	 * whose claim ends at a stop of this station is heading to a <b>partner</b> track
	 * ({@link #isMeetingPair}: same station, different physical track, different OSM way). If another train
	 * is heading to <em>our</em> track, the section must not end here - otherwise both would drive into the
	 * station on one rail and block each other for ever (measured: tiny case {@code m5-meet-same-track}).
	 */
	private boolean meetPossibleNow(TrainPosition position, RailLink stop) {
		String station = linkStation.get(stop.getLinkId());
		if (station == null) {
			return false;
		}
		List<Claim> others = stopClaims.get(station);
		if (others == null) {
			return true;
		}
		for (Claim c : others) {
			if (c.driver() == position.getDriver()) {
				continue;
			}
			if (!isMeetingPair(stop.getLinkId(), c.stopLink())) {
				return false;
			}
		}
		return true;
	}

	/** The stop link at the end of the section {@code [startIdx, end)}, or null if the section does not end at a stop. */
	private Id<Link> sectionStopLink(TrainPosition position, int end) {
		if (end <= 0 || end > position.getRouteSize()) {
			return null;
		}
		Id<Link> last = position.getRoute(end - 1).getLinkId();
		RouteInfo info = routeInfo(position);
		boolean stop;
		if (info != null && info.size() == position.getRouteSize()) {
			stop = info.stop[end - 1];
		} else {
			stop = position.isStop(last);
		}
		return stop ? last : null;
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

	/**
	 * ZH1 retrace rule: true if the train currently holds a capacity-1 reservation on the physical track of
	 * {@code link} - in either direction (a terminus reversal stands on the arrival link and asks for the
	 * departure link of the same resource).
	 */
	private boolean holdsTrack(TrainPosition position, RailLink link) {

		if (rrm == null) {
			return false;
		}
		for (RailLink l : link.getResource().getLinks()) {
			if (rrm.isBlockedBy(l, position)) {
				return true;
			}
		}
		return false;
	}

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
			// BN2: a standing body blocks its track in both directions.
			if (other.standing()) {
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

	// ----------------------------------------------------------- diagnostics

	private void logDenial(double time, MobsimDriverAgent driver, RailLink link, TrainPosition position, int start, int end,
						   Claim conflict) {

		if (denialRows < MAX_DENIAL_ROWS) {
			try {
				if (denialOut == null) {
					Path file = outputDir().resolve(CSV_DENIALS);
					Files.createDirectories(file.toAbsolutePath().getParent());
					denialOut = Files.newBufferedWriter(file, StandardCharsets.UTF_8);
					denialOut.write("time,driver,requested_link,section_start_link,section_end_link,section_km,section_links,"
							+ "conflict_driver,conflict_link,conflict_age_s,conflict_resource\n");
				}
				double km = 0;
				for (int i = start; i < end; i++) {
					km += position.getRoute(i).getLength();
				}
				denialOut.write(String.format(Locale.ROOT, "%.0f,%s,%s,%s,%s,%.3f,%d,%s,%s,%.0f,%s%n",
						time, driver.getId(), link.getLinkId(),
						position.getRoute(start).getLinkId(), position.getRoute(end - 1).getLinkId(),
						km / 1000., end - start,
						conflict.driver().getId(), conflict.linkId(), time - conflict.time(),
						conflict.resource()));
				denialRows++;
				// flush every row: the deadlock avoidance has no end-of-run callback, and an unflushed tail
				// would silently truncate the evidence file
				denialOut.flush();
			} catch (IOException e) {
				log.error("sectionLock: cannot write denial log", e);
				denialRows = MAX_DENIAL_ROWS;
			}
		}

		if (time - lastLogTime >= 3600) {
			lastLogTime = time;
			StringBuilder why = new StringBuilder();
			for (int i = 0; i < retraceRefusalReason.length; i++) {
				why.append(i == 0 ? "" : " ").append(RETRACE_REFUSAL_NAMES[i]).append('=').append(retraceRefusalReason[i]);
			}
			log.info("sectionLock t={}: {} checkLink calls, {} denied, {} not on route, {} drivers with claims, {} claims on {} resources, "
							+ "{} routes without stop info, {} denial rows logged, {} standing-area sections, {} standing-area refusals, "
							+ "{} retrace crossings allowed ({} beyond their section end), {} retrace crossings refused ({})",
					Math.round(time), checkLinkCalls, denied, notOnRoute, claimsByDriver.size(),
					claims.values().stream().mapToInt(List::size).sum(), claims.size(), unknownRouteCalls, denialRows,
					standingSections, standingDenials, retracePassAllowed, retracePassAheadAllowed, retracePassRefused, why);
		}
	}

	/** Per transit route: link sequence and "this link is a scheduled stop" flags. */
	private static final class RouteInfo {
		final List<Id<Link>> links;
		final boolean[] stop;

		/**
		 * ZH1 retrace rule: {@code retrace[i]} is true if the opposite link of {@code links[i]} appears
		 * <em>before</em> index {@code i} on the route. Such a link is traversed twice by the train - the arrival
		 * direction first, the claimed (departure) direction second - which is exactly the terminus reversal over a
		 * platform track and its lead. Only these claims must survive the physical release of their resource.
		 */
		final boolean[] retrace;

		/** BN2: first index at or after this one whose resource is not bidirectional ({@code size} if none). */
		int[] nextNonBi;

		/** BN2: route indices per resource, used for the departure-path tests of the standing-area rule. */
		Map<Id<RailResource>, int[]> resourceIndices;

		/** BN2: for a stop link index, the index of the stop in {@code TransitRoute.getStops()} (-1 if none). */
		int[] stopNumber;

		RouteInfo(List<Id<Link>> sequence) {
			this.links = sequence;
			this.stop = new boolean[sequence.size()];
			this.retrace = new boolean[sequence.size()];
			this.stopNumber = new int[sequence.size()];
			for (int i = 0; i < stopNumber.length; i++) {
				stopNumber[i] = -1;
			}
		}

		int size() {
			return links.size();
		}
	}

	/** BN2: the stop at which an admitted train will stand, with its body resources and departure-path range. */
	private record Standing(RouteInfo info, int stopIdx, int bodyStart, Set<Id<RailResource>> resources, int outEnd,
							double departure) {
	}

	/** One direction claim of a train on one resource. */
	private record Claim(MobsimDriverAgent driver, Id<RailResource> resource, Id<Link> linkId, Id<Link> stopLink,
						 double time, boolean retrace, boolean standing) {
	}

	/** Convenience for tests / debugging. */
	public Collection<Id<RailResource>> getBidirectionalResources() {
		return bidirectional;
	}
}
