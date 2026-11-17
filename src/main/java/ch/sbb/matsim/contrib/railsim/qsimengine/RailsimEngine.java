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

package ch.sbb.matsim.contrib.railsim.qsimengine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.events.Event;
import org.matsim.api.core.v01.events.LinkEnterEvent;
import org.matsim.api.core.v01.events.LinkLeaveEvent;
import org.matsim.api.core.v01.events.PersonEntersVehicleEvent;
import org.matsim.api.core.v01.events.PersonLeavesVehicleEvent;
import org.matsim.api.core.v01.events.PersonStuckEvent;
import org.matsim.api.core.v01.events.VehicleAbortsEvent;
import org.matsim.api.core.v01.events.VehicleEntersTrafficEvent;
import org.matsim.api.core.v01.events.VehicleLeavesTrafficEvent;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.population.Leg;
import org.matsim.api.core.v01.population.PlanElement;
import org.matsim.api.core.v01.population.Route;
import org.matsim.core.api.experimental.events.EventsManager;
import org.matsim.core.mobsim.framework.MobsimDriverAgent;
import org.matsim.core.mobsim.framework.Steppable;
import org.matsim.core.network.NetworkChangeEvent;
import org.matsim.core.population.routes.NetworkRoute;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;
import org.matsim.vehicles.VehicleType;

import ch.sbb.matsim.contrib.railsim.RailsimUtils;
import ch.sbb.matsim.contrib.railsim.config.RailsimConfigGroup;
import ch.sbb.matsim.contrib.railsim.events.RailsimDetourEvent;
import ch.sbb.matsim.contrib.railsim.events.RailsimTrainLeavesLinkEvent;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.DispositionResponse;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.TrainDisposition;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.ResourceType;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailResourceManager;

/**
 * Engine to simulate train movement.
 */
final class RailsimEngine implements Steppable {

	/**
	 * Additional safety distance in meter that is added to the reservation distance.
	 * Ensure that trains always have enough distance to progress.
	 */
	private static final double SAFETY_DIST = 10;
	private static final Logger log = LogManager.getLogger(RailsimEngine.class);

	// SWARM-FIX: prove at start-up that this class replaced the one from the railsim jar.
	static {
		log.info("SWARM-FIX RailsimEngine active (reversal releases stale arrival-direction reservations)");
	}

	// SWARM-FIX (JT1): the punctual-profile hooks (see PunctualProfileHook) are only active for speedProfile=punctual.
	static {
		log.info("SWARM-FIX JT1 punctual-profile engine hooks: {}", PunctualProfileHook.isActive() ? "ON" : "off (base7 behaviour)");
	}

	private static boolean punctualProfile() {
		return PunctualProfileHook.isActive();
	}

	private final EventsManager eventsManager;
	private final RailsimConfigGroup config;
	private final Queue<UpdateEvent> updateQueue = new PriorityQueue<>();
	private final RailResourceManager resources;
	private final TrainManager trainManager;
	private final TrainDisposition disposition;
	private final TrainTimeDistanceHandler ttdHandler;
	/** JT2: incremental distance to the next stop per train. */
	private final Map<TrainState, StopDistance> stopDistance = new java.util.IdentityHashMap<>();

	RailsimEngine(EventsManager eventsManager, RailsimConfigGroup config, RailResourceManager resources,
				  TrainManager trainManager, TrainDisposition disposition, TrainTimeDistanceHandler ttdHandler) {
		this.eventsManager = eventsManager;
		this.config = config;
		this.resources = resources;
		this.trainManager = trainManager;
		this.disposition = disposition;
		this.ttdHandler = ttdHandler;
	}

	@Override
	public void doSimStep(double time) {

		UpdateEvent update = updateQueue.peek();

		// Update loop over all required state updates
		while (update != null && update.plannedTime <= time) {
			updateQueue.poll();

			// Use planned time here, otherwise there will be inaccuracies
			updateState(update.plannedTime, update);

			// Add the update event again
			if (update.type != UpdateEvent.Type.IDLE) {
				updateQueue.add(update);
			}

			update = updateQueue.peek();
		}

		if (time % config.getUpdateInterval() == 0.) {
			updateAllPositions(time);
		}
	}

	/**
	 * Update the current state of all trains, even if no update would be needed.
	 */
	public void updateAllStates(double time) {

		// Process all waiting events first
		doSimStep(time);

		// Potential duplication of position update events here, if time matches updateInterval
		updateAllPositions(time);
	}

	private List<RailLink> createRoute(Id<Link> currentLinkId, NetworkRoute route) {
		List<RailLink> list = route.getLinkIds().stream()
			.map(resources::getLink)
			.collect(Collectors.toList());

		list.addFirst(resources.getLink(currentLinkId));
		list.add(resources.getLink(route.getEndLinkId()));
		return list;
	}

	/**
	 * Handle the departure of a train.
	 */
	public boolean handleDeparture(double now, MobsimDriverAgent agent, Id<Link> linkId, NetworkRoute route) {

		log.debug("Train {} is departing at {}", agent.getVehicle(), now);

		// Queue the update event
		// NO events can be generated here, or temporal ordering is not guaranteed
		// (departures are handled before event queue is processed)

		List<RailLink> list = createRoute(linkId, route);
		VehicleType type = agent.getVehicle().getVehicle().getType();
		TrainState state = new TrainState(agent, new TrainInfo(type, config), now, linkId, list);

		state.train.checkConsistency();

		trainManager.addActiveTrain(state);

		disposition.onDeparture(now, state.driver, state.route);

		updateQueue.add(new UpdateEvent(state, UpdateEvent.Type.DEPARTURE));

		return true;
	}

	/**
	 * Handle {@link NetworkChangeEvent}. Note that certain changes don't come into effect immediately if the train is already on the link.
	 */
	public void handleNetworkChangeEvent(double time, NetworkChangeEvent event) {

		for (Link link : event.getLinks()) {

			RailLink railLink = resources.getLink(link.getId());
			if (railLink == null)
				continue;

			// Set new free speed
			if (event.getFreespeedChange() != null) {
				railLink.setFreeSpeed(RailsimCalc.calculateChange(event.getFreespeedChange(), railLink.getFreeSpeed()));
			}

			for (NetworkChangeEvent.AttributesChangeValue attributesChange : event.getAttributesChanges()) {
				if (RailsimUtils.LINK_ATTRIBUTE_CAPACITY.equals(attributesChange.getAttribute())) {

					double newCapacity = RailsimCalc.calculateChange(attributesChange, railLink.getResource().getTotalCapacity());
					resources.setCapacity(link.getId(), (int) newCapacity);

				}
			}
		}
	}

	/**
	 * Prepare the departure of a train that has been waiting.
	 */
	private void prepareDeparture(double time, UpdateEvent event) {

		TrainState state = event.state;
		RailsimTransitDriverAgent driver = state.pt;
		assert driver != null : "Train must have a pt driver";

		// Prepares the next route and initializes events
		driver.endActivityAndComputeNextState(time);

		PlanElement el = driver.getCurrentPlanElement();

		if (!(el instanceof Leg leg)) {
			throw new IllegalStateException("Plan element of agent " + driver + " is not a leg.");
		}
		Route route = leg.getRoute();
		if (!(route instanceof NetworkRoute networkRoute)) {
			throw new IllegalStateException("A network route is required for agent " + driver + ".");
		}

		// Free reservations of the old route
		for (RailLink link : state.route) {
			if (resources.isBlockedBy(link, state) && link.getLinkId() != state.headLink) {
				disposition.unblockRailLink(time, state.driver, link);
			}
		}

		// Check the consistency of the route
		if (config.getVehicleCirculation() == RailsimConfigGroup.VehicleCirculation.yes) {
			if (!Objects.equals(state.headLink, networkRoute.getStartLinkId())) {
				throw new IllegalStateException("Train " + driver.getId() + " is not at the start link of its route. Expected: " + networkRoute.getStartLinkId() + ", but was: " + state.getHeadLink());
			}
		}

		state.reset(time, createRoute(networkRoute.getStartLinkId(), networkRoute));

		disposition.onDeparture(time, state.driver, state.route);

		event.plannedTime = time;
		event.type = UpdateEvent.Type.DEPARTURE;
	}

	private void updateAllPositions(double time) {
		for (TrainState train : trainManager.getActiveTrains()) {
			if (train.timestamp < time)
				updateState(time, new UpdateEvent(train, UpdateEvent.Type.POSITION));
		}
	}

	private void createEvent(Event event) {
		// Because of the 1s update interval, events need to be rounded to the current simulation step
		event.setTime(Math.ceil(event.getTime()));
		this.eventsManager.processEvent(event);
	}

	private void updateState(double time, UpdateEvent event) {

		// Do different updates depending on the type
		switch (event.type) {
			case DEPARTURE -> updateDeparture(time, event);
			// A position sample is discarded. The queued movement or reservation event owns the next update.
			case POSITION -> updatePosition(time, event);
			case SPEED_CHANGE -> updateSpeed(time, event);
			case ENTER_LINK, REVERSE_TRAIN -> enterLink(time, event);
			case LEAVE_LINK -> leaveLink(time, event);
			case BLOCK_TRACK -> blockTrack(time, event);
			case WAIT_FOR_RESERVATION -> checkTrackReservation(time, event);
			case UNBLOCK_LINK -> {
				unblockTrack(time, event.state, event.unblockLink);
				// event will be removed
				event.type = UpdateEvent.Type.IDLE;
			}
			case WAIT_DEPARTURE -> prepareDeparture(time, event);
			default -> throw new IllegalStateException("Unhandled update type " + event.type);
		}
	}

	private void updateSpeed(double time, UpdateEvent event) {

		TrainState state = event.state;

		updatePosition(time, event);

		decideTargetSpeed(event, state);

		createEvent(state.asEvent(time));

		decideNextUpdate(event);
	}

	private void blockTrack(double time, UpdateEvent event) {

		TrainState state = event.state;

		updatePosition(time, event);

		if (!blockLinkTracks(time, state)) {
			// train needs to stop
			decideTargetSpeed(event, state);

			event.checkReservation = time + config.getPollInterval();
			decideNextUpdate(event);

		} else {
			event.checkReservation = -1;
			decideNextUpdate(event);
		}
	}

	private void checkTrackReservation(double time, UpdateEvent event) {

		TrainState state = event.state;

		// train might be at the end of route already
		RailLink nextLink = state.isRouteAtEnd() ? null : state.route.get(state.routeIdx);

		boolean allBlocked = blockLinkTracks(time, state);

		// Driver can advance if there is approved distance
		if (allBlocked || FuzzyUtils.greaterThan(state.approvedDist, 0)) {

			if (allBlocked)
				event.checkReservation = -1;
			else {
				event.checkReservation = time + config.getPollInterval();
			}

			// Train already waits at the end of previous link, next link is already blocked
			if (event.waitingForLink &&
				nextLink != null &&
				resources.isBlockedBy(nextLink, state) &&
				FuzzyUtils.equals(resources.getLink(state.headLink).length, state.headPosition)) {

				enterLink(time, event);
				event.waitingForLink = false;

			} else {

				updatePosition(time, event);
				decideTargetSpeed(event, state);
				decideNextUpdate(event);

			}

		} else {

			event.checkReservation = time + config.getPollInterval();

			// If train is already standing still and waiting, there is no update needed.
			if (event.waitingForLink && FuzzyUtils.equals(state.speed, 0)) {
				event.plannedTime = time + config.getPollInterval();
			} else {
				decideNextUpdate(event);
			}
		}
	}

	private void updateDeparture(double time, UpdateEvent event) {

		TrainState state = event.state;
		state.timestamp = time;

		state.allowedMaxSpeed = retrieveAllowedMaxSpeed(state);

		RailLink firstLink = resources.getLink(state.headLink);
		boolean preserveGeometry = shouldPreserveDepartureGeometry(state, firstLink);

		// When a train reverses, it is already correctly positioned even across multiple links
		// Otherwise trains are placed initially on the first link
		// Note that tail position can be negative here when the train is longer than the first link
		if (!preserveGeometry) {
			state.headPosition = firstLink.length;
			state.tailPosition = firstLink.length - state.train.length();
		}

		// reserve links and start if first one is free
		if (blockLinkTracks(time, state) || resources.isBlockedBy(firstLink, state)) {

			createEvent(new PersonEntersVehicleEvent(time, state.driver.getId(), state.driver.getVehicle().getId()));
			createEvent(new VehicleEntersTrafficEvent(time, state.driver.getId(),
				state.headLink, state.driver.getVehicle().getId(), state.driver.getMode(), 1.0));

			state.timestamp = time;

			double stopTime = 0;
			if (state.isStop(state.headLink)) {
				stopTime = handleTransitStop(time, state);
			}

			// Train departs at the very end of the first link
			state.routeIdx = 1;

			createEvent(state.asEvent(time));

			if (stopTime <= 0) {
				// Call enter link logic immediately
				enterLink(time, event);
			} else {
				event.plannedTime = time + stopTime;
				event.type = UpdateEvent.Type.ENTER_LINK;
			}

		} else {
			// vehicle will wait and call departure again
			event.plannedTime += config.getPollInterval();
		}
	}

	private boolean shouldPreserveDepartureGeometry(TrainState state, RailLink firstLink) {
		return Objects.equals(state.headLink, firstLink.getLinkId())
			&& FuzzyUtils.equals(state.headPosition, firstLink.length)
			&& RailsimCalc.checkTrainLength(state);
	}

	/**
	 * Reserve links in advance as necessary.
	 */
	private boolean blockLinkTracks(double time, TrainState state) {

		double reserveDist = RailsimCalc.calcReservationDistance(state, resources.getLink(state.headLink));

		// SWARM-FIX (FS2): with the window-stop rule off (below) a train really drives until the end of the
		// window it is granted. Asking for the bare stopping distance then makes the granted window end exactly
		// at the required distance, and the next reservation update has zero slack - which the engine treats as
		// an error ("Reserve distance must be positive", measured at the very first departure of a fixed free
		// run). Ask for the usual safety distance more. The punctual profile keeps the exact request: its
		// behaviour is tuned (JT1) and re-asks the profile every 200 m on the final approach anyway.
		double requestDist = punctualProfile() ? reserveDist : reserveDist + SAFETY_DIST;
		DispositionResponse response = disposition.requestNextSegment(time, state, requestDist);

		// Disposition assigned a detour
		if (state.pt != null && response.detour() != null) {

			List<RailLink> subRoute = state.route.subList(response.detour().startIdx(), response.detour().endIdx());

			TransitStopFacility newStop = state.pt.addDetour(subRoute, response.detour().newRoute());

			subRoute.clear();
			subRoute.addAll(response.detour().newRoute());

			if (newStop != null) {
				state.nextStop = newStop;
			}

			createEvent(new RailsimDetourEvent(
				time, state.driver.getVehicle().getId(),
				response.detour().startLink(), response.detour().endLink(),
				response.detour().newRoute().stream().map(RailLink::getLinkId).toList(),
				newStop != null ? newStop.getId() : null
			));

			// reserve the assigned links from the detour
			response = disposition.requestNextSegment(time, state, reserveDist);
		}

		state.approvedDist = response.approvedDist();

		// SWARM-FIX: see updatePosition(): a negative approved distance produces a NaN target speed.
		if (state.approvedDist < 0) {
			if (state.approvedDist < -1) {
				log.warn("Train {} got a negative approved distance {} from its disposition, using 0", state.driver.getId(), state.approvedDist);
			}
			state.approvedDist = 0;
		}

		assert FuzzyUtils.greaterEqualThan(state.approvedDist, 0) : "Approved distance must be positive, but was " + response.approvedDist();

		// Stop when the approved distance is not enough.
		// SWARM-FIX (JT1): with the punctual profile the disposition's own verdict is used. SimpleDisposition
		// returns approvedSpeed = 0 exactly when a link of the requested segment is only partially reserved (or
		// the current link is), i.e. when the train really has to stop at the end of its window. The additional
		// distance test below fires whenever the window ends less than SAFETY_DIST (10 m) beyond the reserve
		// distance, which on this network (OSM links of a few metres) happens on the open line as well - and then
		// it replaced the profile's speed by "drive at allowedMaxSpeed and brake at the end of the window"
		// (measured: a train accelerated from 11.7 to 25.7 m/s although its cap was 10-12 m/s).
		// SWARM-FIX (FS2): the same artefact is present with every other profile, and it is neither specific to
		// the punctual profile nor harmless: on the open line it sets approvedSpeed = 0, RailsimCalc
		// .calcTargetSpeed(approvedDist, ..., finalSpeed=0) then finds the approved window shorter than the
		// braking distance and returns targetSpeed = 0, i.e. the train brakes at full service deceleration for a
		// stop that does not exist. Measured on the FS2 free run (speedProfile=maxSpeed, 612 trains alone):
		// 812 392 state events with targetSpeed = 0 while the train was moving = 64 471 separate stop commands,
		// none of which ever came to a standstill, and 11.3 s lost on the IC5 Neuchâtel -> Yverdon leg
		// (booked 1 080 s). The legacy rule is therefore only applied when it is asked for explicitly
		// (engine.windowStop=legacy).
		if (PunctualProfileHook.windowStopLegacy() && !punctualProfile()
				&& FuzzyUtils.lessThan(state.approvedDist, reserveDist + SAFETY_DIST))
			state.approvedSpeed = 0;
		else
			state.approvedSpeed = response.approvedSpeed();

		// Return whether the train has to stop
		return state.approvedSpeed != 0;
	}

	private void enterLink(double time, UpdateEvent event) {

		TrainState state = event.state;

		updatePosition(time, event);

		// EN1-FIX (engine.headGuard, default on): the head may only be moved onto the next link when it really
		// stands at the end of the current one. A queued ENTER_LINK / REVERSE_TRAIN can fire early: the time was
		// computed by RailsimCalc.calcRequiredTime, which returned the end of the deceleration phase instead of
		// the arrival at the link end (see the fix there) - measured 153.8 m of free ride for an IC6 at Bern, and
		// the recorded body of the train grows by that distance because the tail cannot follow a teleport
		// (BN4: 301 m Olten / 657 m CH). Without the fix the code below moved the head to position 0 of the next
		// link unconditionally, i.e. the train drove forward without covering the distance.
		RailLink headLinkObject = resources.getLink(state.headLink);
		if (PunctualProfileHook.headGuard() && !event.waitingForLink
			&& FuzzyUtils.greaterThan(headLinkObject.length - state.headPosition, 0.01)) {

			if (FuzzyUtils.equals(state.speed, 0)) {
				// standing short of the link end: nothing moves until a reservation or the profile allows it
				event.type = UpdateEvent.Type.WAIT_FOR_RESERVATION;
				event.waitingForLink = false;
				event.checkReservation = time + config.getPollInterval();
				event.plannedTime = event.checkReservation;
				return;
			}

			// still moving: re-plan this update instead of entering the link
			decideTargetSpeed(event, state);
			decideNextUpdate(event);
			return;
		}

		// current head link is the pt stop, which means the train is at the end of the link when this is called
		if (!event.waitingForLink && state.isStop(state.headLink)) {

			double stopTime = handleTransitStop(time, state);

			assert stopTime >= 0 : "Stop time must be positive";
//			assert FuzzyUtils.equals(state.speed, 0) : "Speed must be 0 at pt stop, but was " + state.speed;

			// Keep the original arrival time so repeated dwell checks do not restart the reverse timer.
			event.plannedTime = time + stopTime;
			if (event.lastArrivalTime < 0) {
				event.lastArrivalTime = time;
			}
			state.speed = 0;

			return;
		}

		// Arrival at destination
		if (!event.waitingForLink && state.isRouteAtEnd()) {

//			assert FuzzyUtils.equals(state.speed, 0) : "Speed must be 0 at end, but was " + state.speed;

			state.speed = 0;

			// LT1: close the leg with the counterparts of the PersonEntersVehicle / VehicleEntersTraffic events of
			// updateDeparture, in the order of the standard QSim (vehicle leaves traffic, driver leaves vehicle,
			// then the driver's arrival from endLegAndComputeNextState). Upstream railsim never emits them, so
			// viewers that hide a vehicle on "vehicle leaves traffic" kept every finished train drawn at its last
			// position. Emitted at the end of EVERY leg, also when the vehicle circulation continues: the next leg
			// (a "Wenden" turn or the next trip) emits its own enter events in updateDeparture.
			createEvent(new VehicleLeavesTrafficEvent(time, state.driver.getId(), state.headLink,
				state.driver.getVehicle().getId(), state.driver.getMode(), 1.0));
			createEvent(new PersonLeavesVehicleEvent(time, state.driver.getId(), state.driver.getVehicle().getId()));

			state.driver.notifyArrivalOnLinkByNonNetworkMode(state.headLink);
			state.driver.endLegAndComputeNextState(Math.ceil(time));

			double nextDeparture = state.driver.getActivityEndTime();

			// Handles the "Umlauf" of pt drivers
			if (state.pt != null && Double.isFinite(nextDeparture)) {

				event.plannedTime = nextDeparture;
				event.type = UpdateEvent.Type.WAIT_DEPARTURE;

			} else {

				// Free all reservations
				for (RailLink link : state.route) {
					if (resources.isBlockedBy(link, state)) {
						disposition.unblockRailLink(time, state.driver, link);
					}
				}

				trainManager.removeActiveTrain(state);
				event.type = UpdateEvent.Type.IDLE;
			}

			return;
		}

		// Train stopped exactly at the end of the link and tries to enter the next link
		if (FuzzyUtils.equals(state.speed, 0) && !blockLinkTracks(time, state)) {

			RailLink currentLink = state.route.get(state.routeIdx);
			// EN1 (engine.reversalEnter, default off): BN4's one-line guard - at a reversal the next link is the
			// opposite of the head link, i.e. the physical track the train already stands on, and the train holds
			// that capacity-1 resource. Entering it can therefore never be unsafe. Measured: the branch is NOT a
			// self-block - the refusal in the one case where it fired (IC6 777.TA…_08:48 at Bern at t=36 240 s,
			// head 508218, next 508217) comes from the IR15's claim on 522491 (7 006 rows in
			// sectionlock_denials.csv), i.e. from the phantom tail reservation of the IC6 itself (defect 1).
			// With the tail fixed the refusal never occurs, and bypassing the section lock turns the clean
			// refusal into a two-train cycle (BN4 §5: 32/81 unchanged). Kept as a switch, off by default.
			boolean ownTrack = PunctualProfileHook.reversalEnter()
				&& currentLink.isOppositeLink(state.headLink) && isHeld(state, currentLink);
			// If this linked is blocked the driver can continue
			if (!resources.isBlockedBy(currentLink, state) && !ownTrack) {
				event.waitingForLink = true;
				event.type = UpdateEvent.Type.WAIT_FOR_RESERVATION;
				event.plannedTime = time + config.getPollInterval();
				return;
			}
		}

		// Peek next link and only commit the route index once the final post-reverse state is known.
		RailLink nextLink = state.route.get(state.routeIdx);

		if (Objects.equals(nextLink.getLinkId(), state.headLink)) {
			throw new IllegalStateException("Train %s is trying to enter link %s from itself. Such Loop connections are not supported".formatted(state.driver.getId(), state.headLink));
		}

		// Check for disallowed links
		if (state.routeIdx > 0) {
			RailLink currentLink = state.route.get(state.routeIdx - 1);

			if (currentLink.isDisallowedNextLink(nextLink.getLinkId()))
				throw new IllegalStateException("Train " + state.driver.getId() + " is trying to enter the disallowed link " + nextLink.getLinkId() + " from link " + currentLink.getLinkId() + ".");
		}

		// Check if needs to reverse
		if (event.type != UpdateEvent.Type.REVERSE_TRAIN && nextLink.isOppositeLink(state.headLink)) {
			if (state.train.reversible() < 0)
				throw new IllegalStateException("Train " + state.driver.getId() + " is trying to enter the opposite link " + nextLink.getLinkId() + " with a non-reversible train.");

			// Reschedule the event
			if (event.lastArrivalTime > -1) {
				// The time used for stopping is used for reversing
				event.plannedTime = Math.max(time, event.lastArrivalTime + state.train.reversible());
			} else
				event.plannedTime = time + state.train.reversible();

			event.type = UpdateEvent.Type.REVERSE_TRAIN;
			return;
		}

		int nextRouteIdx = state.routeIdx + 1;

		// Reverse-state reconstruction must use the pre-move geometry.
		Id<Link> previousHeadLink = state.headLink;
		Id<Link> previousTailLink = state.tailLink;
		double previousHeadPosition = state.headPosition;
		double previousTailPosition = state.tailPosition;

		// SWARM-FIX (E1): a reversal rebuilds the body onto the DEPARTURE route. Only do that when every
		// link the rebuilt body will cover is held by this train itself (the whole physical track, i.e.
		// its resource, is reserved by the train). Otherwise the body would be moved onto track the
		// train never reserved - measured in the baseline: a reversing IC6 at Bern was placed on track
		// held by an IC1 train (5 overlaps, 6-50 m). In that case the train does not reverse yet: it
		// keeps standing on its arrival track (which it holds), asks for the reservation again on every
		// poll and reverses as soon as the departure track under its body is free.
		if (event.type == UpdateEvent.Type.REVERSE_TRAIN) {
			List<RailLink> unheld = unheldReverseBody(state, nextLink, nextRouteIdx, previousHeadLink,
				previousHeadPosition, previousTailLink, previousTailPosition);
			if (!unheld.isEmpty()) {
				logReverseWait(state, time, nextLink, unheld);
				event.plannedTime = time + config.getPollInterval();
				return;
			}
		}

		// On route departure the head link is null
		createEvent(new LinkLeaveEvent(time, state.driver.getVehicle().getId(), state.headLink));

		assert resources.isBlockedBy(nextLink, state) : "Link has to be blocked by driver when entered";

		if (event.type == UpdateEvent.Type.REVERSE_TRAIN) {
			reverseTrain(time, state, nextLink, previousHeadLink, previousHeadPosition, previousTailLink, previousTailPosition, nextRouteIdx);
		} else {
			state.headLink = nextLink.getLinkId();
			state.headPosition = 0;
			state.routeIdx = nextRouteIdx;
		}

		assert RailsimCalc.checkTrainLength(state) : String.format("Train %s length incorrect at t=%.2f, link=%s", state.driver.getId(), time, state.headLink);

		// Reset waiting flag
		event.waitingForLink = false;
		event.lastArrivalTime = -1;
		event.type = UpdateEvent.Type.ENTER_LINK;

		state.driver.notifyMoveOverNode(state.headLink);
		createEvent(new LinkEnterEvent(time, state.driver.getVehicle().getId(), state.headLink));

		ttdHandler.writePosition(state, null);

		decideTargetSpeed(event, state);

		createEvent(state.asEvent(time));

		decideNextUpdate(event);
	}

	/**
	 * Reverse the train.
	 */
	private void reverseTrain(double time, TrainState state, RailLink nextLink, Id<Link> previousHeadLink, double previousHeadPosition,
							  Id<Link> previousTailLink, double previousTailPosition, int nextRouteIdx) {

		applyReverseGeometry(state, computeReverseGeometry(state, nextLink, nextRouteIdx, previousHeadLink,
			previousHeadPosition, previousTailLink, previousTailPosition));

		// The approved distance is measured from the head. The reversal moves the head forward along the
		// departure route without driving, so the distance it jumped is no longer ahead of the train.
		double jump = resources.getLink(previousHeadLink).length - previousHeadPosition;
		for (int i = nextRouteIdx - 1; i < state.routeIdx - 1; i++) {
			jump += state.route.get(i).length;
		}
		jump += state.headPosition;
		state.approvedDist = Math.max(0, state.approvedDist - jump);

		// Keep the time-distance head reference aligned when the train flips direction.
		state.cumulativeDistance += state.train.length();

		// SWARM-FIX
		releaseReverseReservations(time, state);
	}

	/**
	 * SWARM-FIX (E1): the geometry a reversal produces, without touching the state. Same code as the
	 * original {@code reverseTrain} body; {@link #unheldReverseBody} needs it to know which track the
	 * rebuilt body would cover before the state is changed.
	 */
	private record ReverseGeometry(Id<Link> headLink, double headPosition, int routeIdx,
								   Id<Link> tailLink, double tailPosition) {
	}

	private ReverseGeometry computeReverseGeometry(TrainState state, RailLink nextLink, int nextRouteIdx,
												   Id<Link> previousHeadLink, double previousHeadPosition,
												   Id<Link> previousTailLink, double previousTailPosition) {

		// Train is fully contained on the current link. Keep the tail on the current link and flip the head
		// onto the opposite link so routeIdx still means "first link the head has not entered yet".
		if (nextLink.isOppositeLink(previousTailLink)) {

			return new ReverseGeometry(nextLink.getLinkId(), nextLink.length - previousTailPosition, nextRouteIdx,
				previousHeadLink, previousHeadPosition);
		}

		Id<Link> tailLink = nextLink.getLinkId();
		double tailPosition = nextLink.length - previousHeadPosition;

		// Multi-link reverse: find the route element opposite to the old tail.
		RailLink tailReverse = null;
		int tailReverseIdx = nextRouteIdx;
		for (; tailReverseIdx < state.route.size(); tailReverseIdx++) {
			RailLink link = state.route.get(tailReverseIdx);
			if (link.isOppositeLink(previousTailLink)) {
				tailReverse = link;
				break;
			}
		}

		if (tailReverse != null) {
			return new ReverseGeometry(tailReverse.getLinkId(), tailReverse.length - previousTailPosition,
				tailReverseIdx + 1, tailLink, tailPosition);
		}

		// Reconstruct the new head by walking train length along the reversed route.
		double remainingLength = state.train.length();
		RailLink headLink = nextLink;
		double headPosition = tailPosition;
		int headIdx = nextRouteIdx - 1;

		double available = headLink.length - headPosition;
		while (FuzzyUtils.greaterThan(remainingLength, available) || (FuzzyUtils.equals(remainingLength, available) && headIdx + 1 < state.route.size()
			&& !state.isStop(headLink.getLinkId()))) {
			remainingLength -= available;
			headIdx++;

			if (headIdx >= state.route.size()) {
				throw new IllegalStateException(("Train %s is trying to reverse on a route that does not contain enough links for %s. " +
					"Make sure that a single transit line contains all links and stops needed for reversing.")
					.formatted(state.driver.getVehicle().getId(), previousTailLink));
			}

			headLink = state.route.get(headIdx);
			headPosition = 0;
			available = headLink.length;
		}

		return new ReverseGeometry(headLink.getLinkId(), headPosition + remainingLength, headIdx + 1,
			tailLink, tailPosition);
	}

	private void applyReverseGeometry(TrainState state, ReverseGeometry g) {
		state.headLink = g.headLink();
		state.headPosition = g.headPosition();
		state.routeIdx = g.routeIdx();
		state.tailLink = g.tailLink();
		state.tailPosition = g.tailPosition();
	}

	/**
	 * SWARM-FIX (E1): body links of the reversal that the train does NOT hold, i.e. whose physical track
	 * (resource, capacity 1) is reserved by nobody or by another train. Holding any direction link of
	 * the resource is enough: {@code FixedBlockResource} then keeps every other train out of it.
	 */
	private List<RailLink> unheldReverseBody(TrainState state, RailLink nextLink, int nextRouteIdx,
											 Id<Link> previousHeadLink, double previousHeadPosition,
											 Id<Link> previousTailLink, double previousTailPosition) {

		ReverseGeometry g = computeReverseGeometry(state, nextLink, nextRouteIdx, previousHeadLink,
			previousHeadPosition, previousTailLink, previousTailPosition);

		List<RailLink> unheld = new ArrayList<>();
		for (RailLink link : bodyLinks(state.route, g.routeIdx(), g.headPosition(), state.train.length())) {
			if (!isHeld(state, link)) {
				unheld.add(link);
			}
		}
		return unheld;
	}

	/** SWARM-FIX (E1): is the physical track of {@code link} reserved by this train (any direction)? */
	private boolean isHeld(TrainState state, RailLink link) {
		if (link.getResource().getType() != ResourceType.fixedBlock) {
			return resources.isBlockedBy(link, state);
		}
		for (RailLink other : link.getResource().getLinks()) {
			if (resources.isBlockedBy(other, state)) {
				return true;
			}
		}
		return false;
	}

	/** SWARM-FIX (E1): rate-limited logging of a train waiting for its reversal body to be free. */
	private void logReverseWait(TrainState state, double time, RailLink nextLink, List<RailLink> unheld) {
		String vid = state.driver.getVehicle().getId().toString();
		Double since = reverseWaitSince.putIfAbsent(vid, time);
		double waited = time - (since == null ? time : since);
		double nextWarn = reverseWaitWarnAt.getOrDefault(vid, 120.0);
		if (since == null || waited >= nextWarn) {
			reverseWaitWarnAt.put(vid, waited + 120.0);
			log.warn("SWARM-FIX E1: train {} waits to reverse at t={} s ({} s): the body would cover track not held by itself: {}",
				state.driver.getId(), time, Math.round(waited),
				unheld.stream().map(l -> l.getLinkId().toString()).toList());
		}
		if (reverseWaitLogged.add(vid))
			totalReverseWaits.incrementAndGet();
	}

	private final Set<String> reverseWaitLogged = new HashSet<>();
	private final Map<String, Double> reverseWaitSince = new HashMap<>();
	private final Map<String, Double> reverseWaitWarnAt = new HashMap<>();
	private final java.util.concurrent.atomic.AtomicInteger totalReverseWaits = new java.util.concurrent.atomic.AtomicInteger();

	/**
	 * SWARM-FIX: a reversal re-defines the train body in {@link #reverseTrain} without passing through
	 * {@link #leaveLink}. The arrival-direction links behind the new body were reserved one by one
	 * ({@code FixedBlockResource.tracks[]}) and {@code leaveLink} only ever releases {@code state.tailLink},
	 * which after a reversal walks along the *departure* links. The reservations of the arrival links are
	 * therefore never freed, and {@code FixedBlockResource.release} only frees a resource once the driver
	 * holds no link of it any more - so the whole track section stays EXHAUSTED for the rest of the day
	 * (a reversing train leaks the track between the old tail and the stop).
	 * <p>
	 * This releases exactly the reserved links the body no longer covers and keeps every link the train
	 * still stands on: a link is only released if its resource is not part of the new body, or if the
	 * body's link of that resource is already reserved (then {@code release} keeps the resource blocked).
	 */
	private void releaseReverseReservations(double time, TrainState state) {

		// links physically covered by the new body, walking back from the head by the train length
		List<RailLink> body = bodyLinks(state);
		Set<Id<Link>> bodyIds = new HashSet<>();
		for (RailLink link : body) {
			bodyIds.add(link.getLinkId());
		}

		// a route may visit a link twice: never touch a reservation that is still ahead of the head
		Set<Id<Link>> ahead = new HashSet<>();
		for (int i = Math.max(0, state.routeIdx); i < state.route.size(); i++) {
			ahead.add(state.route.get(i).getLinkId());
		}

		for (int i = Math.max(0, state.routeIdx - 1); i >= 0; i--) {
			RailLink link = state.route.get(i);
			if (bodyIds.contains(link.getLinkId()) || ahead.contains(link.getLinkId())) {
				continue;
			}
			if (!resources.isBlockedBy(link, state)) {
				continue;
			}

			// never free a resource the body still occupies: its new-direction link must be held
			boolean safe = true;
			for (RailLink b : body) {
				if (b.getResource() == link.getResource() && !resources.isBlockedBy(b, state)) {
					safe = false;
					break;
				}
			}
			if (!safe) {
				log.warn("Reversed train {} keeps its reservation on {}: the departure direction is not reserved yet",
					state.driver.getId(), link.getLinkId());
				continue;
			}

			if (link.minimumHeadwayTime == 0)
				disposition.unblockRailLink(time, state.driver, link);
			else
				updateQueue.add(new UpdateEvent(state, link, time));
		}
	}

	private void leaveLink(double time, UpdateEvent event) {

		TrainState state = event.state;

		RailLink nextTailLink = nextTailLink(state);
		Objects.requireNonNull(nextTailLink, () -> "Could not find next link in route " + state.tailLink);

		updatePosition(time, event);

		RailLink tailLink = resources.getLink(state.tailLink);

		// Forced position updates can make a queued leave-link event stale before the tail reaches the boundary.
		if (FuzzyUtils.lessThan(state.tailPosition, tailLink.length)) {
			assert RailsimCalc.checkTrainLength(state) : String.format("Train %s length incorrect at t=%.2f, link=%s", state.driver.getId(), time, state.tailLink);
			decideTargetSpeed(event, state);
			decideNextUpdate(event);
			return;
		}

		// JT2-FIX: the tail keeps moving while leaveLink is processed; the distance beyond the end of the
		// left link is the tail's position on the next link, NOT zero. The upstream code set it to 0 and
		// thereby lost up to one simulation step of tail movement at every link boundary. Measured on the
		// IR15 sub-scenario: the modelled train body grew from its real 200.6 m to 1 206 m over one run, so
		// retrieveAllowedMaxSpeed() kept capping the train at the limit of track it had already left (a
		// 60 km/h section 1 km behind held it at 60 km/h for another minute) and leaveLink released the
		// track up to a kilometre late.
		while (!FuzzyUtils.lessThan(state.tailPosition, tailLink.length)) {

			RailLink next = nextTailLink(state);
			if (next == null) {
				break;
			}

			createEvent(new RailsimTrainLeavesLinkEvent(time, state.driver.getVehicle().getId(), state.tailLink));

			double overshoot = state.tailPosition - tailLink.length;
			Id<Link> leftLink = state.tailLink;
			state.tailLink = next.getLinkId();
			state.tailPosition = overshoot;



			assert RailsimCalc.checkTrainLength(state) : String.format("Train %s length incorrect at t=%.2f, link=%s", state.driver.getId(), time, state.headLink);

			if (tailLink.minimumHeadwayTime == 0)
				unblockTrack(time, state, tailLink);
			else
				updateQueue.add(new UpdateEvent(state, tailLink, time));

			if (Objects.equals(state.tailLink, leftLink)) {
				break; // route loop: never spin
			}
			tailLink = resources.getLink(state.tailLink);
		}

		decideTargetSpeed(event, state);

		decideNextUpdate(event);
	}

	/**
	 * Next link of the tail in the route (current route, then the previous route of a reversal).
	 */
	private static RailLink nextTailLink(TrainState state) {

		RailLink nextTailLink = null;
		// Find the next link in the route.
		// JT2-FIX: stop at the first (nearest) occurrence. The original loop ran to i = 1 and kept the
		// LAST occurrence, i.e. it scanned the whole route at every link boundary although the tail is
		// at most a train length behind the head - measured as the hot spot of a Switzerland run
		// (jcmd thread dumps: 4 of 5 samples in this loop).
		for (int i = state.routeIdx; i >= 1; i--) {
			if (state.route.get(i - 1).getLinkId().equals(state.tailLink)) {
				nextTailLink = state.route.get(i);
				break;
			}
		}

		// Check element of previous route as well
		if (nextTailLink == null)
			for (int i = state.previousRoute.size() - 1; i >= 0; i--) {
				if (state.previousRoute.get(i).getLinkId().equals(state.tailLink)) {
					// Use next link from current route if it is the last
					nextTailLink = i == state.previousRoute.size() - 1 ? state.route.getFirst() : state.previousRoute.get(i + 1);
				}
			}

		return nextTailLink;
	}

	/**
	 * Unblocks a link.
	 */
	private void unblockTrack(double time, TrainState state, RailLink unblockLink) {
		disposition.unblockRailLink(time, state.driver, unblockLink);
		// SWARM-FIX: the tail just left this physical track, so the train must also drop its
		// reservations on the other links of the same resource (the other direction, or a
		// parallel track between the same two nodes). FixedBlockResource keeps the resource
		// blocked as long as the driver holds any of its links, so without this the resource
		// stays EXHAUSTED for the rest of the day after a reversal.
		releaseResourceSiblings(time, state, unblockLink);
	}

	/**
	 * SWARM-FIX: release the train's reservations on the other links of {@code released}'s resource,
	 * except links the train body still stands on. Called when the tail leaves {@code released},
	 * i.e. when the train no longer occupies that physical track in any direction.
	 */
	private void releaseResourceSiblings(double time, TrainState state, RailLink released) {

		// A moving block reservation is kept per link and distance and is released link by link when
		// the tail leaves it. Only a fixed block resource blocks all of its links together.
		if (released.getResource().getType() != ResourceType.fixedBlock)
			return;

		// Usually the train holds no other link of the resource; skip the route scans below then.
		boolean holdsSibling = false;
		for (RailLink other : released.getResource().getLinks()) {
			if (other != released && resources.isBlockedBy(other, state)) {
				holdsSibling = true;
				break;
			}
		}
		if (!holdsSibling)
			return;

		Set<Id<Link>> bodyIds = new HashSet<>();
		for (RailLink link : bodyLinks(state)) {
			bodyIds.add(link.getLinkId());
		}

		Set<Id<Link>> ahead = new HashSet<>();
		for (int i = Math.max(0, state.routeIdx); i < state.route.size(); i++) {
			ahead.add(state.route.get(i).getLinkId());
		}

		for (RailLink other : released.getResource().getLinks()) {
			if (other == released || bodyIds.contains(other.getLinkId()) || ahead.contains(other.getLinkId())) {
				continue;
			}
			if (resources.isBlockedBy(other, state)) {
				disposition.unblockRailLink(time, state.driver, other);
			}
		}
	}

	/**
	 * SWARM-FIX: links physically covered by the train body, walking back from the head by the train
	 * length. The recorded tail bookkeeping is intentionally not used here, so a stale tail cannot
	 * hide track the train has already left.
	 */
	private List<RailLink> bodyLinks(TrainState state) {
		return bodyLinks(state.route, state.routeIdx, state.headPosition, state.train.length());
	}

	/**
	 * SWARM-FIX (E1): body links for an arbitrary (possibly not yet applied) geometry.
	 */
	static List<RailLink> bodyLinks(List<RailLink> route, int routeIdx, double headPosition, double length) {

		List<RailLink> body = new ArrayList<>();
		double remaining = length;

		for (int i = routeIdx - 1; i >= 0; i--) {
			RailLink link = route.get(i);
			body.add(link);
			// The head covers its link up to the head position, every link behind it completely.
			double covered = i == routeIdx - 1 ? headPosition : link.length;
			if (FuzzyUtils.lessEqualThan(remaining, covered)) {
				break;
			}
			remaining -= covered;
		}

		return body;
	}

	/**
	 * Update position within a link and decides on next update.
	 */
	private void updatePosition(double time, UpdateEvent event) {

		TrainState state = event.state;

		double elapsed = time - state.timestamp;

		if (elapsed == 0)
			return;

		double accelTime = (state.targetSpeed - state.speed) / state.acceleration;

		double dist;
		if (state.acceleration == 0) {
			dist = state.speed * elapsed;

		} else if (accelTime < elapsed) {

			// Travelled distance under constant acceleration
			dist = RailsimCalc.calcTraveledDist(state.speed, accelTime, state.acceleration);

			// Remaining time at constant speed
			if (state.acceleration > 0)
				dist += RailsimCalc.calcTraveledDist(state.targetSpeed, elapsed - accelTime, 0);

			// Target speed was reached
			state.speed = state.targetSpeed;
			state.acceleration = 0;

		} else {

			// Acceleration was constant the whole time
			dist = RailsimCalc.calcTraveledDist(state.speed, elapsed, state.acceleration);
			state.speed = state.speed + elapsed * state.acceleration;

			if (FuzzyUtils.equals(state.speed, state.targetSpeed)) {
				state.speed = state.targetSpeed;
				state.acceleration = 0;
			}

		}

		assert FuzzyUtils.greaterEqualThan(dist, 0) : "Travel distance must be positive, but was" + dist;

		// This corrects inaccuracy happening when the train is at the end of the link
		// Should very rarely be necessary
		if (FuzzyUtils.greaterThan(state.headPosition + dist, resources.getLink(state.headLink).length)) {
			// Dist will be dist to end of link
			dist = resources.getLink(state.headLink).length - state.headPosition;
		}
		// In the same way also correct the speed
		if (FuzzyUtils.greaterThan(state.speed, state.allowedMaxSpeed)) {
			state.speed = state.allowedMaxSpeed;
			state.acceleration = 0;
		}

		state.cumulativeDistance += dist;
		state.headPosition += dist;
		state.tailPosition += dist;
		state.approvedDist -= dist;

		// SWARM-FIX: approved distance is a *remaining* distance and must not become negative. The train
		// always drives to the end of its head link, so it can overshoot a short approved window (it
		// cannot stop within the remaining distance), and floating point noise adds to that.
		// RailsimCalc.calcTargetSpeed() takes the sqrt of a term proportional to the distance, so a
		// negative value yields NaN as target speed; the train then decelerates through 0 into negative
		// speed and drives backwards for the rest of the simulation (observed: -2.5e9 m at Bern).
		if (state.approvedDist < 0) {
			state.approvedDist = 0;
		}

		if (Double.isFinite(state.targetDecelDist)) {
			state.targetDecelDist -= dist;
		}

		// When trains are put into the network their tail may be longer than the current link
		// this assertion may not hold depending on the network
//		assert state.routeIdx <= 2 || FuzzyUtils.greaterEqualThan(state.tailPosition, 0) : "Illegal state update. Tail position should not be negative";

		assert FuzzyUtils.greaterEqualThan(state.headPosition, 0) : "Head position must be positive";
		assert FuzzyUtils.lessEqualThan(state.speed, state.allowedMaxSpeed) : "Speed must be less equal than the allowed speed";
		assert FuzzyUtils.greaterEqualThan(state.approvedDist, 0) : "Approved distance must be positive";

		state.timestamp = time;


		ttdHandler.calculateDelay(state);

		// Only emit events on certain occasions
		if (event.type == UpdateEvent.Type.ENTER_LINK || event.type == UpdateEvent.Type.LEAVE_LINK || event.type == UpdateEvent.Type.POSITION || event.type == UpdateEvent.Type.SPEED_CHANGE)
			createEvent(state.asEvent(time));
	}

	/**
	 * Handle transit stop and update the state.
	 *
	 * @return stop time
	 */
	private double handleTransitStop(double time, TrainState state) {

		assert state.pt != null : "Pt driver must be present";

		ttdHandler.writePosition(state, state.nextStop);

		// Time needs to be rounded to current sim step
		double stopTime = state.pt.handleTransitStop(state.nextStop, Math.ceil(time));
		// If stop time > 0, train has not yet departed
		if (stopTime <= 0)
			state.nextStop = state.pt.getNextTransitStop();

		return stopTime;
	}

	/**
	 * Decide which update is the earliest and needs to be the next.
	 */
	private void decideNextUpdate(UpdateEvent event) {

		TrainState state = event.state;
		RailLink currentLink = resources.getLink(state.headLink);

		// (1) max speed reached
		double accelDist = Double.POSITIVE_INFINITY;
		if (state.acceleration > 0 && FuzzyUtils.greaterThan(state.targetSpeed, state.speed)) {
			accelDist = RailsimCalc.calcTraveledDist(state.speed, (state.targetSpeed - state.speed) / state.acceleration, state.acceleration);
		}

		// (1b) CH1: the lowest speed limit under the train body falls away once the tail has left the link
		// that limits it. The target speed is only re-evaluated when a link is entered or a reservation is
		// made, so the limit of the link the train has just left caps it for the whole of the next link:
		// on a 25.9 km off-model link behind a 40 km/h platform the train drove 11.1 m/s instead of
		// 23.3 m/s (+1 204 s on the leg). Re-run decideTargetSpeed at the moment the body clears.
		double refreshDist = PunctualProfileHook.speedRefresh() ? bodySpeedRefreshDist(state) : Double.POSITIVE_INFINITY;

		// (2) start deceleration
		double decelDist = state.targetDecelDist;

		assert FuzzyUtils.greaterEqualThan(decelDist, 0) : "Deceleration distance must be larger than 0, but was " + decelDist;

		// (3) next link needs reservation
		double reserveDist = Double.POSITIVE_INFINITY;
		if (!state.isRouteAtEnd() && !event.isAwaitingReservation()) {

			// Check first if more distance than for the next stop is needed
			// SWARM-FIX (FS2): the head link itself can be the stop link (the train is arriving at its next
			// stop). RailsimCalc.calcDistToNextStop only looks AHEAD, so it then reports the stop AFTER the
			// next one (measured: 65 120 m instead of 15.65 m) and the approved window (= the stop link) no
			// longer equals it; reserveDist becomes 0 and the engine aborts with "Reserve distance must be
			// positive". With the window-stop rule off this happens at every arrival, because the train
			// really drives to the stop. JT1's distToNextStop() handles the head link - use it for all
			// profiles.
			double nextStop = distToNextStop(state);

			if (!FuzzyUtils.equals(state.approvedDist, nextStop)) {
				double requiredDist = RailsimCalc.calcReservationDistance(state, currentLink);
				// when the approved distance is equal to the required distance, new reservation needs to be made
				reserveDist = state.approvedDist - requiredDist;
			} else if (punctualProfile() && nextStop < PunctualProfileHook.resolveWindowM()) {
				// SWARM-FIX (JT1): the approved window now ends at the next stop, so no further reservation is
				// needed and the engine would never ask the speed profile again - the cap of the profile would be
				// frozen up to 2.5 km before the stop (measured: 5-7 s early). Re-ask it every resolveDistanceM
				// on the final approach.
				reserveDist = Math.max(1., PunctualProfileHook.resolveDistanceM());
			}

			// With moving blocks the reserve distance might not be set to 0, but rather the accel or decel distance
			if (reserveDist < 0)
				reserveDist = 0;

			// Outside of block track the reserve distance is always greater 0
			// infinite loops would occur otherwise
			if (!(event.type != UpdateEvent.Type.BLOCK_TRACK || FuzzyUtils.greaterThan(reserveDist, 0))) {
				throw new AssertionError("Reserve distance must be positive, but was " + reserveDist);
			}
		}

		// (4) tail link changes
		double tailDist = resources.getLink(state.tailLink).length - state.tailPosition;

		// (5) head link changes
		double headDist = currentLink.length - state.headPosition;

		assert FuzzyUtils.greaterEqualThan(tailDist, 0) : "Tail distance must be positive";
		assert FuzzyUtils.greaterEqualThan(headDist, 0) : "Head distance must be positive";

		// Find the earliest required update

		double dist;
		if (FuzzyUtils.lessEqualThan(tailDist, decelDist) &&
			FuzzyUtils.lessEqualThan(tailDist, reserveDist) &&
			FuzzyUtils.lessEqualThan(tailDist, headDist) &&
			FuzzyUtils.lessEqualThan(tailDist, accelDist)) {

			// leave link has priority, and is ensured to be executed before others at same time step
			// otherwise train might block links of same length as itself

			dist = tailDist;
			event.type = UpdateEvent.Type.LEAVE_LINK;

		} else if (reserveDist <= accelDist && reserveDist <= decelDist && reserveDist <= tailDist && reserveDist <= headDist && reserveDist <= refreshDist) {
			dist = reserveDist;
			event.type = UpdateEvent.Type.BLOCK_TRACK;
		} else if (accelDist <= decelDist && accelDist <= reserveDist && accelDist <= tailDist && accelDist <= headDist && accelDist <= refreshDist) {
			dist = accelDist;
			event.type = UpdateEvent.Type.SPEED_CHANGE;
		} else if (decelDist <= accelDist && decelDist <= reserveDist && decelDist <= tailDist && decelDist <= headDist && decelDist <= refreshDist) {
			dist = decelDist;
			event.type = UpdateEvent.Type.SPEED_CHANGE;
		} else if (refreshDist <= accelDist && refreshDist <= decelDist && refreshDist <= reserveDist && refreshDist <= tailDist && refreshDist <= headDist) {
			// CH1: the body left the slower link behind -> re-evaluate the allowed speed now
			dist = refreshDist;
			event.type = UpdateEvent.Type.SPEED_CHANGE;
		} else {
			dist = headDist;
			event.type = UpdateEvent.Type.ENTER_LINK;
		}

		assert FuzzyUtils.greaterEqualThan(dist, 0) : "Distance for next update must be positive";

		// dist is the minimum of all supplied distances
		event.plannedTime = state.timestamp + RailsimCalc.calcRequiredTime(state, dist);

		// There could be old reservations events that need to be checked first
		if (event.isAwaitingReservation() && event.checkReservation < state.timestamp) {
			event.checkReservation = state.timestamp;
		}

		// insert reservation event if necessary
		if (event.isAwaitingReservation() && event.plannedTime > event.checkReservation) {

			event.type = UpdateEvent.Type.WAIT_FOR_RESERVATION;
			event.plannedTime = event.checkReservation;
		}

		assert Double.isFinite(event.plannedTime) : "Planned update time must be finite, but was " + event.plannedTime;
		assert event.plannedTime >= state.timestamp : "Planned time must be after current time";

	}

	/**
	 * Calculates possible target speed, consequential acceleration depending on current state.
	 */
	private void decideTargetSpeed(UpdateEvent event, TrainState state) {

		// Distance to next link
		RailLink currentLink = resources.getLink(state.headLink);

		double dist = currentLink.length - state.headPosition;
		state.allowedMaxSpeed = retrieveAllowedMaxSpeed(state);

		// Lookahead window
		double window = RailsimCalc.calcTraveledDist(state.allowedMaxSpeed, state.allowedMaxSpeed / state.train.deceleration(),
			-state.train.deceleration()) + currentLink.length;

		double advisedSpeed = state.approvedSpeed != 0 ? Math.min(state.approvedSpeed, state.allowedMaxSpeed) : state.allowedMaxSpeed;
		double minAllowed = advisedSpeed;

		state.targetSpeed = advisedSpeed;
		state.targetDecelDist = Double.POSITIVE_INFINITY;

		boolean stop = false;

		// Set speed approved by disposition
		if (state.approvedSpeed < minAllowed) {

			RailsimCalc.SpeedTarget target = RailsimCalc.calcTargetSpeed(state.approvedDist,
				state.train.acceleration(), state.train.deceleration(),
				state.speed, state.allowedMaxSpeed, state.approvedSpeed);

//			assert FuzzyUtils.greaterEqualThan(target.decelDist(), 0) : "Decel dist is " + target.decelDist() + ", stopping is not possible";

			// Because of numerical inaccuracies, the decel dist might be slightly negative
			if (FuzzyUtils.lessEqualThan(target.decelDist(), 0)) {
				state.targetSpeed = state.approvedSpeed;
				state.targetDecelDist = Double.POSITIVE_INFINITY;
				stop = true;
			} else {
				state.targetSpeed = target.targetSpeed();
				state.targetDecelDist = target.decelDist();
			}
		}

		for (int i = state.routeIdx; i <= state.route.size() && !stop; i++) {

			RailLink link;
			double allowed;
			if (i == state.route.size()) {
				link = null;
				allowed = 0;
			} else {
				link = state.route.get(i);

				// If the previous link is a transit stop the speed needs to be 0 at the next link
				// train stops at the very end of a link
				if (i > 0 && state.isStop(state.route.get(i - 1).getLinkId()))
					allowed = 0;
				else
					allowed = link.getAllowedFreespeed(state.driver);
			}

			// A future limit above the advised speed cannot require braking.
			if (!FuzzyUtils.equals(dist, 0) && allowed <= minAllowed && allowed <= advisedSpeed) {
				RailsimCalc.SpeedTarget target = RailsimCalc.calcTargetSpeed(dist, state.train.acceleration(), state.train.deceleration(),
					state.speed, advisedSpeed, Math.min(state.allowedMaxSpeed, allowed));

				// This assertion does not hold when the max speed on links is adjusted by network changes
				// assert FuzzyUtils.greaterEqualThan(target.decelDist(), 0) : "Decel dist must be greater than 0, or stopping is not possible";

				if (FuzzyUtils.lessEqualThan(target.decelDist(), 0)) {
					// Need to decelerate now
					state.targetSpeed = allowed;
					state.targetDecelDist = Double.POSITIVE_INFINITY;
					break;
				} else if (target.decelDist() < state.targetDecelDist && target.targetSpeed() <= state.targetSpeed) {

					// Decelerate later
					state.targetSpeed = target.targetSpeed();
					state.targetDecelDist = target.decelDist();

				} else if (target.targetSpeed() > state.targetSpeed && !Double.isFinite(state.targetDecelDist)) {
					// Acceleration is required
					state.targetSpeed = target.targetSpeed();
					state.targetDecelDist = target.decelDist();
				}
			}

			if (link != null)
				dist += link.length;

			if (FuzzyUtils.greaterEqualThan(dist, window))
				break;

			minAllowed = allowed;
		}

		// Calc accel depending on target
		if (FuzzyUtils.equals(state.speed, state.targetSpeed)) {
			state.acceleration = 0;
		} else if (FuzzyUtils.lessThan(state.speed, state.targetSpeed)) {
			state.acceleration = state.train.acceleration();
		} else {
			state.acceleration = -state.train.deceleration();
		}

		assert FuzzyUtils.equals(state.targetSpeed, state.speed) || state.acceleration != 0 : "Acceleration must be set if target speed is different than current";
		assert FuzzyUtils.greaterThan(state.targetDecelDist, 0) : "Target decel must be greater than 0 after updating";

	}


	/**
	 * SWARM-FIX (JT1): distance from the head to the end of the next stop link. {@link RailsimCalc#calcDistToNextStop}
	 * only looks AHEAD of the head link, so a train that is already running along its stop link (up to a kilometre)
	 * reported the stop AFTER that one - which made the engine request a new reservation in the middle of a platform
	 * track and broke the speed profile there.
	 */
	private double distToNextStop(TrainState state) {
		RailLink head = resources.getLink(state.headLink);
		if (state.isStop(state.headLink)) {
			return head.length - state.headPosition;
		}
		// JT2-FIX: RailsimCalc.calcDistToNextStop() scans every link from the head to the stop and is
		// called from every decideNextUpdate(), i.e. once per link and per poll (O(links^2) per train).
		// Keep the remaining distance incrementally: it only decreases by the distance the head moved
		// since the last call, so the sum is over the few links crossed in between. Invalidated when the
		// next stop, the route or the direction changes.
		int size = state.route.size();
		Id<Link> firstLink = state.route.get(0).getLinkId();
		Id<TransitStopFacility> stopId = state.nextStop == null ? null : state.nextStop.getId();
		StopDistance sd = stopDistance.get(state);
		if (sd == null || sd.routeSize != size || !sd.firstLink.equals(firstLink)
			|| !Objects.equals(sd.stopId, stopId) || state.routeIdx - 1 < sd.headIdx || state.routeIdx - 1 > sd.headIdx + 64) {
			double d = RailsimCalc.calcDistToNextStop(state, head);
			sd = new StopDistance(size, firstLink, stopId, state.routeIdx - 1, state.headPosition, d);
			stopDistance.put(state, sd);
			return d;
		}
		int headIdx = state.routeIdx - 1;
		double moved;
		if (headIdx == sd.headIdx) {
			moved = state.headPosition - sd.headPos;
		} else {
			moved = state.route.get(sd.headIdx).length - sd.headPos + state.headPosition;
			for (int i = sd.headIdx + 1; i < headIdx; i++) {
				moved += state.route.get(i).length;
			}
		}
		sd.remaining -= moved;
		sd.headIdx = headIdx;
		sd.headPos = state.headPosition;
		return Math.max(0., sd.remaining);
	}

	/** Incremental distance to the next stop (JT2), see {@link #distToNextStop}. */
	private static final class StopDistance {
		final int routeSize;
		final Id<Link> firstLink;
		final Id<TransitStopFacility> stopId;
		int headIdx;
		double headPos;
		double remaining;

		StopDistance(int routeSize, Id<Link> firstLink, Id<TransitStopFacility> stopId, int headIdx, double headPos,
					 double remaining) {
			this.routeSize = routeSize;
			this.firstLink = firstLink;
			this.stopId = stopId;
			this.headIdx = headIdx;
			this.headPos = headPos;
			this.remaining = remaining;
		}
	}

	/**
	 * CH1: distance until the lowest speed limit under the train body no longer applies, i.e. until the
	 * tail has left every link behind the head that is slower than the head link. {@code +inf} when no
	 * such link is under the body (nothing to gain) or when the remaining distance is below 0.5 m.
	 * <p>
	 * The engine re-evaluates the target speed only when a link is entered ({@code enterLink}) or a
	 * reservation is made ({@code blockLinkTracks}); the body-aware cap of {@link #retrieveAllowedMaxSpeed}
	 * therefore applies to a whole link. On short links this is the accepted body effect, but on a long
	 * link the limit of the link the train has just left caps it from end to end: measured on the
	 * 25.9 km off-model link Schaffhausen -> Bülach (RE48), where a 7 m 40 km/h platform held the train at
	 * 11.1 m/s instead of the planned 23.3 m/s for 2 330 s (+1 204 s on the leg, +10 860 s at Bülach).
	 */
	private double bodySpeedRefreshDist(TrainState state) {
		RailLink head = resources.getLink(state.headLink);
		double headLimit = head.getAllowedFreespeed(state.driver);
		double length = state.train.length();
		double back = state.headPosition;
		double dist = Double.POSITIVE_INFINITY;
		for (int i = state.routeIdx - 2; i >= 0 && back < length; i--) {
			RailLink link = state.route.get(i);
			if (link.getAllowedFreespeed(state.driver) < headLimit - 1e-6) {
				// +1 m: the event must fire *after* the tail has really left the link, otherwise the
				// recomputed cap is still the old one and the train never accelerates.
				dist = Math.min(dist, length - back + 1.);
			}
			back += link.length;
		}
		return dist > 0.5 ? dist : Double.POSITIVE_INFINITY;
	}

	/**
	 * Allowed speed for the train.
	 */
	private double retrieveAllowedMaxSpeed(TrainState state) {
		double maxSpeed = resources.getLink(state.headLink).getAllowedFreespeed(state.driver);

		// get minium allowed speed on all links occupied by the train
		// JT2-FIX: the body is defined by the vehicle length measured back from the head, exactly like
		// RailsimCalc.bodyLinks() does for the reservations - NOT by the tailLink pointer, which railsim
		// advances link by link and which can lag far behind the real tail (measured on an IR15 run: up to
		// 1 000 m), so a train was held at the limit of track it had left minutes ago (a 60 km/h section
		// 1 km behind kept it at 60 km/h for another minute, +35 s at Zofingen).
		double back = state.headPosition;
		for (int i = state.routeIdx - 2; i >= 0 && back < state.train.length(); i--) {
			RailLink link = state.route.get(i);
			maxSpeed = Math.min(maxSpeed, link.getAllowedFreespeed(state.driver));
			back += link.length;
		}

		return maxSpeed;
	}

	/**
	 * Remove all trains from simulation and generate events at the end of the day.
	 *
	 * @param now end of daytime
	 */
	void clearTrains(double now) {

		for (TrainState train : trainManager.getActiveTrains()) {
			eventsManager.processEvent(new VehicleAbortsEvent(now, train.driver.getVehicle().getId(), train.headLink));
			eventsManager.processEvent(new PersonStuckEvent(now, train.driver.getId(), train.headLink, train.driver.getMode()));
		}
	}
}
