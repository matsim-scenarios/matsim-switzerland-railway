package ch.sbb.run;

import ch.sbb.matsim.contrib.railsim.qsimengine.TrainPosition;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.PlannedArrival;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.SpeedProfile;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;

/**
 * Drives so that the train reaches the next stop {@code margin} seconds before its
 * published arrival time. Falls back to full speed as soon as the remaining time
 * is not enough (i.e. the train is late or has no slack).
 */
public final class JustInTimeSpeedProfile implements SpeedProfile {

	/** Never drive slower than this (m/s), otherwise block occupation grows without bound. */
	private final double minSpeed;

	/** Target earliness relative to the published arrival time (seconds). */
	private final double margin;

	public JustInTimeSpeedProfile(double margin, double minSpeed) {
		this.margin = margin;
		this.minSpeed = minSpeed;
	}

	@Override
	public double getTargetSpeed(double time, TrainPosition position, PlannedArrival nextArrival) {

		if (nextArrival == PlannedArrival.UNDEFINED || nextArrival.route() == null || nextArrival.route().isEmpty())
			return Double.POSITIVE_INFINITY;

		double remainingTime = nextArrival.time() - margin - time;
		if (remainingTime <= 0)
			return Double.POSITIVE_INFINITY; // late or no slack left -> full speed

		// getRouteUntilNextStop() starts at the current head link, so remove the part already travelled
		double distance = -position.getHeadPosition();
		double length = 0;
		double weightedSpeed = 0;
		for (RailLink link : nextArrival.route()) {
			double v = link.getAllowedFreespeed(position.getDriver());
			distance += link.getLength();
			length += link.getLength();
			weightedSpeed += v * link.getLength();
		}

		if (distance <= 0 || length <= 0)
			return Double.POSITIVE_INFINITY;

		double referenceSpeed = weightedSpeed / length;
		double requiredSpeed = distance / remainingTime;

		// Slower than the reference is not needed: full speed
		if (requiredSpeed >= referenceSpeed)
			return Double.POSITIVE_INFINITY;

		double target = requiredSpeed / referenceSpeed * nextArrival.route().getFirst().getAllowedFreespeed(position.getDriver());

		return Math.max(minSpeed, target);
	}
}
