package ch.sbb.run;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.core.utils.misc.OptionalTime;
import org.matsim.pt.transitSchedule.api.Departure;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;

import ch.sbb.matsim.contrib.railsim.config.RailsimConfigGroup;
import ch.sbb.matsim.contrib.railsim.qsimengine.RailsimCalc;
import ch.sbb.matsim.contrib.railsim.qsimengine.TrainInfo;
import ch.sbb.matsim.contrib.railsim.qsimengine.TrainPosition;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.PlannedArrival;
import ch.sbb.matsim.contrib.railsim.qsimengine.disposition.SpeedProfile;
import ch.sbb.matsim.contrib.railsim.qsimengine.resources.RailLink;

/**
 * Just-in-time speed profile: drive so that the train stops at the next stop exactly at its published arrival
 * time - neither late nor early.
 * <p>
 * <b>Model.</b> A leg is the run from the departure at one stop to the arrival at the next stop. The remaining
 * distance of a leg is the rest of the head link plus every link up to and including the next stop link. For a
 * candidate cruise speed {@code v} the profile predicts the time the train needs for that distance with the
 * <em>same</em> kinematics railsim uses ({@code RailsimCalc}):
 * <ol>
 * <li>a speed limit per link ({@code link.getAllowedFreespeed(driver)} = min(link freespeed, vehicle maximum)),</li>
 * <li>acceleration {@code a} and deceleration {@code d} of the vehicle ({@link TrainInfo}),</li>
 * <li>a backward braking envelope {@code env[k] = min(limit[k], sqrt(env[k+1]^2 + 2 d len[k]))} with
 * {@code env[N] = 0} at the end of the stop link,</li>
 * <li>a forward pass per link: brake at {@code d} to the envelope / accelerate at {@code a} to it, cruise, and
 * brake at {@code d} to the next envelope entry - the same phase formulas as railsim's
 * {@code RailsimCalc.calcTargetSpeed}.</li>
 * </ol>
 * {@code time(v)} is strictly decreasing in {@code v}, so the profile solves {@code time(v) = A - now - aim} by
 * bisection and returns that {@code v}. The engine uses it as approved speed: it accelerates to it and holds it
 * until the braking curve for the stop begins. If even the fastest possible run ({@code v = min limits}) needs more
 * time than is left, the profile returns {@link Double#POSITIVE_INFINITY} (full speed): the controller never makes a
 * train late and recovers after being held. A leg that is infeasible at full speed is logged once.
 * <p>
 * <b>Rounding.</b> The engine rounds event times up ({@code RailsimEngine.createEvent}: {@code Math.ceil(t)}, and
 * {@code handleTransitStop(..., Math.ceil(time))}), so the published second {@code A} is hit when the train really
 * stops in {@code (A-1, A]}. The controller therefore aims at {@code A - aim} with {@code aim = 0.25 s}.
 * <p>
 * <b>Cost (JT2).</b> The leg geometry (merged speed limits, cumulative distances, the route index of every
 * segment) is computed once per train and leg and cached. A call then only copies the suffix from the current head
 * position and runs one forward pass with the cached cruise speed; it re-solves only when that cruise no longer
 * hits the published arrival within {@code RESOLVE_TOL_S} (a disturbance: being held, a model error) or every
 * {@code resolveDistanceM} on the final kilometre. The empirical cost drops from a full route scan plus 19 passes
 * per call to one pass per call plus a solve only when the plan is off.
 * <p>
 * <b>Interaction with the deadlock avoidance.</b> This profile only sets a speed cap; it does not change
 * reservations. A slower train holds a claimed section longer (documented in the report).
 */
public final class PunctualSpeedProfile implements SpeedProfile {

	private static final Logger log = LogManager.getLogger(PunctualSpeedProfile.class);

	/** Default: aim this many seconds before the published arrival (event times are ceil-ed). */
	public static final double DEFAULT_AIM = 0.25;
	/** Default: no floor (exactness first). A floor makes a train arrive early, never late. */
	public static final double DEFAULT_MIN_FACTOR = 0.0;
	public static final double DEFAULT_MIN_SPEED = 0.0;
	public static final double DEFAULT_MAX_FACTOR = 1.0;

	/** Bisection iterations: 2^-24 of the speed range is far below any timing relevance. */
	private static final int BISECTION_ITERS = 18;

	/** Time step used to read the current speed out of RailsimCalc.projectedDistance. */
	private static final double SPEED_EPS = 0.01;

	/** Re-solve when the cached cruise misses the target arrival by more than this. */
	private static final double RESOLVE_TOL_S = 0.3;

	/** On the final approach, re-solve at least every this many metres (as JT1: catches the 1 s discretisation). */
	private static final double FINAL_APPROACH_M = 1000.;
	private static final double FINAL_RESOLVE_M = 100.;

	private final double aim;
	private final double minFactor;
	private final double minSpeed;
	private final double maxFactor;

	private final boolean writeLegs;
	private final Path legsFile;
	private final boolean trace;
	private BufferedWriter traceOut;
	/** Fallback for the offline time-distance approximation, whose SimpleTrainState has no TrainInfo. */
	private final RailsimConfigGroup railsimConfig;

	/** vehicle id + stop index -> already logged. */
	private final Map<String, Boolean> loggedLegs = new HashMap<>();
	private BufferedWriter legsOut;
	private int legRows;
	private long calls;
	private long infeasible;
	private long solves;
	private long rebuilds;
	private long nanos;
	private static final boolean TIMING = Boolean.getBoolean("punctual.timing");

	/** driver id -> geometry of the current leg. */
	private final Map<String, Leg> legCache = new HashMap<>();

	// reusable per-call scratch (the qsim is single threaded; prepareTimeDistanceApproximation runs before it)
	private double[] subLen = new double[64];
	private double[] subLim = new double[64];
	private double[] env = new double[65];

	/** Cached geometry of one leg: merged speed limits from the first link of the leg to the stop link. */
	private static final class Leg {
		int stopIndex = -1;
		int routeSize;
		int headIdx0;
		int n;
		double[] len = new double[32];
		double[] lim = new double[32];
		/** route index of the first link of every segment */
		int[] startIdx = new int[32];
		/** cumulative distance at the start of every segment, and at the end of the leg */
		double[] cum = new double[33];
		/** cumulative distance of every route link of the leg (cumLink[routeIdx - headIdx0]) */
		double[] cumLink = new double[256];
		/** route identity check: the link at lastHeadIdx must still be lastHeadLink (a reversal rebuilds the route) */
		int lastHeadIdx = -1;
		Id<Link> lastHeadLink;
		/** solution cache */
		boolean solved;
		double cruise;
		double solveTime;
		double solveCum;
		double solveHeadCum;
		double solveV0;
		double solveTarget;
	}

	public PunctualSpeedProfile(RailsimConfigGroup railsimConfig, double aim, double minFactor, double minSpeed,
								double maxFactor, boolean writeLegs, Path legsFile, boolean trace) {
		this.railsimConfig = railsimConfig;
		this.trace = trace;
		this.aim = aim;
		this.minFactor = minFactor;
		this.minSpeed = minSpeed;
		this.maxFactor = Math.max(1e-6, maxFactor);
		this.writeLegs = writeLegs;
		this.legsFile = legsFile;
		if (writeLegs) {
			try {
				if (legsFile.toAbsolutePath().getParent() != null) {
					Files.createDirectories(legsFile.toAbsolutePath().getParent());
				}
				legsOut = Files.newBufferedWriter(legsFile, StandardCharsets.UTF_8);
				legsOut.write("vehicle,line,route,departure,stop_index,n_stops,from_link,to_link,now,goal_time,"
						+ "remaining_s,fastest_s,booked_s,cruise_ms,max_limit_ms,distance_m,feasible,v0_ms,accel,decel\n");
			} catch (IOException e) {
				log.warn("punctual: cannot write {} - leg log disabled", legsFile, e);
				legsOut = null;
			}
		}
	}

	public static PunctualSpeedProfile defaults() {
		return new PunctualSpeedProfile(null, DEFAULT_AIM, DEFAULT_MIN_FACTOR, DEFAULT_MIN_SPEED, DEFAULT_MAX_FACTOR,
				false, null, false);
	}

	/**
	 * Same planned arrival as the engine's default implementation, but without the route scan of
	 * {@code getRouteUntilNextStop()} (which starts at route index 0 for every call - a 60 km leg has 2 400 links,
	 * and the reservation logic calls this thousands of times per train). The profile only needs the published
	 * arrival time; the leg geometry is cached separately.
	 */
	@Override
	public PlannedArrival getNextArrival(double time, TrainPosition position) {
		var pt = position.getPt();
		if (pt == null)
			return PlannedArrival.UNDEFINED;
		TransitStopFacility stop = position.getNextStop();
		if (stop == null)
			return PlannedArrival.UNDEFINED;
		TransitRoute tr = pt.getTransitRoute();
		Departure dep = pt.getDeparture();
		if (tr == null || dep == null)
			return PlannedArrival.UNDEFINED;
		int idx = pt.getCurrentStopIndex();
		if (idx < 0 || idx >= tr.getStops().size())
			return PlannedArrival.UNDEFINED;
		OptionalTime arrival = tr.getStops().get(idx).getArrivalOffset();
		if (arrival.isUndefined())
			return PlannedArrival.UNDEFINED;
		return new PlannedArrival(dep.getDepartureTime() + arrival.seconds(), List.of());
	}

	@Override
	public double getTargetSpeed(double time, TrainPosition position, PlannedArrival nextArrival) {
		if (!TIMING) {
			return getTargetSpeed0(time, position, nextArrival);
		}
		long t0 = System.nanoTime();
		try {
			return getTargetSpeed0(time, position, nextArrival);
		} finally {
			nanos += System.nanoTime() - t0;
		}
	}

	private double getTargetSpeed0(double time, TrainPosition position, PlannedArrival nextArrival) {

		calls++;

		if (nextArrival == PlannedArrival.UNDEFINED || !Double.isFinite(nextArrival.time()))
			return Double.POSITIVE_INFINITY;

		double a = 0, d = 0, vmaxVehicle = 0;
		try {
			TrainInfo train = position.getTrain();
			a = train.acceleration();
			d = train.deceleration();
			vmaxVehicle = train.maxVelocity();
		} catch (UnsupportedOperationException e) {
			// TrainTimeDistanceHandler.initialTimeDistanceData() runs on a SimpleTrainState before the mobsim and
			// only writes a diagnostics CSV (railsimTimeDistance.csv). Planning there would cost O(links^2) per
			// route and does not influence the simulation, so the profile stays out of it (full speed).
			return Double.POSITIVE_INFINITY;
		}
		if (!(a > 0) || !(d > 0))
			return Double.POSITIVE_INFINITY; // no vehicle dynamics -> nothing to plan with

		int size = position.getRouteSize();
		int headIdx = Math.max(0, position.getRouteIndex() - 1);
		if (headIdx >= size)
			return Double.POSITIVE_INFINITY;

		// The engine calls this from checkTrackReservation() BEFORE updatePosition(), so the recorded head
		// position can be up to pollInterval (5 s, ~200 m) old. RailsimCalc.projectedDistance() is the distance
		// the train has really covered since then; skipping it keeps the plan consistent with the clock.
		double lag = 0;
		try {
			lag = Math.max(0., RailsimCalc.projectedDistance(time, position));
		} catch (RuntimeException e) {
			// offline / simple state: no projection
		}
		double skip = position.getHeadPosition() + lag;

		int stopIndex = -1;
		try {
			stopIndex = position.getPt() == null ? -1 : position.getPt().getCurrentStopIndex();
		} catch (RuntimeException e) {
			// diagnostics only
		}

		Leg leg = leg(position, headIdx, stopIndex);
		if (leg == null)
			return Double.POSITIVE_INFINITY;

		// cumulative distance of the head from the start of the leg
		int rel = headIdx - leg.headIdx0;
		if (rel < 0 || rel >= leg.cumLink.length)
			return Double.POSITIVE_INFINITY;
		double headCum = leg.cumLink[rel] + skip;

		// segment that contains the head
		int k = 0;
		while (k + 1 < leg.n && leg.cum[k + 1] <= headCum)
			k++;
		double firstLen = leg.cum[k + 1] - headCum;
		if (firstLen < 1e-3) {
			k++;
			if (k >= leg.n)
				return Double.POSITIVE_INFINITY;
			firstLen = leg.len[k];
		}
		double distance = leg.cum[leg.n] - headCum;
		if (distance < 1.0 || leg.n - k == 0)
			return Double.POSITIVE_INFINITY;

		int n = fillSuffix(leg, k, firstLen);
		double minLimit = Double.POSITIVE_INFINITY;
		double maxLimit = 0;
		for (int i = 0; i < n; i++) {
			minLimit = Math.min(minLimit, subLim[i]);
			maxLimit = Math.max(maxLimit, subLim[i]);
		}

		double v0 = currentSpeed(time, position);
		double vmax = Math.min(maxLimit * maxFactor, vmaxVehicle);
		double goal = nextArrival.time() - aim;
		double remaining = goal - time;

		// ---- cached plan: keep the cruise speed as long as it still lands on the published second ----
		double fastest = -1;
		double cruise;
		if (leg.solved && useCached(leg, v0, a, d, n, remaining, headCum)) {
			cruise = leg.cruise;
		} else {
			fastest = time(vmax, v0, a, d, n);
			if (!(remaining > 0) || fastest > remaining) {
				infeasible++;
				leg.solved = false;
				trace(position, time, v0, distance, remaining, fastest, -1., fastest, lag);
				maybeLogLeg(position, time, goal, remaining, fastest, distance, maxLimit, vmax, v0, 0., false, a, d);
				return Double.POSITIVE_INFINITY;
			}
			cruise = solve(remaining, v0, a, d, n, vmax);
			cruise = Math.min(cruise, vmax);
			leg.solved = true;
			leg.cruise = cruise;
			leg.solveTime = time;
			leg.solveV0 = v0;
			leg.solveHeadCum = headCum;
			leg.solveTarget = remaining;
			solves++;
		}

		// Optional floors (off by default: they trade exactness for capacity and are reported in the report)
		double floor = Math.max(minSpeed, minFactor * subLim[0]);
		if (floor > 0) {
			cruise = Math.max(cruise, Math.min(floor, vmax));
		}
		maybeLogLeg(position, time, goal, remaining, fastest, distance, maxLimit, vmax, v0, cruise, true, a, d);
		if (trace && fastest < 0) {
			fastest = time(vmax, v0, a, d, n);
		}
		trace(position, time, v0, distance, remaining, fastest, cruise, time(cruise, v0, a, d, n), lag);

		return cruise;
	}

	/**
	 * True if the cached cruise still predicts the target arrival within {@link #RESOLVE_TOL_S} and the train
	 * followed the plan (so the cached speed is not a leftover of another state). Never on the final approach
	 * closer than {@link #FINAL_RESOLVE_M} to the stop, where the plan is refreshed to catch the 1 s discretisation.
	 */
	private boolean useCached(Leg leg, double v0, double a, double d, int n, double remaining, double headCum) {

		// final approach: refresh the plan every FINAL_RESOLVE_M
		if (leg.cum[leg.n] - headCum < FINAL_APPROACH_M
			&& headCum - leg.solveHeadCum > FINAL_RESOLVE_M) {
			return false;
		}
		// the train must be ahead of the point of the last solve
		if (headCum + 1. < leg.solveHeadCum)
			return false;
		// and the cached cruise must still hit the target
		double predicted = time(leg.cruise, v0, a, d, n);
		return Math.abs(predicted - remaining) <= RESOLVE_TOL_S;
	}

	private int fillSuffix(Leg leg, int k, double firstLen) {
		int n = leg.n - k;
		if (subLen.length < n) {
			grow(n);
		}
		subLen[0] = firstLen;
		subLim[0] = leg.lim[k];
		if (n > 1) {
			System.arraycopy(leg.len, k + 1, subLen, 1, n - 1);
			System.arraycopy(leg.lim, k + 1, subLim, 1, n - 1);
		}
		return n;
	}

	/**
	 * Build (or reuse) the merged geometry of the leg that starts at {@code headIdx} and ends at the next stop
	 * link. Rebuilt only when the train started a new leg or the head went back.
	 */
	private Leg leg(TrainPosition position, int headIdx, int stopIndex) {

		String driver = position.getDriver().getId().toString();
		Leg leg = legCache.get(driver);
		if (leg != null && leg.stopIndex == stopIndex && leg.routeSize == position.getRouteSize()
			&& headIdx >= leg.headIdx0 && headIdx - leg.headIdx0 < leg.cumLink.length
			&& leg.lastHeadIdx >= 0 && leg.lastHeadIdx < position.getRouteSize()
			&& leg.lastHeadLink.equals(position.getRoute(leg.lastHeadIdx).getLinkId())) {
			leg.lastHeadIdx = headIdx;
			leg.lastHeadLink = position.getRoute(headIdx).getLinkId();
			return leg;
		}

		int size = position.getRouteSize();
		Leg fresh = new Leg();
		fresh.stopIndex = stopIndex;
		fresh.routeSize = size;
		fresh.headIdx0 = headIdx;

		int n = 0;
		int links = 0;
		double cum = 0;
		int cap = 32;
		double[] len = new double[cap];
		double[] lim = new double[cap];
		int[] start = new int[cap];
		double[] cumSeg = new double[cap + 1];
		double[] cumLink = new double[Math.max(16, size - headIdx + 2)];
		boolean foundStop = false;

		cumLink[0] = 0;
		for (int i = headIdx; i < size; i++) {
			RailLink link = position.getRoute(i);
			double l = link.getLength();
			double allowed = link.getAllowedFreespeed(position.getDriver());
			if (n > 0 && lim[n - 1] == allowed) {
				len[n - 1] += l;
			} else {
				if (n == cap) {
					cap *= 2;
					double[] nl = new double[cap];
					double[] nm = new double[cap];
					int[] ns = new int[cap];
					double[] nc = new double[cap + 1];
					System.arraycopy(len, 0, nl, 0, n);
					System.arraycopy(lim, 0, nm, 0, n);
					System.arraycopy(start, 0, ns, 0, n);
					System.arraycopy(cumSeg, 0, nc, 0, n + 1);
					len = nl;
					lim = nm;
					start = ns;
					cumSeg = nc;
				}
				len[n] = l;
				lim[n] = allowed;
				start[n] = i;
				cumSeg[n] = cum;
				n++;
			}
			cum += l;
			links++;
			cumLink[links] = cum;
			if (position.isStop(link.getLinkId())) {
				foundStop = true;
				break;
			}
		}
		if (!foundStop || n == 0 || cum < 1.0) {
			return null; // standing on the stop link / nothing to plan
		}
		cumSeg[n] = cum;

		fresh.len = len;
		fresh.lim = lim;
		fresh.startIdx = start;
		fresh.cum = cumSeg;
		fresh.cumLink = cumLink;
		fresh.n = n;
		fresh.lastHeadIdx = headIdx;
		fresh.lastHeadLink = position.getRoute(headIdx).getLinkId();
		legCache.put(driver, fresh);
		rebuilds++;
		return fresh;
	}

	/**
	 * Current speed in m/s: derivative of the engine's own distance projection over a 20 ms window. A single
	 * {@code projectedDistance(time + eps)/eps} is wrong whenever the recorded state is older than {@code time}
	 * (checkTrackReservation calls before updatePosition): measured 1 107 m/s instead of 39 m/s, which made the
	 * controller crawl at 0.01 m/s for 27 minutes.
	 */
	private static double currentSpeed(double time, TrainPosition position) {
		try {
			double d1 = RailsimCalc.projectedDistance(time, position);
			double d2 = RailsimCalc.projectedDistance(time + 2 * SPEED_EPS, position);
			return Math.max(0., (d2 - d1) / (2 * SPEED_EPS));
		} catch (RuntimeException e) {
			return 0.;
		}
	}

	/**
	 * Time for the remaining leg at a constant cruise speed {@code v} (0 = stop), starting at {@code v0}.
	 * <p>
	 * Per segment (link) the speed is {@code min(sqrt(v0^2+2as), cap, link limit, braking curve to the end)}:
	 * accelerate at {@code a}, cruise, brake at {@code d}. That is exactly what the engine does when it is asked
	 * for a target speed at every update (RailsimCalc.calcTargetSpeed / decideTargetSpeed).
	 */
	private double time(double v, double v0, double a, double decel, int n) {

		double d = decel;

		// backward braking envelope: env[k] = highest speed allowed at the START of segment k
		env[n] = 0;
		for (int k = n - 1; k >= 0; k--) {
			double e = Math.min(subLim[k], v);
			double fromNext = Math.sqrt(env[k + 1] * env[k + 1] + 2 * d * subLen[k]);
			env[k] = Math.min(e, fromNext);
		}

		double speed = v0;
		double t = 0;
		for (int k = 0; k < n; k++) {
			double L = subLen[k];
			double vcap = Math.min(subLim[k], v);            // cruise speed on this link
			double c = Math.min(env[k + 1], vcap);       // speed to reach at the end of this link

			// 1. arrive too fast for this link: brake at d, then continue with the rest of the link
			if (speed > vcap + 1e-9) {
				double sBrake = (speed * speed - vcap * vcap) / (2 * d);
				if (sBrake >= L) {
					double vEnd = Math.sqrt(Math.max(0., speed * speed - 2 * d * L));
					t += (speed - vEnd) / d;
					speed = vEnd;
					continue;
				}
				t += (speed - vcap) / d;
				speed = vcap;
				L -= sBrake;
			}
			if (L <= 1e-9) {
				continue;
			}

			// 2. accelerate at a to the cruise speed, cruise, and brake at d to the exit speed.
			// The braking for the exit speed must start at sBrakeStart; if the train cannot reach the
			// cruise speed before that point, the acceleration curve meets the braking curve inside the
			// link and the peak speed follows from that (same triangle as RailsimCalc.calcTargetSpeed).
			double sAcc = (vcap * vcap - speed * speed) / (2 * a);
			double sBrakeStart = L - (vcap * vcap - c * c) / (2 * d);
			if (sAcc <= sBrakeStart + 1e-9) {
				t += (vcap - speed) / a + (sBrakeStart - sAcc) / vcap + (vcap - c) / d;
				speed = c;
			} else {
				double sMeet = (c * c + 2 * d * L - speed * speed) / (2 * (a + d));
				if (sMeet <= L) {
					double vp = Math.sqrt(speed * speed + 2 * a * Math.max(0., sMeet));
					vp = Math.min(vp, vcap);
					t += (vp - speed) / a + (vp - c) / d;
					speed = c;
				} else {
					double vEnd = Math.sqrt(speed * speed + 2 * a * L);
					t += (vEnd - speed) / a;
					speed = vEnd;
				}
			}
		}
		return t;
	}

	/** Solves time(v) = target on [0.01, vmax]; time is strictly decreasing in v. */
	private double solve(double target, double v0, double a, double d, int n, double vmax) {
		double lo = 0.01;
		double hi = vmax;
		if (time(hi, v0, a, d, n) > target)
			return vmax;
		for (int i = 0; i < BISECTION_ITERS; i++) {
			double mid = 0.5 * (lo + hi);
			if (time(mid, v0, a, d, n) > target) {
				lo = mid; // too slow -> need faster
			} else {
				hi = mid;
			}
		}
		return 0.5 * (lo + hi);
	}

	private void grow(int n) {
		int size = Math.max(n, subLen.length * 2);
		subLen = new double[size];
		subLim = new double[size];
		env = new double[size + 1];
	}

	// ------------------------------------------------------------------ diagnostics

	private void maybeLogLeg(TrainPosition position, double time, double goal,
							 double remaining, double fastest, double distance, double maxLimit,
							 double vmax, double v0, double cruise, boolean feasible, double a, double d) {

		if (!writeLegs || legsOut == null)
			return;

		ch.sbb.matsim.contrib.railsim.qsimengine.RailsimTransitDriverAgent pt;
		try {
			pt = position.getPt();
		} catch (UnsupportedOperationException e) {
			return;
		}
		if (pt == null)
			return;
		String key = position.getDriver().getId() + "#" + pt.getCurrentStopIndex();
		if (loggedLegs.putIfAbsent(key, Boolean.TRUE) != null)
			return;

		String line = "";
		String route = "";
		String departure = "";
		double booked = Double.NaN;
		try {
			TransitRoute tr = pt.getTransitRoute();
			Departure dep = pt.getDeparture();
			if (tr != null)
				route = tr.getId().toString();
			if (dep != null)
				departure = dep.getId().toString();
			if (pt.getTransitLine() != null)
				line = pt.getTransitLine().getId().toString();
			int idx = pt.getCurrentStopIndex();
			if (tr != null && idx > 0 && idx < tr.getStops().size()) {
				OptionalTime arr = tr.getStops().get(idx).getArrivalOffset();
				OptionalTime depPrev = tr.getStops().get(idx - 1).getDepartureOffset();
				if (arr.isDefined() && depPrev.isDefined())
					booked = arr.seconds() - depPrev.seconds();
			}
		} catch (RuntimeException e) {
			// diagnostics only
		}

		Id<Link> to = null;
		for (int i = Math.max(0, position.getRouteIndex() - 1); i < position.getRouteSize(); i++) {
			if (position.isStop(position.getRoute(i).getLinkId())) {
				to = position.getRoute(i).getLinkId();
				break;
			}
		}

		try {
			legsOut.write(String.format(Locale.ROOT,
				"%s,%s,%s,%s,%d,%d,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.1f,%s,%.3f,%.3f,%.3f%n",
				csv(position.getDriver().getId().toString()), csv(line), csv(route), csv(departure),
				pt.getCurrentStopIndex(), position.getRouteSize(),
				position.getHeadLink(), to, time, goal, remaining, fastest, booked, cruise, maxLimit, distance,
				feasible ? "yes" : "NO", v0, a, d));
			legRows++;
			if (legRows % 200 == 0)
				legsOut.flush();
		} catch (IOException e) {
			log.warn("punctual: cannot write leg row", e);
			close();
		}
	}

	private void trace(TrainPosition position, double time, double v0, double distance, double remaining,
					   double fastest, double cruise, double predicted, double lag) {
		if (!trace)
			return;
		try {
			if (traceOut == null) {
				Path f = legsFile == null ? java.nio.file.Paths.get("punctual_trace.csv")
						: legsFile.resolveSibling("punctual_trace.csv");
				traceOut = Files.newBufferedWriter(f, StandardCharsets.UTF_8);
				traceOut.write("vehicle,time,head_link,head_pos,route_idx,v0_ms,distance_m,remaining_s,fastest_s,cruise_ms,pred_s,lag_m\n");
			}
			traceOut.write(String.format(Locale.ROOT, "%s,%.2f,%s,%.1f,%d,%.3f,%.1f,%.2f,%.2f,%.3f,%.2f,%.1f%n",
				position.getDriver().getId(), time, position.getHeadLink(), position.getHeadPosition(),
				position.getRouteIndex(), v0, distance, remaining, fastest, cruise, predicted, lag));
			traceOut.flush();
		} catch (IOException e) {
			log.warn("punctual: cannot write trace", e);
		}
	}

	private static String csv(String s) {
		return s == null ? "" : s.replace(',', ';');
	}

	/** Called by the run class after the simulation. */
	public void close() {
		if (traceOut != null) {
			try {
				traceOut.flush();
				traceOut.close();
			} catch (IOException ignored) {
				// nothing to do
			}
			traceOut = null;
		}
		if (legsOut != null) {
			try {
				legsOut.flush();
				legsOut.close();
			} catch (IOException e) {
				log.warn("punctual: cannot close {}", legsFile, e);
			}
			legsOut = null;
		}
		log.info("punctual profile: {} getTargetSpeed calls, {} infeasible, {} solves, {} leg rebuilds, {} leg rows written{}",
			calls, infeasible, solves, rebuilds, legRows, writeLegs ? " to " + legsFile : "");
		if (TIMING) {
			log.info("punctual profile CPU: {} ms in getTargetSpeed ({}.{} % of a 1381 s run)",
				nanos / 1_000_000L, nanos / 10_000_000L, (nanos / 1_000_000L) % 100);
		}
	}

	public long getCalls() {
		return calls;
	}

	public long getSolves() {
		return solves;
	}

	public long getInfeasibleLegs() {
		return infeasible;
	}

	public static Path defaultLegsFile() {
		return Paths.get("punctual_legs.csv");
	}
}
