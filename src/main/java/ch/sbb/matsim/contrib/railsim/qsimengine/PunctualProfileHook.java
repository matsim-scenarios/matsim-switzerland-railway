package ch.sbb.matsim.contrib.railsim.qsimengine;

/**
 * Switch for the JT1 "punctual" speed profile hooks inside {@link RailsimEngine}.
 * <p>
 * The punctual profile needs two small behaviour changes in the engine on the final approach to a stop (the
 * approved speed of the profile must not be replaced by "drive as fast as allowed and brake at the last moment",
 * and the stop on the head link must be recognised as the next stop). Both are only correct for a profile that
 * really controls the arrival time, so they are switched on by {@code RunSwitzerlandRailway} for
 * {@code speedProfile=punctual} and stay off otherwise - with the switch off the engine behaves exactly like
 * base7.
 */
public final class PunctualProfileHook {

	private static volatile boolean active;
	private static volatile double resolveDistanceM = 200.;
	private static volatile double resolveWindowM = 1000.;
	/** FS2: restore the legacy "approved speed = 0 when the reserved window ends within 10 m of the
	 * distance needed to stop" rule. Off by default: on this network it fires on the open line. */
	private static volatile boolean windowStopLegacy;
	/** CH1: re-evaluate the target speed once the tail has left a link that is slower than the head link
	 * (the engine otherwise keeps the cap of the previous link for the whole of the next link). */
	private static volatile boolean speedRefresh;
	/** EN1: {@link RailsimCalc#calcRequiredTime} adds the cruise phase at the target speed when the
	 * requested distance is farther than the deceleration phase (default on; off = base12 behaviour). */
	private static volatile boolean reachTime = true;
	/** EN1: {@code RailsimEngine.enterLink} refuses to move the head onto the next link while the head is
	 * not at the end of the current one (default on; off = base12 behaviour, i.e. the head jumps). */
	private static volatile boolean headGuard = true;
	/** EN1: BN4's reversal guard - a train standing at a reversal may enter the departure link when it is
	 * the opposite direction of its head link and the train holds that physical track itself.
	 * Default off: measured as not needed after the tail fix and it bypasses the section lock. */
	private static volatile boolean reversalEnter;

	private PunctualProfileHook() {
	}

	public static void setReachTime(boolean value) {
		reachTime = value;
	}

	public static boolean reachTime() {
		return reachTime;
	}

	public static void setHeadGuard(boolean value) {
		headGuard = value;
	}

	public static boolean headGuard() {
		return headGuard;
	}

	public static void setReversalEnter(boolean value) {
		reversalEnter = value;
	}

	public static boolean reversalEnter() {
		return reversalEnter;
	}

	public static void setSpeedRefresh(boolean value) {
		speedRefresh = value;
	}

	public static boolean speedRefresh() {
		return speedRefresh;
	}

	public static void setWindowStopLegacy(boolean value) {
		windowStopLegacy = value;
	}

	public static boolean windowStopLegacy() {
		return windowStopLegacy;
	}

	public static void setActive(boolean value) {
		active = value;
	}

	public static void setResolveDistanceM(double metres) {
		resolveDistanceM = metres;
	}

	public static boolean isActive() {
		return active;
	}

	/** Re-ask the speed profile at least after this distance while the train is on its final approach. */
	public static double resolveDistanceM() {
		return resolveDistanceM;
	}

	/** The final approach starts this many metres before the stop. */
	public static double resolveWindowM() {
		return resolveWindowM;
	}
}
