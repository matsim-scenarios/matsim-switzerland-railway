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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
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
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.groups.ControllerConfigGroup;
import org.matsim.core.controler.Controler;
import org.matsim.core.controler.OutputDirectoryHierarchy;
import org.matsim.core.mobsim.qsim.AbstractQSimModule;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.core.utils.misc.OptionalTime;
import org.matsim.core.utils.misc.Time;
import org.matsim.pt.transitSchedule.api.Departure;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.VehicleType;

import ch.sbb.matsim.contrib.railsim.RailsimModule;
import ch.sbb.matsim.contrib.railsim.RailsimUtils;
import ch.sbb.matsim.contrib.railsim.config.RailsimConfigGroup;
import ch.sbb.matsim.contrib.railsim.config.RailsimConfigGroup.SpeedProfile;
import ch.sbb.matsim.contrib.railsim.qsimengine.PunctualProfileHook;
import ch.sbb.matsim.contrib.railsim.qsimengine.RailsimQSimModule;
import ch.sbb.matsim.contrib.railsim.qsimengine.deadlocks.DeadlockAvoidance;
import ch.sbb.matsim.contrib.railsim.qsimengine.deadlocks.NoDeadlockAvoidance;

/**
 * Parametrised Switzerland railway run for railsim {@code 2027.0-2026w45}.
 * <p>
 * Identical to the stock {@code src/main/java/ch/sbb/run/RunSwitzerlandRailway.java} unless a
 * {@code run.properties} file is placed next to the config file. Every property defaults to the stock
 * behaviour, with one unavoidable exception on railsim &gt;= 2026.0-2025w42: a train that enters the
 * opposite link of its current head link must be marked reversible ({@code railsimReversible}), otherwise
 * the engine throws {@code IllegalStateException: ... with a non-reversible train.} So the default
 * {@code reversibleSeconds=0} sets that attribute (reversal takes no time).
 * <p>
 * Property reference (see also swarm/engine/REPORT.md):
 * <pre>
 * vehicles            stock | realistic                     default stock
 * vehicles.file       CSV line_name_prefix,length_m,vmax_kmh,acceleration,deceleration   default none
 * reversibleSeconds   seconds a train needs to reverse; &lt; 0 = not reversible            default 0
 * speedProfile        maxSpeed | adaptive | justInTime | punctual   default maxSpeed
 * punctual.aim        aim this many seconds before the published arrival     default 0.25
 * punctual.minFactor  never slower than this factor of the current link's allowed speed   default 0.0
 * punctual.minSpeed   never slower than this absolute speed [m/s]           default 0.0
 * punctual.maxFactor  cruise at most this factor of the allowed speed       default 1.0
 * punctual.legs       on | off: write one row per leg to punctual.legsFile  default off
 * punctual.legsFile   diagnostics CSV                        default punctual_legs.csv
 * freeRun             off | on: every link capacity 256 + no deadlock avoidance (free-run diagnostic, D2) default off
 * pollInterval        railsim poll interval [s]              default: config value (10)
 * updateInterval      railsim update interval [s]            default: config value (10)
 * endTime             qsim end time, e.g. 36:00:00           default: config value (24:00:00)
 * resources           linkPair | block                       default linkPair
 * resources.file      block table for resources=block        default swarm/data/A05/segment_block.csv
 * boundaryCapacity    capacity of links with boundaryExtension=true   default 1
 * deadlockAvoidance   default | sectionLock | interlocking | none   default default
 * sectionLock.dedup   on | off   default on  (E2: one claim per physical track per section)
 * sectionLock.stopRule any | safe                               default safe
 * sectionLock.reversalStop off | standingArea | standingAreaAny  default off  (BN2)
 * sectionLock.retracePass on | off                               default on   (BN3: time-bounded retrace claim)
 * sectionLock.retracePassMargin  seconds                         default 120  (BN3)
 * clusterGap          metre gap between junction nodes that still counts as one station area (interlocking)   default 300
 * </pre>
 * Relative paths are resolved against the config directory and then against each of its ancestors.
 */
public final class RunSwitzerlandRailway {

	private static final Logger log = LogManager.getLogger(RunSwitzerlandRailway.class);

	private static final String PROPERTIES_FILE = "run.properties";
	private static final String LINK_ATTRIBUTE_OSM_WAY_ID = "osm:way:id";
	private static final String LINK_ATTRIBUTE_BOUNDARY = "boundaryExtension";
	private static final String DEFAULT_SEGMENT_BLOCK_FILE = "swarm/data/A05/segment_block.csv";

	/** Free-run diagnostic (D2): capacity high enough that no two trains can block each other. Never a solution. */
	static final int FREE_RUN_CAPACITY = 256;

	public static void main(String[] args) {

		String configFilename;
		if (args.length != 0) {
			configFilename = args[0];
		} else {
			configFilename = "matsim_input/config.xml";
		}

		Path configPath = Paths.get(configFilename).toAbsolutePath().normalize();
		Path configDir = configPath.getParent();

		Settings settings = Settings.read(configDir);
		settings.log();

		Config config = ConfigUtils.loadConfig(configFilename, new RailsimConfigGroup());
		config.controller().setOverwriteFileSetting(OutputDirectoryHierarchy.OverwriteFileSetting.deleteDirectoryIfExists);
		// railsim 2027 / MATSim 2027 defaults to zstd output (*.xml.zst); the shared bench (swarm/bench/sim.sh and
		// evaluate.py) expects gzip, and this only changes the file encoding, not the simulation.
		config.controller().setCompressionType(ControllerConfigGroup.CompressionType.gzip);

		RailsimConfigGroup railsim = ConfigUtils.addOrGetModule(config, RailsimConfigGroup.class);
		// railsim's "adaptive" aims 180 s early; "justInTime" and "punctual" are our own profiles and use SpeedProfile.custom.
		railsim.setSpeedProfile(switch (settings.speedProfile) {
			case "justInTime", "punctual" -> SpeedProfile.custom;
			default -> SpeedProfile.valueOf(settings.speedProfile);
		});
		if (settings.pollInterval.isDefined()) {
			railsim.setPollInterval(settings.pollInterval.seconds());
		}
		if (settings.updateInterval.isDefined()) {
			railsim.setUpdateInterval(settings.updateInterval.seconds());
		}
		if (settings.endTime.isDefined()) {
			config.qsim().setEndTime(settings.endTime.seconds());
		}

		log.info("effective railsim config: speedProfile={} pollInterval={} updateInterval={} networkModes={} accelerationDefault={} decelerationDefault={}",
				railsim.getSpeedProfile(), railsim.getPollInterval(), railsim.getUpdateInterval(), railsim.getNetworkModes(),
				railsim.getAccelerationDefault(), railsim.getDecelerationDefault());
		log.info("effective qsim config: startTime={} endTime={} simEndtimeInterpretation={}",
				config.qsim().getStartTime(), config.qsim().getEndTime(), config.qsim().getSimEndtimeInterpretation());

		Scenario scenario = ScenarioUtils.loadScenario(config);

		adjustNetwork(scenario, settings);
		adjustVehicles(scenario, settings);

		Controler controler = new Controler(scenario);

		controler.addOverridingModule(new RailsimModule());

		// Deadlock avoidance strategy. railsim's default binding (SimpleDeadlockAvoidance) claims the running direction of
		// every capacity-1 resource of a train's whole remaining route; SectionLockDeadlockAvoidance only locks the
		// sections of resources that the schedule really uses in both directions.
		if ("justInTime".equals(settings.speedProfile)) {
			final double margin = settings.jitMargin, minSpeed = settings.jitMinSpeed;
			log.info("speedProfile=justInTime: aim at published arrival - {} s, never slower than {} m/s", margin, minSpeed);
			controler.addOverridingQSimModule(new AbstractQSimModule() {
				@Override
				protected void configureQSim() {
					bind(ch.sbb.matsim.contrib.railsim.qsimengine.disposition.SpeedProfile.class)
							.toInstance(new JustInTimeSpeedProfile(margin, minSpeed));
				}
			});
		}
		final PunctualSpeedProfile punctual = "punctual".equals(settings.speedProfile) ? settings.punctualProfile(railsim) : null;
		// the engine hooks that keep the approved speed through the final approach are only active for this profile,
		// so speedProfile=maxSpeed stays byte-identical to base7
		PunctualProfileHook.setActive(punctual != null);
		// SWARM-FIX (FS2): the legacy "approvedSpeed = 0 if the reserved window ends within 10 m of the
		// stopping distance" rule makes trains brake to a standstill on the open line (measured 11.3 s on the
		// IC5 Neuchâtel -> Yverdon leg); off by default, engine.windowStop=legacy restores it.
		PunctualProfileHook.setWindowStopLegacy(settings.windowStopLegacy);
		log.info("engine.windowStop={} (legacy window-stop rule {})", settings.windowStopLegacy ? "legacy" : "off",
				settings.windowStopLegacy ? "on" : "off");
		// CH1: re-evaluate the target speed once the tail has left a link slower than the head link
		// (engine.speedRefresh=off restores the base8/src9 behaviour).
		PunctualProfileHook.setSpeedRefresh(settings.speedRefresh);
		log.info("engine.speedRefresh={}", settings.speedRefresh ? "on" : "off");
		// EN1: the time to reach a distance includes the cruise phase after a deceleration, and the head may only
		// enter a link it has really reached (engine.reachTime=off / engine.headGuard=off restore base12 behaviour).
		PunctualProfileHook.setReachTime(settings.reachTime);
		PunctualProfileHook.setHeadGuard(settings.headGuard);
		PunctualProfileHook.setReversalEnter(settings.reversalEnter);
		log.info("engine.reachTime={} engine.headGuard={} engine.reversalEnter={}", settings.reachTime ? "on" : "off",
				settings.headGuard ? "on" : "off", settings.reversalEnter ? "on" : "off");
		if (punctual != null) {
			log.info("speedProfile=punctual: aim = published arrival - {} s, minFactor={} minSpeed={} m/s maxFactor={} legs={}",
					settings.punctualAim, settings.punctualMinFactor, settings.punctualMinSpeed, settings.punctualMaxFactor,
					settings.punctualLegs);
			controler.addOverridingQSimModule(new AbstractQSimModule() {
				@Override
				protected void configureQSim() {
					bind(ch.sbb.matsim.contrib.railsim.qsimengine.disposition.SpeedProfile.class).toInstance(punctual);
				}
			});
		}
		String deadlockAvoidance = settings.freeRun ? Settings.DEADLOCK_NONE : settings.deadlockAvoidance;
		if (settings.freeRun) {
			log.info("freeRun=on: every link capacity {} and no deadlock avoidance - diagnostic only", FREE_RUN_CAPACITY);
		}
		switch (deadlockAvoidance) {
			case Settings.DEADLOCK_SECTION_LOCK -> {
				log.info("deadlockAvoidance=sectionLock: binding {} to {}", DeadlockAvoidance.class.getSimpleName(),
						SectionLockDeadlockAvoidance.class.getSimpleName());
				final String stopRule = settings.sectionLockStopRule;
				final boolean dedup = settings.sectionLockDedup;
				controler.addOverridingQSimModule(new AbstractQSimModule() {
					@Override
					protected void configureQSim() {
						bind(DeadlockAvoidance.class)
								.toProvider(() -> new SectionLockDeadlockAvoidance(scenario, stopRule, dedup))
								.asEagerSingleton();
					}
				});
			}
			case Settings.DEADLOCK_INTERLOCKING -> {
				log.info("deadlockAvoidance=interlocking: binding {} to {} (clusterGap={} m)", DeadlockAvoidance.class.getSimpleName(),
						InterlockingDeadlockAvoidance.class.getSimpleName(), settings.clusterGap);
				final double clusterGap = settings.clusterGap;
				controler.addOverridingQSimModule(new AbstractQSimModule() {
					@Override
					protected void configureQSim() {
						bind(DeadlockAvoidance.class)
								.toProvider(() -> new InterlockingDeadlockAvoidance(scenario, clusterGap))
								.asEagerSingleton();
					}
				});
			}
			case Settings.DEADLOCK_NONE -> {
				log.info("deadlockAvoidance=none: binding {} to {}", DeadlockAvoidance.class.getSimpleName(),
						NoDeadlockAvoidance.class.getSimpleName());
				controler.addOverridingQSimModule(new AbstractQSimModule() {
					@Override
					protected void configureQSim() {
						bind(DeadlockAvoidance.class).to(NoDeadlockAvoidance.class).asEagerSingleton();
					}
				});
			}
			default -> log.info("deadlockAvoidance=default: railsim's own SimpleDeadlockAvoidance is used");
		}

		// if you have other extensions that provide QSim components, call their configure-method here
		controler.configureQSimComponents(components -> new RailsimQSimModule().configure(components));

		controler.run();

		if (punctual != null) {
			punctual.close();
		}

		log.info("Done.");
	}

	// ------------------------------------------------------------------ network

	private static void adjustNetwork(Scenario scenario, Settings settings) {

		int boundaryLinks = 0;
		int blockLinks = 0;
		int pairLinks = 0;
		Map<String, String> pairIds = new HashMap<>();

		Map<String, String> segmentToBlock = null;
		if (settings.resources.equals(Settings.RESOURCES_BLOCK)) {
			Path blockFile = settings.resolveInput(settings.resourcesFile, "resources.file");
			segmentToBlock = readSegmentBlocks(blockFile);
			log.info("resources=block: {} segment rows from {}", segmentToBlock.size(), blockFile);
		}

		for (Link link : scenario.getNetwork().getLinks().values()) {

			boolean boundary = isBoundaryExtension(link);
			int capacity = settings.freeRun ? FREE_RUN_CAPACITY : (boundary ? settings.boundaryCapacity : 1);
			if (boundary) {
				boundaryLinks++;
			}
			RailsimUtils.setTrainCapacity(link, capacity);

			String resourceId = null;
			if (segmentToBlock != null) {
				resourceId = lookupBlock(segmentToBlock, link);
			}
			if (resourceId != null) {
				blockLinks++;
			} else {
				resourceId = linkPairResourceId(pairIds, link);
				pairLinks++;
			}
			RailsimUtils.setResourceId(link, resourceId);
		}

		log.info("network adjusted: {} links, resources={}, {} links on a block id, {} links on link-pair resources ({} distinct link-pair groups), {} boundaryExtension links with capacity {}",
				scenario.getNetwork().getLinks().size(), settings.resources, blockLinks, pairLinks, pairIds.size(),
				boundaryLinks, settings.boundaryCapacity);
	}

	/**
	 * Stock resource layout: a link and its reverse link(s) share one resource of capacity 1. Equivalently, all
	 * links between the same unordered pair of nodes share one resource (the stock loop also merges genuine
	 * parallel tracks between two nodes - that is the "before" state, not a modelling choice made here).
	 */
	private static String linkPairResourceId(Map<String, String> pairIds, Link link) {
		String from = link.getFromNode().getId().toString();
		String to = link.getToNode().getId().toString();
		String key = from.compareTo(to) <= 0 ? from + "|" + to : to + "|" + from;
		return pairIds.computeIfAbsent(key, k -> "resource_" + pairIds.size());
	}

	private static boolean isBoundaryExtension(Link link) {
		Object attr = link.getAttributes().getAttribute(LINK_ATTRIBUTE_BOUNDARY);
		return Boolean.TRUE.equals(attr) || (attr != null && Boolean.parseBoolean(attr.toString().trim()));
	}

	/**
	 * Reads {@code way_id,from_node,to_node,block_id} (one row per OSM segment) into a map keyed
	 * {@code way|from|to}.
	 */
	private static Map<String, String> readSegmentBlocks(Path file) {
		Map<String, String> map = new HashMap<>();
		try (BufferedReader in = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
			String line;
			boolean first = true;
			while ((line = in.readLine()) != null) {
				line = line.trim();
				if (line.isEmpty() || line.startsWith("#")) {
					continue;
				}
				String[] p = line.split(",");
				if (first) {
					first = false;
					if (!p[0].trim().matches("\\d+")) {
						continue; // header
					}
				}
				if (p.length != 4) {
					throw new IllegalArgumentException("invalid row in " + file + ": " + line);
				}
				map.put(p[0].trim() + "|" + p[1].trim() + "|" + p[2].trim(), p[3].trim());
			}
		} catch (IOException e) {
			throw new IllegalArgumentException("cannot read segment block file " + file, e);
		}
		return map;
	}

	/** Block id of this link, checking both directions of the OSM segment. Null if not in the table. */
	private static String lookupBlock(Map<String, String> segmentToBlock, Link link) {
		Object way = link.getAttributes().getAttribute(LINK_ATTRIBUTE_OSM_WAY_ID);
		if (way == null) {
			return null;
		}
		String from = link.getFromNode().getId().toString();
		String to = link.getToNode().getId().toString();
		String block = segmentToBlock.get(way + "|" + from + "|" + to);
		if (block == null) {
			block = segmentToBlock.get(way + "|" + to + "|" + from);
		}
		return block;
	}

	// ----------------------------------------------------------------- vehicles

	private static void adjustVehicles(Scenario scenario, Settings settings) {

		// remove the input vehicles and types; this run class defines its own fleet per line category
		for (Vehicle vehicle : new ArrayList<>(scenario.getTransitVehicles().getVehicles().values())) {
			scenario.getTransitVehicles().removeVehicle(vehicle.getId());
		}
		for (VehicleType vehicleType : new ArrayList<>(scenario.getTransitVehicles().getVehicleTypes().values())) {
			scenario.getTransitVehicles().removeVehicleType(vehicleType.getId());
		}

		List<VehicleSpec> table = settings.vehicles.equals(Settings.VEHICLES_REALISTIC)
				? VehicleSpec.realisticTable()
				: VehicleSpec.stockTable();
		List<VehicleSpec> fileSpecs = settings.vehiclesFile == null ? List.of() : readVehicleFile(settings.vehiclesFile);

		log.info("vehicles={}: {} category types, {} types from vehicles.file", settings.vehicles, table.size(), fileSpecs.size());
		for (VehicleSpec spec : table) {
			log.info("  category {}: length={} m vmax={} km/h acceleration={} deceleration={}",
					spec.prefix, spec.length, Math.round(spec.vmax * 3.6 * 100) / 100., spec.acceleration, spec.deceleration);
		}
		for (VehicleSpec spec : fileSpecs) {
			log.info("  file {}: length={} m vmax={} km/h acceleration={} deceleration={}",
					spec.prefix, spec.length, Math.round(spec.vmax * 3.6 * 100) / 100., spec.acceleration, spec.deceleration);
		}

		Map<String, VehicleType> types = new HashMap<>();

		for (TransitLine line : scenario.getTransitSchedule().getTransitLines().values()) {
			VehicleSpec spec = selectVehicleSpec(line.getName(), fileSpecs, table);
			VehicleType vehicleType = types.computeIfAbsent(spec.prefix, p -> createVehicleType(scenario, spec, settings));

			for (TransitRoute route : line.getRoutes().values()) {
				for (Departure dep : route.getDepartures().values()) {

					Id<Vehicle> newVehId = Id.createVehicleId("line_" + line.getName() + "_" + dep.getId());
					dep.setVehicleId(newVehId);
					Vehicle newVehicle = scenario.getTransitVehicles().getFactory().createVehicle(newVehId, vehicleType);
					scenario.getTransitVehicles().addVehicle(newVehicle);
				}
			}
		}

		log.info("created {} train vehicles of {} types", scenario.getTransitVehicles().getVehicles().size(), types.size());
	}

	private static VehicleType createVehicleType(Scenario scenario, VehicleSpec spec, Settings settings) {
		VehicleType type = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create(spec.prefix, VehicleType.class));
		type.getCapacity().setStandingRoom(1000);
		type.setLength(spec.length);
		type.setMaximumVelocity(spec.vmax);
		type.setNetworkMode("rail");
		if (spec.acceleration > 0) {
			RailsimUtils.setTrainAcceleration(type, spec.acceleration);
		}
		if (spec.deceleration > 0) {
			RailsimUtils.setTrainDeceleration(type, spec.deceleration);
		}
		// railsim >= 2026.0-2025w42 throws if a train enters the opposite link of its head link and is not
		// marked reversible. A negative reversibleSeconds means "not reversible" and leaves the attribute unset.
		if (settings.reversibleSeconds >= 0) {
			RailsimUtils.setTrainReversible(type, settings.reversibleSeconds);
		}
		scenario.getTransitVehicles().addVehicleType(type);
		return type;
	}

	/** Longest matching line-name prefix wins; file entries take precedence over the category table. */
	private static VehicleSpec selectVehicleSpec(String lineName, List<VehicleSpec> fileSpecs, List<VehicleSpec> table) {
		VehicleSpec spec = longestPrefix(fileSpecs, lineName);
		if (spec != null) {
			return spec;
		}
		spec = longestPrefix(table, lineName);
		if (spec != null) {
			return spec;
		}
		return table.stream().filter(s -> s.prefix.equals(VehicleSpec.FALLBACK)).findFirst().orElseThrow();
	}

	private static VehicleSpec longestPrefix(List<VehicleSpec> specs, String lineName) {
		VehicleSpec best = null;
		if (lineName == null) {
			return null;
		}
		for (VehicleSpec spec : specs) {
			if (lineName.startsWith(spec.prefix) && (best == null || spec.prefix.length() > best.prefix.length())) {
				best = spec;
			}
		}
		return best;
	}

	/** CSV {@code line_name_prefix,length_m,vmax_kmh,acceleration,deceleration}. */
	private static List<VehicleSpec> readVehicleFile(Path file) {
		List<VehicleSpec> specs = new ArrayList<>();
		try (BufferedReader in = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
			String line;
			int lineNo = 0;
			while ((line = in.readLine()) != null) {
				lineNo++;
				line = line.trim();
				if (line.isEmpty() || line.startsWith("#")) {
					continue;
				}
				String[] p = line.split(",");
				if (p.length != 5) {
					throw new IllegalArgumentException("invalid row " + lineNo + " in " + file + ": " + line);
				}
				try {
					specs.add(new VehicleSpec(p[0].trim(), Double.parseDouble(p[1].trim()),
							Double.parseDouble(p[2].trim()) / 3.6, Double.parseDouble(p[3].trim()),
							Double.parseDouble(p[4].trim())));
				} catch (NumberFormatException e) {
					if (lineNo == 1) {
						continue; // header
					}
					throw new IllegalArgumentException("invalid number in row " + lineNo + " of " + file + ": " + line, e);
				}
			}
		} catch (IOException e) {
			throw new IllegalArgumentException("cannot read vehicles file " + file, e);
		}
		if (specs.isEmpty()) {
			throw new IllegalArgumentException("no vehicle rows in " + file);
		}
		return specs;
	}

	/** One rolling-stock parameter set; {@code prefix} is both the line-name prefix and the vehicle type id. */
	private record VehicleSpec(String prefix, double length, double vmax, double acceleration, double deceleration) {

		static final String FALLBACK = "other";

		/** Values of the stock run class: 100 m/s (360 km/h) for IC/EC/IR, 100 km/h for RE/S/other, no railsim accel/decel. */
		static List<VehicleSpec> stockTable() {
			return List.of(
					new VehicleSpec("IC", 250., 100., 0., 0.),
					new VehicleSpec("EC", 250., 100., 0., 0.),
					new VehicleSpec("IR", 250., 100., 0., 0.),
					new VehicleSpec("RE", 300., 27.777, 0., 0.),
					new VehicleSpec("S", 150., 27.777, 0., 0.),
					new VehicleSpec(FALLBACK, 150., 27.777, 0., 0.));
		}

		/** Real Swiss rolling stock, swarm/research/R09-rolling-stock.md (constant/mean dynamics, ICE like IC). */
		static List<VehicleSpec> realisticTable() {
			return List.of(
					new VehicleSpec("IC", 205., 55.556, 0.45, 0.60),
					new VehicleSpec("EC", 195., 55.556, 0.45, 0.60),
					new VehicleSpec("IR", 150., 44.444, 0.60, 0.65),
					new VehicleSpec("RE", 120., 44.444, 0.70, 0.70),
					new VehicleSpec("S", 100., 38.889, 0.85, 0.85),
					new VehicleSpec(FALLBACK, 100., 38.889, 0.70, 0.70));
		}
	}

	// ---------------------------------------------------------------- settings

	/** All run.properties values, with defaults reproducing the stock logic. */
	private static final class Settings {

		static final String VEHICLES_STOCK = "stock";
		static final String VEHICLES_REALISTIC = "realistic";
		static final String RESOURCES_LINK_PAIR = "linkPair";
		static final String RESOURCES_BLOCK = "block";
		static final String DEADLOCK_DEFAULT = "default";
		static final String DEADLOCK_SECTION_LOCK = "sectionLock";
		static final String DEADLOCK_INTERLOCKING = "interlocking";
		static final String DEADLOCK_NONE = "none";

		final Path configDir;
		final String vehicles;
		final Path vehiclesFile;
		final double reversibleSeconds;
		final String speedProfile;
		final double jitMargin;
		final double jitMinSpeed;
		final double punctualAim;
		final double punctualMinFactor;
		final double punctualMinSpeed;
		final double punctualMaxFactor;
		final boolean punctualLegs;
		final Path punctualLegsFile;
		final boolean punctualTrace;
		final boolean freeRun;
		final boolean windowStopLegacy;
		final boolean speedRefresh;
		final boolean reachTime;
		final boolean headGuard;
		final boolean reversalEnter;
		final OptionalTime pollInterval;
		final OptionalTime updateInterval;
		final OptionalTime endTime;
		final String resources;
		final String resourcesFile;
		final int boundaryCapacity;
		final String deadlockAvoidance;
		final String sectionLockStopRule;
		final boolean sectionLockDedup;
		final double clusterGap;

		private Settings(Path configDir, Properties p) {
			this.configDir = configDir;
			this.vehicles = oneOf(p, "vehicles", VEHICLES_STOCK, VEHICLES_STOCK, VEHICLES_REALISTIC);
			this.vehiclesFile = optionalPath(p, "vehicles.file");
			this.reversibleSeconds = number(p, "reversibleSeconds", 0.);
			this.speedProfile = oneOf(p, "speedProfile", SpeedProfile.maxSpeed.name(),
					SpeedProfile.maxSpeed.name(), SpeedProfile.adaptive.name(), "justInTime", "punctual");
			this.jitMargin = number(p, "jitMargin", 30.);
			this.jitMinSpeed = number(p, "jitMinSpeed", 10.);
			this.punctualAim = number(p, "punctual.aim", PunctualSpeedProfile.DEFAULT_AIM);
			this.punctualMinFactor = number(p, "punctual.minFactor", PunctualSpeedProfile.DEFAULT_MIN_FACTOR);
			this.punctualMinSpeed = number(p, "punctual.minSpeed", PunctualSpeedProfile.DEFAULT_MIN_SPEED);
			this.punctualMaxFactor = number(p, "punctual.maxFactor", PunctualSpeedProfile.DEFAULT_MAX_FACTOR);
			this.punctualLegs = oneOf(p, "punctual.legs", "off", "off", "on").equals("on");
			this.punctualLegsFile = Paths.get(p.getProperty("punctual.legsFile", "punctual_legs.csv").trim());
			this.punctualTrace = oneOf(p, "punctual.trace", "off", "off", "on").equals("on");
			this.freeRun = oneOf(p, "freeRun", "off", "off", "on").equals("on");
			this.windowStopLegacy = oneOf(p, "engine.windowStop", "off", "off", "legacy").equals("legacy");
			this.speedRefresh = oneOf(p, "engine.speedRefresh", "on", "on", "off").equals("on");
			this.reachTime = oneOf(p, "engine.reachTime", "on", "on", "off").equals("on");
			this.headGuard = oneOf(p, "engine.headGuard", "on", "on", "off").equals("on");
			this.reversalEnter = oneOf(p, "engine.reversalEnter", "off", "on", "off").equals("on");
			this.pollInterval = optionalTime(p, "pollInterval");
			this.updateInterval = optionalTime(p, "updateInterval");
			this.endTime = optionalTime(p, "endTime");
			this.resources = oneOf(p, "resources", RESOURCES_LINK_PAIR, RESOURCES_LINK_PAIR, RESOURCES_BLOCK);
			this.resourcesFile = p.getProperty("resources.file", DEFAULT_SEGMENT_BLOCK_FILE).trim();
			this.boundaryCapacity = (int) number(p, "boundaryCapacity", 1.);
			this.deadlockAvoidance = oneOf(p, "deadlockAvoidance", DEADLOCK_DEFAULT,
					DEADLOCK_DEFAULT, DEADLOCK_SECTION_LOCK, DEADLOCK_INTERLOCKING, DEADLOCK_NONE);
			this.sectionLockStopRule = oneOf(p, "sectionLock.stopRule", SectionLockDeadlockAvoidance.STOP_RULE_SAFE,
					SectionLockDeadlockAvoidance.STOP_RULE_SAFE, SectionLockDeadlockAvoidance.STOP_RULE_ANY);
			this.sectionLockDedup = oneOf(p, "sectionLock.dedup", "on", "on", "off").equals("on");
			this.clusterGap = number(p, "clusterGap", InterlockingDeadlockAvoidance.DEFAULT_CLUSTER_GAP_M);
		}

		static Settings read(Path configDir) {
			Path file = configDir == null ? Paths.get(PROPERTIES_FILE) : configDir.resolve(PROPERTIES_FILE);
			Properties p = new Properties();
			if (Files.isRegularFile(file)) {
				try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
					p.load(reader);
				} catch (IOException e) {
					throw new IllegalArgumentException("cannot read " + file, e);
				}
				log.info("loaded settings from {}", file);
				Set<String> known = new LinkedHashSet<>(List.of("vehicles", "vehicles.file", "reversibleSeconds",
						"speedProfile", "jitMargin", "jitMinSpeed", "punctual.aim", "punctual.minFactor", "punctual.minSpeed",
						"punctual.maxFactor", "punctual.legs", "punctual.legsFile", "punctual.trace", "freeRun",
						"engine.windowStop", "engine.speedRefresh", "engine.reachTime", "engine.headGuard", "engine.reversalEnter",
						"pollInterval", "updateInterval", "endTime", "resources", "resources.file",
						"boundaryCapacity", "deadlockAvoidance", "clusterGap", "sectionLock.stopRule", "sectionLock.dedup",
						"sectionLock.meet", "sectionLock.retrace", "sectionLock.reversalStop",
						"sectionLock.retracePass", "sectionLock.retracePassMargin",
						"sectionLock.retracePassAhead", "sectionLock.retracePassAheadMaxM"));
				for (String key : p.stringPropertyNames()) {
					if (!known.contains(key)) {
						log.warn("unknown property '{}' in {} - ignored", key, file);
					}
				}
			} else {
				log.info("no {} next to the config file ({}) - using stock defaults", PROPERTIES_FILE, file);
			}
			return new Settings(configDir, p);
		}

		PunctualSpeedProfile punctualProfile(RailsimConfigGroup railsim) {
			return new PunctualSpeedProfile(railsim, punctualAim, punctualMinFactor, punctualMinSpeed, punctualMaxFactor,
					punctualLegs, punctualLegsFileResolved(), punctualTrace);
		}

		private Path punctualLegsFileResolved() {
			return punctualLegsFile.isAbsolute() ? punctualLegsFile
					: (configDir == null ? punctualLegsFile : configDir.resolve(punctualLegsFile)).normalize();
		}

		void log() {
			log.info("run.properties: vehicles={} vehicles.file={} reversibleSeconds={} speedProfile={} pollInterval={} updateInterval={} endTime={} resources={} resources.file={} boundaryCapacity={} deadlockAvoidance={} sectionLock.stopRule={} sectionLock.dedup={} clusterGap={}",
					vehicles, vehiclesFile == null ? "-" : vehiclesFile, reversibleSeconds, speedProfile,
					pollInterval.isDefined() ? pollInterval.seconds() : "config", updateInterval.isDefined() ? updateInterval.seconds() : "config",
					endTime.isDefined() ? Time.writeTime(endTime.seconds()) : "config", resources, resourcesFile, boundaryCapacity,
					deadlockAvoidance, sectionLockStopRule, sectionLockDedup ? "on" : "off", clusterGap);
			log.info("run.properties: speedProfile={} jitMargin={} jitMinSpeed={} punctual.aim={} punctual.minFactor={} "
							+ "punctual.minSpeed={} punctual.maxFactor={} punctual.legs={} freeRun={}",
					speedProfile, jitMargin, jitMinSpeed, punctualAim, punctualMinFactor, punctualMinSpeed,
					punctualMaxFactor, punctualLegs, freeRun);
			if (!deadlockAvoidance.equals(DEADLOCK_DEFAULT)) {
				log.info("deadlockAvoidance={} is an experiment binding", deadlockAvoidance);
			}
		}

		/**
		 * Resolve a relative input path against the config directory and then its ancestors (an experiment directory
		 * below the repository root can reference {@code swarm/data/A05/segment_block.csv}).
		 */
		Path resolveInput(String path, String property) {
			Path p = Paths.get(path);
			if (p.isAbsolute()) {
				if (!Files.isRegularFile(p)) {
					throw new IllegalArgumentException(property + "=" + path + " does not exist");
				}
				return p;
			}
			for (Path dir = configDir; dir != null; dir = dir.getParent()) {
				Path candidate = dir.resolve(path);
				if (Files.isRegularFile(candidate)) {
					return candidate;
				}
			}
			throw new IllegalArgumentException(property + "=" + path + " not found in " + configDir + " or any parent directory");
		}

		private Path optionalPath(Properties p, String key) {
			String value = p.getProperty(key, "").trim();
			return value.isEmpty() ? null : resolveInput(value, key);
		}

		private static String oneOf(Properties p, String key, String defaultValue, String... allowed) {
			String value = p.getProperty(key, defaultValue).trim();
			for (String a : allowed) {
				if (a.equals(value)) {
					return value;
				}
			}
			throw new IllegalArgumentException("property " + key + "=" + value + " is not one of " + List.of(allowed));
		}

		private static double number(Properties p, String key, double defaultValue) {
			String value = p.getProperty(key, "").trim();
			if (value.isEmpty()) {
				return defaultValue;
			}
			try {
				return Double.parseDouble(value);
			} catch (NumberFormatException e) {
				throw new IllegalArgumentException("property " + key + "=" + value + " is not a number", e);
			}
		}

		/** Optional seconds value; accepts plain seconds or MATSim time format (hh:mm:ss). */
		private static OptionalTime optionalTime(Properties p, String key) {
			String value = p.getProperty(key, "").trim();
			if (value.isEmpty()) {
				return OptionalTime.undefined();
			}
			try {
				return OptionalTime.defined(Double.parseDouble(value));
			} catch (NumberFormatException e) {
				return OptionalTime.defined(Time.parseTime(value));
			}
		}

		@Override
		public String toString() {
			return String.format(Locale.ROOT, "Settings[vehicles=%s, vehiclesFile=%s, reversibleSeconds=%s, speedProfile=%s]",
					vehicles, vehiclesFile, reversibleSeconds, speedProfile);
		}
	}
}
