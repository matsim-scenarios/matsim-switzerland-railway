/* *********************************************************************** *
 * project: org.matsim.*
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2026 by the members listed in the COPYING,        *
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

package ch.sbb.prepare.platform;

import java.io.File;
import java.util.HashSet;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigGroup;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.ConfigWriter;
import org.matsim.core.network.algorithms.MultimodalNetworkCleaner;
import org.matsim.core.utils.collections.CollectionUtils;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt.transitSchedule.api.TransitStopFacility;
import org.matsim.pt2matsim.config.OsmConverterConfigGroup;
import org.matsim.pt2matsim.config.PublicTransitMappingConfigGroup;
import org.matsim.pt2matsim.config.PublicTransitMappingConfigGroup.PlatformStopLink;
import org.matsim.pt2matsim.config.PublicTransitMappingStrings;
import org.matsim.pt2matsim.config.TransportModeParameterSet;
import org.matsim.pt2matsim.mapping.PTMapper;
import org.matsim.pt2matsim.run.CreateDefaultOsmConfig;
import org.matsim.pt2matsim.run.CreateDefaultPTMapperConfig;
import org.matsim.pt2matsim.run.Osm2MultimodalNetwork;
import org.matsim.pt2matsim.tools.NetworkTools;
import org.matsim.pt2matsim.tools.ScheduleTools;
import org.matsim.pt2matsim.tools.debug.ScheduleCleaner;
import org.matsim.vehicles.Vehicles;

/**
 * Optional alternative to steps 2-15 of {@code docs/pipeline.md}: map the filtered schedule of step 1 with
 * pt2matsim (&ge; 26.12) onto the OSM network so that every rail stop lies on the OSM stop position
 * ({@code railway=stop}, {@code local_ref}) of its GTFS platform, and write a runnable scenario for
 * {@code ch.sbb.run.RunSwitzerlandRailway} (see issue #16).
 * <p>
 * Unlike the {@code TrackRouter} chain, this uses pt2matsim's shortest-path mapping and none of the reviewed
 * corrections in {@code prepare/}; it reads the outputs of step 1 but changes nothing the default chain uses.
 * <p>
 * Reads {@code <tmp>/schedule_gtfs_filtered.xml.gz} and {@code <tmp>/vehicles_gtfs.xml.gz} (written by
 * {@code GenerateRailsimInput}) and the OSM extract; writes {@code <out>/transitNetwork.xml.gz},
 * {@code <out>/transitSchedule.xml.gz} and {@code <out>/transitVehicles.xml.gz} (consumed by
 * {@code matsim_input/platform/config.xml}) and the intermediate files to {@code <out>/tmp/}.
 *
 * <pre>
 * java -cp matsim-switzerland-railway-&lt;ver&gt;.jar ch.sbb.prepare.platform.GeneratePlatformMappedInput [--osm F] [--tmp DIR] [--out DIR]
 * </pre>
 */
public final class GeneratePlatformMappedInput {
	private static final Logger log = LogManager.getLogger(GeneratePlatformMappedInput.class);

	private static final String EPSG2056 = "EPSG:2056";
	private static final String RAIL = "rail";

	/** narrow gauge and tram tracks are not reachable from the standard gauge network and attract wrong stop links (e.g. Liestal) */
	private static final Set<String> RAILWAY_TYPES_TO_REMOVE = Set.of("light_rail", "tram");

	/** the train head halts at most this far behind the OSM stop position (or earlier at the next switch) */
	private static final double MAX_STOP_LINK_EXTENSION = 200.0;

	private static final String USAGE = "usage: GeneratePlatformMappedInput [--osm F] [--tmp DIR] [--out DIR]";

	private String inputOsmFile = "original_data/osm/switzerland_railways.osm";
	private String stepOneDir = "matsim_input/tmp/";
	private String outDir = "matsim_input/platform/";

	public static void main(String[] args) throws Exception {
		if (args.length == 1 && (args[0].equals("--help") || args[0].equals("-h"))) {
			System.out.println(USAGE);
			return;
		}
		GeneratePlatformMappedInput generator = new GeneratePlatformMappedInput();
		generator.parseArgs(args);
		generator.run();
	}

	private void parseArgs(String[] args) {
		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
				case "--osm" -> inputOsmFile = args[++i];
				case "--tmp" -> stepOneDir = withSlash(args[++i]);
				case "--out" -> outDir = withSlash(args[++i]);
				default -> throw new IllegalArgumentException("unknown argument " + args[i] + " (" + USAGE + ")");
			}
		}
	}

	private static String withSlash(String directory) {
		return directory.endsWith("/") ? directory : directory + "/";
	}

	private String tmp(String file) {
		return outDir + "tmp/" + file;
	}

	private void run() throws Exception {
		log.info("GeneratePlatformMappedInput: osm={} step-1 dir={} out={}", inputOsmFile, stepOneDir, outDir);
		new File(outDir + "tmp/").mkdirs();

		String networkOsm = tmp("network_osm_platform.xml.gz");
		convertOsm(networkOsm);
		removeRailwayTypes(networkOsm, RAILWAY_TYPES_TO_REMOVE);

		String mapperConfigFile = tmp("pt2matsim_mapper_config.xml");
		writeMapperConfig(mapperConfigFile, networkOsm);
		mapScheduleToNetwork(mapperConfigFile);

		writeRunnableScenario();
		log.info("done: {}", outDir);
	}

	private void convertOsm(String networkFile) {
		String defaultFile = tmp("pt2matsim_osm_converter_config_default.xml");
		String configFile = tmp("pt2matsim_osm_converter_config.xml");
		CreateDefaultOsmConfig.main(new String[]{defaultFile});
		Config config = ConfigUtils.loadConfig(defaultFile, new OsmConverterConfigGroup());
		OsmConverterConfigGroup osmConfig = ConfigUtils.addOrGetModule(config, OsmConverterConfigGroup.class);
		osmConfig.setOsmFile(inputOsmFile);
		osmConfig.setOutputCoordinateSystem(EPSG2056);
		osmConfig.setOutputNetworkFile(networkFile);
		osmConfig.setKeepPaths(true);
		osmConfig.keepRailStopPositions = true;
		new ConfigWriter(config).write(configFile);
		Osm2MultimodalNetwork.main(new String[]{configFile});
	}

	private static void removeRailwayTypes(String networkFile, Set<String> railwayTypes) {
		Network network = NetworkTools.readNetwork(networkFile);
		network.getLinks().values().stream()
				.filter(l -> railwayTypes.contains(l.getAttributes().getAttribute("osm:way:railway")))
				.map(Link::getId).toList()
				.forEach(network::removeLink);
		new MultimodalNetworkCleaner(network).run(Set.of(RAIL));
		NetworkTools.writeNetwork(network, networkFile);
	}

	private void writeMapperConfig(String configFile, String networkFile) {
		String defaultFile = tmp("pt2matsim_mapper_config_default.xml");
		CreateDefaultPTMapperConfig.main(new String[]{defaultFile});
		Config config = ConfigUtils.loadConfig(defaultFile, PublicTransitMappingConfigGroup.createDefaultConfig());
		PublicTransitMappingConfigGroup ptmConfig = ConfigUtils.addOrGetModule(config, PublicTransitMappingConfigGroup.class);

		ptmConfig.setInputNetworkFile(networkFile);
		ptmConfig.setInputScheduleFile(stepOneDir + "schedule_gtfs_filtered.xml.gz");
		ptmConfig.setOutputNetworkFile(tmp("network_mapped.xml.gz"));
		ptmConfig.setOutputScheduleFile(tmp("schedule_mapped.xml.gz"));
		ptmConfig.setModesToKeepOnCleanUp(CollectionUtils.stringToSet(RAIL));
		ptmConfig.setScheduleFreespeedModes(CollectionUtils.stringToSet(RAIL));
		// default is rail -> rail,light_rail, which makes the pseudo routing very slow
		for (ConfigGroup set : ptmConfig.getParameterSets(TransportModeParameterSet.GROUP_NAME)) {
			TransportModeParameterSet assignment = (TransportModeParameterSet) set;
			if (RAIL.equals(assignment.getScheduleMode())) {
				assignment.setNetworkModes(Set.of(RAIL));
			}
		}
		ptmConfig.setPlatformModes(Set.of(RAIL));
		ptmConfig.setPlatformStopLink(PlatformStopLink.outLink);
		ptmConfig.setPlatformStopLinkMaxExtension(MAX_STOP_LINK_EXTENSION);
		ptmConfig.setPlatformRemoveTerminalReversal(true);
		new ConfigWriter(config).write(configFile);
	}

	private static void mapScheduleToNetwork(String mapperConfigFile) throws Exception {
		PublicTransitMappingConfigGroup config = PublicTransitMappingConfigGroup.loadConfig(mapperConfigFile);
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(config.getInputScheduleFile());
		Network network = NetworkTools.readNetwork(config.getInputNetworkFile());

		for (TransitStopFacility facility : schedule.getFacilities().values()) {
			String platformCode = platformCodeFromStopId(facility.getId().toString());
			if (platformCode != null) {
				facility.getAttributes().putAttribute(PublicTransitMappingStrings.STOP_FACILITY_ATTRIBUTE_PLATFORM_CODE, platformCode);
			}
		}

		new PTMapper(schedule, network).run(config);

		ScheduleTools.writeTransitSchedule(schedule, config.getOutputScheduleFile());
		NetworkTools.writeNetwork(network, config.getOutputNetworkFile());
	}

	/**
	 * @return the platform code of a Swiss GTFS stop id {@code <UIC>:0:<platform>}, or {@code null}
	 */
	static String platformCodeFromStopId(String stopId) {
		String[] parts = stopId.split(":");
		if (parts.length < 3) {
			return null;
		}
		String code = parts[parts.length - 1].trim();
		return code.isEmpty() ? null : code;
	}

	private void writeRunnableScenario() {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(tmp("schedule_mapped.xml.gz"));
		Network network = NetworkTools.readNetwork(tmp("network_mapped.xml.gz"));
		Vehicles vehicles = ScheduleTools.readVehicles(stepOneDir + "vehicles_gtfs.xml.gz");

		for (TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
			if (transitLine.getRoutes().isEmpty()) {
				schedule.removeTransitLine(transitLine);
			}
		}
		ScheduleCleaner.cleanVehicles(schedule, vehicles);
		ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleCleaner.removeNotUsedMinimalTransferTimes(schedule);

		ScheduleTools.writeTransitSchedule(schedule, outDir + "transitSchedule.xml.gz");
		NetworkTools.writeNetwork(network, outDir + "transitNetwork.xml.gz");
		ScheduleTools.writeVehicles(vehicles, outDir + "transitVehicles.xml.gz");
	}
}
