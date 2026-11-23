/* *********************************************************************** *
 * project: org.matsim.*
 * *********************************************************************** *
 *                                                                         *
 * copyright       : (C) 2016 by the members listed in the COPYING,        *
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

package ch.sbb.prepare;

import java.io.File;
import java.net.MalformedURLException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.ConfigWriter;
import org.matsim.core.utils.collections.CollectionUtils;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitRouteStop;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt2matsim.config.OsmConverterConfigGroup;
import org.matsim.pt2matsim.run.CreateDefaultOsmConfig;
import org.matsim.pt2matsim.run.Gtfs2TransitSchedule;
import org.matsim.pt2matsim.run.Osm2MultimodalNetwork;
import org.matsim.pt2matsim.tools.ScheduleTools;
import org.matsim.utils.gis.shp2matsim.ShpGeometryUtils;

/**
 * Step 1 of the pipeline: the original open data into the two MATSim inputs the rest of the chain
 * consumes (see {@code docs/pipeline.md}).
 *
 * <ol>
 *   <li>GTFS → unmapped MATSim transit schedule, filtered to rail lines named IC/IR/EC/RE whose stops
 *       are in Switzerland → {@code <out>/schedule_gtfs_filtered.xml.gz} (1231 routes, 2541 departures);</li>
 *   <li>OSM railway extract → MATSim network (one link per OSM way segment and direction,
 *       {@code keepPaths=true}, LV95 metres, {@code osm:way:id} on every link)
 *       → {@code <out>/network_osm.xml.gz} (665 238 links).</li>
 * </ol>
 *
 * <b>What changed against the upstream class</b> (round 2, agent P2):
 * <ul>
 *   <li>the old pt2matsim shortest-path mapping ({@code PublicTransitMapper}) and the trimming to the
 *       Olten polygon are <b>gone</b>: the tracks are assigned by {@code ch.sbb.prepare.router.TrackRouter},
 *       which does not use a shortest path on the OSM network but a cost over the legal track graph;</li>
 *   <li>the class is parametrised by case: {@code java ... GenerateRailsimInput <olten|switzerland>}
 *       (or {@code --gtfs/--osm/--out} for other paths).  Both cases are simulated over the whole of
 *       Switzerland, so both consume the <i>same</i> two files; the Olten case is the cut of the filtered
 *       schedule to the Olten area (polygon + reviewed route list), made by {@code prepare/build_tables.py}
 *       because that cut is a reviewed data decision, not a mapping;</li>
 *   <li>the intermediate config files and the mapped network are no longer written.</li>
 * </ul>
 */
public final class GenerateRailsimInput {
    private static final Logger log = LogManager.getLogger(GenerateRailsimInput.class);

	// input osm and gtfs file (defaults; overridable from the command line)
	private static String inputOsmFile = "original_data/osm/switzerland_railways.osm";
	private static String inputGtfsFile = "original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip";

	/** the area a route must touch (a stop inside it) to be kept; the whole simulated area */
	private static String removeLinesOutsideThisArea = "original_data/shp/switzerland/switzerland.shp";

	// filter by line name prefix
	private static final Set<String> transitLineNamePrefixesToKeep = CollectionUtils.stringToSet("IC,IR,EC,RE");

	private static final String EPSG2056 = "EPSG:2056";

	// output directory of the two files the router and the schedule cutter consume
	private static String matsimInputTmp = "matsim_input/tmp/";
	private static String caseName = "switzerland";

	private static final String PT2MATSIM_OSM_CONVERTER_CONFIG_DEFAULT = "pt2matsim_osm_converter_config_default.xml";
	private static final String PT2MATSIM_OSM_CONVERTER_CONFIG = "pt2matsim_osm_converter_config.xml";

	private static String scheduleGtfs() { return matsimInputTmp + "schedule_gtfs.xml.gz"; }
	private static String scheduleGtfsFiltered() { return matsimInputTmp + "schedule_gtfs_filtered.xml.gz"; }
	private static String networkOsm() { return matsimInputTmp + "network_osm.xml.gz"; }
	private static String vehiclesGtfs() { return matsimInputTmp + "vehicles_gtfs.xml.gz"; }

	public static void main(String[] args) throws MalformedURLException {

		if (args.length == 1 && (args[0].equals("--help") || args[0].equals("-h"))) {
			System.out.println("usage: GenerateRailsimInput [olten|switzerland] [--gtfs F] [--osm F] [--out DIR] [--area SHP]");
			return;
		}
		parseArgs(args);
		log.info("GenerateRailsimInput: case={} gtfs={} osm={} out={}", caseName, inputGtfsFile, inputOsmFile, matsimInputTmp);
		if ("olten".equals(caseName)) {
			log.info("case olten: this step produces the whole-Switzerland inputs the Olten trains are simulated on; "
					+ "the Olten case schedule (boundary polygon + reviewed route list) is cut by prepare/build_tables.py");
		}

		new File(matsimInputTmp).mkdirs();

		// 1. Convert the gtfs schedule to an unmapped transit schedule and filter it
		gtfsToSchedule();
		filterSchedule();

		// 2. Convert the osm extract to a MATSim network (raw, unmapped - the router maps the schedule)
		createOsmConfigFile(matsimInputTmp + PT2MATSIM_OSM_CONVERTER_CONFIG);
		Osm2MultimodalNetwork.main(new String[]{matsimInputTmp + PT2MATSIM_OSM_CONVERTER_CONFIG});

		log.info("done: {} and {}", scheduleGtfsFiltered(), networkOsm());
	}

	private static void parseArgs(String[] args) {
		boolean caseSeen = false;
		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
				case "--gtfs" -> inputGtfsFile = args[++i];
				case "--osm" -> inputOsmFile = args[++i];
				case "--out" -> {
					String directory = args[++i];
					matsimInputTmp = directory.endsWith("/") ? directory : directory + "/";
				}
				case "--area" -> removeLinesOutsideThisArea = args[++i];
				default -> {
					if (!caseSeen && (args[i].equals("olten") || args[i].equals("switzerland") || args[i].equals("ch"))) {
						caseName = args[i].equals("ch") ? "switzerland" : args[i];
						caseSeen = true;
					} else {
						throw new IllegalArgumentException("unknown argument " + args[i]
								+ " (usage: GenerateRailsimInput [olten|switzerland] [--gtfs F] [--osm F] [--out DIR])");
					}
				}
			}
		}
	}

	private static boolean routeHasStopInArea(TransitRoute route, List<PreparedGeometry> geometries) {
		 for (TransitRouteStop stop : route.getStops()) {
	        	if (ShpGeometryUtils.isCoordInPreparedGeometries(stop.getStopFacility().getCoord(), geometries)) {
	                return true;
	            }
	        }
			return false;
	}

	public static void filterSchedule() throws MalformedURLException {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(scheduleGtfs());

		// remove non-rail transit lines, e.g. buses, light-rail, ...
		for(TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
			for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
				if(!transitRoute.getTransportMode().equals("rail")) {
					transitLine.removeRoute(transitRoute);
				}
			}
		}

		// remove lines outside the provided area, e.g. Switzerland
		if (removeLinesOutsideThisArea != null) {
			List<PreparedGeometry> geometries = ShpGeometryUtils.loadPreparedGeometries(new File(removeLinesOutsideThisArea).toURI().toURL());

			for (TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
				for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
					if (routeHasStopInArea(transitRoute, geometries)) {
						// keep
					} else {
						// remove
						transitLine.removeRoute(transitRoute);
					}
				}
			}
		}

		if (transitLineNamePrefixesToKeep == null || transitLineNamePrefixesToKeep.isEmpty()) {
			// do not filter by line name prefix
		} else {
			for(TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
				boolean lineToKeep = false;
				for (String prefix : transitLineNamePrefixesToKeep) {
					if (transitLine.getName().startsWith(prefix)) {
						lineToKeep = true;
						break;
					}
				}

				if(lineToKeep) {
					// keep
				} else {
					// remove
					for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
						transitLine.removeRoute(transitRoute);
					}
				}
			}
		}

		org.matsim.pt2matsim.tools.debug.ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleTools.writeTransitSchedule(schedule, scheduleGtfsFiltered());
	}

	/**
	 * 	1. A GTFS or HAFAS Schedule or a OSM map with information on public transport
	 * 	has to be converted to an unmapped MATSim Transit Schedule.
	 *
	 * 	Here the GTFS schedule of Switzerland is converted (day with most trips, LV95 coordinates).
	 */
	public static void gtfsToSchedule() {
		String[] gtfsConverterArgs = new String[]{
				// [0] gtfs zip file
				inputGtfsFile,
				// [1] which service ids should be used. One of the following:
				//		dayWithMostTrips, date in the format yyyymmdd, , dayWithMostServices, all
				"dayWithMostTrips",
				// [2] the output coordinate system. Use WGS84 for no transformation.
				EPSG2056,
				// [3] output transit schedule file
				scheduleGtfs(),
				// [4] output default vehicles file (optional)
				vehiclesGtfs(),
		};
		Gtfs2TransitSchedule.main(gtfsConverterArgs);
	}

	/**
	 * 	2. A MATSim network of the area is required. If no such network is already available,
	 * the PT2MATSim package provides the possibility to use OSM-maps as data-input.
	 *
	 */
	public static void createOsmConfigFile(String configFile) {
		String defaultFile = matsimInputTmp + PT2MATSIM_OSM_CONVERTER_CONFIG_DEFAULT;
		CreateDefaultOsmConfig.main(new String[]{defaultFile});
		Config osmConverterConfig = ConfigUtils.loadConfig(defaultFile, new OsmConverterConfigGroup());
		OsmConverterConfigGroup osmConfig = ConfigUtils.addOrGetModule(osmConverterConfig, OsmConverterConfigGroup.class);
		osmConfig.setOsmFile(inputOsmFile);
		osmConfig.setOutputCoordinateSystem(EPSG2056);
		osmConfig.setOutputNetworkFile(networkOsm());
		osmConfig.setKeepPaths(true);
		new ConfigWriter(osmConverterConfig).write(configFile);
	}

}
