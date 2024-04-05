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
import org.matsim.pt2matsim.config.PublicTransitMappingConfigGroup;
import org.matsim.pt2matsim.run.CreateDefaultOsmConfig;
import org.matsim.pt2matsim.run.CreateDefaultPTMapperConfig;
import org.matsim.pt2matsim.run.Gtfs2TransitSchedule;
import org.matsim.pt2matsim.run.Osm2MultimodalNetwork;
import org.matsim.pt2matsim.run.PublicTransitMapper;
import org.matsim.pt2matsim.tools.ScheduleTools;
import org.matsim.pt2matsim.tools.debug.ScheduleCleaner;
import org.matsim.utils.gis.shp2matsim.ShpGeometryUtils;

public final class GenerateRailsimInput {

	private static final String ORIGINAL_DATA = "original_data/";
	private static final String MATSIM_INPUT = "matsim_input/";

	private static final String inputOSM = ORIGINAL_DATA + "osm/";
	private static final String inputGTFS = ORIGINAL_DATA + "gtfs/";

	private static final String EPSG2056 = "EPSG:2056";
	
    private static final String areaShpFile = ORIGINAL_DATA + "shp/olten/olten.shp";

	public static void main(String[] args) throws MalformedURLException {
		prepare();

		// 1. Convert a gtfs schedule to an unmapped transit schedule
		gtfsToSchedule();
		filterSchedule();
		trimSchedule();

		// 2. Convert an osm map to a MATSim network
		// create a config file (or adjust an existing one by hand)
		createOsmConfigFile( MATSIM_INPUT + "OsmConverterConfig.xml" );
		// Convert the OSM file using the config
		Osm2MultimodalNetwork.main(new String[]{ MATSIM_INPUT + "OsmConverterConfig.xml" });

		// 3. Map the schedule onto the network
		// create a config file (or adjust an existing one by hand)
		createMapperConfigFile(MATSIM_INPUT + "MapperConfigAdjusted.xml");
		// Map the schedule using the config
		PublicTransitMapper.main(new String[]{MATSIM_INPUT + "MapperConfigAdjusted.xml"});
	}

	private static void trimSchedule() throws MalformedURLException {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(MATSIM_INPUT + "schedule_gtfs_filtered.xml.gz");

		List<PreparedGeometry> geometries = ShpGeometryUtils.loadPreparedGeometries(new File(areaShpFile).toURI().toURL());

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
		
		ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleTools.writeTransitSchedule(schedule, MATSIM_INPUT + "schedule_gtfs_filtered_trimmed.xml.gz");		
	}
	
	private static boolean routeHasStopInArea(TransitRoute route, List<PreparedGeometry> geometries) {
        for (TransitRouteStop stop : route.getStops()) {
        	if (ShpGeometryUtils.isCoordInPreparedGeometries(stop.getStopFacility().getCoord(), geometries)) {
                return true;
            }
        }   
		return false;	
	}

	/** Create output folder if not existing **/
	public static void prepare() {
		new File(MATSIM_INPUT).mkdirs();
	}
	
	public static void filterSchedule() {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(MATSIM_INPUT + "schedule_gtfs.xml.gz");

		for(TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
			for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
				if(!transitRoute.getTransportMode().equals("rail")) {
					transitLine.removeRoute(transitRoute);
				}
			}
		}
		
		for(TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
			if(transitLine.getName().startsWith("IC") ||
					transitLine.getName().startsWith("IR") ||
					transitLine.getName().startsWith("RE")) {
				// keep
				
			} else {
				// remove
				
				for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
					transitLine.removeRoute(transitRoute);
				}
			}
		}
		ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleTools.writeTransitSchedule(schedule, MATSIM_INPUT + "schedule_gtfs_filtered.xml.gz");
	}


	/**
	 * 	1. A GTFS or HAFAS Schedule or a OSM map with information on public transport
	 * 	has to be converted to an unmapped MATSim Transit Schedule.
	 *
	 * 	Here as a first example, the GTFS-schedule of GrandRiverTransit, Waterloo-Area, Canada, is converted.
	 */
	public static void gtfsToSchedule() {
		String[] gtfsConverterArgs = new String[]{
				// [0] gtfs zip file
				inputGTFS + "gtfs_fp2024_2024-03-20_04-15.zip",
				// [1] which service ids should be used. One of the following:
				//		dayWithMostTrips, date in the format yyyymmdd, , dayWithMostServices, all
				"dayWithMostTrips",
				// [2] the output coordinate system. Use WGS84 for no transformation.
				EPSG2056,
				// [3] output transit schedule file
				MATSIM_INPUT + "schedule_gtfs.xml.gz",
				// [4] output default vehicles file (optional)
				MATSIM_INPUT + "vehicles_gtfs.xml.gz",
		};
		Gtfs2TransitSchedule.main(gtfsConverterArgs);
	}

	/**
	 * 2. A MATSim network of the area is required. If no such network is already available,
	 * the PT2MATSim package provides the possibility to use OSM-maps as data-input.
	 *
	 */
	public static void createOsmConfigFile(String configFile) {
		// Create a default createOsmConfigFile-Config:
		CreateDefaultOsmConfig.main(new String[]{MATSIM_INPUT + "OsmConverterConfigDefault.xml"});

		// Open the createOsmConfigFile Config and set the parameters to the required values
		// (usually done manually by opening the config with a simple editor)
		Config osmConverterConfig = ConfigUtils.loadConfig(
				MATSIM_INPUT + "OsmConverterConfigDefault.xml",
				new OsmConverterConfigGroup());

		OsmConverterConfigGroup osmConfig = ConfigUtils.addOrGetModule(osmConverterConfig, OsmConverterConfigGroup.class);
		osmConfig.setOsmFile(inputOSM + "switzerland_railways.osm");
		osmConfig.setOutputCoordinateSystem(EPSG2056);
		osmConfig.setOutputNetworkFile(MATSIM_INPUT + "network_osm.xml.gz");
		osmConfig.setKeepPaths(true);

		// Save the createOsmConfigFile config (usually done manually)
		new ConfigWriter(osmConverterConfig).write(configFile);
	}

	/**
	 * 	3. The core of the PT2MATSim-package is the mapping process of the schedule to the network.
	 */
	public static void createMapperConfigFile(String configFile) {
		// Create a mapping config:
		CreateDefaultPTMapperConfig.main(new String[]{ MATSIM_INPUT + "MapperConfigDefault.xml"});
		// Open the mapping config and set the parameters to the required values
		// (usually done manually by opening the config with a simple editor)
		Config config = ConfigUtils.loadConfig(
				MATSIM_INPUT + "MapperConfigDefault.xml",
				PublicTransitMappingConfigGroup.createDefaultConfig());
		PublicTransitMappingConfigGroup ptmConfig = ConfigUtils.addOrGetModule(config, PublicTransitMappingConfigGroup.class);

		ptmConfig.setInputNetworkFile(MATSIM_INPUT + "network_osm.xml.gz");
		ptmConfig.setOutputNetworkFile(MATSIM_INPUT + "network_multimodal_final.xml.gz");
		ptmConfig.setOutputScheduleFile(MATSIM_INPUT + "schedule_final.xml.gz");
		ptmConfig.setOutputStreetNetworkFile(MATSIM_INPUT + "network_streets_final.xml.gz");
		ptmConfig.setInputScheduleFile(MATSIM_INPUT + "schedule_gtfs_filtered_trimmed.xml.gz");
		ptmConfig.setScheduleFreespeedModes(CollectionUtils.stringToSet("rail, light_rail"));
		// Save the mapping config
		// (usually done manually)
		new ConfigWriter(config).write(configFile);
	}

}