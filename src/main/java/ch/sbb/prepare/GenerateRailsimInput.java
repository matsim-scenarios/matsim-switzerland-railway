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

import org.locationtech.jts.geom.prep.PreparedGeometry;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.config.ConfigWriter;
import org.matsim.core.utils.collections.CollectionUtils;
import org.matsim.pt.transitSchedule.api.Departure;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt2matsim.config.OsmConverterConfigGroup;
import org.matsim.pt2matsim.config.PublicTransitMappingConfigGroup;
import org.matsim.pt2matsim.run.CreateDefaultOsmConfig;
import org.matsim.pt2matsim.run.CreateDefaultPTMapperConfig;
import org.matsim.pt2matsim.run.Gtfs2TransitSchedule;
import org.matsim.pt2matsim.run.Osm2MultimodalNetwork;
import org.matsim.pt2matsim.run.PublicTransitMapper;
import org.matsim.pt2matsim.tools.NetworkTools;
import org.matsim.pt2matsim.tools.ScheduleTools;
import org.matsim.pt2matsim.tools.debug.ScheduleCleaner;
import org.matsim.utils.gis.shp2matsim.ShpGeometryUtils;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.Vehicles;

public final class GenerateRailsimInput {

	// input osm and gtfs file
	private static final String INPUT_OSM_FILE = "original_data/osm/switzerland_railways.osm";
	private static final String INPUT_GTFS_FILE = "original_data/gtfs/gtfs_fp2024_2024-03-20_04-15.zip";
	
	// optional: trim the schedule
    // private static final String areaShpFileForTrimming = null;
    private static final String areaShpFileForTrimming = "original_data/shp/olten/olten.shp";
    // private static final String areaShpFileForTrimming = "original_data/shp/switzerland/switzerland.shp";
    
    // optional: filter by line name prefix
    private static final Set<String> transitLineNamePrefixesToKeep = CollectionUtils.stringToSet("IC");
    // private static final Set<String> transitLineNamePrefixesToKeep = null;

	private static final String EPSG2056 = "EPSG:2056";
	
	// matsim input files to write
	private static final String MATSIM_INPUT = "matsim_input/";
	private static final String VEHICLES_FINAL = MATSIM_INPUT + "transitVehicles.xml.gz";
	private static final String NETWORK_FINAL = MATSIM_INPUT + "transitNetwork.xml.gz";
	private static final String SCHEDULE_FINAL = MATSIM_INPUT + "transitSchedule.xml.gz";
	
	// intermediate files
	
	private static final String MATSIM_INPUT_TMP = "matsim_input/tmp/";
    
	private static final String PT2MATSIM_OSM_CONVERTER_CONFIG_DEFAULT = MATSIM_INPUT_TMP + "pt2matsim_osm_converter_config_default.xml";
	private static final String PT2MATSIM_OSM_CONVERTER_CONFIG = MATSIM_INPUT_TMP + "pt2matsim_osm_converter_config.xml";
	private static final String PT2MATSIM_MAPPER_CONFIG_DEFAULT = MATSIM_INPUT_TMP + "pt2matsim_mapper_config_default.xml";
	private static final String PT2MATSIM_MAPPER_CONFIG = MATSIM_INPUT_TMP + "pt2matsim_mapper_config_adjusted.xml";

	private static final String SCHEDULE_GTFS = MATSIM_INPUT_TMP + "schedule_gtfs.xml.gz";
	private static final String SCHEDULE_GTFS_FILTERED = MATSIM_INPUT_TMP + "schedule_gtfs_filtered.xml.gz";
	private static final String SCHEDULE_GTFS_FILTERED_MAPPED = MATSIM_INPUT_TMP + "schedule_gtfs_filtered_mapped.xml.gz";
	private static final String SCHEDULE_GTFS_FILTERED_MAPPED_TRIMMED = MATSIM_INPUT_TMP + "schedule_gtfs_filtered_mapped_trimmed.xml.gz";

	private static final String NETWORK_OSM = MATSIM_INPUT_TMP + "network_osm.xml.gz";
	private static final String NETWORK_OSM_MAPPED = MATSIM_INPUT_TMP + "network_osm_mapped.xml.gz";
	
	private static final String VEHICLES_GTFS = MATSIM_INPUT_TMP + "vehicles_gtfs.xml.gz";
	
	public static void main(String[] args) throws MalformedURLException {

		new File(MATSIM_INPUT).mkdirs();
		new File(MATSIM_INPUT_TMP).mkdirs();

		// 1. Convert a gtfs schedule to an unmapped transit schedule
		gtfsToSchedule();
		filterSchedule();

		// 2. Convert an osm map to a MATSim network
		createOsmConfigFile( PT2MATSIM_OSM_CONVERTER_CONFIG );
		Osm2MultimodalNetwork.main(new String[]{ PT2MATSIM_OSM_CONVERTER_CONFIG });

		// 3. Map the schedule onto the network
		createMapperConfigFile(PT2MATSIM_MAPPER_CONFIG);
		PublicTransitMapper.main(new String[]{PT2MATSIM_MAPPER_CONFIG});
		
		trimSchedule();
		writeFinalFiles();
	}

	private static void writeFinalFiles() {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(SCHEDULE_GTFS_FILTERED_MAPPED_TRIMMED);
		Network network = NetworkTools.readNetwork(NETWORK_OSM_MAPPED);
		Vehicles vehicles = ScheduleTools.readVehicles(VEHICLES_GTFS);
				
		for (TransitLine line : schedule.getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {
				for (Departure dep : route.getDepartures().values()) {
					Id<Vehicle> vehicleId = dep.getVehicleId();
					Vehicle vehicle = vehicles.getVehicles().get(vehicleId);
					vehicle.getAttributes().putAttribute("line_name", line.getName());
				}
			}
		}
				
		ScheduleTools.writeTransitSchedule(schedule, SCHEDULE_FINAL);	
		NetworkTools.writeNetwork(network, NETWORK_FINAL);
		ScheduleTools.writeVehicles(vehicles, VEHICLES_FINAL);
	}

	private static void trimSchedule() throws MalformedURLException {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(SCHEDULE_GTFS_FILTERED_MAPPED);
		Network network = NetworkTools.readNetwork(NETWORK_OSM_MAPPED);
		
		if (areaShpFileForTrimming != null) {
			List<PreparedGeometry> geometries = ShpGeometryUtils.loadPreparedGeometries(new File(areaShpFileForTrimming).toURI().toURL());

			for (TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
				for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
					if (routeHasLinkInArea(transitRoute, network, geometries)) {
						// keep
					} else {
						// remove
						transitLine.removeRoute(transitRoute);
					}
				}

			}
		}
		
		ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleTools.writeTransitSchedule(schedule, SCHEDULE_GTFS_FILTERED_MAPPED_TRIMMED);		
	}
	
	private static boolean routeHasLinkInArea(TransitRoute route, Network network, List<PreparedGeometry> geometries) {
        for (Id<Link> id : route.getRoute().getLinkIds()) {
        	Link link = network.getLinks().get(id);
        	if (ShpGeometryUtils.isCoordInPreparedGeometries(link.getCoord(), geometries)) {
                return true;
            }
        }   
		return false;	
	}
	
	public static void filterSchedule() {
		TransitSchedule schedule = ScheduleTools.readTransitSchedule(SCHEDULE_GTFS);

		// remove non-rail transit lines, e.g. buses, light-rail, ...
		for(TransitLine transitLine : new HashSet<>(schedule.getTransitLines().values())) {
			for(TransitRoute transitRoute : new HashSet<>(transitLine.getRoutes().values())) {
				if(!transitRoute.getTransportMode().equals("rail")) {
					transitLine.removeRoute(transitRoute);
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
		
		ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		ScheduleTools.writeTransitSchedule(schedule, SCHEDULE_GTFS_FILTERED);
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
				INPUT_GTFS_FILE,
				// [1] which service ids should be used. One of the following:
				//		dayWithMostTrips, date in the format yyyymmdd, , dayWithMostServices, all
				"dayWithMostTrips",
				// [2] the output coordinate system. Use WGS84 for no transformation.
				EPSG2056,
				// [3] output transit schedule file
				SCHEDULE_GTFS,
				// [4] output default vehicles file (optional)
				VEHICLES_GTFS,
		};
		Gtfs2TransitSchedule.main(gtfsConverterArgs);
	}

	/**
	 * 2. A MATSim network of the area is required. If no such network is already available,
	 * the PT2MATSim package provides the possibility to use OSM-maps as data-input.
	 *
	 */
	public static void createOsmConfigFile(String configFile) {
		CreateDefaultOsmConfig.main(new String[]{PT2MATSIM_OSM_CONVERTER_CONFIG_DEFAULT});
		Config osmConverterConfig = ConfigUtils.loadConfig(
				PT2MATSIM_OSM_CONVERTER_CONFIG_DEFAULT,
				new OsmConverterConfigGroup());
		OsmConverterConfigGroup osmConfig = ConfigUtils.addOrGetModule(osmConverterConfig, OsmConverterConfigGroup.class);
		osmConfig.setOsmFile(INPUT_OSM_FILE);
		osmConfig.setOutputCoordinateSystem(EPSG2056);
		osmConfig.setOutputNetworkFile(NETWORK_OSM);
		osmConfig.setKeepPaths(true);
		new ConfigWriter(osmConverterConfig).write(configFile);
	}

	/**
	 * 	3. The core of the PT2MATSim-package is the mapping process of the schedule to the network.
	 */
	public static void createMapperConfigFile(String configFile) {
		CreateDefaultPTMapperConfig.main(new String[]{ PT2MATSIM_MAPPER_CONFIG_DEFAULT});
		Config config = ConfigUtils.loadConfig(
				PT2MATSIM_MAPPER_CONFIG_DEFAULT,
				PublicTransitMappingConfigGroup.createDefaultConfig());
		PublicTransitMappingConfigGroup ptmConfig = ConfigUtils.addOrGetModule(config, PublicTransitMappingConfigGroup.class);

		ptmConfig.setInputNetworkFile(NETWORK_OSM);
		ptmConfig.setOutputNetworkFile(NETWORK_OSM_MAPPED);
		ptmConfig.setOutputScheduleFile(SCHEDULE_GTFS_FILTERED_MAPPED);
		ptmConfig.setInputScheduleFile(SCHEDULE_GTFS_FILTERED);
//		ptmConfig.setModesToKeepOnCleanUp(CollectionUtils.stringToSet("rail"));
		ptmConfig.setScheduleFreespeedModes(CollectionUtils.stringToSet("rail"));
		new ConfigWriter(config).write(configFile);
		
	}

}