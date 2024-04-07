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

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.Scenario;
import org.matsim.api.core.v01.network.Link;
import org.matsim.core.config.Config;
import org.matsim.core.config.ConfigUtils;
import org.matsim.core.controler.Controler;
import org.matsim.core.controler.OutputDirectoryHierarchy;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.pt.transitSchedule.api.Departure;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitRoute;
import org.matsim.vehicles.Vehicle;
import org.matsim.vehicles.VehicleType;

import ch.sbb.matsim.contrib.railsim.RailsimModule;
import ch.sbb.matsim.contrib.railsim.RailsimUtils;
import ch.sbb.matsim.contrib.railsim.qsimengine.RailsimQSimModule;

public final class RunSwitzerlandRailway {
    private static final Logger log = LogManager.getLogger(RunSwitzerlandRailway.class);

	public static void main(String[] args) {

		String configFilename;
		if (args.length != 0) {
			configFilename = args[0];
		} else {
			configFilename = "matsim_input/config.xml";
		}

		Config config = ConfigUtils.loadConfig(configFilename);
		config.controller().setOverwriteFileSetting(OutputDirectoryHierarchy.OverwriteFileSetting.deleteDirectoryIfExists);		
		
		Scenario scenario = ScenarioUtils.loadScenario(config);
		
		adjustVehicles(scenario);
		adjustNetwork(scenario);
			
		Controler controler = new Controler(scenario);

		controler.addOverridingModule(new RailsimModule());

		// if you have other extensions that provide QSim components, call their configure-method here
		controler.configureQSimComponents(components -> new RailsimQSimModule().configure(components));

		controler.run();
		
		log.info("Done.");
	}

	private static void adjustNetwork(Scenario scenario) {
		// set some network attributes which are used by the railsim engine
		int resourceCnt = 0;
		for (Link link : scenario.getNetwork().getLinks().values()) {
			RailsimUtils.setTrainCapacity(link, 1);
			
			RailsimUtils.setResourceId(link, "resource_" + resourceCnt);

			// find inverse links
			for (Link toNodeOutLink : link.getToNode().getOutLinks().values()) {
				if (toNodeOutLink.getToNode() == link.getFromNode()) {
					// inverse link
					RailsimUtils.setResourceId(toNodeOutLink, "resource_" + resourceCnt);
				}
			}
			resourceCnt++;
		}
	}

	private static void adjustVehicles(Scenario scenario) {
		// nicer vehicle names
		scenario.getTransitVehicles().getVehicles().values().forEach(vehicle -> {
			scenario.getTransitVehicles().removeVehicle(vehicle.getId());
		});
		scenario.getTransitVehicles().getVehicleTypes().values().forEach(vehicleType -> {
			scenario.getTransitVehicles().removeVehicleType(vehicleType.getId());
		});
		
		VehicleType vehTypeIC = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("IC", VehicleType.class));
		vehTypeIC.getCapacity().setStandingRoom(1000);
		vehTypeIC.setMaximumVelocity(100.);
		vehTypeIC.setLength(1000.);
		vehTypeIC.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeIC);
		
		VehicleType vehTypeEC = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("EC", VehicleType.class));
		vehTypeEC.getCapacity().setStandingRoom(1000);
		vehTypeEC.setMaximumVelocity(100.);
		vehTypeEC.setLength(1000.);
		vehTypeEC.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeEC);
		
		VehicleType vehTypeIR = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("IR", VehicleType.class));
		vehTypeIR.getCapacity().setStandingRoom(1000);
		vehTypeIR.setMaximumVelocity(100.);
		vehTypeIR.setLength(1000.);
		vehTypeIR.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeIR);
		
		VehicleType vehTypeRE = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("RE", VehicleType.class));
		vehTypeRE.getCapacity().setStandingRoom(1000);
		vehTypeRE.setMaximumVelocity(27.777);
		vehTypeRE.setLength(300.);
		vehTypeRE.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeRE);
		
		VehicleType vehTypeS = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("S", VehicleType.class));
		vehTypeS.getCapacity().setStandingRoom(1000);
		vehTypeS.setMaximumVelocity(27.777);
		vehTypeS.setLength(300.);
		vehTypeS.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeS);
		
		VehicleType vehTypeOther = scenario.getTransitVehicles().getFactory().createVehicleType(Id.create("other", VehicleType.class));
		vehTypeOther.getCapacity().setStandingRoom(1000);
		vehTypeOther.setMaximumVelocity(27.777);
		vehTypeOther.setLength(300.);
		vehTypeOther.setNetworkMode("rail");
		scenario.getTransitVehicles().addVehicleType(vehTypeOther);
		
		for (TransitLine line : scenario.getTransitSchedule().getTransitLines().values()) {
			for (TransitRoute route : line.getRoutes().values()) {
				for (Departure dep : route.getDepartures().values()) {
					
					VehicleType newVehicleType = null;
					if (line.getName().startsWith("IC")) {
						newVehicleType = vehTypeIC;

					} else if (line.getName().startsWith("EC")) {
						newVehicleType = vehTypeEC;

					} else if (line.getName().startsWith("IR")) {
						newVehicleType = vehTypeIR;

					} else if (line.getName().startsWith("RE")) {
						newVehicleType = vehTypeRE;

					} else if (line.getName().startsWith("S")) {
						newVehicleType = vehTypeS;

					} else {
						newVehicleType = vehTypeOther;
					}
					
					Id<Vehicle> newVehId = Id.createVehicleId("line_" + line.getName() + "_" + dep.getId());
					dep.setVehicleId(newVehId);
					Vehicle newVehicle = scenario.getTransitVehicles().getFactory().createVehicle(newVehId, newVehicleType);
					scenario.getTransitVehicles().addVehicle(newVehicle);	
				}
			}
		}
	}

}
