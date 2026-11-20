package ch.sbb.prepare.platform;

import java.util.function.Consumer;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.network.Network;
import org.matsim.pt.transitSchedule.api.TransitLine;
import org.matsim.pt.transitSchedule.api.TransitSchedule;
import org.matsim.pt.utils.TransitScheduleValidator;
import org.matsim.pt2matsim.config.PublicTransitMappingConfigGroup;
import org.matsim.pt2matsim.config.PublicTransitMappingStrings;
import org.matsim.pt2matsim.mapping.Progress;
import org.matsim.pt2matsim.mapping.PseudoRouting;
import org.matsim.pt2matsim.mapping.PseudoRoutingImpl;
import org.matsim.pt2matsim.mapping.linkCandidateCreation.LinkCandidateCreator;
import org.matsim.pt2matsim.mapping.networkRouter.ScheduleRoutersFactory;
import org.matsim.pt2matsim.mapping.networkRouter.ScheduleRoutersStandard;
import org.matsim.pt2matsim.mapping.pseudoRouter.PseudoSchedule;
import org.matsim.pt2matsim.mapping.pseudoRouter.PseudoScheduleImpl;
import org.matsim.pt2matsim.tools.NetworkTools;
import org.matsim.pt2matsim.tools.PTMapperTools;
import org.matsim.pt2matsim.tools.ScheduleTools;
import org.matsim.pt2matsim.tools.debug.ScheduleCleaner;

/**
 * Same as pt2matsim's {@code PTMapper.run(config, linkCandidateCreator, null)} (version 24.4), but without
 * {@code PTMapperTools.pullChildStopFacilitiesTogether}: that step moves every stop link by link towards the stop
 * facility coordinate (= station centre for Swiss GTFS) and would undo the platform placement of
 * {@link RailPlatformLinkCandidateCreator}.
 * <p>
 * Upstream, this would be a config switch in pt2matsim's {@code PTMapper} instead of a copy.
 */
public final class PlatformAwarePTMapper {

	private static final Logger log = LogManager.getLogger(PlatformAwarePTMapper.class);

	private PlatformAwarePTMapper() {
	}

	public static void run(TransitSchedule schedule, Network network, PublicTransitMappingConfigGroup config,
			LinkCandidateCreator linkCandidateCreator, Consumer<TransitSchedule> afterRouting) {
		if (ScheduleTools.idsContainChildStopString(schedule)) {
			throw new RuntimeException("Some stopFacility ids contain the string \"" + PublicTransitMappingStrings.SUFFIX_CHILD_STOP_FACILITIES + "\"! Schedule cannot be mapped.");
		}
		PTMapperTools.setLogLevels();

		ScheduleRoutersFactory routersFactory = new ScheduleRoutersStandard.Factory(schedule, network,
				config.getTransportModeAssignment(), config.getTravelCostType(), config.getRoutingWithCandidateDistance());

		int nRoutes = schedule.getTransitLines().values().stream().mapToInt(l -> l.getRoutes().size()).sum();
		Progress progress = new Progress(nRoutes, "Calculating pseudoTransitRoutes ...");
		int numThreads = config.getNumOfThreads();
		PseudoRouting[] routings = new PseudoRouting[numThreads];
		for (int i = 0; i < numThreads; i++) {
			routings[i] = new PseudoRoutingImpl(routersFactory, linkCandidateCreator, config.getMaxTravelCostFactor(), progress);
		}
		int thr = 0;
		for (TransitLine line : schedule.getTransitLines().values()) {
			routings[thr++ % numThreads].addTransitLineToQueue(line);
		}
		Thread[] threads = new Thread[numThreads];
		for (int i = 0; i < numThreads; i++) {
			threads[i] = new Thread(routings[i]);
			threads[i].start();
		}
		for (Thread thread : threads) {
			try {
				thread.join();
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new RuntimeException(e);
			}
		}

		PseudoSchedule pseudoSchedule = new PseudoScheduleImpl();
		for (PseudoRouting routing : routings) {
			routing.addArtificialLinks(network);
			pseudoSchedule.mergePseudoSchedule(routing.getPseudoSchedule());
		}
		pseudoSchedule.createFacilitiesAndLinkSequences(schedule);

		log.info("Skipping pullChildStopFacilitiesTogether to keep rail stops on their platform position.");
		afterRouting.accept(schedule);

		PTMapperTools.addTransfersForChildStopFacilities(schedule);

		NetworkTools.resetLinkLength(network, PublicTransitMappingStrings.ARTIFICIAL_LINK_MODE);
		ScheduleTools.setFreeSpeedBasedOnSchedule(network, schedule, config.getScheduleFreespeedModes());
		ScheduleCleaner.removeNotUsedTransitLinks(schedule, network, config.getModesToKeepOnCleanUp(), true);
		if (config.getRemoveNotUsedStopFacilities()) {
			ScheduleCleaner.removeNotUsedStopFacilities(schedule);
		}
		ScheduleCleaner.removeNotUsedMinimalTransferTimes(schedule);
		ScheduleTools.assignScheduleModesToLinks(schedule, network);

		TransitScheduleValidator.ValidationResult result = TransitScheduleValidator.validateAll(schedule, network);
		log.info("Schedule valid: {} (errors: {}, warnings: {})", result.isValid(), result.getErrors().size(), result.getWarnings().size());
		result.getErrors().forEach(log::warn);

		long nArtificial = network.getLinks().values().stream()
				.filter(l -> l.getAllowedModes().contains(PublicTransitMappingStrings.ARTIFICIAL_LINK_MODE)).count();
		log.info("Artificial links created: {}", nArtificial);
	}
}
