package ch.sbb.prepare.platform;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;

/**
 * Copies OSM rail stop positions ({@code railway=stop} / {@code public_transport=stop_position} with {@code train=yes})
 * onto the matching MATSim network nodes as node attributes. Requires a network converted with pt2matsim and
 * {@code keepPaths=true}, so that network node ids equal OSM node ids.
 * <p>
 * Upstream, this belongs into pt2matsim's {@code OsmMultimodalNetworkConverter} (behind a config switch).
 */
public final class OsmStopPositionNetworkEnricher {

	private static final Logger log = LogManager.getLogger(OsmStopPositionNetworkEnricher.class);

	public static final String ATTR_STOP_POSITION = "osm:node:railway:stop";
	public static final String ATTR_LOCAL_REF = "osm:node:local_ref";
	public static final String ATTR_NAME = "osm:node:name";

	private OsmStopPositionNetworkEnricher() {
	}

	public static int enrich(Network network, String osmFile) {
		Map<String, Map<String, String>> stopNodes = readRailStopPositions(osmFile);
		int matched = 0;
		for (Map.Entry<String, Map<String, String>> e : stopNodes.entrySet()) {
			Node node = network.getNodes().get(Id.createNodeId(e.getKey()));
			if (node == null) {
				continue;
			}
			node.getAttributes().putAttribute(ATTR_STOP_POSITION, true);
			String localRef = e.getValue().get("local_ref");
			if (localRef != null) {
				node.getAttributes().putAttribute(ATTR_LOCAL_REF, localRef);
			}
			String name = e.getValue().get("name");
			if (name != null) {
				node.getAttributes().putAttribute(ATTR_NAME, name);
			}
			matched++;
		}
		log.info("OSM rail stop positions: {} read, {} found in network.", stopNodes.size(), matched);
		return matched;
	}

	static Map<String, Map<String, String>> readRailStopPositions(String osmFile) {
		Map<String, Map<String, String>> result = new HashMap<>();
		XMLInputFactory factory = XMLInputFactory.newInstance();
		try (InputStream in = new BufferedInputStream(new FileInputStream(osmFile))) {
			XMLStreamReader reader = factory.createXMLStreamReader(in);
			String nodeId = null;
			Map<String, String> tags = null;
			while (reader.hasNext()) {
				int event = reader.next();
				if (event == XMLStreamConstants.START_ELEMENT) {
					String name = reader.getLocalName();
					if ("node".equals(name)) {
						nodeId = reader.getAttributeValue(null, "id");
						tags = new HashMap<>();
					} else if ("tag".equals(name) && tags != null) {
						tags.put(reader.getAttributeValue(null, "k"), reader.getAttributeValue(null, "v"));
					}
				} else if (event == XMLStreamConstants.END_ELEMENT && "node".equals(reader.getLocalName())) {
					if (tags != null && isRailStopPosition(tags)) {
						result.put(nodeId, tags);
					}
					nodeId = null;
					tags = null;
				}
			}
			reader.close();
		} catch (IOException | XMLStreamException e) {
			throw new RuntimeException("Could not read OSM file " + osmFile, e);
		}
		return result;
	}

	static boolean isRailStopPosition(Map<String, String> tags) {
		if ("stop".equals(tags.get("railway"))) {
			return true;
		}
		return "stop_position".equals(tags.get("public_transport")) && "yes".equals(tags.get("train"));
	}
}
