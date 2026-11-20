package ch.sbb.prepare.platform;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import org.matsim.api.core.v01.network.Link;

/**
 * Track traversal helpers: follow a track through switches preferring the same OSM way, then the straightest
 * continuation. U-turns are never taken.
 */
final class PlatformTrack {

	private static final String ATTR_WAY_ID = "osm:way:id";
	private static final int MAX_LINKS = 500;

	private PlatformTrack() {
	}

	/**
	 * Distance available behind the end of {@code stopLink} (including {@code stopLink}) until a dead end, capped at
	 * {@code max}.
	 */
	static double spaceBehind(Link stopLink, Set<String> networkModes, double max) {
		Link current = stopLink;
		double travelled = stopLink.getLength();
		int n = 0;
		while (travelled < max && n++ < MAX_LINKS) {
			Link prev = previousLink(current, networkModes);
			if (prev == null) {
				return travelled;
			}
			current = prev;
			travelled += prev.getLength();
		}
		return Math.min(travelled, max);
	}

	/**
	 * Links following {@code from} until their accumulated length reaches {@code distance}, or until a dead end.
	 */
	static List<Link> forward(Link from, Set<String> networkModes, double distance) {
		List<Link> links = new ArrayList<>();
		Link current = from;
		double travelled = 0;
		int n = 0;
		while (travelled < distance && n++ < MAX_LINKS) {
			Link next = nextLink(current, networkModes);
			if (next == null) {
				break;
			}
			links.add(next);
			travelled += next.getLength();
			current = next;
		}
		return links;
	}

	static Link nextLink(Link current, Set<String> networkModes) {
		Link best = null;
		double bestScore = Double.MAX_VALUE;
		for (Link l : current.getToNode().getOutLinks().values()) {
			if (!allowed(l, networkModes) || l.getToNode() == current.getFromNode()) {
				continue;
			}
			double score = score(current, l);
			if (score < bestScore) {
				best = l;
				bestScore = score;
			}
		}
		return best;
	}

	static Link previousLink(Link current, Set<String> networkModes) {
		Link best = null;
		double bestScore = Double.MAX_VALUE;
		for (Link l : current.getFromNode().getInLinks().values()) {
			if (!allowed(l, networkModes) || l.getFromNode() == current.getToNode()) {
				continue;
			}
			double score = score(l, current);
			if (score < bestScore) {
				best = l;
				bestScore = score;
			}
		}
		return best;
	}

	static boolean allowed(Link link, Set<String> networkModes) {
		for (String mode : link.getAllowedModes()) {
			if (networkModes.contains(mode)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Lower is better: same OSM way first, then the straightest continuation.
	 */
	private static double score(Link from, Link to) {
		Object wayFrom = from.getAttributes().getAttribute(ATTR_WAY_ID);
		boolean sameWay = wayFrom != null && Objects.equals(wayFrom, to.getAttributes().getAttribute(ATTR_WAY_ID));
		return (sameWay ? 0 : 10) + Math.abs(angleDiff(from, to));
	}

	private static double angleDiff(Link a, Link b) {
		double a1 = Math.atan2(a.getToNode().getCoord().getY() - a.getFromNode().getCoord().getY(),
				a.getToNode().getCoord().getX() - a.getFromNode().getCoord().getX());
		double a2 = Math.atan2(b.getToNode().getCoord().getY() - b.getFromNode().getCoord().getY(),
				b.getToNode().getCoord().getX() - b.getFromNode().getCoord().getX());
		double d = a2 - a1;
		while (d > Math.PI) d -= 2 * Math.PI;
		while (d < -Math.PI) d += 2 * Math.PI;
		return d;
	}
}
