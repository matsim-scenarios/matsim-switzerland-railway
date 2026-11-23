package ch.sbb.prepare.platform;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class GeneratePlatformMappedInputTest {

	@Test
	public void platformCodeIsLastPartOfSwissStopId() {
		assertEquals("7", GeneratePlatformMappedInput.platformCodeFromStopId("8500218:0:7"));
		assertEquals("12AB", GeneratePlatformMappedInput.platformCodeFromStopId("8503000:0:12AB"));
	}

	@Test
	public void noPlatformCodeWithoutPlatformPart() {
		assertNull(GeneratePlatformMappedInput.platformCodeFromStopId("8500218"));
		assertNull(GeneratePlatformMappedInput.platformCodeFromStopId("8500218:0"));
		assertNull(GeneratePlatformMappedInput.platformCodeFromStopId("8500218:0: "));
	}
}
