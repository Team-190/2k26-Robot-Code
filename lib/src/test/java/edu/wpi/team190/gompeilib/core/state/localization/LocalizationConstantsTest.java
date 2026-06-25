package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class LocalizationConstantsTest {
  @Test
  public void testConstants() {
    new LocalizationConstants(); // Cover constructor
    assertEquals(0.1, LocalizationConstants.XY_STDDEV_COEFFICIENT);
    assertEquals(1.2, LocalizationConstants.XY_STDDEV_DISTANCE_EXPONENT);
  }
}
