package edu.wpi.team190.gompeilib.core.utility.sysid;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class CustomUnitsTest {
  @Test
  public void testCustomUnits() {
    new CustomUnits(); // Constructor coverage
    assertNotNull(CustomUnits.ampsPerSecond);
    assertNotNull(CustomUnits.voltsPerSecond);
  }
}
