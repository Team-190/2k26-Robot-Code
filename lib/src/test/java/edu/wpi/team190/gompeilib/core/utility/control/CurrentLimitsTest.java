package edu.wpi.team190.gompeilib.core.utility.control;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Current;

public class CurrentLimitsTest {
  @Test
  public void testCurrentLimits() {
    Current supply = Units.Amps.of(40);
    Current stator = Units.Amps.of(80);

    CurrentLimits limits1 =
        CurrentLimits.builder()
            .withSupplyCurrentLimit(supply)
            .withStatorCurrentLimit(stator)
            .build();

    assertEquals(supply, limits1.supplyCurrentLimit());
    assertEquals(stator, limits1.statorCurrentLimit());

    CurrentLimits limits2 =
        CurrentLimits.fromDoubles()
            .withSupplyCurrentLimit(40.0)
            .withStatorCurrentLimit(80.0)
            .build();

    assertEquals(40.0, limits2.supplyCurrentLimit().in(Units.Amps));
    assertEquals(80.0, limits2.statorCurrentLimit().in(Units.Amps));
  }
}
