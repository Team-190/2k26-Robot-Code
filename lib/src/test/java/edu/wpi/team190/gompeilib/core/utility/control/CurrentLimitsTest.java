package edu.wpi.team190.gompeilib.core.utility.control;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import org.junit.jupiter.api.Test;

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
