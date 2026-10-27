package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.Test;
import org.wpilib.units.Units;

public class ExtensionIOTest {
  @Test
  public void testExtensionIODefaults() {
    ExtensionIO io = new ExtensionIO() {};
    ExtensionIO.ExtensionIOInputs inputs = new ExtensionIO.ExtensionIOInputs();

    // Call defaults - should not throw
    io.updateInputs(inputs);
    io.setVoltageGoal(Units.Volts.of(1.0));
    io.setPositionGoal(Units.Meters.of(1.0));
    assertFalse(io.atVoltageGoal(Units.Volts.of(1.0)));
    assertFalse(io.atPositionGoal(Units.Meters.of(1.0)));
    io.setPosition(Units.Meters.of(0.5));
    io.setGainSlot(GainSlot.ZERO);
    io.updateGains(Gains.builder().withPrefix("test").build(), GainSlot.ZERO);
    io.updateConstraints(
        LinearConstraints.builder()
            .withPrefix("test")
            .withGoalTolerance(Units.Meters.of(0.01))
            .withMaxVelocity(Units.MetersPerSecond.of(1.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
            .build());

    // Inputs defaults
    assertEquals(0.0, inputs.position.in(Units.Meters));
    assertEquals(0.0, inputs.velocity.in(Units.MetersPerSecond));
    assertEquals(0.0, inputs.acceleration.in(Units.MetersPerSecondPerSecond));
    assertEquals(0, inputs.appliedVolts.length);
    assertEquals(0, inputs.supplyCurrentAmps.length);
    assertEquals(0, inputs.torqueCurrentAmps.length);
    assertEquals(0, inputs.temperatureCelsius.length);
    assertEquals(0.0, inputs.positionGoalMeters.in(Units.Meters));
    assertEquals(0.0, inputs.positionSetpointMeters.in(Units.Meters));
    assertEquals(0.0, inputs.positionErrorMeters.in(Units.Meters));
    assertNull(inputs.gainSlot);
  }
}
