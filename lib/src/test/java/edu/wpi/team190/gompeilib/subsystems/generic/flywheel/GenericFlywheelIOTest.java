package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.Test;

public class GenericFlywheelIOTest {
  @Test
  public void testGenericFlywheelIODefaults() {
    GenericFlywheelIO io = new GenericFlywheelIO() {};
    GenericFlywheelIO.GenericFlywheelIOInputs inputs =
        new GenericFlywheelIO.GenericFlywheelIOInputs();

    // Call defaults - should not throw
    io.updateInputs(inputs);
    io.setVoltageGoal(Units.Volts.of(1.0));
    io.setCurrentGoal(Units.Amps.of(1.0));
    io.setVelocityGoal(Units.RadiansPerSecond.of(1.0));
    io.setVelocityGoal(Units.RadiansPerSecond.of(1.0), Units.Amps.of(0.5));
    io.setNeutralControl();
    assertFalse(io.atVoltageGoal(Units.Volts.of(1.0)));
    assertFalse(io.atCurrentGoal(Units.Amps.of(1.0)));
    assertFalse(io.atVelocityGoal(Units.RadiansPerSecond.of(1.0)));
    io.updateGains(Gains.fromDoubles().withPrefix("test").build(), GainSlot.ZERO);
    io.updateConstraints(
        AngularVelocityConstraints.fromMeasures()
            .withPrefix("test")
            .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
            .withMaxVelocity(Units.RadiansPerSecond.of(100.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(200.0))
            .build());

    // Inputs defaults
    assertEquals(0.0, inputs.position.getDegrees());
    assertEquals(0.0, inputs.velocity.in(Units.RadiansPerSecond));
    assertEquals(0, inputs.appliedVolts.length);
    assertEquals(0, inputs.supplyCurrentAmps.length);
    assertEquals(0, inputs.torqueCurrentAmps.length);
    assertEquals(0, inputs.temperatureCelsius.length);
    assertEquals(0.0, inputs.velocityGoal.in(Units.RadiansPerSecond));
    assertEquals(0.0, inputs.velocitySetpoint.in(Units.RadiansPerSecond));
    assertEquals(0.0, inputs.velocityError.in(Units.RadiansPerSecond));
    assertNull(inputs.gainSlot);
  }
}
