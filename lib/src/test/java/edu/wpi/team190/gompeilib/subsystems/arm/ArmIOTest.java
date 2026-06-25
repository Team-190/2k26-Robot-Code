package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.Test;

public class ArmIOTest {
  @Test
  public void testArmIODefaults() {
    ArmIO io = new ArmIO() {};
    ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

    // Call defaults - should not throw
    io.updateInputs(inputs);
    io.setVoltageGoal(Units.Volts.of(1.0));
    io.setPositionGoal(Rotation2d.fromDegrees(10));
    assertFalse(io.atVoltageGoal(Units.Volts.of(1.0)));
    assertFalse(io.atPositionGoal(Rotation2d.fromDegrees(10)));
    io.setPosition(Rotation2d.fromDegrees(5));
    io.setGainSlot(GainSlot.ZERO);
    io.updateGains(Gains.builder().build(), GainSlot.ZERO);
    io.updateConstraints(AngularPositionConstraints.builder().build());

    // Inputs defaults
    assertEquals(0.0, inputs.position.getDegrees());
    assertEquals(0.0, inputs.velocity.in(Units.RadiansPerSecond));
    assertEquals(0.0, inputs.acceleration.in(Units.RadiansPerSecondPerSecond));
    assertEquals(0, inputs.appliedVolts.length);
    assertEquals(0, inputs.supplyCurrentAmps.length);
    assertEquals(0, inputs.torqueCurrentAmps.length);
    assertEquals(0, inputs.temperatureCelsius.length);
    assertEquals(0.0, inputs.positionGoal.getDegrees());
    assertEquals(0.0, inputs.positionSetpoint.getDegrees());
    assertEquals(0.0, inputs.positionError.getDegrees());
    assertEquals(GainSlot.ZERO, inputs.gainSlot);
  }
}
