package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.units.Units;

public class GenericRollerIOTest {
  @Test
  public void testDefaultIO() {
    GenericRollerIO io = new GenericRollerIO() {};
    GenericRollerIO.GenericRollerIOInputs inputs = new GenericRollerIO.GenericRollerIOInputs();

    assertNotNull(inputs.position);
    assertEquals(0.0, inputs.velocity.in(Units.RadiansPerSecond));

    // Calling defaults should not throw
    io.updateInputs(inputs);
    io.setVoltageGoal(Units.Volts.of(0.0));
    assertFalse(io.atVoltageGoal(Units.Volts.of(0.0)));
  }
}
