package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class GenericRollerIOSimTest {
  private GenericRollerConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    constants =
        GenericRollerConstants.builder()
            .withLeaderCANID(1)
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30.0)
                    .withStatorCurrentLimit(30.0)
                    .build())
            .withRollerGearbox(DCMotor.getNeo550(1))
            .withRollerMotorGearRatio(5.0)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.001))
            .withNeutralMode(NeutralModeValue.Brake)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();
  }

  @Test
  public void testRollerIOSim() {
    GenericRollerIOSim sim = new GenericRollerIOSim(constants);
    GenericRollerIO.GenericRollerIOInputs inputs = new GenericRollerIO.GenericRollerIOInputs();

    // Initial check
    sim.updateInputs(inputs);
    assertEquals(0.0, inputs.position.in(Units.Radians), 1e-6);
    assertEquals(0.0, inputs.velocity.in(Units.RadiansPerSecond), 1e-6);
    assertEquals(1, inputs.appliedVolts.length);

    // Set voltage goal
    sim.setVoltageGoal(Units.Volts.of(6.0));
    sim.updateInputs(inputs);
    assertEquals(6.0, inputs.appliedVolts[0], 0.01);
    assertTrue(sim.atVoltageGoal(Units.Volts.of(6.0)));
    assertFalse(sim.atVoltageGoal(Units.Volts.of(0.0)));

    // Let simulation update and check position and velocity change
    sim.updateInputs(inputs);
    assertTrue(inputs.velocity.in(Units.RadiansPerSecond) > 0.0);
    assertTrue(inputs.position.in(Units.Radians) > 0.0);
  }
}
