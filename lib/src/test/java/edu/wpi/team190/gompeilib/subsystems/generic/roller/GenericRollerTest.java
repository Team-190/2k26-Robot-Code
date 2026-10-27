package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class GenericRollerTest {
  private GenericRollerIO io;
  private Subsystem subsystem;
  private GenericRollerConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(GenericRollerIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestRollerSubsystem");

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
  public void testRoller() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericRoller roller = new GenericRoller(io, subsystem, constants, "Test");
      assertNotNull(roller);

      // Alternative constructor
      GenericRoller roller2 =
          new GenericRoller(io, subsystem, constants, "Test", roller.getVoltageGoal());
      assertNotNull(roller2);

      // periodic
      roller.periodic();
      verify(io).updateInputs(any());
      verify(io).setVoltageGoal(Units.Volts.of(0.0));

      // setVoltageGoal (Voltage)
      roller.setVoltageGoal(Units.Volts.of(6.0));
      roller.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // setVoltageGoal (Setpoint)
      Setpoint<org.wpilib.units.VoltageUnit> voltSetpoint =
          new Setpoint<>(
              Units.Volts.of(4.0),
              Units.Volts.of(0.5),
              Units.Volts.of(-12.0),
              Units.Volts.of(12.0));
      roller.setVoltageGoal(voltSetpoint);
      roller.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(4.0));

      // atVoltageGoal helpers
      when(io.atVoltageGoal(any())).thenReturn(true);
      assertTrue(roller.atVoltageGoal(Units.Volts.of(1.0)));
      assertTrue(roller.atVoltageGoal());

      // getTorqueCurrent
      assertNotNull(roller.getTorqueCurrent());
    }
  }
}
