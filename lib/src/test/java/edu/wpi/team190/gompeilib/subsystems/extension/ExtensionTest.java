package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Distance;

public class ExtensionTest {
  private ExtensionIO io;
  private Subsystem subsystem;
  private ExtensionConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(ExtensionIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestExtensionSubsystem");

    DCMotor motor = DCMotor.getNeo550(1);
    ExtensionConstants.ExtensionParameters params =
        ExtensionConstants.ExtensionParameters.builder()
            .withEXTENSION_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_LENGTH(Units.Meters.of(0.0))
            .withMAX_LENGTH(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    constants =
        ExtensionConstants.builder()
            .withLeaderCANID(5)
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withExtensionGearRatio(10.0)
            .withDrumRadius(0.02)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withExtensionParameters(params)
            .withSlot0Gains(Gains.builder().withPrefix("slot0").build())
            .withConstraints(
                LinearConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.MetersPerSecond.of(2.0))
                    .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Meters.of(0.05))
                    .build())
            .withVerticalGravity(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();
  }

  @Test
  public void testExtension() throws Exception {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Extension extension = new Extension(constants, subsystem, 0, io);

      Extension _ =
          new Extension(
              constants,
              subsystem,
              0,
              io,
              new Setpoint<DistanceUnit>(
                  Units.Meters.of(0.5),
                  Units.Meters.of(0.01),
                  Units.Meters.of(0.0),
                  Units.Meters.of(1.5)));

      assertNotNull(extension);

      // Test periodic in IDLE state
      extension.periodic();
      verify(io).updateInputs(any());

      // Test open loop control state
      extension.setVoltageGoal(Units.Volts.of(6.0));
      extension.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // Test closed loop control state
      extension.setPositionGoal(Units.Meters.of(1.0));
      extension.periodic();
      verify(io).setPositionGoal(any(Distance.class));

      // Test setters/getters
      extension.setVoltageGoal(
          new Setpoint<>(
              Units.Volts.of(2.0), Units.Volts.of(0.1), Units.Volts.of(-12), Units.Volts.of(12)));
      extension.setPositionGoal(
          new Setpoint<>(
              Units.Meters.of(0.5),
              Units.Meters.of(0.01),
              Units.Meters.of(0.0),
              Units.Meters.of(1.5)));

      extension.getExtensionPosition();
      // extension.getTorqueCurrent();
      extension.getVelocity();

      // Test delegates
      extension.setPosition(Units.Meters.of(0.4));
      verify(io).setPosition(Units.Meters.of(0.4));

      extension.setGainSlot(GainSlot.ONE);
      verify(io).setGainSlot(GainSlot.ONE);

      extension.updateGains(Gains.builder().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      extension.updateConstraints(
          LinearConstraints.builder()
              .withPrefix("constraints")
              .withGoalTolerance(Units.Meters.of(0.01))
              .withMaxVelocity(Units.MetersPerSecond.of(1.0))
              .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // test goals
      when(io.atVoltageGoal(any())).thenReturn(true);
      assertTrue(extension.atVoltageGoal());
      assertTrue(extension.atVoltageGoal(Units.Volts.of(2.0)));

      when(io.atPositionGoal(any())).thenReturn(true);
      assertTrue(extension.atPositionGoal());
      assertTrue(extension.atPositionGoal(Units.Meters.of(0.5)));

      Command waitCmd = extension.waitUntilAtGoal();
      assertNotNull(waitCmd);

      Command sysIdCmd = extension.runSysIdRoutine();
      assertNotNull(sysIdCmd);

      sysIdCmd.initialize();
      sysIdCmd.execute();
      sysIdCmd.end(true);
    }
  }
}
