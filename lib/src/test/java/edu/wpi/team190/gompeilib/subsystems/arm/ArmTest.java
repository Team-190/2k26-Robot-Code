package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.Units;

public class ArmTest {
  private ArmIO io;
  private Subsystem subsystem;
  private ArmConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(ArmIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestArmSubsystem");

    constants =
        ArmConstants.builder()
            .withArmCANID(4)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(
                ArmConstants.ArmParameters.builder()
                    .withMotorConfig(org.wpilib.math.system.DCMotor.getNeo550(1))
                    .withMinAngle(Rotation2d.fromDegrees(-90))
                    .withMaxAngle(Rotation2d.fromDegrees(90))
                    .withContinuousOutput(false)
                    .withNumMotors(1)
                    .withGearRatio(100.0)
                    .withLengthMeters(0.5)
                    .withMomentOfInertia(0.1)
                    .build())
            .withSlot0Gains(Gains.builder().withPrefix("slot0").build())
            .withConstraints(
                AngularPositionConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(1.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Units.Radians.of(0.01))
                    .build())
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withEnableFOC(false)
            .withInvertedValue(com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();
  }

  @Test
  public void testArm() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Arm arm = new Arm(io, subsystem, 0, constants);

      assertNotNull(arm);
      assertEquals(ArmState.IDLE, arm.getCurrentState());

      // Test alternative constructors
      Arm arm5 = new Arm(io, subsystem, 0, constants, arm.getPositionGoal());
      assertNotNull(arm5);

      Arm arm6 = new Arm(io, subsystem, 0, constants, arm.getPositionGoal(), arm.getVoltageGoal());
      assertNotNull(arm6);

      // Test periodic in IDLE state
      arm.periodic();
      verify(io).updateInputs(any());

      // Test open loop control state
      arm.setVoltageGoal(Units.Volts.of(6.0));
      assertEquals(ArmState.OPEN_LOOP_VOLTAGE_CONTROL, arm.getCurrentState());
      arm.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // Test closed loop control state
      arm.setPositionGoal(Rotation2d.fromDegrees(30));
      assertEquals(ArmState.CLOSED_LOOP_POSITION_CONTROL, arm.getCurrentState());
      arm.periodic();
      verify(io).setPositionGoal(any(Rotation2d.class));

      // Test setters/getters
      arm.setVoltageGoal(
          new Setpoint<>(
              Units.Volts.of(2.0), Units.Volts.of(0.1), Units.Volts.of(-12), Units.Volts.of(12)));
      arm.setPositionGoal(
          new Setpoint<>(
              Units.Radians.of(0.5),
              Units.Radians.of(0.01),
              Units.Radians.of(-3.14),
              Units.Radians.of(3.14)));

      arm.getArmPosition();

      // Test delegates
      arm.setPosition(Rotation2d.fromDegrees(10));
      verify(io).setPosition(Rotation2d.fromDegrees(10));

      arm.setGainSlot(GainSlot.ONE);
      verify(io).setGainSlot(GainSlot.ONE);

      arm.updateGains(Gains.builder().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      arm.updateConstraints(
          AngularPositionConstraints.builder()
              .withPrefix("constraints")
              .withGoalTolerance(Units.Radians.of(0.01))
              .withMaxVelocity(Units.RadiansPerSecond.of(1.0))
              .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // test goals
      when(io.atVoltageGoal(any())).thenReturn(true);
      assertTrue(arm.atVoltageGoal());
      assertTrue(arm.atVoltageGoal(Units.Volts.of(2.0)));

      when(io.atPositionGoal(any())).thenReturn(true);
      assertTrue(arm.atPositionGoal());
      assertTrue(arm.atPositionGoal(Rotation2d.fromDegrees(30)));

      Command waitCmd = arm.waitUntilAtGoal();
      assertNotNull(waitCmd);

      Command sysIdCmd = arm.sysIdRoutine();
      assertNotNull(sysIdCmd);

      sysIdCmd.initialize();
      sysIdCmd.execute();
      sysIdCmd.end(true);
    }
  }
}
