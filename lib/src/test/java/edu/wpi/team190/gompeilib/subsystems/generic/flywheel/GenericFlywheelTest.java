package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class GenericFlywheelTest {
  private GenericFlywheelIO io;
  private Subsystem subsystem;
  private GenericFlywheelConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(GenericFlywheelIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestFlywheelSubsystem");

    constants =
        GenericFlywheelConstants.builder()
            .withLeaderCANID(5)
            .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withCurrentLimit(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withMomentOfInertia(0.01)
            .withGearRatio(1.0)
            .withMotorConfig(DCMotor.getNeo550(1))
            .withVoltageGains(Gains.fromDoubles().withPrefix("voltage").build())
            .withTorqueGains(Gains.fromDoubles().withPrefix("torque").build())
            .withConstraints(
                AngularVelocityConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(100.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(50.0))
                    .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
                    .build())
            .withVelocityOffsetStep(Units.RadiansPerSecond.of(5.0))
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();
  }

  @Test
  public void testGenericFlywheel() {
    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {
      GenericFlywheel flywheel = new GenericFlywheel(io, subsystem, constants, "Test");
      assertNotNull(flywheel);
      assertEquals(GenericFlywheelState.IDLE, flywheel.getCurrentState());

      // Test alternative constructors
      GenericFlywheel flywheel2 =
          new GenericFlywheel(io, subsystem, constants, "Test", flywheel.getVelocityGoal());
      assertNotNull(flywheel2);

      GenericFlywheel flywheel3 =
          new GenericFlywheel(
              io,
              subsystem,
              constants,
              "Test",
              flywheel.getVelocityGoal(),
              flywheel.getVoltageGoal());
      assertNotNull(flywheel3);

      // periodic in IDLE
      flywheel.periodic();
      verify(io).updateInputs(any());

      // setVoltageGoal (Voltage)
      flywheel.setVoltageGoal(Units.Volts.of(6.0));
      assertEquals(GenericFlywheelState.VOLTAGE_CONTROL, flywheel.getCurrentState());
      flywheel.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // setVoltageGoal (Setpoint)
      Setpoint<edu.wpi.first.units.VoltageUnit> voltSetpoint =
          new Setpoint<>(
              Units.Volts.of(4.0),
              Units.Volts.of(0.5),
              Units.Volts.of(-12.0),
              Units.Volts.of(12.0));
      flywheel.setVoltageGoal(voltSetpoint);
      flywheel.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(4.0));

      // setVelocityGoal (AngularVelocity)
      flywheel.setVelocityGoal(Units.RadiansPerSecond.of(20.0));
      assertEquals(GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL, flywheel.getCurrentState());
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(20.0));

      // setVelocityGoal (Setpoint)
      Setpoint<edu.wpi.first.units.AngularVelocityUnit> velocitySetpoint =
          new Setpoint<>(
              Units.RadiansPerSecond.of(30.0),
              Units.RadiansPerSecond.of(5.0),
              Units.RadiansPerSecond.of(-100.0),
              Units.RadiansPerSecond.of(100.0));
      flywheel.setVelocityGoal(velocitySetpoint);
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(30.0));

      // setVelocityGoal (Supplier)
      flywheel.setVelocityGoal(() -> Units.RadiansPerSecond.of(40.0));
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(40.0));

      // setVelocityGoal (AngularVelocity, Current)
      flywheel.setVelocityGoal(Units.RadiansPerSecond.of(50.0), Units.Amps.of(2.0));
      assertEquals(GenericFlywheelState.VELOCITY_TORQUE_CONTROL, flywheel.getCurrentState());
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(50.0), Units.Amps.of(2.0));

      // setVelocityGoal (Setpoint, Current)
      velocitySetpoint =
          new Setpoint<>(
              Units.RadiansPerSecond.of(30.0),
              Units.RadiansPerSecond.of(5.0),
              Units.RadiansPerSecond.of(-100.0),
              Units.RadiansPerSecond.of(100.0));
      flywheel.setVelocityGoal(velocitySetpoint, Units.Amps.of(3.0));
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(30.0), Units.Amps.of(3.0));

      // setVelocityGoal (Supplier, Supplier)
      flywheel.setVelocityGoal(() -> Units.RadiansPerSecond.of(60.0), () -> Units.Amps.of(4.0));
      flywheel.periodic();
      verify(io).setVelocityGoal(Units.RadiansPerSecond.of(60.0), Units.Amps.of(4.0));

      // stop
      flywheel.stop();
      assertEquals(GenericFlywheelState.STOP, flywheel.getCurrentState());
      flywheel.periodic();
      verify(io).setNeutralControl();

      // getters
      assertNotNull(flywheel.getFlywheelVelocity());
      assertNotNull(flywheel.getFlywheelPosition());

      // atGoal helpers
      when(io.atVoltageGoal(any())).thenReturn(true);
      when(io.atVelocityGoal(any())).thenReturn(true);
      when(io.atCurrentGoal(any())).thenReturn(true);

      assertTrue(flywheel.atVoltageGoal(Units.Volts.of(1.0)));
      assertTrue(flywheel.atVelocityGoal(Units.RadiansPerSecond.of(1.0)));
      assertTrue(flywheel.atCurrentGoal(Units.Amps.of(1.0)));

      assertTrue(flywheel.atVoltageGoal());
      assertTrue(flywheel.atVelocityGoal());
      assertTrue(flywheel.atCurrentGoal());

      Command waitCmd = flywheel.waitUntilAtGoal();
      assertNotNull(waitCmd);

      // updates
      flywheel.updateGains(Gains.fromDoubles().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      flywheel.updateConstraints(
          AngularVelocityConstraints.fromMeasures()
              .withPrefix("constraints")
              .withMaxVelocity(Units.RadiansPerSecond.of(1.0))
              .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(1.0))
              .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // sysid routines
      assertNotNull(flywheel.sysIdRoutineVoltage());
      assertNotNull(flywheel.sysIdRoutineTorque());
    }
  }
}
