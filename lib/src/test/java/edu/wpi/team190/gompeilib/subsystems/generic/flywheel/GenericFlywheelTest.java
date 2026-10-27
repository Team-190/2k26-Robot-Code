package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomSysIdRoutine;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class GenericFlywheelTest {
  private GenericFlywheelIO io;
  private Subsystem subsystem;
  private GenericFlywheelConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
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
            .withVoltageGains(Gains.builder().withPrefix("voltage").build())
            .withTorqueGains(Gains.builder().withPrefix("torque").build())
            .withConstraints(
                AngularVelocityConstraints.builder()
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
  public void testGenericFlywheel() throws Exception {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
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
      Setpoint<org.wpilib.units.VoltageUnit> voltSetpoint =
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
      Setpoint<org.wpilib.units.AngularVelocityUnit> velocitySetpoint =
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
      flywheel.updateGains(Gains.builder().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      flywheel.updateConstraints(
          AngularVelocityConstraints.builder()
              .withPrefix("constraints")
              .withMaxVelocity(Units.RadiansPerSecond.of(1.0))
              .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(1.0))
              .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // sysid routines
      Command voltageSysIdCmd = flywheel.sysIdRoutineVoltage();
      assertNotNull(voltageSysIdCmd);
      voltageSysIdCmd.initialize();

      Command torqueSysIdCmd = flywheel.sysIdRoutineTorque();
      assertNotNull(torqueSysIdCmd);
      torqueSysIdCmd.initialize();

      java.lang.reflect.Field voltageRoutineField =
          GenericFlywheel.class.getDeclaredField("voltageCharacterizationRoutine");
      voltageRoutineField.setAccessible(true);
      CustomSysIdRoutine<org.wpilib.units.VoltageUnit> voltageRoutine =
          (CustomSysIdRoutine<org.wpilib.units.VoltageUnit>) voltageRoutineField.get(flywheel);
      Command voltageQuasistatic =
          voltageRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward);
      voltageQuasistatic.initialize();
      voltageQuasistatic.execute();
      voltageQuasistatic.end(true);

      java.lang.reflect.Field torqueRoutineField =
          GenericFlywheel.class.getDeclaredField("torqueCharacterizationRoutine");
      torqueRoutineField.setAccessible(true);
      CustomSysIdRoutine<org.wpilib.units.CurrentUnit> torqueRoutine =
          (CustomSysIdRoutine<org.wpilib.units.CurrentUnit>) torqueRoutineField.get(flywheel);
      Command torqueQuasistatic = torqueRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward);
      torqueQuasistatic.initialize();
      torqueQuasistatic.execute();
      torqueQuasistatic.end(true);
    }
  }
}
