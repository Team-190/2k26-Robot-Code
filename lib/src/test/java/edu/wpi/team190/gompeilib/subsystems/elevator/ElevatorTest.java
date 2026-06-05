package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

public class ElevatorTest {
  private ElevatorIO io;
  private Subsystem subsystem;
  private ElevatorConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(ElevatorIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestElevatorSubsystem");

    DCMotor motor = DCMotor.getNeo550(1);
    ElevatorConstants.ElevatorParameters params =
        ElevatorConstants.ElevatorParameters.builder()
            .withELEVATOR_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_HEIGHT(Units.Meters.of(0.0))
            .withMAX_HEIGHT(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    constants =
        ElevatorConstants.builder()
            .withLeaderCANID(5)
            .withElevatorGearRatio(10.0)
            .withDrumRadius(0.02)
            .withElevatorSupplyCurrentLimit(40.0)
            .withElevatorStatorCurrentLimit(40.0)
            .withElevatorParameters(params)
            .withSlot0Gains(Gains.fromDoubles().withPrefix("slot0").build())
            .withConstraints(
                LinearConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.MetersPerSecond.of(2.0))
                    .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Meters.of(0.05))
                    .build())
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();
  }

  @Test
  public void testElevator() {
    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {
      Elevator elevator = new Elevator(constants, subsystem, 0, io);

      assertNotNull(elevator);

      // Test periodic in IDLE state
      elevator.periodic();
      verify(io).updateInputs(any());

      // Test open loop control state
      elevator.setVoltageGoal(Units.Volts.of(6.0));
      elevator.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // Test closed loop control state
      elevator.setPositionGoal(Units.Meters.of(1.0));
      elevator.periodic();
      verify(io).setPositionGoal(any(Distance.class));

      // Test setters/getters
      elevator.setVoltageGoal(
          new Setpoint<>(
              Units.Volts.of(2.0), Units.Volts.of(0.1), Units.Volts.of(-12), Units.Volts.of(12)));
      elevator.setPositionGoal(
          new Setpoint<>(
              Units.Meters.of(0.5),
              Units.Meters.of(0.01),
              Units.Meters.of(0.0),
              Units.Meters.of(1.5)));

      elevator.getElevatorPosition();

      // Test delegates
      elevator.setPosition(Units.Meters.of(0.4));
      verify(io).setPosition(Units.Meters.of(0.4));

      elevator.setGainSlot(GainSlot.ONE);
      verify(io).setGainSlot(GainSlot.ONE);

      elevator.updateGains(Gains.fromDoubles().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      elevator.updateConstraints(
          LinearConstraints.fromMeasures()
              .withPrefix("constraints")
              .withGoalTolerance(Units.Meters.of(0.01))
              .withMaxVelocity(Units.MetersPerSecond.of(1.0))
              .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // test goals
      when(io.atVoltageGoal(any())).thenReturn(true);
      assertTrue(elevator.atVoltageGoal());
      assertTrue(elevator.atVoltageGoal(Units.Volts.of(2.0)));

      when(io.atPositionGoal(any())).thenReturn(true);
      assertTrue(elevator.atPositionGoal());
      assertTrue(elevator.atPositionGoal(Units.Meters.of(0.5)));

      Command waitCmd = elevator.waitUntilAtGoal();
      assertNotNull(waitCmd);

      Command sysIdCmd = elevator.runSysIdRoutine();
      assertNotNull(sysIdCmd);
    }
  }
}
