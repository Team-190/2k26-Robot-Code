package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import org.junit.jupiter.api.Test;

public class ElevatorConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    DCMotor motor = DCMotor.getNeo550(1);
    ElevatorConstants.ElevatorParameters params =
        ElevatorConstants.ElevatorParameters.builder()
            .withELEVATOR_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_HEIGHT(Units.Meters.of(0.0))
            .withMAX_HEIGHT(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    Gains slot0 = Gains.fromDoubles().withPrefix("test").withKP(1.0).build();
    LinearConstraints constraints =
        LinearConstraints.fromMeasures()
            .withPrefix("test")
            .withMaxVelocity(Units.MetersPerSecond.of(1.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
            .withGoalTolerance(Units.Meters.of(0.01))
            .build();

    ElevatorConstants constants =
        ElevatorConstants.builder()
            .withLeaderCANID(5)
            .withElevatorGearRatio(10.0)
            .withDrumRadius(0.02)
            .withElevatorSupplyCurrentLimit(40.0)
            .withElevatorStatorCurrentLimit(40.0)
            .withElevatorParameters(params)
            .withSlot0Gains(slot0)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();

    assertNotNull(constants);
    assertEquals(5, constants.leaderCANID);
    assertEquals(10.0, constants.elevatorGearRatio);
    assertEquals(0.02, constants.drumRadius);
    assertEquals(40.0, constants.elevatorSupplyCurrentLimit);
    assertEquals(40.0, constants.elevatorStatorCurrentLimit);
    assertEquals(params, constants.elevatorParameters);
    assertEquals(slot0, constants.slot0Gains);
    assertEquals(constraints, constants.constraints);
    assertEquals(0.5, constants.voltageOffsetStep.in(Units.Volts));
    assertEquals(0.05, constants.heightOffsetStep.in(Units.Meters));

    // Test record fields
    assertEquals(motor, params.ELEVATOR_MOTOR_CONFIG());
    assertEquals(15.0, params.CARRIAGE_MASS_KG());
    assertEquals(0.0, params.MIN_HEIGHT().in(Units.Meters));
    assertEquals(1.5, params.MAX_HEIGHT().in(Units.Meters));
    assertEquals(1, params.NUM_MOTORS());
  }
}
