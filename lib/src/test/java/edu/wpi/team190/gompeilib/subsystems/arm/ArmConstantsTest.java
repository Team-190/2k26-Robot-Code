package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.junit.jupiter.api.Test;

public class ArmConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    DCMotor motor = DCMotor.getNeo550(1);
    ArmConstants.ArmParameters params =
        ArmConstants.ArmParameters.builder()
            .withMotorConfig(motor)
            .withMinAngle(Rotation2d.fromDegrees(0))
            .withMaxAngle(Rotation2d.fromDegrees(90))
            .withContinuousOutput(false)
            .withNumMotors(1)
            .withGearRatio(100.0)
            .withLengthMeters(0.5)
            .withMomentOfInertia(0.1)
            .build();

    Gains slot0 = Gains.fromDoubles().withPrefix("test").withKP(1.0).build();
    AngularPositionConstraints constraints =
        AngularPositionConstraints.fromMeasures()
            .withPrefix("test")
            .withMaxVelocity(Units.RadiansPerSecond.of(1.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(1.0))
            .withGoalTolerance(Units.Radians.of(0.01))
            .build();
    CurrentLimits limits = CurrentLimits.builder().build();

    ArmConstants constants =
        ArmConstants.builder()
            .withArmCANID(4)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(params)
            .withSlot0Gains(slot0)
            .withConstraints(constraints)
            .withCurrentLimits(limits)
            .withEnableFOC(false)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();

    assertNotNull(constants);
    assertEquals(4, constants.armCANID);
    assertEquals("rio", constants.canBus.getName());
    assertEquals(params, constants.armParameters);
    assertEquals(slot0, constants.slot0Gains);
    assertNotNull(constants.slot1Gains);
    assertNotNull(constants.slot2Gains);
    assertEquals(constraints, constants.constraints);
    assertEquals(limits, constants.currentLimits);
    assertFalse(constants.enableFOC);
    assertEquals(InvertedValue.Clockwise_Positive, constants.invertedValue);
    assertEquals(0.5, constants.voltageOffsetStep.in(Units.Volts));
    assertEquals(5.0, constants.positionOffsetStep.getDegrees());

    // Test record fields on ArmParameters
    assertEquals(motor, params.motorConfig());
    assertEquals(0.0, params.minAngle().getDegrees());
    assertEquals(90.0, params.maxAngle().getDegrees());
    assertFalse(params.continuousOutput());
    assertEquals(1, params.numMotors());
    assertEquals(100.0, params.gearRatio());
    assertEquals(0.5, params.lengthMeters());
    assertEquals(0.1, params.momentOfInertia());
  }
}
