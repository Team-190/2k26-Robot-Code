package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import org.junit.jupiter.api.Test;

public class GenericFlywheelConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    DCMotor motor = DCMotor.getNeo550(1);
    Gains gains = Gains.fromDoubles().withPrefix("test").withKP(1.0).build();
    AngularVelocityConstraints constraints =
        AngularVelocityConstraints.fromMeasures()
            .withPrefix("test")
            .withMaxVelocity(Units.RadiansPerSecond.of(100.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(200.0))
            .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
            .build();

    GenericFlywheelConstants constants =
        GenericFlywheelConstants.builder()
            .withLeaderCANID(6)
            .withLeaderInversion(InvertedValue.Clockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withEnableFOC(false)
            .withCurrentLimit(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withMomentOfInertia(0.01)
            .withGearRatio(1.0)
            .withMotorConfig(motor)
            .withVoltageGains(gains)
            .withTorqueGains(gains)
            .withConstraints(constraints)
            .withVelocityOffsetStep(Units.RadiansPerSecond.of(5.0))
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();

    assertNotNull(constants);
    assertEquals(6, constants.leaderCANID);
    assertEquals(InvertedValue.Clockwise_Positive, constants.leaderInversion);
    assertEquals("rio", constants.canBus.getName());
    assertFalse(constants.enableFOC);
    assertEquals(0.01, constants.momentOfInertia);
    assertEquals(1.0, constants.gearRatio);
    assertEquals(motor, constants.motorConfig);
    assertEquals(gains, constants.voltageGains);
    assertEquals(gains, constants.torqueGains);
    assertEquals(constraints, constants.constraints);
    assertEquals(5.0, constants.velocityOffsetStep.in(Units.RadiansPerSecond));
    assertEquals(0.5, constants.voltageOffsetStep.in(Units.Volts));
  }
}
