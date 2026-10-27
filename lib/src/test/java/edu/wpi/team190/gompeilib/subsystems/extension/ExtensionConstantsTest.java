package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class ExtensionConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    GompeiLib.init(null, false, 0.02);
    DCMotor motor = DCMotor.getNeo550(1);
    ExtensionConstants.ExtensionParameters params =
        ExtensionConstants.ExtensionParameters.builder()
            .withEXTENSION_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_LENGTH(Units.Meters.of(0.0))
            .withMAX_LENGTH(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    Gains slot0 =
        Gains.builder()
            .withPrefix("slot0")
            .withKP(1.0)
            .withKI(0.0)
            .withKD(0.1)
            .withKS(0.01)
            .withKV(0.01)
            .withKA(0.01)
            .withKG(0.1)
            .build();
    LinearConstraints constraints =
        LinearConstraints.builder()
            .withPrefix("test")
            .withMaxVelocity(Units.MetersPerSecond.of(1.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
            .withGoalTolerance(Units.Meters.of(0.01))
            .build();

    ExtensionConstants constants =
        ExtensionConstants.builder()
            .withLeaderCANID(5)
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withCanBus(new CANBus("rio"))
            .withExtensionGearRatio(10.0)
            .withDrumRadius(0.02)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withExtensionParameters(params)
            .withSlot0Gains(slot0)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .withVerticalGravity(true)
            .build();

    assertNotNull(constants);
    assertEquals(5, constants.leaderCANID);
    assertEquals(10.0, constants.extensionGearRatio);
    assertEquals(0.02, constants.drumRadius);
    assertEquals(40.0, constants.extensionSupplyCurrentLimit);
    assertEquals(40.0, constants.extensionStatorCurrentLimit);
    assertEquals(params, constants.extensionParameters);
    assertEquals(slot0, constants.slot0Gains);
    assertEquals(constraints, constants.constraints);
    assertEquals(0.5, constants.voltageOffsetStep.in(Units.Volts));
    assertEquals(0.05, constants.heightOffsetStep.in(Units.Meters));

    // Test record fields
    assertEquals(motor, params.EXTENSION_MOTOR_CONFIG());
    assertEquals(15.0, params.CARRIAGE_MASS_KG());
    assertEquals(0.0, params.MIN_LENGTH().in(Units.Meters));
    assertEquals(1.5, params.MAX_LENGTH().in(Units.Meters));
    assertEquals(1, params.NUM_MOTORS());
  }
}
