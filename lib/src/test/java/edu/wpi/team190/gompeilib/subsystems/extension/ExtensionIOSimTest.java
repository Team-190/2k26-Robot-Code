package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class ExtensionIOSimTest {
  private ExtensionConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ExtensionConstants.ExtensionParameters params =
        ExtensionConstants.ExtensionParameters.builder()
            .withEXTENSION_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_LENGTH(Units.Meters.of(0.0))
            .withMAX_LENGTH(Units.Meters.of(1.5))
            .withNUM_MOTORS(2)
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
            .withSlot0Gains(
                Gains.builder()
                    .withPrefix("slot0")
                    .withKP(1.0)
                    .withKI(0.0)
                    .withKD(0.1)
                    .withKS(0.01)
                    .withKV(0.01)
                    .withKA(0.01)
                    .withKG(0.1)
                    .build())
            .withSlot1Gains(
                Gains.builder().withPrefix("slot1").withKP(2.0).withKI(0.0).withKD(0.2).build())
            .withSlot2Gains(
                Gains.builder().withPrefix("slot2").withKP(3.0).withKI(0.0).withKD(0.3).build())
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
  public void testExtensionIOSim() {
    ExtensionIOSim sim = new ExtensionIOSim(constants);
    ExtensionIO.ExtensionIOInputs inputs = new ExtensionIO.ExtensionIOInputs();

    // Initial state set position
    sim.setPosition(Units.Meters.of(0.5));
    sim.setVoltageGoal(Units.Volts.of(0.0));
    sim.updateInputs(inputs);
    assertEquals(0.5, inputs.position.in(Units.Meters), 0.1);

    // Set position goal
    sim.setPositionGoal(Units.Meters.of(1.0));
    sim.updateInputs(inputs);
    assertEquals(2, inputs.appliedVolts.length);
    assertEquals(GainSlot.ZERO, inputs.gainSlot);

    // Test gain slots
    sim.setGainSlot(GainSlot.ONE);
    sim.updateInputs(inputs);
    assertEquals(GainSlot.ONE, inputs.gainSlot);

    sim.setGainSlot(GainSlot.TWO);
    sim.updateInputs(inputs);
    assertEquals(GainSlot.TWO, inputs.gainSlot);

    sim.setGainSlot(GainSlot.ZERO);
    sim.updateInputs(inputs);
    assertEquals(GainSlot.ZERO, inputs.gainSlot);

    // Set voltage goal (open loop)
    sim.setVoltageGoal(Units.Volts.of(6.0));
    sim.updateInputs(inputs);
    assertEquals(6.0, inputs.appliedVolts[0], 0.01);
    assertTrue(sim.atVoltageGoal(Units.Volts.of(6.0)));
    assertFalse(sim.atVoltageGoal(Units.Volts.of(0.0)));

    // update gains & constraints
    sim.updateGains(
        Gains.builder()
            .withPrefix("slot0")
            .withKP(4.0)
            .withKI(0.0)
            .withKD(0.4)
            .withKS(0.02)
            .withKG(0.2)
            .withKV(0.02)
            .withKA(0.0)
            .build(),
        GainSlot.ZERO);
    sim.updateConstraints(
        LinearConstraints.builder()
            .withPrefix("constraints")
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(3.0))
            .withGoalTolerance(Units.Meters.of(0.1))
            .build());

    sim.setPosition(Units.Meters.of(0.8));
    assertTrue(sim.atPositionGoal(Units.Meters.of(0.8)));
    assertFalse(sim.atPositionGoal(Units.Meters.of(1.2)));
  }

  @Test
  public void testExtensionIOSimUnknownGainSlotThrows() throws Exception {
    ExtensionIOSim sim = new ExtensionIOSim(constants);

    // GainSlot has exactly 3 values, all handled by setGainSlot's switch, so its compiler-
    // generated $SwitchMap lookup table has no unmapped (default-triggering) entry under normal
    // use. Corrupt that lookup table directly to force the switch's default branch, proving it
    // throws rather than silently doing nothing.
    Class<?> switchMapClass = Class.forName(ExtensionIOSim.class.getName() + "$1");
    java.lang.reflect.Field field =
        switchMapClass.getDeclaredField(
            "$SwitchMap$edu$wpi$team190$gompeilib$core$utility$phoenix$GainSlot");
    field.setAccessible(true);
    int[] switchMap = (int[]) field.get(null);
    int original = switchMap[GainSlot.ZERO.ordinal()];
    switchMap[GainSlot.ZERO.ordinal()] = 0;
    try {
      assertThrows(IllegalStateException.class, () -> sim.setGainSlot(GainSlot.ZERO));
    } finally {
      switchMap[GainSlot.ZERO.ordinal()] = original;
    }
  }
}
