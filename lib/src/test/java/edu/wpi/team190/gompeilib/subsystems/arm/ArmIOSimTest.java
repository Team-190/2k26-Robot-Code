package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class ArmIOSimTest {
  private ArmConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ArmConstants.ArmParameters params =
        ArmConstants.ArmParameters.builder()
            .withMotorConfig(motor)
            .withMinAngle(Rotation2d.fromDegrees(-90))
            .withMaxAngle(Rotation2d.fromDegrees(90))
            .withContinuousOutput(true)
            .withNumMotors(2)
            .withGearRatio(100.0)
            .withLengthMeters(0.5)
            .withMomentOfInertia(0.1)
            .build();

    constants =
        ArmConstants.builder()
            .withArmCANID(4)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(params)
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
                Gains.builder()
                    .withPrefix("slot1")
                    .withKP(2.0)
                    .withKI(0.0)
                    .withKD(0.2)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withSlot2Gains(
                Gains.builder()
                    .withPrefix("slot2")
                    .withKP(3.0)
                    .withKI(0.0)
                    .withKD(0.3)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withConstraints(
                AngularPositionConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(2.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Radians.of(0.05))
                    .build())
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withEnableFOC(false)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();
  }

  @Test
  public void testArmIOSim() {
    ArmIOSim sim = new ArmIOSim(constants);
    ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

    // Initial state set position
    sim.setPosition(Rotation2d.fromDegrees(10));
    sim.setVoltageGoal(Units.Volts.of(0.0));
    sim.updateInputs(inputs);
    assertEquals(10.0, inputs.position.getDegrees(), 1.0);

    // Set position goal
    sim.setPositionGoal(Rotation2d.fromDegrees(30));
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
        AngularPositionConstraints.builder()
            .withPrefix("constraints")
            .withMaxVelocity(Units.RadiansPerSecond.of(3.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(3.0))
            .withGoalTolerance(Units.Radians.of(0.1))
            .build());

    sim.setPosition(Rotation2d.fromDegrees(20));
    assertTrue(sim.atPositionGoal(Rotation2d.fromDegrees(20)));
    assertFalse(sim.atPositionGoal(Rotation2d.fromDegrees(45)));
  }

  @Test
  public void testArmIOSimNonContinuousOutput() {
    DCMotor motor = DCMotor.getNeo550(1);
    ArmConstants.ArmParameters nonContinuousParams =
        ArmConstants.ArmParameters.builder()
            .withMotorConfig(motor)
            .withMinAngle(Rotation2d.fromDegrees(-90))
            .withMaxAngle(Rotation2d.fromDegrees(90))
            .withContinuousOutput(false)
            .withNumMotors(2)
            .withGearRatio(100.0)
            .withLengthMeters(0.5)
            .withMomentOfInertia(0.1)
            .build();

    ArmConstants nonContinuousConstants =
        ArmConstants.builder()
            .withArmCANID(4)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(nonContinuousParams)
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
            .withConstraints(
                AngularPositionConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(2.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Radians.of(0.05))
                    .build())
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40.0)
                    .withStatorCurrentLimit(40.0)
                    .build())
            .withEnableFOC(false)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();

    ArmIOSim sim = new ArmIOSim(nonContinuousConstants);
    ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

    sim.setPosition(Rotation2d.fromDegrees(10));
    sim.updateInputs(inputs);
    assertEquals(10.0, inputs.position.getDegrees(), 1.0);
  }

  @Test
  public void testArmIOSimUnknownGainSlotThrows() throws Exception {
    ArmIOSim sim = new ArmIOSim(constants);

    // GainSlot has exactly 3 values, all handled by setGainSlot's switch, so its compiler-
    // generated $SwitchMap lookup table has no unmapped (default-triggering) entry under normal
    // use. Corrupt that lookup table directly to force the switch's default branch, proving it
    // throws rather than silently doing nothing.
    Class<?> switchMapClass = Class.forName(ArmIOSim.class.getName() + "$1");
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
