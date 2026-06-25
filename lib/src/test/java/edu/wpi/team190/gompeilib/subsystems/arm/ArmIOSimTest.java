package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ArmIOSimTest {
  private ArmConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
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
                Gains.fromDoubles()
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
                Gains.fromDoubles()
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
                Gains.fromDoubles()
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
                AngularPositionConstraints.fromMeasures()
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
        Gains.fromDoubles()
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
        AngularPositionConstraints.fromMeasures()
            .withPrefix("constraints")
            .withMaxVelocity(Units.RadiansPerSecond.of(3.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(3.0))
            .withGoalTolerance(Units.Radians.of(0.1))
            .build());

    sim.setPosition(Rotation2d.fromDegrees(20));
    assertTrue(sim.atPositionGoal(Rotation2d.fromDegrees(20)));
    assertFalse(sim.atPositionGoal(Rotation2d.fromDegrees(45)));
  }
}
