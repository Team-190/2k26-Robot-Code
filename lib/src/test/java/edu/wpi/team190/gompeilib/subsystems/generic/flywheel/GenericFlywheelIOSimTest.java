package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class GenericFlywheelIOSimTest {
  private GenericFlywheelConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

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
            .withVoltageGains(
                Gains.fromDoubles()
                    .withPrefix("voltage")
                    .withKP(1.0)
                    .withKI(0.0)
                    .withKD(0.1)
                    .withKS(0.01)
                    .withKV(0.0104)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withTorqueGains(
                Gains.fromDoubles()
                    .withPrefix("torque")
                    .withKP(0.5)
                    .withKI(0.0)
                    .withKD(0.05)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
                    .build())
            .withConstraints(
                AngularVelocityConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.RadiansPerSecond.of(100.0))
                    .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(50.0))
                    .withGoalTolerance(Units.RadiansPerSecond.of(1.0))
                    .build())
            .withVelocityOffsetStep(Units.RadiansPerSecond.of(5.0))
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withAlignedFollowerCANID(6)
            .withOpposedFollowerCANID(7)
            .build();
  }

  @Test
  public void testGenericFlywheelIOSim() {
    GenericFlywheelIOSim sim = new GenericFlywheelIOSim(constants);
    GenericFlywheelIO.GenericFlywheelIOInputs inputs =
        new GenericFlywheelIO.GenericFlywheelIOInputs();

    // Initial check
    sim.updateInputs(inputs);
    assertEquals(0.0, inputs.position.getRadians(), 1e-6);
    assertEquals(0.0, inputs.velocity.in(Units.RadiansPerSecond), 1e-6);
    assertEquals(3, inputs.appliedVolts.length);

    // Set voltage goal
    sim.setVoltageGoal(Units.Volts.of(6.0));
    sim.updateInputs(inputs);
    assertEquals(6.0, inputs.appliedVolts[0], 0.01);
    assertTrue(sim.atVoltageGoal(Units.Volts.of(6.0)));
    assertFalse(sim.atVoltageGoal(Units.Volts.of(0.0)));

    // Set velocity goal
    sim.setVelocityGoal(Units.RadiansPerSecond.of(100.0));
    sim.updateInputs(inputs);
    assertTrue(inputs.appliedVolts[0] > 0, "Actual applied volts: " + inputs.appliedVolts[0]);

    // atVelocityGoal
    assertFalse(sim.atVelocityGoal(Units.RadiansPerSecond.of(100.0)));
    // Let simulation run to reach goal
    for (int i = 0; i < 150; i++) {
      sim.updateInputs(inputs);
    }
    assertTrue(sim.atVelocityGoal(Units.RadiansPerSecond.of(100.0)));

    // setNeutralControl
    sim.setNeutralControl();
    sim.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVolts[0], 0.01);

    // updateGains and updateConstraints
    sim.updateGains(
        Gains.fromDoubles()
            .withPrefix("newGains")
            .withKP(2.0)
            .withKI(0.0)
            .withKD(0.2)
            .withKS(0.02)
            .withKV(0.2)
            .withKA(0.01)
            .withKG(0.0)
            .build(),
        GainSlot.ZERO);

    sim.updateConstraints(
        AngularVelocityConstraints.fromMeasures()
            .withPrefix("newConstraints")
            .withMaxVelocity(Units.RadiansPerSecond.of(120.0))
            .withMaxAcceleration(Units.RadiansPerSecondPerSecond.of(60.0))
            .withGoalTolerance(Units.RadiansPerSecond.of(2.0))
            .build());

    sim.setVelocityGoal(Units.RadiansPerSecond.of(20.0));
    sim.updateInputs(inputs);
    assertEquals(GainSlot.ZERO, inputs.gainSlot);
  }
}
