package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ElevatorIOSimTest {
  private ElevatorConstants constants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    DCMotor motor = DCMotor.getNeo550(1);
    ElevatorConstants.ElevatorParameters params =
        ElevatorConstants.ElevatorParameters.builder()
            .withELEVATOR_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_HEIGHT(Units.Meters.of(0.0))
            .withMAX_HEIGHT(Units.Meters.of(1.5))
            .withNUM_MOTORS(2)
            .build();

    constants =
        ElevatorConstants.builder()
            .withLeaderCANID(5)
            .withElevatorGearRatio(10.0)
            .withDrumRadius(0.02)
            .withElevatorSupplyCurrentLimit(40.0)
            .withElevatorStatorCurrentLimit(40.0)
            .withElevatorParameters(params)
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
                Gains.fromDoubles().withPrefix("slot1").withKP(2.0).withKI(0.0).withKD(0.2).build())
            .withSlot2Gains(
                Gains.fromDoubles().withPrefix("slot2").withKP(3.0).withKI(0.0).withKD(0.3).build())
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
  public void testElevatorIOSim() {
    ElevatorIOSim sim = new ElevatorIOSim(constants);
    ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

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
        LinearConstraints.fromMeasures()
            .withPrefix("constraints")
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(3.0))
            .withGoalTolerance(Units.Meters.of(0.1))
            .build());

    sim.setPosition(Units.Meters.of(0.8));
    assertTrue(sim.atPositionGoal(Units.Meters.of(0.8)));
    assertFalse(sim.atPositionGoal(Units.Meters.of(1.2)));
  }
}
