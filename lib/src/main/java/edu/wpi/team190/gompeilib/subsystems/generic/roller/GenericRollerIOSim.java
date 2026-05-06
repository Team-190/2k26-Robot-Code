package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.Arrays;

public class GenericRollerIOSim implements GenericRollerIO {
  private final DCMotorSim motorSim;

  private Voltage appliedVolts;

  private Angle accumulatedPosition;

  public GenericRollerIOSim(GenericRollerConstants constants) {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.rollerGearbox,
                constants.momentOfInertia.baseUnitMagnitude(),
                constants.rollerMotorGearRatio),
            constants.rollerGearbox);

    appliedVolts = Volts.of(0.0);

    accumulatedPosition = Radians.of(0.0);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    motorSim.setInputVoltage(appliedVolts.in(Volts));
    motorSim.update(GompeiLib.getLoopPeriod());

    accumulatedPosition =
        Radians.of(
            accumulatedPosition.in(Radians)
                + (motorSim.getAngularVelocityRadPerSec() * GompeiLib.getLoopPeriod()));

    inputs.position = Rotation2d.fromRadians(accumulatedPosition.in(Radians));
    inputs.velocity = motorSim.getAngularVelocity();
    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, motorSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    appliedVolts = voltageGoal;
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolts.of(500));
  }
}
