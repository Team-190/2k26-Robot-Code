package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.Arrays;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

public class GenericRollerIOSim implements GenericRollerIO {
  private final DCMotorSim motorSim;
  private final GenericRollerConstants constants;

  private Voltage appliedVolts;

  private Angle accumulatedPosition;

  public GenericRollerIOSim(GenericRollerConstants constants) {
    this.constants = constants;
    motorSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                constants.rollerGearbox,
                constants.momentOfInertia.baseUnitMagnitude(),
                constants.rollerMotorGearRatio),
            constants.rollerGearbox);

    appliedVolts = Volts.of(0.0);

    accumulatedPosition = Radians.of(0.0);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    appliedVolts = Volts.of(Math.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    motorSim.setInputVoltage(appliedVolts.in(Volts));
    motorSim.update(GompeiLib.getLoopPeriod());

    accumulatedPosition =
        Radians.of(
            accumulatedPosition.in(Radians)
                + (motorSim.getAngularVelocity() * GompeiLib.getLoopPeriod()));

    inputs.position = accumulatedPosition;
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocity());

    int numMotors =
        1 + constants.alignedFollowerCANIDs.size() + constants.opposedFollowerCANIDs.size();
    inputs.appliedVolts = new double[numMotors];
    inputs.supplyCurrentAmps = new double[numMotors];
    inputs.torqueCurrentAmps = new double[numMotors];
    inputs.temperatureCelsius = new double[numMotors];

    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, motorSim.getCurrentDraw());
    Arrays.fill(inputs.torqueCurrentAmps, motorSim.getCurrentDraw());
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
