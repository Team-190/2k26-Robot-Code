package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim armSim;

  private Voltage appliedVolts;
  private boolean isClosedLoop;
  private GainSlot gainSlot;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private final ArmConstants constants;

  public ArmIOSim(ArmConstants constants) {
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                constants.armParameters.motorConfig(),
                constants.armParameters.momentOfInertia(),
                constants.armParameters.gearRatio()),
            constants.armParameters.motorConfig(),
            constants.armParameters.gearRatio(),
            constants.armParameters.lengthMeters(),
            constants.armParameters.minAngle().getRadians(),
            constants.armParameters.maxAngle().getRadians(),
            true,
            constants.armParameters.minAngle().getRadians());

    appliedVolts = Volts.of(0.0);
    isClosedLoop = true;
    gainSlot = GainSlot.ZERO;

    feedback =
        new ProfiledPIDController(
            constants.slot0Gains.kP().get(),
            0.0,
            constants.slot0Gains.kD().get(),
            new Constraints(
                constants.constraints.maxVelocity().get().in(RadiansPerSecond),
                constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    if (constants.armParameters.continuousOutput()) {
      feedback.enableContinuousInput(
          constants.armParameters.minAngle().getRadians(),
          constants.armParameters.maxAngle().getRadians());
    }
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(Radians));
    feedforward =
        new ArmFeedforward(
            constants.slot0Gains.kS().get(),
            constants.slot0Gains.kV().get(),
            constants.slot0Gains.kA().get(),
            constants.slot0Gains.kG().get());

    this.constants = constants;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (isClosedLoop)
      appliedVolts =
          Volts.of(
              feedback.calculate(armSim.getAngleRads())
                  + feedforward.calculate(
                      feedback.getSetpoint().position, feedback.getSetpoint().velocity));

    appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    armSim.setInputVoltage(appliedVolts.in(Volts));
    armSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());

    inputs.appliedVolts = new double[constants.armParameters.numMotors()];
    inputs.supplyCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.torqueCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.temperatureCelsius = new double[constants.armParameters.numMotors()];

    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, armSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, armSim.getCurrentDrawAmps());

    inputs.positionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());

    inputs.gainSlot = gainSlot;
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    isClosedLoop = false;
    this.appliedVolts = voltageGoal;
  }

  @Override
  public void setPositionGoal(Rotation2d rotationGoal) {
    isClosedLoop = true;
    feedback.setGoal(rotationGoal.getRadians());
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRadians() - armSim.getAngleRads())
        < constants.constraints.goalTolerance().get(Radians);
  }

  @Override
  public void setPosition(Rotation2d position) {
    armSim.setState(position.getRadians(), 0);
  }

  @Override
  public void setGainSlot(GainSlot gainSlot) {
    this.gainSlot = gainSlot;
    switch (gainSlot) {
      case ZERO:
        feedback.setPID(constants.slot0Gains.kP().get(), 0.0, constants.slot0Gains.kD().get());
        break;
      case ONE:
        feedback.setPID(constants.slot1Gains.kP().get(), 0.0, constants.slot1Gains.kD().get());
        break;
      case TWO:
        feedback.setPID(constants.slot2Gains.kP().get(), 0.0, constants.slot2Gains.kD().get());
        break;
    }
  }

  @Override
  public void updateGains(Gains gains, GainSlot gainSlot) {
    feedback.setPID(gains.kP().get(), gains.kI().get(), gains.kD().get());
    feedforward = new ArmFeedforward(gains.kS().get(), gains.kG().get(), gains.kV().get());
  }

  @Override
  public void updateConstraints(AngularPositionConstraints constraints) {
    feedback.setConstraints(
        new Constraints(
            constraints.maxVelocity().get().in(RadiansPerSecond),
            constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get().in(Radians));
  }
}
