package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.*;

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
            Models.singleJointedArmFromPhysicalConstants(
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
              feedback.calculate(armSim.getAngle())
                  + feedforward.calculate(
                      feedback.getSetpoint().position, feedback.getSetpoint().velocity));

    appliedVolts = Volts.of(Math.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    armSim.setInputVoltage(appliedVolts.in(Volts));
    armSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(armSim.getAngle());
    inputs.velocity = RadiansPerSecond.of(armSim.getVelocity());

    inputs.appliedVolts = new double[constants.armParameters.numMotors()];
    inputs.supplyCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.torqueCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.temperatureCelsius = new double[constants.armParameters.numMotors()];

    Arrays.fill(inputs.appliedVolts, appliedVolts.in(Volts));
    Arrays.fill(inputs.supplyCurrentAmps, armSim.getCurrentDraw());
    Arrays.fill(inputs.torqueCurrentAmps, armSim.getCurrentDraw());

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
    return Math.abs(positionReference.getRadians() - armSim.getAngle())
        < constants.constraints.goalTolerance().get().in(Radians);
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
      default:
        throw new IllegalStateException("Unknown gain slot: " + gainSlot);
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
