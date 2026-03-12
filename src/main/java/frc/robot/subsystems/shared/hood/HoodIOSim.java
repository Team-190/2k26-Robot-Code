package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim motorSim;

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  private final HoodConstants constants;

  public HoodIOSim(HoodConstants constants) {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig,
            constants.gearRatio,
            constants.lengthMeters,
            constants.minAngle.getRadians(),
            constants.maxAngle.getRadians(),
            true,
            constants.minAngle.getRadians());

    feedback =
        new ProfiledPIDController(
            constants.gains.kP().get(),
            0.0,
            constants.gains.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.constraints.maxVelocity().get().in(RadiansPerSecond),
                constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(Radians));
    feedforward =
        new SimpleMotorFeedforward(constants.gains.kS().get(), constants.gains.kV().get());

    this.constants = constants;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(motorSim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint = Rotation2d.fromRotations(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRotations(feedback.getPositionError());
  }

  @Override
  public void setVoltage(Voltage volts) {
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    positionGoal = position;
    feedback.setGoal(position.getRadians());
    appliedVolts =
        feedback.calculate(position.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  public void setPosition(Rotation2d position) {
    motorSim.setState(position.getRadians(), motorSim.getVelocityRadPerSec());
    feedback.reset(position.getRadians(), motorSim.getVelocityRadPerSec());
  }

  @Override
  public void setGains(Gains gains) {
    feedback.setPID(gains.kP().get(), 0.0, gains.kD().get());
    feedforward.setKa(gains.kA().get());
    feedforward.setKv(gains.kV().get());
    feedforward.setKs(gains.kS().get());
  }

  @Override
  public void setProfile(AngularPositionConstraints constraints) {
    feedback.setConstraints(
        new Constraints(
            constraints.maxVelocity().get(RadiansPerSecond),
            constraints.maxAcceleration().get(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get(Radians));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRadians() - motorSim.getAngleRads())
        <= constants.constraints.goalTolerance().get(Radians);
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(Volts.of(appliedVolts), Millivolts.of(500));
  }
}
