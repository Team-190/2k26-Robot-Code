package frc.robot.subsystems.shared.linkage;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.shared.linkage.LinkageIO.LinkageIOInputs;

public class LinkageIOSim {

  private final SingleJointedArmSim motorSimLeft;
  private final SingleJointedArmSim motorSimRight;

  private final ProfiledPIDController feedbackRight;
  private final ProfiledPIDController feedbackLeft;

  private final SimpleMotorFeedforward feedforwardRight;
  private final SimpleMotorFeedforward feedforwardLeft;

  private Rotation2d positionGoal = new Rotation2d();

  private double appliedVoltsRight = 0.0;
  private double appliedVoltsLeft = 0.0;

  public LinkageIOSim() {
    motorSimRight =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                LinkageConstants.MOTOR_CONFIG_RIGHT,
                LinkageConstants.MOMENT_OF_INERTIA,
                LinkageConstants.GEAR_RATIO),
            LinkageConstants.MOTOR_CONFIG_RIGHT,
            LinkageConstants.GEAR_RATIO,
            LinkageConstants.LENGTH_METERS,
            LinkageConstants.MIN_ANGLE,
            LinkageConstants.MAX_ANGLE,
            true,
            LinkageConstants.MIN_ANGLE);

    motorSimLeft =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                LinkageConstants.MOTOR_CONFIG_LEFT,
                LinkageConstants.MOMENT_OF_INERTIA,
                LinkageConstants.GEAR_RATIO),
            LinkageConstants.MOTOR_CONFIG_LEFT,
            LinkageConstants.GEAR_RATIO,
            LinkageConstants.LENGTH_METERS,
            LinkageConstants.MIN_ANGLE,
            LinkageConstants.MAX_ANGLE,
            true,
            LinkageConstants.MIN_ANGLE);

    feedbackLeft =
        new ProfiledPIDController(
            LinkageConstants.GAINS.kp().get(),
            0.0,
            LinkageConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                LinkageConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                LinkageConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedbackLeft.setTolerance(LinkageConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforwardLeft =
        new SimpleMotorFeedforward(
            LinkageConstants.GAINS.ks().get(), LinkageConstants.GAINS.kv().get());

    feedbackRight =
        new ProfiledPIDController(
            LinkageConstants.GAINS.kp().get(),
            0.0,
            LinkageConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                LinkageConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                LinkageConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedbackRight.setTolerance(LinkageConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforwardRight =
        new SimpleMotorFeedforward(
            LinkageConstants.GAINS.ks().get(), LinkageConstants.GAINS.kv().get());
  }

  public void updateInputs(LinkageIOInputs inputs) {}

  /** Set voltage. */
  public void setVoltageRight(double volts) {
    appliedVoltsRight = volts;
  }

  public void setVoltageLeft(double volts) {
    appliedVoltsLeft = volts;
  }

  /** Set closed loop position setpoint. */
  public void setPositionRight(Rotation2d position) {
    positionGoal = position;
    appliedVoltsRight =
        feedbackRight.calculate(position.getRadians())
            + feedforwardRight.calculate(feedbackRight.getSetpoint().velocity);
  }

  public void setPositionLeft(Rotation2d position) {
    positionGoal = position;
    appliedVoltsLeft =
        feedbackLeft.calculate(position.getRadians())
            + feedforwardLeft.calculate(feedbackLeft.getSetpoint().velocity);
  }

  public void setPIDRight(double kp, double ki, double kd) {
    feedbackRight.setPID(kp, ki, kd);
  }

  public void setPIDLeft(double kp, double ki, double kd) {
    feedbackLeft.setPID(kp, ki, kd);
  }

  public void setProfileRight(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedbackRight.setConstraints(
        new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedbackRight.setTolerance(goalToleranceRadians);
  }

  public void setProfileLeft(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedbackLeft.setConstraints(
        new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedbackLeft.setTolerance(goalToleranceRadians);
  }

  /** Check if the linkage is within tolerance */

  // TODO: REPLACE WITH ACTUAL GEOMETRIC CALCULATION

  public boolean atGoalRight() {
    return false;
  }

  public boolean atGoalLeft() {
    return false;
  }
}
