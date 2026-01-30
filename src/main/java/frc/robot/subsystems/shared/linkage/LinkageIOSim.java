package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import frc.robot.subsystems.shared.linkage.LinkageIO.LinkageIOInputs;

public class LinkageIOSim {

  private final SingleJointedArmSim motorSim;

  private final ProfiledPIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private double appliedVolts = 0.0;

  public LinkageIOSim() {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                LinkageConstants.MOTOR_CONFIG,
                LinkageConstants.MOMENT_OF_INERTIA,
                LinkageConstants.GEAR_RATIO),
            LinkageConstants.MOTOR_CONFIG,
            LinkageConstants.GEAR_RATIO,
            LinkageConstants.LENGTH_METERS,
            LinkageConstants.MIN_ANGLE,
            LinkageConstants.MAX_ANGLE,
            true,
            LinkageConstants.MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            LinkageConstants.GAINS.kp().get(),
            0.0,
            LinkageConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                LinkageConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                LinkageConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(LinkageConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new SimpleMotorFeedforward(
            LinkageConstants.GAINS.ks().get(), LinkageConstants.GAINS.kv().get());
  }

  public void updateInputs(LinkageIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);

    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(motorSim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(motorSim.getVelocityRadPerSec());
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.positionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  /** Set voltage. */
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  /** Set closed loop position setpoint. */
  public void setPositionGoal(Rotation2d position) {
    appliedVolts =
        feedback.calculate(motorSim.getAngleRads(), position.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  public void setPosition(Rotation2d position) {
    motorSim.setState(position.getRadians(), motorSim.getVelocityRadPerSec());
    feedback.reset(position.getRadians(), motorSim.getVelocityRadPerSec());
  }

  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  public void setFeedforward(double ks, double kv, double ka) {
    feedforward.setKs(ks);
    feedforward.setKv(kv);
    feedforward.setKa(ka);
  }

  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedback.setConstraints(
        new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedback.setTolerance(goalToleranceRadians);
  }

  /** Check if the linkage is within tolerance */
  public boolean atGoalRight() {
    return feedback.atGoal();
  }
}
