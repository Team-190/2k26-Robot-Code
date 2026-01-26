package frc.robot.subsystems.shared.hood;

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

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim motorSim;

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                HoodConstants.MOTOR_CONFIG,
                HoodConstants.MOMENT_OF_INERTIA,
                HoodConstants.GEAR_RATIO),
            HoodConstants.MOTOR_CONFIG,
            HoodConstants.GEAR_RATIO,
            HoodConstants.LENGTH_METERS,
            HoodConstants.MIN_ANGLE,
            HoodConstants.MAX_ANGLE,
            true,
            HoodConstants.MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            HoodConstants.GAINS.kp().get(),
            0.0,
            HoodConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                HoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                HoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(HoodConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new SimpleMotorFeedforward(HoodConstants.GAINS.ks().get(), HoodConstants.GAINS.kv().get());
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
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionGoal = position;
    appliedVolts =
        feedback.calculate(position.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    feedforward.setKa(ka);
    feedforward.setKs(ks);
    feedforward.setKv(kv);
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedback.setConstraints(
        new Constraints(maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedback.setTolerance(goalToleranceRadians);
  }

  @Override
  public boolean atGoal() {
    return feedback.atGoal();
  }
}
