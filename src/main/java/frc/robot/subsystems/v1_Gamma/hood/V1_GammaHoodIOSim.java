package frc.robot.subsystems.v0_Funky.hood;

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

public class V1_GammaHoodIOSim implements V1_GammaHoodIO {
  private final SingleJointedArmSim motorSim;

  private ProfiledPIDController feedback;
  private SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  public V1_GammaHoodIOSim() {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaHoodConstants.MOTOR_CONFIG,
                V1_GammaHoodConstants.MOMENT_OF_INERTIA,
                V1_GammaHoodConstants.GEAR_RATIO),
            V1_GammaHoodConstants.MOTOR_CONFIG,
            V1_GammaHoodConstants.GEAR_RATIO,
            V1_GammaHoodConstants.LENGTH_METERS,
            V1_GammaHoodConstants.MIN_ANGLE,
            V1_GammaHoodConstants.MAX_ANGLE,
            true,
            V1_GammaHoodConstants.MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            V1_GammaHoodConstants.GAINS.kp().get(),
            0.0,
            V1_GammaHoodConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                V1_GammaHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                V1_GammaHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(V1_GammaHoodConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new SimpleMotorFeedforward(
            V1_GammaHoodConstants.GAINS.ks().get(), V1_GammaHoodConstants.GAINS.kv().get());
  }

  @Override
  public void updateInputs(V0_FunkyHoodIOInputs inputs) {
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
    feedforward = new SimpleMotorFeedforward(ks, kv, ka);
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
