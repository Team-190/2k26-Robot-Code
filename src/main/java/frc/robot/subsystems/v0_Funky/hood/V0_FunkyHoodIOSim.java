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

public class V0_FunkyHoodIOSim implements V0_FunkyHoodIO {
  private final SingleJointedArmSim motorSim;

  private ProfiledPIDController feedback;
  private SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;

  public V0_FunkyHoodIOSim() {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                V0_FunkyHoodConstants.MOTOR_CONFIG,
                V0_FunkyHoodConstants.MOMENT_OF_INERTIA,
                V0_FunkyHoodConstants.GEAR_RATIO),
            V0_FunkyHoodConstants.MOTOR_CONFIG,
            V0_FunkyHoodConstants.GEAR_RATIO,
            V0_FunkyHoodConstants.LENGTH_METERS,
            V0_FunkyHoodConstants.MIN_ANGLE,
            V0_FunkyHoodConstants.MAX_ANGLE,
            true,
            V0_FunkyHoodConstants.MIN_ANGLE);

    feedback =
        new ProfiledPIDController(
            V0_FunkyHoodConstants.GAINS.kp().get(),
            0.0,
            V0_FunkyHoodConstants.GAINS.kd().get(),
            new TrapezoidProfile.Constraints(
                V0_FunkyHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get(),
                V0_FunkyHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get()));
    feedback.setTolerance(V0_FunkyHoodConstants.CONSTRAINTS.goalToleranceRadians().get());
    feedforward =
        new SimpleMotorFeedforward(
            V0_FunkyHoodConstants.GAINS.ks().get(), V0_FunkyHoodConstants.GAINS.kv().get());
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
