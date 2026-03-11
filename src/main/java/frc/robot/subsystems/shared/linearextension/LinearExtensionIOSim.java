package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class LinearExtensionIOSim implements LinearExtensionIO {

  private final DCMotorSim motorSim;

  private final ProfiledPIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private double appliedVolts = 0.0;

  public LinearExtensionIOSim(LinearExtensionConstants constants) {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.MOTOR_CONFIG, constants.MOMENT_OF_INERTIA, constants.GEAR_RATIO),
            constants.MOTOR_CONFIG);

    feedback =
        new ProfiledPIDController(
            constants.GAINS.kP().get(),
            0.0,
            constants.GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.CONSTRAINTS.maxVelocity().get().in(RadiansPerSecond),
                constants.CONSTRAINTS.maxAcceleration().get().in(RadiansPerSecondPerSecond)));

    feedback.setTolerance(constants.CONSTRAINTS.goalTolerance().get().in(Radians));
    feedforward =
        new SimpleMotorFeedforward(constants.GAINS.kS().get(), constants.GAINS.kV().get());
  }

  public void updateInputs(LinearExtensionIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);

    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = motorSim.getAngularPositionRad();
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.positionGoal = feedback.getGoal().position;
    inputs.positionSetpoint = feedback.getSetpoint().position;
    inputs.positionError = feedback.getPositionError();
  }

  /** Set voltage. */
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  /** Set closed loop position setpoint. */
  public void setPositionGoal(Rotation2d position) {
    appliedVolts =
        feedback.calculate(motorSim.getAngularPositionRad(), position.getRadians())
            + feedforward.calculate(feedback.getSetpoint().velocity);
  }

  public void setPosition(Rotation2d position) {
    motorSim.setState(position.getRadians(), motorSim.getAngularVelocityRadPerSec());
    feedback.reset(position.getRadians(), motorSim.getAngularVelocityRadPerSec());
  }

  public void setPID(double kp, double ki, double kd) {
    feedback.setPID(kp, ki, kd);
  }

  public void setFeedforward(double ks, double kv, double kg, double ka) {
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
  public boolean atGoal() {
    return feedback.atGoal();
  }
}
