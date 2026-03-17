package frc.robot.subsystems.shared.fourbarlinkage;

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

public class FourBarLinkageIOSim implements FourBarLinkageIO {

  private final SingleJointedArmSim motorSim;

  private final ProfiledPIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private double appliedVolts;

  private final FourBarLinkageConstants constants;

  public FourBarLinkageIOSim(FourBarLinkageConstants constants) {
    motorSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig,
            constants.gearRatio,
            constants.pinLength,
            constants.minAngle.getRadians(),
            constants.maxAngle.getRadians(),
            false,
            0.0);

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
    appliedVolts = 0.0;

    this.constants = constants;
  }

  public void updateInputs(FourBarLinkageIOInputs inputs) {
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
  public void setVoltageGoal(Voltage volts) {
    appliedVolts = volts.in(Volts);
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

  /** Check if the linkage is within tolerance */
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRadians() - motorSim.getAngleRads())
        <= constants.constraints.goalTolerance().get(Radians);
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(Volts.of(appliedVolts), Millivolts.of(500));
  }
}
