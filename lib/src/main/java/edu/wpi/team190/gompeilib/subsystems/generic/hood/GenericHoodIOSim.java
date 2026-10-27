package edu.wpi.team190.gompeilib.subsystems.generic.hood;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Voltage;

public class GenericHoodIOSim implements GenericHoodIO {
  private static final double CONTROL_PERIOD_SECS = 0.001;
  // Gains are tuned in Phoenix units (V per mechanism rotation); scale them to V per radian
  private static final double ROTATIONS_PER_RADIAN = 1.0 / (2.0 * Math.PI);

  private final SingleJointedArmSim motorSim;

  private final ProfiledPIDController feedback;
  private final SimpleMotorFeedforward feedforward;

  private Rotation2d positionGoal = new Rotation2d();
  private double appliedVolts = 0.0;
  private boolean isClosedLoop = false;

  private final GenericHoodConstants constants;

  public GenericHoodIOSim(GenericHoodConstants constants) {
    motorSim =
        new SingleJointedArmSim(
            constants.motorConfig,
            constants.gearRatio,
            constants.momentOfInertia,
            constants.lengthMeters,
            constants.minAngle.getRadians(),
            constants.maxAngle.getRadians(),
            // Gravity off: the sim infers mass from MOI, which overstates load on a geared hood
            false,
            constants.minAngle.getRadians());
    feedback =
        new ProfiledPIDController(
            constants.gains.kP().get() * ROTATIONS_PER_RADIAN,
            0.0,
            constants.gains.kD().get() * ROTATIONS_PER_RADIAN,
            new TrapezoidProfile.Constraints(
                constants.constraints.maxVelocity().get().in(RadiansPerSecond),
                constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)),
            CONTROL_PERIOD_SECS);
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(Radians));
    feedforward =
        new SimpleMotorFeedforward(
            constants.gains.kS().get(), constants.gains.kV().get() * ROTATIONS_PER_RADIAN);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericHoodIOInputs inputs) {
    // Substep at the TalonFX's 1 kHz closed-loop rate so the real gains (tuned onboard) stay stable
    int substeps = (int) Math.max(1, Math.round(GompeiLib.getLoopPeriod() / CONTROL_PERIOD_SECS));
    for (int i = 0; i < substeps; i++) {
      if (isClosedLoop) {
        appliedVolts =
            feedback.calculate(motorSim.getAngle())
                + feedforward.calculate(feedback.getSetpoint().velocity);
      }
      motorSim.setInputVoltage(Math.clamp(appliedVolts, -12.0, 12.0));
      motorSim.update(GompeiLib.getLoopPeriod() / substeps);
    }

    inputs.position = Rotation2d.fromRadians(motorSim.getAngle());
    inputs.velocity = RadiansPerSecond.of(motorSim.getVelocity());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDraw());
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  @Override
  public void setVoltage(Voltage volts) {
    isClosedLoop = false;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    if (!isClosedLoop) {
      // Start the profile from where the hood actually is, like MotionMagic does
      feedback.reset(motorSim.getAngle(), motorSim.getVelocity());
      isClosedLoop = true;
    }
    positionGoal = position;
    feedback.setGoal(position.getRadians());
  }

  public void setPosition(Rotation2d position) {
    motorSim.setState(position.getRadians(), motorSim.getVelocity());
    feedback.reset(position.getRadians(), motorSim.getVelocity());
  }

  @Override
  public void setGains(Gains gains) {
    feedback.setPID(
        gains.kP().get() * ROTATIONS_PER_RADIAN, 0.0, gains.kD().get() * ROTATIONS_PER_RADIAN);
    feedforward.setKa(gains.kA().get() * ROTATIONS_PER_RADIAN);
    feedforward.setKv(gains.kV().get() * ROTATIONS_PER_RADIAN);
    feedforward.setKs(gains.kS().get());
  }

  @Override
  public void setProfile(AngularPositionConstraints constraints) {
    feedback.setConstraints(
        new Constraints(
            constraints.maxVelocity().get().in(RadiansPerSecond),
            constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond)));
    feedback.setTolerance(constraints.goalTolerance().get().in(Radians));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRadians() - motorSim.getAngle())
        <= constants.constraints.goalTolerance().get().in(Radians);
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(Volts.of(appliedVolts), Millivolts.of(500));
  }
}
