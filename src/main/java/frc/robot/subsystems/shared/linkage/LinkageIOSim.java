package frc.robot.subsystems.shared.linkage;

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

import static edu.wpi.first.units.Units.*;

public class LinkageIOSim {

    private final SingleJointedArmSim motorSimLeft;
    private final SingleJointedArmSim motorSimRight;

    private final ProfiledPIDController feedbackRight;
    private final ProfiledPIDController feedbackLeft;

    private final SimpleMotorFeedforward feedforwardRight;
    private final SimpleMotorFeedforward feedforwardLeft;

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

    public void updateInputs(LinkageIOInputs inputs) {
        appliedVoltsLeft = MathUtil.clamp(appliedVoltsLeft, -12.0, 12.0);
        appliedVoltsRight = MathUtil.clamp(appliedVoltsRight, -12.0, 12.0);

        motorSimLeft.setInputVoltage(appliedVoltsLeft);
        motorSimRight.setInputVoltage(appliedVoltsRight);

        motorSimRight.update(GompeiLib.getLoopPeriod());
        motorSimLeft.update(GompeiLib.getLoopPeriod());

        inputs.leftPosition = Rotation2d.fromRadians(motorSimLeft.getAngleRads());
        inputs.leftVelocity = Radians.per(Second).of(motorSimLeft.getVelocityRadPerSec());
        inputs.leftSupplyCurrent = Amps.of(motorSimLeft.getCurrentDrawAmps());
        inputs.leftAppliedVolts = Volts.of(appliedVoltsLeft);
        inputs.leftPositionGoal = Rotation2d.fromRadians(feedbackLeft.getGoal().position);
        inputs.leftPositionSetpoint = Rotation2d.fromRadians(feedbackLeft.getSetpoint().position);
        inputs.leftPositionError = Rotation2d.fromRadians(feedbackLeft.getPositionError());

        inputs.rightPosition = Rotation2d.fromRadians(motorSimRight.getAngleRads());
        inputs.rightVelocity = RadiansPerSecond.of(motorSimRight.getVelocityRadPerSec());
        inputs.rightSupplyCurrent = Amps.of(motorSimRight.getCurrentDrawAmps());
        inputs.rightAppliedVolts = Volts.of(appliedVoltsRight);
        inputs.rightPositionGoal = Rotation2d.fromRadians(feedbackRight.getGoal().position);
        inputs.rightPositionSetpoint = Rotation2d.fromRadians(feedbackRight.getSetpoint().position);
        inputs.rightPositionError = Rotation2d.fromRadians(feedbackRight.getPositionError());
    }

    /**
     * Set voltage.
     */
    public void setVoltageRight(double volts) {
        appliedVoltsRight = volts;
    }

    public void setVoltageLeft(double volts) {
        appliedVoltsLeft = volts;
    }

    /**
     * Set closed loop position setpoint.
     */
    public void setPositionGoalRight(Rotation2d position) {
        appliedVoltsRight =
                feedbackRight.calculate(motorSimRight.getAngleRads(), position.getRadians())
                        + feedforwardRight.calculate(feedbackRight.getSetpoint().velocity);
    }

    public void setPositionGoalLeft(Rotation2d position) {
        appliedVoltsLeft =
                feedbackLeft.calculate(motorSimLeft.getAngleRads(), position.getRadians())
                        + feedforwardLeft.calculate(feedbackLeft.getSetpoint().velocity);
    }

    public void setPositionRight(Rotation2d position) {
        motorSimRight.setState(position.getRadians(), motorSimRight.getVelocityRadPerSec());
        feedbackRight.reset(position.getRadians(), motorSimRight.getVelocityRadPerSec());
    }

    public void setPositionLeft(Rotation2d position) {
        motorSimLeft.setState(position.getRadians(), motorSimLeft.getVelocityRadPerSec());
        feedbackLeft.reset(position.getRadians(), motorSimLeft.getVelocityRadPerSec());
    }

    public void setPIDRight(double kp, double ki, double kd) {
        feedbackRight.setPID(kp, ki, kd);
    }

    public void setPIDLeft(double kp, double ki, double kd) {
        feedbackLeft.setPID(kp, ki, kd);
    }

    public void setFeedforwardRight(double ks, double kv, double ka) {
        feedforwardRight.setKs(ks);
        feedforwardRight.setKv(kv);
        feedforwardRight.setKa(ka);
    }

    public void setFeedforwardLeft(double ks, double kv, double ka) {
        feedforwardLeft.setKs(ks);
        feedforwardLeft.setKv(kv);
        feedforwardLeft.setKa(ka);
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

    /**
     * Check if the linkage is within tolerance
     */

    public boolean atGoalRight() {
        return feedbackRight.atGoal();
    }

    public boolean atGoalLeft() {
        return feedbackLeft.atGoal();
    }
}
