package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRadians = 0.0;
    public double driveVelocityRadiansPerSecond = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTemperatureCelcius = 0.0;
    public double driveVelocitySetpointRadiansPerSecond = 0.0;
    public double driveVelocityErrorRadiansPerSecond = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadiansPerSecond = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTemperatureCelcius = 0.0;
    public Rotation2d turnPositionGoal = new Rotation2d();
    public Rotation2d turnPositionSetpoint = new Rotation2d();
    public Rotation2d turnPositionError = new Rotation2d();

    public boolean driveConnected = false;
    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRadians = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveAmps(double currentAmps) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnAmps(double currentAmps) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(
      double velocityRadiansPerSecond, double currentFeedforward) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d position) {}

  /** Sets the module PID gains */
  public default void setPID(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {}

  /** Sets the module FF gains */
  public default void setFeedforward(double drive_Ks, double drive_Kv) {}

  /**
   * Updates current limits.
   *
   * @param driveCurrentLimit The drive current limit.
   * @param turnCurrentLimit The turn current limit.
   */
  public default void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {}
}
