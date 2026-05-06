package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final SwerveDriveConstants driveConstants;
  private final SwerveModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public SwerveModule(SwerveDriveConstants driveConstants, SwerveModuleIO io, int index) {
    this.driveConstants = driveConstants;
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  @Trace
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  @Trace
  public void periodic() {
    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRadians[i] * driveConstants.driveConfig.wheelRadiusMeters();
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  @Trace
  public void runSetpoint(SwerveModuleState state, SwerveModuleState torqueFeedforward) {
    // Optimize veloci[ty setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    double wheelTorqueNewtonMeters = torqueFeedforward.speedMetersPerSecond;
    // Apply setpoints
    io.setDriveVelocity(
        state.speedMetersPerSecond / driveConstants.driveConfig.wheelRadiusMeters(),
        driveConstants
            .driveConfig
            .driveModel()
            .getCurrent(
                wheelTorqueNewtonMeters
                    / driveConstants.driveConfig.frontLeft().DriveMotorGearRatio));
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  @Trace
  public void runCharacterization(double amps) {
    io.setDriveAmps(amps);
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  @Trace
  public void stop() {
    io.setDriveAmps(0.0);
    io.setTurnAmps(0.0);
  }

  /** Returns the current turn angle of the module. */
  @Trace
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  @Trace
  public double getPositionMeters() {
    return inputs.drivePositionRadians * driveConstants.driveConfig.wheelRadiusMeters();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  @Trace
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadiansPerSecond * driveConstants.driveConfig.wheelRadiusMeters();
  }

  /** Returns the module position (turn angle and drive position). */
  @Trace
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  @Trace
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle. */
  @Trace
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  @Trace
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRadians;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  @Trace
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadiansPerSecond);
  }

  /** Sets module PID gains */
  @Trace
  public void setPID(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {
    io.setPID(drive_Kp, drive_Kd, turn_Kp, turn_Kd);
  }

  /** Sets module FF gains */
  @Trace
  public void setFF(double kS, double kV) {
    io.setFeedforward(kS, kV);
  }

  /**
   * Updates current limits.
   *
   * @param driveCurrentLimit The drive current limit.
   * @param turnCurrentLimit The turn current limit.
   */
  @Trace
  public void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {
    io.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit);
  }
}
