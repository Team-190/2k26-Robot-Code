package frc.robot.subsystems.shared.linkage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;

public class FourBarLinkageConstants {

  public final boolean IS_CAN_FD;

  public final int MOTOR_CAN_ID;

  public final int CAN_CODER_CAN_ID;

  public final Rotation2d INTAKE_ANGLE_OFFSET;
  public final Rotation2d ZERO_OFFSET;

  public final double GEAR_RATIO;
  public final double SUPPLY_CURRENT_LIMIT;
  public final double STATOR_CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final DCMotor MOTOR_CONFIG;
  public final double LENGTH_METERS;
  public final Rotation2d MIN_ANGLE;
  public final Rotation2d MAX_ANGLE;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;
  public final DriveConfig DRIVE_CONFIG;

  public final LinkLengths LINK_LENGTHS;
  public final LinkBounds LINK_BOUNDS;

  public final LinkConstants LINK_CONST;

  public final double PIN_LENGTH;
  public final Translation3d LINKAGE_OFFSET;

  public final boolean ENABLE_FOC;

  public FourBarLinkageConstants(
      boolean IS_CAN_FD,
      int MOTOR_CAN_ID,
      int CAN_CODER_CAN_ID,
      Rotation2d INTAKE_ANGLE_OFFSET,
      Rotation2d ZERO_OFFSET,
      double GEAR_RATIO,
      double SUPPLY_CURRENT_LIMIT,
      double STATOR_CURRENT_LIMIT,
      double MOMENT_OF_INERTIA,
      DCMotor MOTOR_CONFIG,
      double LENGTH_METERS,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE,
      Gains GAINS,
      Constraints CONSTRAINTS,
      DriveConfig DRIVE_CONFIG,
      LinkLengths LINK_LENGTHS,
      LinkBounds LINK_BOUNDS,
      LinkConstants LINK_CONST,
      double PIN_LENGTH,
      Translation3d LINKAGE_OFFSET,
      boolean ENABLE_FOC) {

    this.IS_CAN_FD = IS_CAN_FD;
    this.MOTOR_CAN_ID = MOTOR_CAN_ID;
    this.CAN_CODER_CAN_ID = CAN_CODER_CAN_ID;
    this.INTAKE_ANGLE_OFFSET = INTAKE_ANGLE_OFFSET;
    this.ZERO_OFFSET = ZERO_OFFSET;
    this.GEAR_RATIO = GEAR_RATIO;
    this.SUPPLY_CURRENT_LIMIT = SUPPLY_CURRENT_LIMIT;
    this.STATOR_CURRENT_LIMIT = STATOR_CURRENT_LIMIT;
    this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
    this.MOTOR_CONFIG = MOTOR_CONFIG;
    this.LENGTH_METERS = LENGTH_METERS;
    this.MIN_ANGLE = MIN_ANGLE;
    this.MAX_ANGLE = MAX_ANGLE;
    this.GAINS = GAINS;
    this.CONSTRAINTS = CONSTRAINTS;
    this.DRIVE_CONFIG = DRIVE_CONFIG;
    this.LINK_LENGTHS = LINK_LENGTHS;
    this.LINK_BOUNDS = LINK_BOUNDS;
    this.LINK_CONST = LINK_CONST;
    this.PIN_LENGTH = PIN_LENGTH;
    this.LINKAGE_OFFSET = LINKAGE_OFFSET;
    this.ENABLE_FOC = ENABLE_FOC;
  }

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber kd,
      LoggedTunableNumber ks,
      LoggedTunableNumber kv,
      LoggedTunableNumber ka) {}

  public record Constraints(
      LoggedTunableNumber maxVelocityRadiansPerSecond,
      LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      LoggedTunableNumber goalToleranceRadians) {}

  public record LinkLengths(double AB, double BC, double CD, double DA) {}

  public record LinkBounds(double MIN, double PHASE_1, double PHASE_2, double MAX) {}

  public record LinkConstants(double RADIUS_1, double RADIUS_2, double CENTER_OFFSET) {}

  public enum LinkageGoal {
    SCORE,
    FEED,
    STOW
  }

  public record LinkageState(Pose3d pose, Rotation2d rotation) {}
}
