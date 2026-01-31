package frc.robot.subsystems.shared.linkage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import frc.robot.RobotConfig;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntakeConstants;

public class LinkageConstants {

  public static final boolean IS_CAN_FD;

  public static final int MOTOR_CAN_ID;

  public static final int CAN_CODER_CAN_ID;

  public static final Rotation2d INTAKE_ANGLE_OFFSET;

  public static final double GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final double LENGTH_METERS;
  public static final double MIN_ANGLE;
  public static final double MAX_ANGLE;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;
  public static final DriveConfig DRIVE_CONFIG;

  public static final LinkLengths LINK_LENGTHS;
  public static final LinkBounds LINK_BOUNDS;

  public static final LinkConstants LINK_CONST;

  public static final double PIN_LENGTH;
  public static Translation3d LINKAGE_OFFSET;

  static {
    IS_CAN_FD = false;
    MOTOR_CAN_ID = 8; // TODO: set correct ID
    CAN_CODER_CAN_ID = 9; // TODO: set correct ID
    GEAR_RATIO = 1;
    CURRENT_LIMIT = 40;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    LENGTH_METERS = 0.3;
    MIN_ANGLE = 0;
    MAX_ANGLE = 2 * Math.PI;
    INTAKE_ANGLE_OFFSET =
        Rotation2d.fromDegrees(0); // TODO: Figure out the actual static angle offset between
    // points A and D on the intake.

    PIN_LENGTH = 0; // TODO: set to distance of pin from point of rotation.

    GAINS =
        new Gains(
            new LoggedTunableNumber("Linkage/KP", 0.0),
            new LoggedTunableNumber("Linkage/KD", 0.0),
            new LoggedTunableNumber("Linkage/KS", 0.0),
            new LoggedTunableNumber("Linkage/KV", 0.0),
            new LoggedTunableNumber("Linkage/KA", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Linkage/Max Velocity", 120.0),
            new LoggedTunableNumber("Linkage/Max Acceleration", 120.0),
            new LoggedTunableNumber("Linkage/Goal Tolerance", Units.degreesToRadians(1.0)));

    DRIVE_CONFIG = V1_GammaConstants.DRIVE_CONFIG;

    LINK_LENGTHS = new LinkLengths(6, 6, 6, 6);

    LINK_BOUNDS = new LinkBounds(0.810921, 2.86545, 4.752162, 6.46545);

    LINK_CONST = new LinkConstants(6.092560, 2.446682, 5.376661);
  }

  public enum LinkageGoal {
    SCORE,
    FEED,
    STOW
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

  static {
    switch (RobotConfig.ROBOT) {
      case V1_GAMMA:
      case V1_GAMMA_SIM:
        LINKAGE_OFFSET = V1_GammaIntakeConstants.INTAKE_GLOBAL_OFFSET;
        break;
      default:
        LINKAGE_OFFSET = V1_GammaIntakeConstants.INTAKE_GLOBAL_OFFSET;
        break;
    }
  }
}
