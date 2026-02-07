package frc.robot.subsystems.shared.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import frc.robot.RobotConfig;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;

public class HoodConstants {
  public static final int MOTOR_CAN_ID;
  public static final boolean IS_CAN_FD;

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

  static {
    switch (RobotConfig.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        MOTOR_CAN_ID = 7; // TODO: IDEK what the correct ids are
        IS_CAN_FD = true;

        DRIVE_CONFIG = V0_FunkyConstants.DRIVE_CONFIG;

        GEAR_RATIO = 85.0;
        CURRENT_LIMIT = 40.0;
        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        LENGTH_METERS = 0.381;
        MIN_ANGLE = Units.degreesToRadians(0.0);
        MAX_ANGLE = Units.degreesToRadians(90.0);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Hood/KP", 0.0),
                new LoggedTunableNumber("Hood/KD", 0.0),
                new LoggedTunableNumber("Hood/KS", 0.0),
                new LoggedTunableNumber("Hood/KV", 0.0),
                new LoggedTunableNumber("Hood/KA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Hood/Max Velocity", 120.0),
                new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
                new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0)));
        break;

      case V1_DoomSpiral:
      case V1_DoomSpiral_SIM:
        MOTOR_CAN_ID = 7; // TODO: IDEK what the correct ids are
        IS_CAN_FD = true;

        DRIVE_CONFIG = V1_DoomSpiralConstants.DRIVE_CONFIG;

        GEAR_RATIO = 85.0;
        CURRENT_LIMIT = 40.0;
        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        LENGTH_METERS = 0.381;
        MIN_ANGLE = Units.degreesToRadians(0.0);
        MAX_ANGLE = Units.degreesToRadians(90.0);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Hood/KP", 0.0),
                new LoggedTunableNumber("Hood/KD", 0.0),
                new LoggedTunableNumber("Hood/KS", 0.0),
                new LoggedTunableNumber("Hood/KV", 0.0),
                new LoggedTunableNumber("Hood/KA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Hood/Max Velocity", 120.0),
                new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
                new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0)));

        break;
      default:
        MOTOR_CAN_ID = 7; // TODO: IDEK what the correct ids are
        IS_CAN_FD = true;

        DRIVE_CONFIG = V1_DoomSpiralConstants.DRIVE_CONFIG;

        GEAR_RATIO = 85.0;
        CURRENT_LIMIT = 40.0;
        MOMENT_OF_INERTIA = 0.004;
        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
        LENGTH_METERS = 0.381;
        MIN_ANGLE = Units.degreesToRadians(0.0);
        MAX_ANGLE = Units.degreesToRadians(90.0);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Hood/KP", 0.0),
                new LoggedTunableNumber("Hood/KD", 0.0),
                new LoggedTunableNumber("Hood/KS", 0.0),
                new LoggedTunableNumber("Hood/KV", 0.0),
                new LoggedTunableNumber("Hood/KA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Hood/Max Velocity", 120.0),
                new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
                new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0)));
        break;
    }
  }

  public enum HoodGoal {
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
}
