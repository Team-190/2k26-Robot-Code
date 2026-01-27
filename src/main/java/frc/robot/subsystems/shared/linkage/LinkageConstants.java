package frc.robot.subsystems.shared.linkage;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;

public class LinkageConstants {

  public static final boolean IS_CAN_FD;

  public static final int RIGHT_MOTOR_CAN_ID;
  public static final int LEFT_MOTOR_CAN_ID;

  public static final double GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG_RIGHT;
  public static final DCMotor MOTOR_CONFIG_LEFT;
  public static final double LENGTH_METERS;
  public static final double MIN_ANGLE;
  public static final double MAX_ANGLE;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;
  public static final DriveConfig DRIVE_CONFIG;

  static {
    IS_CAN_FD = false;
    RIGHT_MOTOR_CAN_ID = 8; // TODO: set correct ID
    LEFT_MOTOR_CAN_ID = 9; // TODO: set correct ID
    GEAR_RATIO = 1;
    CURRENT_LIMIT = 40;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG_RIGHT = DCMotor.getKrakenX60Foc(1);
    MOTOR_CONFIG_LEFT = DCMotor.getKrakenX60Foc(1);
    LENGTH_METERS = 0.3;
    MIN_ANGLE = 0;
    MAX_ANGLE = 2 * Math.PI;

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
  }

  public enum LinkageGoal {
    SCORE,
    FEED,
    STOW
  }

  public enum LinkageState {
    OPEN_LOOP_VOLTAGE_CONTROL,
    CLOSED_LOOP_POSITION_CONTROL,
    IDLE
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
