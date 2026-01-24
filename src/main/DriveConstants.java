package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT;

  public static final DriveConfig DRIVE_CONFIG;

  public static final Gains GAINS;
  public static final AutoAlignGains AUTO_ALIGN_GAINS;

  public static final AutoAlignGains AUTO_GAINS;

  public static final double ODOMETRY_FREQUENCY;
  public static final double DRIVER_DEADBAND;
  public static final double OPERATOR_DEADBAND;

  public static final AlignRobotToAprilTagConstants ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS;

  static {
    switch (Constants.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        FRONT_LEFT = TunerConstantsV0_Funky.FrontLeft;
        FRONT_RIGHT = TunerConstantsV0_Funky.FrontRight;
        BACK_LEFT = TunerConstantsV0_Funky.BackLeft;
        BACK_RIGHT = TunerConstantsV0_Funky.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV0_Funky.DrivetrainConstants.CANBusName,
                TunerConstantsV0_Funky.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV0_Funky.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV0_Funky.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(34.5),
                Units.inchesToMeters(34.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", TunerConstantsV0_Funky.driveGains.kS),
                new LoggedTunableNumber("Drive/Drive KV", TunerConstantsV0_Funky.driveGains.kV),
                new LoggedTunableNumber("Drive/Drive KP", TunerConstantsV0_Funky.driveGains.kP),
                new LoggedTunableNumber("Drive/Drive KD", TunerConstantsV0_Funky.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV0_Funky.steerGains.kP),
                new LoggedTunableNumber("Drive/Turn KD", TunerConstantsV0_Funky.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 40.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 40.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.00));
        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
  }

  public record Gains(
      LoggedTunableNumber drive_Ks,
      LoggedTunableNumber drive_Kv,
      LoggedTunableNumber drive_Kp,
      LoggedTunableNumber drive_Kd,
      LoggedTunableNumber turn_Kp,
      LoggedTunableNumber turn_Kd) {}

  public record AutoAlignGains(
      LoggedTunableNumber translation_Kp,
      LoggedTunableNumber translation_Kd,
      LoggedTunableNumber rotation_Kp,
      LoggedTunableNumber rotation_Kd) {}

  public record PIDControllerConstants(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber tolerance,
      LoggedTunableNumber maxVelocity) {}

  public static record AlignRobotToAprilTagConstants(
      PIDControllerConstants xPIDConstants,
      PIDControllerConstants yPIDConstants,
      PIDControllerConstants omegaPIDConstants,
      LoggedTunableNumber positionThresholdMeters) {}
}
}
