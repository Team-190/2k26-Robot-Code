package frc.robot.subsystems.v0_Funky;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoGains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.Gains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.PIDControllerConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;

public class V0_FunkyConstants {
  public static final DriveConfig DRIVE_CONFIG =
      DriveConfig.builder()
          .withCanBus(V0_FunkyTunerConstants.kCANBus)
          .withPigeon2Id(V0_FunkyTunerConstants.DrivetrainConstants.Pigeon2Id)
          .withMaxLinearVelocityMetersPerSecond(
              V0_FunkyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
          .withWheelRadiusMeters(V0_FunkyTunerConstants.kWheelRadius.in(Meters))
          .withDriveModel(DCMotor.getKrakenX60Foc(1))
          .withTurnModel(DCMotor.getKrakenX44Foc(1))
          .withFrontLeft(V0_FunkyTunerConstants.FrontLeft)
          .withFrontRight(V0_FunkyTunerConstants.FrontRight)
          .withBackLeft(V0_FunkyTunerConstants.BackLeft)
          .withBackRight(V0_FunkyTunerConstants.BackRight)
          .withDriveClosedLoopOutputType(V0_FunkyTunerConstants.kDriveClosedLoopOutput)
          .withSteerClosedLoopOutputType(V0_FunkyTunerConstants.kSteerClosedLoopOutput)
          .withBumperLength(Units.inchesToMeters(34.5))
          .withBumperWidth(Units.inchesToMeters(34.5))
          .build();

  public static final Gains GAINS =
      Gains.builder()
          .withDriveKs(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Ks", V0_FunkyTunerConstants.driveGains.kS))
          .withDriveKv(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kv", V0_FunkyTunerConstants.driveGains.kV))
          .withDriveKp(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kp", V0_FunkyTunerConstants.driveGains.kP))
          .withDriveKd(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kd", V0_FunkyTunerConstants.driveGains.kD))
          .withTurnKp(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kp", V0_FunkyTunerConstants.steerGains.kP))
          .withTurnKd(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kd", V0_FunkyTunerConstants.steerGains.kD))
          .build();

  public static final AutoGains AUTO_GAINS =
      AutoGains.builder()
          .withTranslationKp(new LoggedTunableNumber("Drive/Auto/Translation Kp", 0.0))
          .withTranslationKd(new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0))
          .withRotationKp(new LoggedTunableNumber("Drive/Auto/Rotation Kp", 0.0))
          .withRotationKd(new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0))
          .build();

  public static final PIDControllerConstants X_PID_CONSTANTS =
      PIDControllerConstants.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/X/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.0))
          .withTolerance(new LoggedTunableNumber("Drive/Auto Align/X/Tolerance", 0.0))
          .withMaxVelocity(new LoggedTunableNumber("Drive/Auto Align/X/Max Velocity", 0.0))
          .build();

  public static final PIDControllerConstants Y_PID_CONSTANTS =
      PIDControllerConstants.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.0))
          .withTolerance(new LoggedTunableNumber("Drive/Auto Align/Y/Tolerance", 0.0))
          .withMaxVelocity(new LoggedTunableNumber("Drive/Auto Align/Y/Max Velocity", 0.0))
          .build();

  public static final PIDControllerConstants OMEGA_PID_CONSTANTS =
      PIDControllerConstants.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.0))
          .withTolerance(new LoggedTunableNumber("Drive/Auto Align/Theta/Tolerance", 0.0))
          .withMaxVelocity(new LoggedTunableNumber("Drive/Auto Align/Theta/Max Velocity", 0.0))
          .build();

  public static final AutoAlignNearConstants AUTO_ALIGN_NEAR_CONSTANTS =
      AutoAlignNearConstants.builder()
          .withXPIDConstants(X_PID_CONSTANTS)
          .withYPIDConstants(Y_PID_CONSTANTS)
          .withOmegaPIDConstants(OMEGA_PID_CONSTANTS)
          .withPositionThresholdMeters(
              new LoggedTunableNumber(
                  "Drive/Auto Align/Position Threshold Meters", Units.inchesToMeters(0.25)))
          .build();

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;
  public static final double TRIGGER_DEADBAND = 0.05;

  public static final double SHOOTER_VOLTAGE = 12.0;
  public static final double FEEDER_VOLTAGE = 6.0;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      SwerveDriveConstants.builder()
          .withDriveConfig(DRIVE_CONFIG)
          .withGains(GAINS)
          .withAutoGains(AUTO_GAINS)
          .withAutoAlignConstants(AUTO_ALIGN_NEAR_CONSTANTS)
          .withOdometryFrequency(ODOMETRY_FREQUENCY)
          .withDriverDeadband(DRIVER_DEADBAND)
          .withOperatorDeadband(OPERATOR_DEADBAND)
          .build();

  public static final LimelightConfig LIMELIGHT_CONFIG =
      LimelightConfig.builder()
          .key("limelight")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(82.0)
          .verticalFOV(56.2)
          .megatagXYStdev(0.1)
          .metatagThetaStdev(0.0015)
          .megatag2XYStdev(0.001)
          .robotToCameraTransform(new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
          .build();
}
