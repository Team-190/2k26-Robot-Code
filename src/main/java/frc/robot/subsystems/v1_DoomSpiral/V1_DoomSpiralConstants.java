package frc.robot.subsystems.v1_DoomSpiral;

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

public class V1_DoomSpiralConstants {
  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
          V1_DoomSpiralTunerConstants.kCANBus,
          V1_DoomSpiralTunerConstants.DrivetrainConstants.Pigeon2Id,
          V1_DoomSpiralTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          V1_DoomSpiralTunerConstants.kWheelRadius.in(Meters),
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          V1_DoomSpiralTunerConstants.FrontLeft,
          V1_DoomSpiralTunerConstants.FrontRight,
          V1_DoomSpiralTunerConstants.BackLeft,
          V1_DoomSpiralTunerConstants.BackRight,
          V1_DoomSpiralTunerConstants.kDriveClosedLoopOutput,
          V1_DoomSpiralTunerConstants.kSteerClosedLoopOutput,
          Units.inchesToMeters(37.5),
          Units.inchesToMeters(28.5));

  public static final Gains GAINS =
      new Gains(
          new LoggedTunableNumber(
              "Drive/Teleop/Drive Ks", V1_DoomSpiralTunerConstants.driveGains.kS),
          new LoggedTunableNumber(
              "Drive/Teleop/Drive Kv", V1_DoomSpiralTunerConstants.driveGains.kV),
          new LoggedTunableNumber(
              "Drive/Teleop/Drive Kp", V1_DoomSpiralTunerConstants.driveGains.kP),
          new LoggedTunableNumber(
              "Drive/Teleop/Drive Kd", V1_DoomSpiralTunerConstants.driveGains.kD),
          new LoggedTunableNumber(
              "Drive/Teleop/Turn Kp", V1_DoomSpiralTunerConstants.steerGains.kP),
          new LoggedTunableNumber(
              "Drive/Teleop/Turn Kd", V1_DoomSpiralTunerConstants.steerGains.kD));

  public static final AutoGains AUTO_GAINS =
      new AutoGains(
          new LoggedTunableNumber("Drive/Auto/Translation Kp", 10.0),
          new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kp", 5.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0));

  public static final AutoAlignNearConstants AUTO_ALIGN_NEAR_CONSTANTS =
      new AutoAlignNearConstants(
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/X Constants/kP", 3),
              new LoggedTunableNumber("Drive/Auto Align/X Constants/kD", 0.15),
              new LoggedTunableNumber("Drive/Auto Align/X Constants/tolerance", 0.03),
              new LoggedTunableNumber("Drive/Auto Align/X Constants/maxVelocity", 2.5)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Y Constants/kP", 3),
              new LoggedTunableNumber("Drive/Auto Align/Y Constants/kD", 0.15),
              new LoggedTunableNumber("Drive/Auto Align/Y Constants/tolerance", 0.03),
              new LoggedTunableNumber("Drive/Auto Align/Y Constants/maxVelocity", 2.5)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Omega Constants/kP", 2 * Math.PI),
              new LoggedTunableNumber("Drive/Auto Align/Omega Constants/kD", 0.05),
              new LoggedTunableNumber(
                  "Drive/Auto Align/Omega Constants/tolerance", Units.degreesToRadians(0.25)),
              new LoggedTunableNumber("Drive/Auto Align/Omega Constants/maxVelocity", Math.PI)),
          new LoggedTunableNumber("Drive/Auto Align/positionThresholdDegrees", 0.03));

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

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

  public static final LimelightConfig LIMELIGHT_SHOOTER_CONFIG =
      LimelightConfig.builder()
          .key("shooter")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.055,
                  -0.054,
                  0.538,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90 - 62.000),
                      Units.degreesToRadians(-90.000))))
          .build();

  public static final LimelightConfig LIMELIGHT_LEFT_CONFIG =
      LimelightConfig.builder()
          .key("left")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.239,
                  -0.387,
                  0.305,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(6.7),
                      Units.degreesToRadians(106))))
          .build();

  public static final LimelightConfig LIMELIGHT_RIGHT_CONFIG =
      LimelightConfig.builder()
          .key("right")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.264,
                  0.337,
                  0.484,
                  // camera is upside down so need to build the angles sequentially
                  new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 - 21.6))
                      .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(2.79), 0.0))
                      .rotateBy(new Rotation3d(Math.PI, 0, 0))))
          .build();
}
