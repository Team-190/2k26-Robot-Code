package frc.robot.subsystems.v1_DoomSpiral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import frc.robot.subsystems.v0_Funky.V0_FunkyTunerConstants;

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

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kp", V0_FunkyTunerConstants.driveGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kd", V0_FunkyTunerConstants.driveGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Ks", V0_FunkyTunerConstants.driveGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kv", V0_FunkyTunerConstants.driveGains.kV))
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kp", V0_FunkyTunerConstants.steerGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kd", V0_FunkyTunerConstants.steerGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Ks", V0_FunkyTunerConstants.steerGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kv", V0_FunkyTunerConstants.steerGains.kV))
          .build();

  public static final Gains TRANSLATION_AUTO_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Translation Kp", 5.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0))
          .build();

  public static final Gains ROTATION_AUTO_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Rotation Kp", 5.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0))
          .build();

  public static final Gains AUTO_ALIGN_X_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/X/Kp", 3.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.15))
          .build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS =
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Velocity", MetersPerSecond.of(2.5)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/X/Max Velocity", Meters.of(0.03)))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 3.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.15))
          .build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS =
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Velocity", MetersPerSecond.of(2.5)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Y/Goal Tolerance", Meters.of(0.05)))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 2.0 * Math.PI))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.05))
          .build();

  public static final AngularConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Velocity", RadiansPerSecond.of(Math.PI)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Acceleration", RadiansPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Theta/Max Velocity", Degrees.of(0.5)))
          .build();

  public static final SwerveDriveConstants.AutoAlignConstants AUTO_ALIGN_CONSTANTS =
      SwerveDriveConstants.AutoAlignConstants.builder()
          .withXGains(AUTO_ALIGN_X_GAINS)
          .withXConstraints(AUTO_ALIGN_X_CONSTRAINTS)
          .withYGains(AUTO_ALIGN_Y_GAINS)
          .withYConstraints(AUTO_ALIGN_Y_CONSTRAINTS)
          .withRotationGains(AUTO_ALIGN_THETA_GAINS)
          .withRotationConstraints(AUTO_ALIGN_THETA_CONSTRAINTS)
          .withLinearThreshold(
              new LoggedTunableMeasure<>("Drive/Auto Align/Position Threshold", Inches.of(0.25)))
          .withAngularThreshold(
              new LoggedTunableMeasure<>("Drive/Auto Align/Angular Threshold", Radians.of(0.25)))
          .build();

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      SwerveDriveConstants.builder()
          .withDriveConfig(DRIVE_CONFIG)
          .withDriveGains(DRIVE_GAINS)
          .withTurnGains(TURN_GAINS)
          .withAutoTranslationGains(TRANSLATION_AUTO_GAINS)
          .withAutoRotationGains(ROTATION_AUTO_GAINS)
          .withAutoAlignConstants(AUTO_ALIGN_CONSTANTS)
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

  public static final LimelightConfig LIMELIGHT_CLIMBER_CONFIG =
      LimelightConfig.builder()
          .key("climber")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.060142,
                  -0.398769,
                  0.305,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90.409532))))
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
