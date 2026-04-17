package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;

public class V2_DeltaConstants {

  public static final Transform3d ROBOT_TO_SHOOTER_TRANSFORM =
      new Transform3d(
          -.017463,
          -.163513,
          .371475,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.703125 - 6.328125)));

  public static final Transform2d ROBOT_TO_SHOOTER_TRANSFORM_2D =
      new Transform2d(-.017463, -.163513, Rotation2d.fromDegrees(90.0));
  public static final Transform3d SHOOTER_TO_LIMELIGHT_TRANSFORM =
      new Transform3d(.015672, -.149274, .170432, new Rotation3d(0.0, .580224, 0.0));

  public static final DriveConfig DRIVE_CONFIG =
      DriveConfig.builder()
          .withCanBus(V2_DeltaTunerConstants.kCANBus)
          .withPigeon2Id(V2_DeltaTunerConstants.DrivetrainConstants.Pigeon2Id)
          .withMaxLinearVelocityMetersPerSecond(
              V2_DeltaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
          .withWheelRadiusMeters(V2_DeltaTunerConstants.kWheelRadius.in(Meters))
          .withDriveModel(DCMotor.getKrakenX60Foc(1))
          .withTurnModel(DCMotor.getKrakenX44Foc(1))
          .withFrontLeft(V2_DeltaTunerConstants.FrontLeft)
          .withFrontRight(V2_DeltaTunerConstants.FrontRight)
          .withBackLeft(V2_DeltaTunerConstants.BackLeft)
          .withBackRight(V2_DeltaTunerConstants.BackRight)
          .withDriveClosedLoopOutputType(V2_DeltaTunerConstants.kDriveClosedLoopOutput)
          .withSteerClosedLoopOutputType(V2_DeltaTunerConstants.kSteerClosedLoopOutput)
          .withBumperWidth(Units.inchesToMeters(37.5))
          .withBumperLength(Units.inchesToMeters(28.5))
          .withTrackWidth(Units.inchesToMeters(0))
          .withRobotMOI(7.897)
          .withModuleCurrentLimit(60.0)
          .withRobotMassKilograms(67.000)
          .withWheelCOF(2.0)
          .build();

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kp", V2_DeltaTunerConstants.driveGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kd", V2_DeltaTunerConstants.driveGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Ks", V2_DeltaTunerConstants.driveGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kv", V2_DeltaTunerConstants.driveGains.kV))
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kp", V2_DeltaTunerConstants.steerGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kd", V2_DeltaTunerConstants.steerGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Ks", V2_DeltaTunerConstants.steerGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kv", V2_DeltaTunerConstants.steerGains.kV))
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

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularPositionConstraints.builder()
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

  public static final StaticLimelightConfig LIMELIGHT_LEFT_CONFIG = // CLIMBER
      StaticLimelightConfig.builder()
          .key("left")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.275571,
                  0.305,
                  0.443327,
                  Rotation3d.kZero
                      .rotateBy(new Rotation3d(0, Units.degreesToRadians(17.114), 0))
                      .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(134)))))
          .enableRewind(true)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_RIGHT_CONFIG = // SHOOTER
      StaticLimelightConfig.builder()
          .key("right")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -.202,
                  -.326,
                  .456,
                  Rotation3d.kZero
                      .rotateBy(new Rotation3d(0, Units.degreesToRadians(20.0), 0))
                      .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(207.9627321)))))
          .enableRewind(true)
          .build();
  public static final StaticLimelightConfig LIMELIGHT_INTAKE_CONFIG =
      StaticLimelightConfig.builder()
          .key("intake")
          .cameraType(CameraType.LIMELIGHT_4)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  Inches.of(-7.176137),
                  Inches.of(6.540012),
                  Inches.of(20.459907),
                  new Rotation3d(0.0, Units.degreesToRadians(90 - 57.826644), 0)))
          .enableRewind(true)
          .build();
}
