package first.robot.v2_Turnover;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.GompeiTunables;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

public class V2_TurnoverConstants {

  public static final Transform3d ROBOT_TO_SHOOTER_TRANSFORM =
      new Transform3d(
          -.017463, -.163513, .371475, new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));

  public static final Transform2d ROBOT_TO_SHOOTER_TRANSFORM_2D =
      new Transform2d(-.017463, -.163513, Rotation2d.fromDegrees(90.0));
  public static final Transform3d SHOOTER_TO_LIMELIGHT_TRANSFORM =
      new Transform3d(.015672, -.149274, .170432, new Rotation3d(0.0, .580224, 0.0));

  public static final DriveConfig DRIVE_CONFIG =
      DriveConfig.builder()
          .withCanBus(V2_TurnoverTunerConstants.kCANBus)
          .withPigeon2Id(V2_TurnoverTunerConstants.DrivetrainConstants.Pigeon2Id)
          .withMaxLinearVelocityMetersPerSecond(
              V2_TurnoverTunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
          .withWheelRadiusMeters(V2_TurnoverTunerConstants.kWheelRadius.in(Meters))
          .withDriveModel(DCMotor.getKrakenX60Foc(1))
          .withTurnModel(DCMotor.getKrakenX44Foc(1))
          .withFrontLeft(V2_TurnoverTunerConstants.FrontLeft)
          .withFrontRight(V2_TurnoverTunerConstants.FrontRight)
          .withBackLeft(V2_TurnoverTunerConstants.BackLeft)
          .withBackRight(V2_TurnoverTunerConstants.BackRight)
          .withDriveClosedLoopOutputType(V2_TurnoverTunerConstants.kDriveClosedLoopOutput)
          .withSteerClosedLoopOutputType(V2_TurnoverTunerConstants.kSteerClosedLoopOutput)
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
          .withPrefix("Drive/Teleoperated/Drive")
          .withKP(V2_TurnoverTunerConstants.driveGains.kP)
          .withKD(V2_TurnoverTunerConstants.driveGains.kD)
          .withKS(V2_TurnoverTunerConstants.driveGains.kS)
          .withKV(V2_TurnoverTunerConstants.driveGains.kV)
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Turn")
          .withKP(V2_TurnoverTunerConstants.steerGains.kP)
          .withKD(V2_TurnoverTunerConstants.steerGains.kD)
          .withKS(V2_TurnoverTunerConstants.steerGains.kS)
          .withKV(V2_TurnoverTunerConstants.steerGains.kV)
          .build();

  public static final Gains TRANSLATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Translation").withKP(5.0).withKD(0.0).build();

  public static final Gains ROTATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Rotation").withKP(5.0).withKD(0.0).build();

  public static final Gains AUTO_ALIGN_X_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/X").withKP(3.0).withKD(0.15).build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/X")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.03))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/Y").withKP(3.0).withKD(0.15).build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/Y")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.05))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS =
      Gains.builder()
          .withPrefix("Drive/Auto Align/Theta")
          .withKP(2.0 * Math.PI)
          .withKD(0.05)
          .build();

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularPositionConstraints.builder()
          .withPrefix("Drive/Auto Align/Theta")
          .withMaxVelocity(RadiansPerSecond.of(Math.PI))
          .withMaxAcceleration(RadiansPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Degrees.of(0.5))
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
              GompeiTunables.value("Drive/Auto Align/Position Threshold", Inches.of(0.25)))
          .withAngularThreshold(
              GompeiTunables.value("Drive/Auto Align/Angular Threshold", Radians.of(0.25)))
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
                  -0.275571 + Units.inchesToMeters(2.3125),
                  0.305,
                  0.443327,
                  Rotation3d.ZERO
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
                  Rotation3d.ZERO
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
  public static final double AUTO_CORRECTION_THRESHOLD_METERS = 0.5;
}
