package first.robot.v1_DoomSpiral;

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
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

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
          Units.inchesToMeters(28.5),
          0.0,
          0.0,
          0.0,
          0.0,
          2.0);

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Drive")
          .withKP(V1_DoomSpiralTunerConstants.driveGains.kP)
          .withKD(V1_DoomSpiralTunerConstants.driveGains.kD)
          .withKS(V1_DoomSpiralTunerConstants.driveGains.kS)
          .withKV(V1_DoomSpiralTunerConstants.driveGains.kV)
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Turn")
          .withKP(V1_DoomSpiralTunerConstants.steerGains.kP)
          .withKD(V1_DoomSpiralTunerConstants.steerGains.kD)
          .withKS(V1_DoomSpiralTunerConstants.steerGains.kS)
          .withKV(V1_DoomSpiralTunerConstants.steerGains.kV)
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
      Gains.builder().withPrefix("Drive/Auto Align/Theta").withKP(6.5).withKD(0.2).build();

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

  public static final StaticLimelightConfig LIMELIGHT_SHOOTER_CONFIG =
      StaticLimelightConfig.builder()
          .key("shooter")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.055,
                  0.054,
                  0.538,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90 - 62.000),
                      Units.degreesToRadians(-90.000))))
          .enableRewind(true)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_CLIMBER_CONFIG =
      StaticLimelightConfig.builder()
          .key("climber")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.060142,
                  0.398769,
                  0.305,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90.409532))))
          .enableRewind(true)
          .build();

  public static final double AUTO_CORRECTION_THRESHOLD_METERS = 0.2;
}
