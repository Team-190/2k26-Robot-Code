package first.robot.v0_Funky;

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

public class V0_FunkyConstants {
  public static final DriveConfig DRIVE_CONFIG =
      DriveConfig.builder()
          .withCanBus(V0_FunkyTunerConstants.kCANBus)
          .withPigeon2Id(V0_FunkyTunerConstants.DrivetrainConstants.Pigeon2Id)
          .withMaxLinearVelocityMetersPerSecond(
              V0_FunkyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
          .withWheelRadiusMeters(V0_FunkyTunerConstants.kWheelRadius.in(Meters))
          .withDriveModel(DCMotor.getKrakenX60Foc(1))
          .withTurnModel(DCMotor.getKrakenX60Foc(1))
          .withFrontLeft(V0_FunkyTunerConstants.FrontLeft)
          .withFrontRight(V0_FunkyTunerConstants.FrontRight)
          .withBackLeft(V0_FunkyTunerConstants.BackLeft)
          .withBackRight(V0_FunkyTunerConstants.BackRight)
          .withDriveClosedLoopOutputType(V0_FunkyTunerConstants.kDriveClosedLoopOutput)
          .withSteerClosedLoopOutputType(V0_FunkyTunerConstants.kSteerClosedLoopOutput)
          .withBumperLength(Units.inchesToMeters(34.5))
          .withBumperWidth(Units.inchesToMeters(34.5))
          .withTrackWidth(Units.inchesToMeters(0))
          .withRobotMOI(7.897)
          .withModuleCurrentLimit(60.0)
          .withRobotMassKilograms(67.000)
          .withWheelCOF(2.0)
          .build();

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Drive")
          .withKP(V0_FunkyTunerConstants.driveGains.kP)
          .withKD(V0_FunkyTunerConstants.driveGains.kD)
          .withKS(V0_FunkyTunerConstants.driveGains.kS)
          .withKV(V0_FunkyTunerConstants.driveGains.kV)
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Turn")
          .withKP(V0_FunkyTunerConstants.steerGains.kP)
          .withKD(V0_FunkyTunerConstants.steerGains.kD)
          .withKS(V0_FunkyTunerConstants.steerGains.kS)
          .withKV(V0_FunkyTunerConstants.steerGains.kV)
          .build();

  public static final Gains TRANSLATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Translation").withKP(0.0).withKD(0.0).build();

  public static final Gains ROTATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Rotation").withKP(0.0).withKD(0.0).build();

  public static final Gains AUTO_ALIGN_X_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/X").withKP(0.0).withKD(0.0).build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/X")
          .withMaxVelocity(MetersPerSecond.of(0.0))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.0))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/Y").withKP(0.0).withKD(0.0).build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/Y")
          .withMaxVelocity(MetersPerSecond.of(0.0))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.0))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/Theta").withKP(0.0).withKD(0.0).build();

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularPositionConstraints.builder()
          .withPrefix("Drive/Auto Align/Theta")
          .withMaxVelocity(RadiansPerSecond.of(0.0))
          .withMaxAcceleration(RadiansPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Radians.zero())
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
              GompeiTunables.value("Drive/Auto Align/Position Threshold Meters", Inches.of(0.25)))
          .withAngularThreshold(
              GompeiTunables.value("Drive/Auto Align/Angular Threshold Meters", Radians.of(0.25)))
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
          .withDriveGains(DRIVE_GAINS)
          .withTurnGains(TURN_GAINS)
          .withAutoTranslationGains(TRANSLATION_AUTO_GAINS)
          .withAutoRotationGains(ROTATION_AUTO_GAINS)
          .withAutoAlignConstants(AUTO_ALIGN_CONSTANTS)
          .withOdometryFrequency(ODOMETRY_FREQUENCY)
          .withDriverDeadband(DRIVER_DEADBAND)
          .withOperatorDeadband(OPERATOR_DEADBAND)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_CONFIG =
      StaticLimelightConfig.builder()
          .key("right")
          .cameraType(CameraType.LIMELIGHT_4)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .enableRewind(false)
          .robotToCameraTransform(
              new Transform3d(
                  Inches.of(11.25),
                  Inches.of(0.0),
                  Inches.of(13.3),
                  new Rotation3d(0.0, 0.6974, 0.0)))
          .build();
}
