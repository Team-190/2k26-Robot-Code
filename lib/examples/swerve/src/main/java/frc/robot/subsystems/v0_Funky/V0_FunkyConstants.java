package frc.robot.subsystems.v0_Funky;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;

public class V0_FunkyConstants {
  public static final SwerveDriveConstants.DriveConfig DRIVE_CONFIG =
      SwerveDriveConstants.DriveConfig.builder()
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

  public static final Gains TRANSLATION_AUTO_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Translation Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0))
          .build();

  public static final Gains ROTATION_AUTO_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Rotation Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0))
          .build();

  public static final Gains AUTO_ALIGN_X_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/X/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.0))
          .build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS =
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Velocity", MetersPerSecond.of(0.0)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/X/Max Velocity", Meters.of(0.0)))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.0))
          .build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS =
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Velocity", MetersPerSecond.of(0.0)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Y/Max Velocity", Meters.of(0.0)))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS =
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 0.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.0))
          .build();

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularPositionConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Velocity", RadiansPerSecond.of(0.0)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Acceleration", RadiansPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Theta/Max Velocity", Radians.of(0.0)))
          .build();

  public static final SwerveDriveConstants.AutoAlignConstants AUTO_ALIGN_NEAR_CONSTANTS =
      SwerveDriveConstants.AutoAlignConstants.builder()
          .withXGains(AUTO_ALIGN_X_GAINS)
          .withXConstraints(AUTO_ALIGN_X_CONSTRAINTS)
          .withYGains(AUTO_ALIGN_Y_GAINS)
          .withYConstraints(AUTO_ALIGN_Y_CONSTRAINTS)
          .withRotationGains(AUTO_ALIGN_THETA_GAINS)
          .withRotationConstraints(AUTO_ALIGN_THETA_CONSTRAINTS)
          .withLinearThreshold(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Position Threshold Meters", Inches.of(0.25)))
          .withAngularThreshold(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Angular Threshold Meters", Radians.of(0.25)))
          .build();

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      SwerveDriveConstants.builder()
          .withDriveConfig(DRIVE_CONFIG)
          .withAutoTranslationGains(TRANSLATION_AUTO_GAINS)
          .withAutoRotationGains(ROTATION_AUTO_GAINS)
          .withAutoAlignConstants(AUTO_ALIGN_NEAR_CONSTANTS)
          .withOdometryFrequency(ODOMETRY_FREQUENCY)
          .withDriverDeadband(DRIVER_DEADBAND)
          .withOperatorDeadband(OPERATOR_DEADBAND)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_CONFIG =
      StaticLimelightConfig.builder()
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
