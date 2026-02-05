package frc.robot.subsystems.v1_Gamma;

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

public class V1_GammaConstants {
  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
          V1_GammaTunerConstants.kCANBus,
          V1_GammaTunerConstants.DrivetrainConstants.Pigeon2Id,
          V1_GammaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          V1_GammaTunerConstants.kWheelRadius.in(Meters),
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          V1_GammaTunerConstants.FrontLeft,
          V1_GammaTunerConstants.FrontRight,
          V1_GammaTunerConstants.BackLeft,
          V1_GammaTunerConstants.BackRight,
          V1_GammaTunerConstants.kDriveClosedLoopOutput,
          V1_GammaTunerConstants.kSteerClosedLoopOutput,
          Units.inchesToMeters(37.25),
          Units.inchesToMeters(28.25)); // TODO: Double check these

  public static final Gains GAINS =
      new Gains(
          new LoggedTunableNumber("Drive/Teleop/Drive Ks", V1_GammaTunerConstants.driveGains.kS),
          new LoggedTunableNumber("Drive/Teleop/Drive Kv", V1_GammaTunerConstants.driveGains.kV),
          new LoggedTunableNumber("Drive/Teleop/Drive Kp", V1_GammaTunerConstants.driveGains.kP),
          new LoggedTunableNumber("Drive/Teleop/Drive Kd", V1_GammaTunerConstants.driveGains.kD),
          new LoggedTunableNumber("Drive/Teleop/Turn Kp", V1_GammaTunerConstants.steerGains.kP),
          new LoggedTunableNumber("Drive/Teleop/Turn Kd", V1_GammaTunerConstants.steerGains.kD));

  public static final AutoGains AUTO_GAINS =
      new AutoGains(
          new LoggedTunableNumber("Drive/Auto/Translation Kp", 0.0),
          new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kp", 0.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0));

  public static final AutoAlignNearConstants AUTO_ALIGN_NEAR_CONSTANTS =
      new AutoAlignNearConstants(
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/X/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Max Velocity", 0.0)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Max Velocity", 0.0)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Max Velocity", 0.0)),
          new LoggedTunableNumber("Drive/Auto Align/Position Threshold Meters", 0.0));

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      new SwerveDriveConstants(
          DRIVE_CONFIG,
          GAINS,
          AUTO_GAINS,
          AUTO_ALIGN_NEAR_CONSTANTS,
          ODOMETRY_FREQUENCY,
          DRIVER_DEADBAND,
          OPERATOR_DEADBAND);

  public static final LimelightConfig LIMELIGHT_CONFIG = // TODO: Put in actual values
      LimelightConfig.builder()
          .key("limelight")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(0)
          .verticalFOV(0)
          .megatagXYStdev(0)
          .metatagThetaStdev(0)
          .megatag2XYStdev(0)
          .robotToCameraTransform(new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
          .build();
}
