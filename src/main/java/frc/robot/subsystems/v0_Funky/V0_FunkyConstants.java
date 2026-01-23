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
      new DriveConfig(
          V0_FunkyTunerConstants.kCANBus,
          V0_FunkyTunerConstants.DrivetrainConstants.Pigeon2Id,
          V0_FunkyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          V0_FunkyTunerConstants.kWheelRadius.in(Meters),
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          V0_FunkyTunerConstants.FrontLeft,
          V0_FunkyTunerConstants.FrontRight,
          V0_FunkyTunerConstants.BackLeft,
          V0_FunkyTunerConstants.BackRight,
          V0_FunkyTunerConstants.kDriveClosedLoopOutput,
          V0_FunkyTunerConstants.kSteerClosedLoopOutput,
          Units.inchesToMeters(34.5),
          Units.inchesToMeters(34.5));
  public static final Gains GAINS =
      new Gains(
          new LoggedTunableNumber("Drive/Teleop/Drive Ks", V0_FunkyTunerConstants.driveGains.kS),
          new LoggedTunableNumber("Drive/Teleop/Drive Kv", V0_FunkyTunerConstants.driveGains.kV),
          new LoggedTunableNumber("Drive/Teleop/Drive Kp", V0_FunkyTunerConstants.driveGains.kP),
          new LoggedTunableNumber("Drive/Teleop/Drive Kd", V0_FunkyTunerConstants.driveGains.kD),
          new LoggedTunableNumber("Drive/Teleop/Turn Kp", V0_FunkyTunerConstants.steerGains.kP),
          new LoggedTunableNumber("Drive/Teleop/Turn Kd", V0_FunkyTunerConstants.steerGains.kD));
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

  public static final GenericRollerConstants FEED_CONSTANTS =
      new GenericRollerConstants(
          10,
          60,
          DCMotor.getKrakenX60(1),
          2,
          MomentOfInertia.ofBaseUnits(0.05, KilogramSquareMeters),
          false);
  public static final int[] SHOOTER_IDS = {11};
  public static final DCMotor[] SHOOTER_MOTORS = {DCMotor.getKrakenX60(1)};
  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      new GenericFlywheelConstants(
          SHOOTER_IDS,
          false,
          true,
          60.0,
          0.05,
          new GenericFlywheelConstants.Gains(
              new LoggedTunableNumber("Shooter/Kp"),
              new LoggedTunableNumber("Shooter/Kd"),
              new LoggedTunableNumber("Shooter/Ks"),
              new LoggedTunableNumber("Shooter/Kv"),
              new LoggedTunableNumber("Shooter/Ka")),
          SHOOTER_MOTORS[0],
          new GenericFlywheelConstants.Constraints(
              new LoggedTunableNumber("Shooter/MaxAccel"),
              new LoggedTunableNumber("Shooter/CruiseVel"),
              new LoggedTunableNumber("Shooter/GoalTolerance")),
          1.0,
          InvertedValue.CounterClockwise_Positive);

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      new SwerveDriveConstants(
          DRIVE_CONFIG,
          GAINS,
          AUTO_GAINS,
          AUTO_ALIGN_NEAR_CONSTANTS,
          OPERATOR_DEADBAND,
          ODOMETRY_FREQUENCY,
          DRIVER_DEADBAND);

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
