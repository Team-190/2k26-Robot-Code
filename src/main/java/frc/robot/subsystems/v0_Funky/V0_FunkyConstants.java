package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.units.Units;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoGains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.Gains;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;

public class V0_FunkyConstants {
  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(null, 0, 0, 0, null, null, null, null, null, null, 0, 0);
  public static final Gains GAINS = new Gains(null, null, null, null, null, null);
  public static final AutoGains AUTO_GAINS = new AutoGains(null, null, null, null);
  public static final AutoAlignNearConstants AUTO_ALIGN_NEAR_CONSTANTS =
      new AutoAlignNearConstants(null, null, null, null);
  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      new SwerveDriveConstants(
          DRIVE_CONFIG,
          GAINS,
          AUTO_GAINS,
          AUTO_ALIGN_NEAR_CONSTANTS,
          OPERATOR_DEADBAND,
          ODOMETRY_FREQUENCY,
          DRIVER_DEADBAND);

  public static final LimelightConfig CENTER_LIMELIGHT_CONSTANTS =
      LimelightConfig.builder()
          .key("center")
          .robotToCameraTransform(
              new Transform3d(
                  0,
                  0.241,
                  0.2,
                  new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-90))))
          .build();
}
