package frc.robot.subsystems.v0_Poot;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoGains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.Gains;

public class V0_PootConstants {
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
}
