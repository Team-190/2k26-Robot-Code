package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import java.util.List;
import java.util.stream.Collectors;

public class V0_FunkyRobotState {
  private static AprilTagFieldLayout fieldLayout;
  private static List<FieldZone> fieldZones;
  private static Localization localization;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    FieldZone globalZone =
        new FieldZone(fieldLayout.getTags().stream().collect(Collectors.toSet()));

    fieldZones = List.of(globalZone);

    localization =
        new Localization(
            fieldZones, V0_FunkyConstants.DRIVE_CONSTANTS.DRIVE_CONFIG.kinematics(), 2);
  }

  public static void periodic(Rotation2d heading, SwerveModulePosition[] modulePositions) {

    localization.addOdometryObservation(Timer.getTimestamp(), heading, modulePositions);
  }

  public static void resetPose(Pose2d pose) {
    localization.resetPose(pose);
  }

  public static Rotation2d getHeading() {
    return localization.getHeading();
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(fieldZones.get(0));
  }
}
