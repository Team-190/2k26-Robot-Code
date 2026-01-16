package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import frc.robot.util.InternalLoggedTracer;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyRobotState {
  private static AprilTagFieldLayout fieldLayout;
  private static List<FieldZone> fieldZones;
  private static Localization localization;

  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    FieldZone globalZone =
        new FieldZone(fieldLayout.getTags().stream().collect(Collectors.toSet()));

    fieldZones = List.of(globalZone);

    localization =
        new Localization(
            fieldZones, V0_FunkyConstants.DRIVE_CONSTANTS.DRIVE_CONFIG.kinematics(), 2);
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions) {
    InternalLoggedTracer.reset();
    V0_FunkyRobotState.robotHeading = robotHeading;
    V0_FunkyRobotState.modulePositions = modulePositions;
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Logger.recordOutput("Robot/Pose/GlobalPose", getGlobalPose());
  }

  public static void addFieldLocalizerVisionMeasurement(List<VisionPoseObservation> observations) {
    localization.addPoseObservations(observations);
  }

  public static void resetPose(Pose2d pose) {
    localization.resetPose(pose);
  }

  public static Rotation2d getHeading() {
    return localization.getEstimatedPose(fieldZones.get(0)).getRotation();
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(fieldZones.get(0));
  }
}
