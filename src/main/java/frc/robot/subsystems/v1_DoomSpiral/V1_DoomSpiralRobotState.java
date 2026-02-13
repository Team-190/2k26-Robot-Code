package frc.robot.subsystems.v1_DoomSpiral;

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
import frc.robot.util.NTPrefixes;
import java.util.HashSet;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotState {
  private static final AprilTagFieldLayout fieldLayout;
  private static final Localization localization;

  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  public static final V1_DoomSpiralShooterOffsets SHOOTER_OFFSETS;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Score Angle")
  @Getter
  private static final Rotation2d scoreAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Shooter/Score Velocity")
  @Getter
  private static final double scoreVelocity;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Feed Angle")
  @Getter
  private static final Rotation2d feedAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Shooter/Feed Velocity")
  @Getter
  private static final double feedVelocity;

  private static final FieldZone globalZone;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    globalZone = new FieldZone(new HashSet<>(fieldLayout.getTags()));

    localization =
        new Localization(
            List.of(globalZone),
            V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);

    scoreAngle = new Rotation2d();
    scoreVelocity = 0;

    feedAngle = new Rotation2d();
    feedVelocity = 0;

    SHOOTER_OFFSETS = new V1_DoomSpiralShooterOffsets(0, new Rotation2d(0));
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions) {
    InternalLoggedTracer.reset();
    V1_DoomSpiralRobotState.robotHeading = robotHeading;
    V1_DoomSpiralRobotState.modulePositions = modulePositions;
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
    return localization.getEstimatedPose(globalZone).getRotation();
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(globalZone);
  }
}
