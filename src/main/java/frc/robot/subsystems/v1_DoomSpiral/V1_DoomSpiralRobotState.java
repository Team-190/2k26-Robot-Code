package frc.robot.subsystems.v1_DoomSpiral;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

  public static final InterpolatingTreeMap<Distance, Rotation2d> hoodAngleTree;
  public static final InterpolatingTreeMap<Distance, AngularVelocity> flywheelSpeedTree;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Score Angle")
  @Getter
  private static final Rotation2d scoreAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Feed Angle")
  @Getter
  private static final Rotation2d feedAngle;

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
    feedAngle = new Rotation2d();
    hoodAngleTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) -> start.interpolate(end, t));
    flywheelSpeedTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                AngularVelocity.ofBaseUnits(
                    Interpolator.forDouble()
                        .interpolate(start.in(RadiansPerSecond), end.in(RadiansPerSecond), t),
                    RadiansPerSecond));
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
