package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.*;
import frc.robot.util.*;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final SwerveDrivePoseEstimator fieldLocalizer;
  private static final SwerveDriveOdometry odometry;
  private static final TimeInterpolatableBuffer<Pose2d> poseBuffer;

  @AutoLogOutput(key = NTPrefixes.POSE_DATA + "Robot Heading")
  private static Rotation2d robotHeading;

  @AutoLogOutput(key = NTPrefixes.POSE_DATA + "Heading Offset")
  private static Rotation2d headingOffset;

  private static SwerveModulePosition[] modulePositions;
  @Getter private static HeadingData headingData;
  @Getter @Setter private static RobotMode mode;

  static {
    robotHeading = new Rotation2d();
    headingOffset = new Rotation2d();
    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    fieldLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());

    odometry =
        new SwerveDriveOdometry(
            DriveConstants.DRIVE_CONFIG.kinematics(), new Rotation2d(), modulePositions);
    headingData = new HeadingData(robotHeading, NetworkTablesJNI.now(), 0.0);

    poseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions) {

    InternalLoggedTracer.reset();
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);

    odometry.update(robotHeading, modulePositions);

    // Add pose to buffer at timestamp
    poseBuffer.addSample(Timer.getTimestamp(), odometry.getPoseMeters());

    InternalLoggedTracer.record("Update Localizers", "RobotState/Periodic");

    headingData =
        new HeadingData(
            robotHeading.minus(headingOffset), latestRobotHeadingTimestamp, robotYawVelocity);

    InternalLoggedTracer.record("Generate Records", "RobotState/Periodic");

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);
  }

  public static void addFieldLocalizerVisionMeasurement(VisionObservation observation) {
    if (!GeometryUtil.isZero(observation.pose())) {
      fieldLocalizer.addVisionMeasurement(
          observation.pose(), observation.timestamp(), observation.stddevs());
    }
  }

  @AutoLogOutput(key = NTPrefixes.POSE_DATA + "Field Pose")
  public static Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  @AutoLogOutput(key = NTPrefixes.POSE_DATA + "Odometry Pose")
  public static Pose2d getRobotPoseOdometry() {
    return odometry.getPoseMeters();
  }

  public static void resetRobotPose(Pose2d pose) {
    headingOffset = robotHeading.minus(pose.getRotation());
    fieldLocalizer.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
    poseBuffer.clear();
  }

  public static Optional<Pose2d> getBufferedPose(double timestamp) {
    return poseBuffer.getSample(timestamp);
  }

  public enum RobotMode {
    DISABLED,
    TELEOP,
    AUTO;

    public static boolean enabled(RobotMode mode) {
      return mode.equals(TELEOP) || mode.equals(AUTO);
    }

    public static boolean disabled(RobotMode mode) {
      return mode.equals(DISABLED);
    }

    public static boolean teleop(RobotMode mode) {
      return mode.equals(TELEOP);
    }

    public static boolean auto(RobotMode mode) {
      return mode.equals(AUTO);
    }

    public static boolean enabled() {
      return enabled(RobotState.getMode());
    }

    public static boolean disabled() {
      return disabled(RobotState.getMode());
    }

    public static boolean teleop() {
      return teleop(RobotState.getMode());
    }

    public static boolean auto() {
      return auto(RobotState.getMode());
    }
  }

  public record HeadingData(
      Rotation2d robotHeading, long latestRobotHeadingTimestamp, double robotYawVelocity) {}

  public record VisionObservation(Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {}
}
