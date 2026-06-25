package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import lombok.Getter;

/**
 * Represents an independent localization context that estimates the robot's field-relative pose
 * using a constrained subset of vision targets.
 *
 * <p>An {@code EstimationRegion} owns its own {@link SwerveDrivePoseEstimator} and is configured
 * with a fixed set of AprilTags that are considered valid measurement sources for this region. This
 * allows multiple estimators to run in parallel, each optimized for different areas of the field,
 * tag groupings, or sensing conditions.
 *
 * <p>The estimator continuously integrates drivetrain odometry and accepts vision updates in two
 * forms:
 *
 * <ul>
 *   <li>Full field-relative pose observations (e.g. solvePNP-based estimates)
 *   <li>Angular {@code tx/ty}-based observations that are projected into a 2D field pose using
 *       known AprilTag locations and the camera transform
 * </ul>
 *
 * Vision measurements are assumed to originate only from the tags assigned to this region;
 * observations referencing unknown tags are ignored.
 *
 * <p>This abstraction enables higher-level localization logic to dynamically select or weight
 * estimators based on robot position, visibility, confidence, or game-specific constraints, without
 * entangling tag-selection logic with the underlying state estimation.
 */
public class EstimationRegion {
  @Getter private final Map<Integer, Pose3d> aprilTags;
  private final SwerveDrivePoseEstimator poseEstimator;

  public EstimationRegion(Set<AprilTag> aprilTags, SwerveDriveKinematics kinematics) {
    this.aprilTags =
        aprilTags.stream()
            .map(aprilTag -> Map.entry(aprilTag.ID, aprilTag.pose))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < swerveModulePositions.length; i++) {
      swerveModulePositions[i] = new SwerveModulePosition();
    }

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, swerveModulePositions, Pose2d.kZero);
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d heading, SwerveModulePosition[] modulePositions) {
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
  }

  public void addPoseObservation(VisionPoseObservation observation) {
    poseEstimator.addVisionMeasurement(
        observation.pose(), observation.timestamp(), observation.stddevs());
  }

  public void addTxTyObservation(VisionMultiTxTyObservation observation) {

    // Get odometry-based pose at the timestamp
    var sample = poseEstimator.sampleAt(observation.timestamp());
    if (sample.isEmpty()) return;

    // Average tx and ty over four corners
    double tx = 0.0;
    double ty = 0.0;
    for (int j = 0; j < 4; j++) {
      tx += observation.tx()[j];
      ty += observation.ty()[j];
    }
    tx /= 4.0;
    ty /= 4.0;

    Pose3d cameraPose = observation.cameraPose();

    // Project 3D distance onto horizontal plane
    Rotation3d camToTagRotation3d = new Rotation3d(0.0, -cameraPose.getRotation().getY() - ty, 0.0);

    Translation3d camToTag = new Translation3d(observation.distance(), camToTagRotation3d);

    double distance2d = camToTag.toTranslation2d().getNorm();

    // Compute rotation from camera to tag
    // tx and ty are flipped from WPILib convention.
    Rotation2d camToTagRotation2d =
        sample
            .get()
            .getRotation()
            .plus(cameraPose.toPose2d().getRotation().plus(Rotation2d.fromRadians(-tx)));
    int tagId = observation.tagId();

    Pose2d tagPose2d = aprilTags.get(tagId).toPose2d();
    if (tagPose2d == null) return;

    // Compute camera position in field frame
    Rotation2d tagToCameraRotation = camToTagRotation2d.plus(Rotation2d.kPi);

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), tagToCameraRotation)
            .transformBy(GeometryUtil.toTransform2d(distance2d, 0.0))
            .getTranslation();

    // Compute robot poses
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                sample.get().getRotation().plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

    // Use odometry rotation only
    robotPose = new Pose2d(robotPose.getTranslation(), sample.get().getRotation());

    double xystdev =
        LocalizationConstants.XY_STDDEV_COEFFICIENT
            * Math.pow(observation.distance(), LocalizationConstants.XY_STDDEV_DISTANCE_EXPONENT);

    poseEstimator.addVisionMeasurement(
        robotPose,
        observation.timestamp(),
        VecBuilder.fill(xystdev, xystdev, Double.POSITIVE_INFINITY));
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }
}
