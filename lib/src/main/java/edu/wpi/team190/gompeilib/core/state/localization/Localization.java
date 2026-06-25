package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class Localization {
  private final List<EstimationRegion> estimationRegions;

  private final SwerveDrivePoseEstimator globalPoseEstimator;

  public Localization(
      List<FieldZone> estimationZones,
      SwerveDriveKinematics kinematics,
      double bufferLengthSeconds) {
    this.globalPoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.kZero,
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            Pose2d.kZero);

    this.estimationRegions =
        estimationZones.stream()
            .map(zone -> new EstimationRegion(zone.aprilTags(), kinematics))
            .toList();
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d rawHeading, SwerveModulePosition[] modulePositions) {
    globalPoseEstimator.updateWithTime(timestamp, rawHeading, modulePositions);
    estimationRegions.forEach(
        region -> region.addOdometryObservation(timestamp, rawHeading, modulePositions));
  }

  public void addPoseObservations(List<VisionPoseObservation> poseObservations) {
    poseObservations.stream()
        .filter(observation -> !GeometryUtil.isNaN(observation.pose()))
        .forEach(
            observation ->
                globalPoseEstimator.addVisionMeasurement(
                    observation.pose(), observation.timestamp(), observation.stddevs()));
    estimationRegions.forEach(
        zone ->
            poseObservations.stream()
                .filter(
                    observation -> zone.getAprilTags().keySet().containsAll(observation.tagIds()))
                .filter(observation -> !GeometryUtil.isNaN(observation.pose()))
                .forEach(zone::addPoseObservation));
  }

  public void addTxTyObservations(List<VisionMultiTxTyObservation> txTyObservations) {
    estimationRegions.forEach(
        zone ->
            txTyObservations.stream()
                .filter(observation -> zone.getAprilTags().containsKey(observation.tagId()))
                .forEach(zone::addTxTyObservation));
  }

  public Pose2d getEstimatedPose(FieldZone fieldZone) {
    Set<Integer> zoneTagIDs =
        fieldZone.aprilTags().stream().map(t -> t.ID).collect(Collectors.toSet());

    return estimationRegions.stream()
        .filter(region -> zoneTagIDs.equals(region.getAprilTags().keySet()))
        .findFirst()
        .map(EstimationRegion::getEstimatedPose)
        .orElseGet(globalPoseEstimator::getEstimatedPosition);
  }

  public Rotation2d getHeading() {
    return globalPoseEstimator.getEstimatedPosition().getRotation();
  }

  public void resetPose(Pose2d pose) {
    for (EstimationRegion region : estimationRegions) {
      region.resetPose(pose);
    }
    globalPoseEstimator.resetPose(pose);
  }
}
