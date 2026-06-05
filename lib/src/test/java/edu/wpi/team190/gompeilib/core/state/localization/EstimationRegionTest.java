package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class EstimationRegionTest {
  @Test
  public void testEstimationRegion() {
    AprilTag tag1 = new AprilTag(1, new Pose3d(1.0, 2.0, 3.0, new Rotation3d()));
    AprilTag tag2 = new AprilTag(2, new Pose3d(4.0, 5.0, 6.0, new Rotation3d()));

    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    EstimationRegion region = new EstimationRegion(Set.of(tag1, tag2), kinematics);

    assertNotNull(region.getAprilTags());
    assertEquals(2, region.getAprilTags().size());
    assertEquals(new Pose3d(1.0, 2.0, 3.0, new Rotation3d()), region.getAprilTags().get(1));

    region.resetPose(new Pose2d(1.0, 2.0, new Rotation2d()));
    assertEquals(new Pose2d(1.0, 2.0, new Rotation2d()), region.getEstimatedPose());

    SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d())
        };
    region.addOdometryObservation(1.0, new Rotation2d(), modulePositions);

    VisionPoseObservation poseObs =
        new VisionPoseObservation(
            new Pose2d(1.1, 2.1, new Rotation2d()),
            Set.of(1),
            1.1,
            edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 0.1));
    region.addPoseObservation(poseObs);

    // Test addTxTyObservation
    VisionMultiTxTyObservation txTyObs =
        new VisionMultiTxTyObservation(
            1,
            new double[] {0.1, 0.1, 0.1, 0.1},
            new double[] {0.2, 0.2, 0.2, 0.2},
            2.0,
            1.2,
            new Pose3d(1.0, 2.0, 0.5, new Rotation3d()));
    region.addTxTyObservation(txTyObs);
  }
}
