package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

public class EstimationRegionTest {
  @Test
  public void testEstimationRegion() {
    FieldTag tag1 = new FieldTag(1, new Pose3d(1.0, 2.0, 3.0, new Rotation3d()));
    FieldTag tag2 = new FieldTag(2, new Pose3d(4.0, 5.0, 6.0, new Rotation3d()));

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
            org.wpilib.math.linalg.VecBuilder.fill(0.1, 0.1, 0.1));
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

    // Test addTxTyObservation with empty sample (timestamp far in past)
    VisionMultiTxTyObservation txTyObsPast =
        new VisionMultiTxTyObservation(
            1,
            new double[] {0.1, 0.1, 0.1, 0.1},
            new double[] {0.2, 0.2, 0.2, 0.2},
            2.0,
            -100.0,
            new Pose3d(1.0, 2.0, 0.5, new Rotation3d()));
    region.addTxTyObservation(txTyObsPast);

    // Test addTxTyObservation with missing tag (tag ID 99 is not in the region)
    VisionMultiTxTyObservation txTyObsMissingTag =
        new VisionMultiTxTyObservation(
            99,
            new double[] {0.1, 0.1, 0.1, 0.1},
            new double[] {0.2, 0.2, 0.2, 0.2},
            2.0,
            1.2,
            new Pose3d(1.0, 2.0, 0.5, new Rotation3d()));
    region.addTxTyObservation(txTyObsMissingTag);

    // Test addTxTyObservation with empty sample using Mockito/reflection to force isEmpty() return
    // path
    try {
      org.wpilib.math.estimator.SwerveDrivePoseEstimator mockEstimator =
          mock(org.wpilib.math.estimator.SwerveDrivePoseEstimator.class);
      when(mockEstimator.sampleAt(anyDouble())).thenReturn(Optional.empty());

      java.lang.reflect.Field field = EstimationRegion.class.getDeclaredField("poseEstimator");
      field.setAccessible(true);
      field.set(region, mockEstimator);

      VisionMultiTxTyObservation txTyObsMockedEmpty =
          new VisionMultiTxTyObservation(
              1,
              new double[] {0.1, 0.1, 0.1, 0.1},
              new double[] {0.2, 0.2, 0.2, 0.2},
              2.0,
              1.2,
              new Pose3d(1.0, 2.0, 0.5, new Rotation3d()));
      region.addTxTyObservation(txTyObsMockedEmpty);
    } catch (Exception e) {
      fail(e);
    }
  }
}
