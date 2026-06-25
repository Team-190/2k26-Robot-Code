package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class LocalizationTest {
  @Test
  public void testLocalization() {
    AprilTag tag1 = new AprilTag(1, new Pose3d(1.0, 2.0, 3.0, new Rotation3d()));
    FieldZone zone = new FieldZone(Set.of(tag1));

    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    Localization loc = new Localization(List.of(zone), kinematics, 1.0);

    loc.resetPose(new Pose2d(1.0, 2.0, new Rotation2d()));
    assertEquals(new Pose2d(1.0, 2.0, new Rotation2d()), loc.getEstimatedPose(zone));
    assertEquals(new Rotation2d(), loc.getHeading());

    SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d()),
          new SwerveModulePosition(0.0, new Rotation2d())
        };
    loc.addOdometryObservation(1.0, new Rotation2d(), modulePositions);

    VisionPoseObservation poseObs =
        new VisionPoseObservation(
            new Pose2d(1.1, 2.1, new Rotation2d()),
            Set.of(1),
            1.1,
            edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 0.1));
    loc.addPoseObservations(List.of(poseObs));

    // Test filtering out NaN poses
    VisionPoseObservation nanPoseObs =
        new VisionPoseObservation(
            new Pose2d(Double.NaN, 2.1, new Rotation2d()),
            Set.of(1),
            1.1,
            edu.wpi.first.math.VecBuilder.fill(0.1, 0.1, 0.1));
    loc.addPoseObservations(List.of(nanPoseObs));

    VisionMultiTxTyObservation txTyObs =
        new VisionMultiTxTyObservation(
            1,
            new double[] {0.1, 0.1, 0.1, 0.1},
            new double[] {0.2, 0.2, 0.2, 0.2},
            2.0,
            1.2,
            new Pose3d(1.0, 2.0, 0.5, new Rotation3d()));
    loc.addTxTyObservations(List.of(txTyObs));

    // Also get estimated pose for an unknown/empty zone to hit else branch
    FieldZone emptyZone = new FieldZone(Set.of());
    assertNotNull(loc.getEstimatedPose(emptyZone));
  }
}
