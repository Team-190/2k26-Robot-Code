package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOGompeiVision;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class CameraGompeiVisionTest {

  @Test
  public void testGompeiVisionCamera() {
    CameraIOGompeiVision io = mock(CameraIOGompeiVision.class);

    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d cameraPoseRel = new Pose3d(0.5, 0.0, 0.5, new edu.wpi.first.math.geometry.Rotation3d());

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei")
            .cameraType(CameraType.THRIFTYCAM)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(cameraPoseRel)
            .build();

    AprilTag tag1 =
        new AprilTag(1, new Pose3d(1.0, 2.0, 3.0, new edu.wpi.first.math.geometry.Rotation3d()));
    AprilTagFieldLayout layout = new AprilTagFieldLayout(List.of(tag1), 16.0, 8.0);

    List<VisionPoseObservation> poses = new ArrayList<>();
    List<VisionMultiTxTyObservation> txtys = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poses::addAll;
    Consumer<List<VisionMultiTxTyObservation>> txtyObserver = txtys::addAll;

    CameraGompeiVision camera =
        new CameraGompeiVision(
            io,
            config,
            () -> layout,
            0.5,
            () -> new Pose2d(0.0, 0.0, new Rotation2d()),
            List.of(poseObserver),
            List.of(txtyObserver));

    assertEquals("gompei", camera.getName());
    assertEquals(cameraPoseRel, camera.getCurrentCameraPose());

    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class)) {

      // --- Test empty frames ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames = new double[][] {new double[] {}};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Invalid Switch Case ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames = new double[][] {new double[] {3.0}};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      camera.periodic();
      mockDS.verify(() -> DriverStation.reportWarning("FAILED TO CAPTURE FRAMES", false), times(1));

      // --- Test Case 1: One Pose (Valid, Inside Field) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0,
                        0.0, // values[0]=1, values[1]=0
                        1.0,
                        2.0,
                        0.0, // translation (1, 2, 0)
                        1.0,
                        0.0,
                        0.0,
                        0.0, // quaternion (w=1, x=0, y=0, z=0)
                        1.0, // tagId = 1
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8, // corners
                        2.0 // distance = 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      txtys.clear();
      camera.periodic();

      assertFalse(poses.isEmpty());
      assertEquals(1, poses.size());
      assertEquals(1, txtys.size());
      assertEquals(1, poses.get(0).tagIds().size());
      assertTrue(poses.get(0).tagIds().contains(1));

      // --- Test Case 1: Outside Field boundary ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 20.0, 2.0,
                        0.0, // X is 20, field is 16. With 0.5 margin, it's outside.
                        1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 2: Multiple Poses ---
      // We will provide two poses, error0 = 0.05, error1 = 0.5 (ambiguity resolution should favor
      // pose 0)
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.05, // values[0]=2, error0=0.05
                        1.0, 2.0, 0.0, // pose0 translation
                        1.0, 0.0, 0.0, 0.0, // pose0 quaternion (no rot)
                        0.5, // error1=0.5
                        1.0, 5.0, 0.0, // pose1 translation
                        1.0, 0.0, 0.0, 0.0, // pose1 quaternion
                        1.0, // tagId=1
                        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());
      // Pose 0 is selected, robot X should be around 1.0 - camera translation
      assertTrue(poses.get(0).pose().getX() < 2.0);

      // Test Case 2: Ambiguity favoring pose 1
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.5, // values[0]=2, error0=0.5
                        1.0, 2.0, 0.0, // pose0 translation
                        1.0, 0.0, 0.0, 0.0, // pose0 quaternion
                        0.05, // error1=0.05
                        1.5, 2.0, 0.0, // pose1 translation
                        1.0, 0.0, 0.0, 0.0, // pose1 quaternion
                        1.0, // tagId=1
                        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());
    }
  }
}
