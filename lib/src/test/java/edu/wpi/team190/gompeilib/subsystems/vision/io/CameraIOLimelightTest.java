package edu.wpi.team190.gompeilib.subsystems.vision.io;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

public class CameraIOLimelightTest {

  @Test
  public void testCameraIOLimelightStaticConfig() {
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static_ll")
            .robotToCameraTransform(new Transform3d())
            .build();

    CameraIOLimelight io = new CameraIOLimelight(config);
    assertEquals("limelight-static_ll", io.getName());

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class)) {
      LimelightHelpers.PoseEstimate mockEst1 =
          new LimelightHelpers.PoseEstimate(
              new Pose2d(1.0, 2.0, new edu.wpi.first.math.geometry.Rotation2d()),
              1.0,
              2.0,
              1,
              0.0,
              0.0,
              0.0,
              new LimelightHelpers.RawFiducial[] {
                new LimelightHelpers.RawFiducial(5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
              },
              false);

      LimelightHelpers.PoseEstimate mockEst2 =
          new LimelightHelpers.PoseEstimate(
              new Pose2d(3.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()),
              3.0,
              4.0,
              2,
              0.0,
              0.0,
              0.0,
              new LimelightHelpers.RawFiducial[] {
                new LimelightHelpers.RawFiducial(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
              },
              true);

      LimelightHelpers.RawFiducial[] mockFids =
          new LimelightHelpers.RawFiducial[] {
            new LimelightHelpers.RawFiducial(7, 1.0, 2.0, 3.0, 4.0, 5.0, 0.6)
          };

      mockHelpers
          .when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-static_ll"))
          .thenReturn(mockEst1);
      mockHelpers
          .when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-static_ll"))
          .thenReturn(mockEst2);
      mockHelpers
          .when(() -> LimelightHelpers.getRawFiducials("limelight-static_ll"))
          .thenReturn(mockFids);

      CameraIO.LimelightIOInputs inputs = new CameraIO.LimelightIOInputs();
      io.updateInputs(inputs);

      assertEquals(1.0, inputs.mt1PoseEstimate.pose().getX());
      assertEquals(3.0, inputs.mt2PoseEstimate.pose().getX());
      assertEquals(1, inputs.rawFiducials.length);
      assertEquals(7, inputs.rawFiducials[0].id());
      assertEquals(0.6, inputs.rawFiducials[0].ambiguity());
    }
  }

  @Test
  public void testCameraIOLimelightMovingConfig() {
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving_ll")
            .robotToRotationAxisTransform(new Transform3d())
            .rotationAxisToLensTransform(new Transform3d())
            .build();

    CameraIOLimelight io = new CameraIOLimelight(config);
    assertEquals("limelight-moving_ll", io.getName());
  }
}
