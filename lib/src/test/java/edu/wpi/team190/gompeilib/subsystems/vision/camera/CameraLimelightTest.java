package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class CameraLimelightTest {

  @Test
  public void testCameraStaticLimelight() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static")
            .cameraType(CameraType.LIMELIGHT_3)
            .horizontalFOV(60.0)
            .verticalFOV(40.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToCameraTransform(
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
            .enableRewind(true)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    List<VisionSingleTxTyObservation> singleObs = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poseObs::addAll;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = singleObs::addAll;

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        MockedStatic<GompeiLib> mockLib = mockStatic(GompeiLib.class);
        MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {

      mockDS.when(DriverStation::isEnabled).thenReturn(false);
      mockDS.when(DriverStation::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(10.0);
      mockLib.when(GompeiLib::isTuning).thenReturn(false);

      CameraStaticLimelight camera =
          new CameraStaticLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> new ChassisSpeeds(),
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      assertEquals("limelight-static", camera.getName());

      // --- Setup input data ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);

                // Setup MT1 Pose Estimate
                CameraIO.RawFiducial fid =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        10.0,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        false);

                // Setup MT2 Pose Estimate
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        10.0,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        true);

                // Setup raw fiducials
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};

                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      // Periodic check while disabled
      camera.periodic();
      assertFalse(poseObs.isEmpty());
      assertEquals(2, poseObs.size()); // MT1 & MT2
      assertEquals(1, singleObs.size());

      // Transition to enabled
      mockDS.when(DriverStation::isEnabled).thenReturn(true);
      mockDS.when(DriverStation::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(11.0);

      poseObs.clear();
      singleObs.clear();
      camera.periodic();

      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-static", 0), times(1));
      mockHelpers.verify(() -> LimelightHelpers.SetThrottle("limelight-static", 0), times(1));

      // Rerun enabled periodic with time advance to trigger rewind
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(180.0); // > 165
      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture("limelight-static", 165), times(1));

      // Transition to disabled while tuning
      mockDS.when(DriverStation::isEnabled).thenReturn(false);
      mockDS.when(DriverStation::isDisabled).thenReturn(true);
      mockLib.when(GompeiLib::isTuning).thenReturn(true);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(185.0);

      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.SetIMUMode("limelight-static", 1),
          times(2)); // First in constructor/constructor states
    }
  }

  @Test
  public void testCameraMovingLimelight() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(80.0)
            .verticalFOV(50.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToRotationAxisTransform(
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
            .rotationAxisToLensTransform(
                new Transform3d(new Translation3d(0.1, 0.0, 0.0), new Rotation3d()))
            .enableRewind(true)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    List<VisionSingleTxTyObservation> singleObs = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poseObs::addAll;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = singleObs::addAll;

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {

      mockDS.when(DriverStation::isEnabled).thenReturn(false);
      mockDS.when(DriverStation::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(10.0);

      CameraMovingLimelight camera =
          new CameraMovingLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> Rotation2d.fromDegrees(45), // rotated 45 degrees
              () -> new ChassisSpeeds(),
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      assertEquals("limelight-moving", camera.getName());

      // --- Setup input data ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.subsystems.vision.io.LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);
                CameraIO.RawFiducial fid =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        10.0,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        false);
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        10.0,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        true);
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      camera.periodic();
      assertFalse(poseObs.isEmpty());
      assertEquals(2, poseObs.size());

      // Transition to enabled
      mockDS.when(DriverStation::isEnabled).thenReturn(true);
      mockDS.when(DriverStation::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(11.0);

      camera.periodic();
      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-moving", 0), times(1));
      mockHelpers.verify(() -> LimelightHelpers.SetThrottle("limelight-moving", 0), times(1));

      // Advance time to trigger rewind capture
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(180.0);
      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture("limelight-moving", 165), times(1));

      // Transition back to disabled
      mockDS.when(DriverStation::isEnabled).thenReturn(false);
      mockDS.when(DriverStation::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getFPGATimestamp).thenReturn(185.0);

      camera.periodic();
      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-moving", 1), times(2));
    }
  }
}
