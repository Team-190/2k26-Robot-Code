package edu.wpi.team190.gompeilib.subsystems.vision.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import org.junit.jupiter.api.Test;

public class CameraIOTest {

  private static class DummyCameraIO implements CameraIO {}

  @Test
  public void testDefaultIO() {
    DummyCameraIO io = new DummyCameraIO();
    assertEquals("", io.getName());

    CameraIO.GompeiVisionIOInputs gompeiInputs = new CameraIO.GompeiVisionIOInputs();
    CameraIO.LimelightIOInputs limelightInputs = new CameraIO.LimelightIOInputs();

    // Default no-ops
    io.updateInputs(gompeiInputs);
    io.updateInputs(limelightInputs);

    assertNotNull(gompeiInputs);
    assertNotNull(limelightInputs);
  }

  @Test
  public void testPoseEstimateRecord() {
    // Default constructor
    CameraIO.PoseEstimate defaultEst = new CameraIO.PoseEstimate();
    assertEquals(Pose2d.kZero, defaultEst.pose());
    assertEquals(0.0, defaultEst.timestampSeconds());
    assertEquals(0.0, defaultEst.latency());
    assertEquals(0, defaultEst.tagCount());
    assertEquals(0.0, defaultEst.tagSpan());
    assertEquals(0.0, defaultEst.avgTagDist());
    assertEquals(0.0, defaultEst.avgTagArea());
    assertEquals(0, defaultEst.rawFiducials().length);
    assertFalse(defaultEst.isMegaTag2());

    // Custom constructor
    Pose2d pose = new Pose2d();
    CameraIO.RawFiducial[] fids = new CameraIO.RawFiducial[] {new CameraIO.RawFiducial()};
    CameraIO.PoseEstimate customEst =
        new CameraIO.PoseEstimate(pose, 1.23, 4.56, 1, 2.0, 3.0, 4.0, fids, true);

    assertEquals(pose, customEst.pose());
    assertEquals(1.23, customEst.timestampSeconds());
    assertEquals(4.56, customEst.latency());
    assertEquals(1, customEst.tagCount());
    assertEquals(2.0, customEst.tagSpan());
    assertEquals(3.0, customEst.avgTagDist());
    assertEquals(4.0, customEst.avgTagArea());
    assertSame(fids, customEst.rawFiducials());
    assertTrue(customEst.isMegaTag2());

    // LimelightHelpers.PoseEstimate constructor
    LimelightHelpers.RawFiducial rawFid =
        new LimelightHelpers.RawFiducial(1, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
    LimelightHelpers.PoseEstimate llEst =
        new LimelightHelpers.PoseEstimate(
            pose,
            10.0,
            20.0,
            1,
            30.0,
            40.0,
            50.0,
            new LimelightHelpers.RawFiducial[] {rawFid},
            true);
    CameraIO.PoseEstimate llConverted = new CameraIO.PoseEstimate(llEst);

    assertEquals(pose, llConverted.pose());
    assertEquals(10.0, llConverted.timestampSeconds());
    assertEquals(20.0, llConverted.latency());
    assertEquals(1, llConverted.tagCount());
    assertEquals(30.0, llConverted.tagSpan());
    assertEquals(40.0, llConverted.avgTagDist());
    assertEquals(50.0, llConverted.avgTagArea());
    assertEquals(1, llConverted.rawFiducials().length);
    assertEquals(1, llConverted.rawFiducials()[0].id());
    assertTrue(llConverted.isMegaTag2());

    // Record properties
    assertNotNull(customEst.toString());
    assertEquals(customEst.hashCode(), customEst.hashCode());
    assertEquals(customEst, customEst);
    assertNotEquals(customEst, defaultEst);
  }

  @Test
  public void testRawFiducialRecord() {
    CameraIO.RawFiducial defaultFid = new CameraIO.RawFiducial();
    assertEquals(0, defaultFid.id());
    assertEquals(0.0, defaultFid.txnc());
    assertEquals(0.0, defaultFid.tync());
    assertEquals(0.0, defaultFid.ta());
    assertEquals(0.0, defaultFid.distToCamera());
    assertEquals(0.0, defaultFid.distToRobot());
    assertEquals(0.0, defaultFid.ambiguity());

    CameraIO.RawFiducial customFid = new CameraIO.RawFiducial(2, 1.0, 2.0, 3.0, 4.0, 5.0, 0.5);
    assertEquals(2, customFid.id());
    assertEquals(1.0, customFid.txnc());
    assertEquals(2.0, customFid.tync());
    assertEquals(3.0, customFid.ta());
    assertEquals(4.0, customFid.distToCamera());
    assertEquals(5.0, customFid.distToRobot());
    assertEquals(0.5, customFid.ambiguity());

    // LimelightHelpers constructor
    LimelightHelpers.RawFiducial llFid =
        new LimelightHelpers.RawFiducial(3, 1.1, 2.2, 3.3, 4.4, 5.5, 0.6);
    CameraIO.RawFiducial converted = new CameraIO.RawFiducial(llFid);
    assertEquals(3, converted.id());
    assertEquals(1.1, converted.txnc());
    assertEquals(2.2, converted.tync());
    assertEquals(3.3, converted.ta());
    assertEquals(4.4, converted.distToCamera());
    assertEquals(5.5, converted.distToRobot());
    assertEquals(0.6, converted.ambiguity());

    assertNotNull(customFid.toString());
    assertEquals(customFid.hashCode(), customFid.hashCode());
    assertEquals(customFid, customFid);
  }

  @Test
  public void testRawDetectionRecord() {
    CameraIO.RawDetection defaultDet = new CameraIO.RawDetection();
    assertEquals(0, defaultDet.classId());
    assertEquals(0.0, defaultDet.txnc());
    assertEquals(0.0, defaultDet.tync());
    assertEquals(0.0, defaultDet.ta());
    assertEquals(0.0, defaultDet.corner0_X());
    assertEquals(0.0, defaultDet.corner0_Y());

    CameraIO.RawDetection customDet =
        new CameraIO.RawDetection(1, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0);
    assertEquals(1, customDet.classId());
    assertEquals(2.0, customDet.txnc());
    assertEquals(3.0, customDet.tync());
    assertEquals(4.0, customDet.ta());
    assertEquals(5.0, customDet.corner0_X());
    assertEquals(6.0, customDet.corner0_Y());
    assertEquals(7.0, customDet.corner1_X());
    assertEquals(8.0, customDet.corner1_Y());
    assertEquals(9.0, customDet.corner2_X());
    assertEquals(10.0, customDet.corner2_Y());
    assertEquals(11.0, customDet.corner3_X());
    assertEquals(12.0, customDet.corner3_Y());

    assertNotNull(customDet.toString());
    assertEquals(customDet.hashCode(), customDet.hashCode());
    assertEquals(customDet, customDet);
  }
}
