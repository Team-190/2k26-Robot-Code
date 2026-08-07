package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

public class LimelightHelpersTest {

  private NetworkTableInstance nt;
  private String llName = "limelight-test";

  @BeforeEach
  public void setUp() {
    nt = NetworkTableInstance.getDefault();
  }

  @Test
  public void testMathConversions() {
    // toPose3D
    double[] badPose = {1.0, 2.0};
    Pose3d resultBad = LimelightHelpers.toPose3D(badPose);
    assertEquals(new Pose3d(), resultBad);

    double[] goodPose = {1.0, 2.0, 3.0, 0.0, 0.0, 90.0};
    Pose3d resultGood = LimelightHelpers.toPose3D(goodPose);
    assertEquals(1.0, resultGood.getX());
    assertEquals(2.0, resultGood.getY());
    assertEquals(3.0, resultGood.getZ());
    assertEquals(Math.PI / 2.0, resultGood.getRotation().getZ(), 1e-6);

    // toPose2D
    Pose2d result2DBad = LimelightHelpers.toPose2D(badPose);
    assertEquals(new Pose2d(), result2DBad);

    Pose2d result2DGood = LimelightHelpers.toPose2D(goodPose);
    assertEquals(1.0, result2DGood.getX());
    assertEquals(2.0, result2DGood.getY());
    assertEquals(Math.PI / 2.0, result2DGood.getRotation().getRadians(), 1e-6);

    // pose3dToArray
    double[] arr3d = LimelightHelpers.pose3dToArray(resultGood);
    assertEquals(6, arr3d.length);
    assertEquals(1.0, arr3d[0]);
    assertEquals(90.0, arr3d[5], 1e-6);

    // pose2dToArray
    double[] arr2d = LimelightHelpers.pose2dToArray(result2DGood);
    assertEquals(6, arr2d.length);
    assertEquals(1.0, arr2d[0]);
    assertEquals(90.0, arr2d[5], 1e-6);
  }

  @Test
  public void testGettersAndSettersNT() {
    // Flush
    LimelightHelpers.Flush();

    // tv, tx, ty, txnc, tync, ta
    nt.getTable(llName).getEntry("tv").setDouble(1.0);
    nt.getTable(llName).getEntry("tx").setDouble(2.5);
    nt.getTable(llName).getEntry("ty").setDouble(-3.5);
    nt.getTable(llName).getEntry("txnc").setDouble(1.2);
    nt.getTable(llName).getEntry("tync").setDouble(-1.2);
    nt.getTable(llName).getEntry("ta").setDouble(15.0);

    assertTrue(LimelightHelpers.getTV(llName));
    assertEquals(2.5, LimelightHelpers.getTX(llName));
    assertEquals(-3.5, LimelightHelpers.getTY(llName));
    assertEquals(1.2, LimelightHelpers.getTXNC(llName));
    assertEquals(-1.2, LimelightHelpers.getTYNC(llName));
    assertEquals(15.0, LimelightHelpers.getTA(llName));

    // t2d
    double[] t2dData = new double[17];
    t2dData[1] = 2.0; // targetCount
    t2dData[10] = 3.0; // targetClassIndexDetector
    t2dData[11] = 4.0; // targetClassIndexClassifier
    nt.getTable(llName).getEntry("t2d").setDoubleArray(t2dData);

    assertArrayEquals(t2dData, LimelightHelpers.getT2DArray(llName));
    assertEquals(2, LimelightHelpers.getTargetCount(llName));
    assertEquals(3, LimelightHelpers.getDetectorClassIndex(llName));
    assertEquals(4, LimelightHelpers.getClassifierClassIndex(llName));

    // strings tcclass, tdclass, getpipetype, json
    nt.getTable(llName).getEntry("tcclass").setString("cls1");
    nt.getTable(llName).getEntry("tdclass").setString("det1");
    nt.getTable(llName).getEntry("getpipetype").setString("tag");
    nt.getTable(llName).getEntry("json").setString("{test}");

    assertEquals("cls1", LimelightHelpers.getClassifierClass(llName));
    assertEquals("det1", LimelightHelpers.getDetectorClass(llName));
    assertEquals("tag", LimelightHelpers.getCurrentPipelineType(llName));
    assertEquals("{test}", LimelightHelpers.getJSONDump(llName));

    // latencies and active pipeline
    nt.getTable(llName).getEntry("tl").setDouble(12.3);
    nt.getTable(llName).getEntry("cl").setDouble(1.1);
    nt.getTable(llName).getEntry("getpipe").setDouble(2.0);

    assertEquals(12.3, LimelightHelpers.getLatency_Pipeline(llName));
    assertEquals(1.1, LimelightHelpers.getLatency_Capture(llName));
    assertEquals(2.0, LimelightHelpers.getCurrentPipelineIndex(llName));

    // botpose methods
    double[] botpose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
    nt.getTable(llName).getEntry("botpose").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("botpose_wpired").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("botpose_wpiblue").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("botpose_targetspace").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("camerapose_targetspace").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("targetpose_cameraspace").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("targetpose_robotspace").setDoubleArray(botpose);
    nt.getTable(llName).getEntry("camerapose_robotspace").setDoubleArray(botpose);

    assertArrayEquals(botpose, LimelightHelpers.getBotpose(llName));
    assertArrayEquals(botpose, LimelightHelpers.getBotpose_wpiRed(llName));
    assertArrayEquals(botpose, LimelightHelpers.getBotpose_wpiBlue(llName));

    assertArrayEquals(botpose, LimelightHelpers.getBotPose(llName));
    assertArrayEquals(botpose, LimelightHelpers.getBotPose_wpiRed(llName));
    assertArrayEquals(botpose, LimelightHelpers.getBotPose_wpiBlue(llName));
    assertArrayEquals(botpose, LimelightHelpers.getBotPose_TargetSpace(llName));
    assertArrayEquals(botpose, LimelightHelpers.getCameraPose_TargetSpace(llName));
    assertArrayEquals(botpose, LimelightHelpers.getTargetPose_CameraSpace(llName));
    assertArrayEquals(botpose, LimelightHelpers.getTargetPose_RobotSpace(llName));

    // pose3d getters
    assertNotNull(LimelightHelpers.getBotPose3d(llName));
    assertNotNull(LimelightHelpers.getBotPose3d_wpiRed(llName));
    assertNotNull(LimelightHelpers.getBotPose3d_wpiBlue(llName));
    assertNotNull(LimelightHelpers.getBotPose3d_TargetSpace(llName));
    assertNotNull(LimelightHelpers.getCameraPose3d_TargetSpace(llName));
    assertNotNull(LimelightHelpers.getTargetPose3d_CameraSpace(llName));
    assertNotNull(LimelightHelpers.getTargetPose3d_RobotSpace(llName));
    assertNotNull(LimelightHelpers.getCameraPose3d_RobotSpace(llName));

    // pose2d getters
    assertNotNull(LimelightHelpers.getBotPose2d(llName));
    assertNotNull(LimelightHelpers.getBotPose2d_wpiRed(llName));
    assertNotNull(LimelightHelpers.getBotPose2d_wpiBlue(llName));

    // set/get python data
    double[] pyOut = {9.9, 8.8};
    LimelightHelpers.setPythonScriptData(llName, pyOut);
    assertArrayEquals(pyOut, nt.getTable(llName).getEntry("llrobot").getDoubleArray(new double[0]));

    double[] pyIn = {7.7, 6.6};
    nt.getTable(llName).getEntry("llpython").setDoubleArray(pyIn);
    assertArrayEquals(pyIn, LimelightHelpers.getPythonScriptData(llName));

    // other set operations
    LimelightHelpers.setPipelineIndex(llName, 4);
    assertEquals(4.0, nt.getTable(llName).getEntry("pipeline").getDouble(0.0));

    LimelightHelpers.setPriorityTagID(llName, 12);
    assertEquals(12.0, nt.getTable(llName).getEntry("priorityid").getDouble(0.0));

    LimelightHelpers.setLEDMode_ForceBlink(llName);
    assertEquals(2.0, nt.getTable(llName).getEntry("ledMode").getDouble(0.0));

    LimelightHelpers.setLEDMode_ForceOff(llName);
    assertEquals(1.0, nt.getTable(llName).getEntry("ledMode").getDouble(0.0));

    LimelightHelpers.setLEDMode_ForceOn(llName);
    assertEquals(3.0, nt.getTable(llName).getEntry("ledMode").getDouble(0.0));

    LimelightHelpers.setLEDMode_PipelineControl(llName);
    assertEquals(0.0, nt.getTable(llName).getEntry("ledMode").getDouble(0.0));

    LimelightHelpers.setStreamMode_PiPMain(llName);
    assertEquals(1.0, nt.getTable(llName).getEntry("stream").getDouble(0.0));

    LimelightHelpers.setStreamMode_PiPSecondary(llName);
    assertEquals(2.0, nt.getTable(llName).getEntry("stream").getDouble(0.0));

    LimelightHelpers.setStreamMode_Standard(llName);
    assertEquals(0.0, nt.getTable(llName).getEntry("stream").getDouble(0.0));

    LimelightHelpers.setCropWindow(llName, -0.5, 0.5, -0.5, 0.5);
    assertArrayEquals(
        new double[] {-0.5, 0.5, -0.5, 0.5},
        nt.getTable(llName).getEntry("crop").getDoubleArray(new double[0]));

    LimelightHelpers.setKeystone(llName, 0.1, 0.2);
    assertArrayEquals(
        new double[] {0.1, 0.2},
        nt.getTable(llName).getEntry("keystone_set").getDoubleArray(new double[0]));

    LimelightHelpers.setFiducial3DOffset(llName, 1.0, 2.0, 3.0);
    assertArrayEquals(
        new double[] {1.0, 2.0, 3.0},
        nt.getTable(llName).getEntry("fiducial_offset_set").getDoubleArray(new double[0]));

    LimelightHelpers.SetRobotOrientation(llName, 10.0, 1.0, 2.0, 3.0, 4.0, 5.0);
    assertArrayEquals(
        new double[] {10.0, 1.0, 2.0, 3.0, 4.0, 5.0},
        nt.getTable(llName).getEntry("robot_orientation_set").getDoubleArray(new double[0]));

    LimelightHelpers.SetRobotOrientation_NoFlush(llName, 11.0, 1.1, 2.1, 3.1, 4.1, 5.1);
    assertArrayEquals(
        new double[] {11.0, 1.1, 2.1, 3.1, 4.1, 5.1},
        nt.getTable(llName).getEntry("robot_orientation_set").getDoubleArray(new double[0]));

    LimelightHelpers.SetIMUMode(llName, 2);
    assertEquals(2.0, nt.getTable(llName).getEntry("imumode_set").getDouble(0.0));

    LimelightHelpers.SetIMUAssistAlpha(llName, 0.05);
    assertEquals(0.05, nt.getTable(llName).getEntry("imuassistalpha_set").getDouble(0.0));

    LimelightHelpers.SetThrottle(llName, 5);
    assertEquals(5.0, nt.getTable(llName).getEntry("throttle_set").getDouble(0.0));

    LimelightHelpers.SetFiducialIDFiltersOverride(llName, new int[] {1, 2});
    assertArrayEquals(
        new double[] {1.0, 2.0},
        nt.getTable(llName).getEntry("fiducial_id_filters_set").getDoubleArray(new double[0]));

    LimelightHelpers.SetFiducialDownscalingOverride(llName, 1.5f);
    assertEquals(2.0, nt.getTable(llName).getEntry("fiducial_downscale_set").getDouble(0.0));

    // Test other downscale factors
    LimelightHelpers.SetFiducialDownscalingOverride(llName, 1.0f);
    assertEquals(1.0, nt.getTable(llName).getEntry("fiducial_downscale_set").getDouble(0.0));
    LimelightHelpers.SetFiducialDownscalingOverride(llName, 2.0f);
    assertEquals(3.0, nt.getTable(llName).getEntry("fiducial_downscale_set").getDouble(0.0));
    LimelightHelpers.SetFiducialDownscalingOverride(llName, 3.0f);
    assertEquals(4.0, nt.getTable(llName).getEntry("fiducial_downscale_set").getDouble(0.0));
    LimelightHelpers.SetFiducialDownscalingOverride(llName, 4.0f);
    assertEquals(5.0, nt.getTable(llName).getEntry("fiducial_downscale_set").getDouble(0.0));

    LimelightHelpers.setCameraPose_RobotSpace(llName, 1.0, 2.0, 3.0, 10.0, 20.0, 30.0);
    assertArrayEquals(
        new double[] {1.0, 2.0, 3.0, 10.0, 20.0, 30.0},
        nt.getTable(llName).getEntry("camerapose_robotspace_set").getDoubleArray(new double[0]));

    LimelightHelpers.setRewindEnabled(llName, true);
    assertEquals(1.0, nt.getTable(llName).getEntry("rewind_enable_set").getDouble(0.0));

    LimelightHelpers.setRewindEnabled(llName, false);
    assertEquals(0.0, nt.getTable(llName).getEntry("rewind_enable_set").getDouble(0.0));

    nt.getTable(llName).getEntry("capture_rewind").setDoubleArray(new double[] {0.0, 0.0});
    LimelightHelpers.triggerRewindCapture(llName, 5.0);
    assertArrayEquals(
        new double[] {1.0, 5.0},
        nt.getTable(llName).getEntry("capture_rewind").getDoubleArray(new double[0]));

    nt.getTable(llName).getEntry("snapshot").setDouble(5.0);
    LimelightHelpers.triggerSnapshot(llName);
    assertEquals(6.0, nt.getTable(llName).getEntry("snapshot").getDouble(0.0));
  }

  @Test
  public void testPoseEstimateRetrieval() {
    // Empty array
    nt.getTable(llName).getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
    LimelightHelpers.PoseEstimate emptyEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
    assertEquals(new Pose2d(), emptyEst.pose);

    // Mismatched fiducial values
    double[] invalidPoseArr = new double[15]; // expected 11 + 7*1 = 18 for 1 tag
    invalidPoseArr[7] = 1.0; // tagCount = 1
    nt.getTable(llName).getEntry("botpose_wpiblue").setDoubleArray(invalidPoseArr);
    LimelightHelpers.PoseEstimate invalidEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
    assertEquals(0, invalidEst.rawFiducials.length);

    // Valid pose with fiducials
    double[] poseArr = new double[18];
    poseArr[0] = 2.0; // X
    poseArr[1] = 3.0; // Y
    // index 6 is latency
    poseArr[6] = 10.0;
    // index 7 is tagCount
    poseArr[7] = 1.0;
    // fiducials start at index 11
    poseArr[11] = 5.0; // ID
    poseArr[12] = 0.1; // txnc
    poseArr[13] = 0.2; // tync
    poseArr[14] = 0.3; // ta
    poseArr[15] = 1.5; // distToCamera
    poseArr[16] = 1.6; // distToRobot
    poseArr[17] = 0.05; // ambiguity

    nt.getTable(llName).getEntry("botpose_wpiblue").setDoubleArray(poseArr);
    LimelightHelpers.PoseEstimate validEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);
    assertEquals(2.0, validEst.pose.getX());
    assertEquals(1, validEst.rawFiducials.length);
    assertEquals(5, validEst.rawFiducials[0].id);

    // printPoseEstimate
    LimelightHelpers.printPoseEstimate(null);
    LimelightHelpers.printPoseEstimate(validEst);

    // validPoseEstimate
    assertFalse(LimelightHelpers.validPoseEstimate(null));
    assertTrue(LimelightHelpers.validPoseEstimate(validEst));

    // getBotPoseEstimate wpiRed and MegaTag2
    assertNotNull(LimelightHelpers.getBotPoseEstimate_wpiRed(llName));
    assertNotNull(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(llName));
    assertNotNull(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName));
  }

  @Test
  public void testRawDetectionsAndTargets() {
    // getRawFiducials
    double[] rawFids = {5, 0.1, 0.2, 0.3, 1.0, 1.1, 0.05};
    nt.getTable(llName).getEntry("rawfiducials").setDoubleArray(rawFids);
    LimelightHelpers.RawFiducial[] fids = LimelightHelpers.getRawFiducials(llName);
    assertEquals(1, fids.length);
    assertEquals(5, fids[0].id);
    assertEquals(0.05, fids[0].ambiguity);

    // Mismatched size
    nt.getTable(llName).getEntry("rawfiducials").setDoubleArray(new double[] {1, 2});
    assertEquals(0, LimelightHelpers.getRawFiducials(llName).length);

    // getRawDetections
    double[] rawDets = {1, 0.1, 0.2, 0.3, 1, 2, 3, 4, 5, 6, 7, 8};
    nt.getTable(llName).getEntry("rawdetections").setDoubleArray(rawDets);
    LimelightHelpers.RawDetection[] dets = LimelightHelpers.getRawDetections(llName);
    assertEquals(1, dets.length);
    assertEquals(1, dets[0].classId);

    // Mismatched size
    nt.getTable(llName).getEntry("rawdetections").setDoubleArray(new double[] {1, 2});
    assertEquals(0, LimelightHelpers.getRawDetections(llName).length);

    // getRawTargets
    double[] rawTargets = {0.1, 0.2, 0.3};
    nt.getTable(llName).getEntry("rawtargets").setDoubleArray(rawTargets);
    LimelightHelpers.RawTarget[] targets = LimelightHelpers.getRawTargets(llName);
    assertEquals(1, targets.length);
    assertEquals(0.1, targets[0].txnc);

    // Mismatched size
    nt.getTable(llName).getEntry("rawtargets").setDoubleArray(new double[] {1, 2});
    assertEquals(0, LimelightHelpers.getRawTargets(llName).length);

    // getCornerCoordinates
    double[] corners = {1.0, 2.0, 3.0, 4.0};
    nt.getTable(llName).getEntry("tcornxy").setDoubleArray(corners);
    assertArrayEquals(corners, LimelightHelpers.getCornerCoordinates(llName));

    // getIMUData
    double[] imu = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    nt.getTable(llName).getEntry("imu").setDoubleArray(imu);
    LimelightHelpers.IMUData imuData = LimelightHelpers.getIMUData(llName);
    assertEquals(1.0, imuData.robotYaw);
    assertEquals(10.0, imuData.accelZ);

    // getIMUData empty/bad
    nt.getTable(llName).getEntry("imu").setDoubleArray(new double[0]);
    assertEquals(0.0, LimelightHelpers.getIMUData(llName).robotYaw);
  }

  @Test
  public void testLatestResultsAndPortForwarding() {
    // Port forwarding (runs safely by mocking PortForwarder)
    try (MockedStatic<PortForwarder> mockPF = mockStatic(PortForwarder.class)) {
      LimelightHelpers.setupPortForwardingUSB(0);
      mockPF.verify(() -> PortForwarder.add(anyInt(), anyString(), anyInt()), times(10));
    }

    // getLatestResults empty
    nt.getTable(llName).getEntry("json").setString("");
    assertNotNull(LimelightHelpers.getLatestResults(llName).error);

    // getLatestResults valid json
    String json =
        "{\n"
            + "  \"pID\": 1.0,\n"
            + "  \"tl\": 10.0,\n"
            + "  \"cl\": 5.0,\n"
            + "  \"v\": true,\n"
            + "  \"botpose\": [1.0, 2.0, 3.0, 0.0, 0.0, 45.0],\n"
            + "  \"Retro\": [],\n"
            + "  \"Fiducial\": [],\n"
            + "  \"Classifier\": [],\n"
            + "  \"Detector\": [],\n"
            + "  \"Barcode\": [],\n"
            + "  \"imu\": {\n"
            + "    \"data\": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],\n"
            + "    \"quat\": [0.0, 0.0, 0.0, 1.0],\n"
            + "    \"yaw\": 90.0\n"
            + "  }\n"
            + "}";
    nt.getTable(llName).getEntry("json").setString(json);
    LimelightHelpers.LimelightResults res = LimelightHelpers.getLatestResults(llName);
    assertNull(res.error);
    assertTrue(res.valid);
    assertEquals(1.0, res.pipelineID);
    assertEquals(90.0, res.imuResults.yaw);
    assertEquals(1.0, res.imuResults.robotYaw);
  }
}
