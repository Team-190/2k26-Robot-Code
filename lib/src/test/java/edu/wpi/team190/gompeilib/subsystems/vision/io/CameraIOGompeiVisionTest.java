package edu.wpi.team190.gompeilib.subsystems.vision.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

public class CameraIOGompeiVisionTest {

  @Test
  public void testGompeiVisionIO() {
    // NetworkTable clean up / setup
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    Matrix<N3, N3> cameraMatrix =
        new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                  {1.0, 0.0, 2.0},
                  {0.0, 1.0, 3.0},
                  {0.0, 0.0, 1.0}
                }));
    Matrix<N5, N1> distortion =
        new Matrix<>(new SimpleMatrix(new double[][] {{0.1}, {0.2}, {0.3}, {0.4}, {0.5}}));

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("test_gompei")
            .hardwareID("HW001")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(100.0)
            .gain(1.5)
            .width(1280)
            .height(720)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .horizontalFOV(82.0)
            .verticalFOV(46.0)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(new Pose3d())
            .build();

    CameraIOGompeiVision io = new CameraIOGompeiVision(config);
    assertEquals("test_gompei", io.getName());

    // Verify NT Config outputs
    NetworkTable configTable =
        nt.getTable("cameras").getSubTable("test_gompei").getSubTable("config");
    assertEquals("test_gompei", configTable.getStringTopic("role").subscribe("").get());
    assertEquals("test_gompei", configTable.getStringTopic("hardware_id").subscribe("").get());
    assertEquals(100.0, configTable.getDoubleTopic("exposure").subscribe(0.0).get());
    assertEquals(1.5, configTable.getDoubleTopic("gain").subscribe(0.0).get());
    assertEquals(1280, configTable.getIntegerTopic("width").subscribe(0).get());
    assertEquals(720, configTable.getIntegerTopic("height").subscribe(0).get());

    // Setup input topics to publish observations and fps
    NetworkTable outputTable =
        nt.getTable("cameras").getSubTable("test_gompei").getSubTable("output");
    DoubleArrayPublisher obsPub = outputTable.getDoubleArrayTopic("observations").publish();
    IntegerPublisher captureFPSPub = outputTable.getIntegerTopic("capture_fps").publish();
    IntegerPublisher processingFPSPub = outputTable.getIntegerTopic("processing_fps").publish();

    obsPub.set(new double[] {1.0, 2.0, 3.0});
    captureFPSPub.set(60);
    processingFPSPub.set(55);

    // Let the subscribers receive it
    nt.flush();
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {
      // ignore
    }

    CameraIO.GompeiVisionIOInputs inputs = new CameraIO.GompeiVisionIOInputs();
    io.updateInputs(inputs);

    assertEquals(60.0, inputs.captureFPS);
    assertEquals(55.0, inputs.processingFPS);
    assertTrue(inputs.frames.length >= 1);
    assertArrayEquals(new double[] {1.0, 2.0, 3.0}, inputs.frames[inputs.frames.length - 1]);
  }
}
