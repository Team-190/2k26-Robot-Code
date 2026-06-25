package edu.wpi.team190.gompeilib.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

public class VisionConstantsTest {

  @Test
  public void testConstants() {
    assertEquals(0.4, VisionConstants.AMBIGUITY_THRESHOLD);
    assertEquals(1.2, VisionConstants.XY_STDEV_DISTANCE_EXPONENT);
    assertEquals(2.0, VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
  }

  @Test
  public void testStaticLimelightConfig() {
    Transform3d transform = new Transform3d(new Translation3d(1, 2, 3), new Rotation3d());
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static-cam")
            .cameraType(CameraType.LIMELIGHT_3)
            .horizontalFOV(60.0)
            .verticalFOV(40.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToCameraTransform(transform)
            .enableRewind(true)
            .build();

    assertEquals("static-cam", config.key());
    assertEquals(CameraType.LIMELIGHT_3, config.cameraType());
    assertEquals(60.0, config.horizontalFOV());
    assertEquals(40.0, config.verticalFOV());
    assertEquals(0.1, config.megatagXYStdev());
    assertEquals(0.2, config.metatagThetaStdev());
    assertEquals(0.05, config.megatag2XYStdev());
    assertEquals(transform, config.robotToCameraTransform());
    assertTrue(config.enableRewind());

    // For full record coverage, test toString, hashCode, equals
    assertNotNull(config.toString());
    assertEquals(config.hashCode(), config.hashCode());
    assertEquals(config, config);
    assertNotEquals(config, null);
  }

  @Test
  public void testMovingLimelightConfig() {
    Transform3d t1 = new Transform3d(new Translation3d(1, 2, 3), new Rotation3d());
    Transform3d t2 = new Transform3d(new Translation3d(4, 5, 6), new Rotation3d());
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving-cam")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(80.0)
            .verticalFOV(50.0)
            .megatagXYStdev(0.15)
            .metatagThetaStdev(0.25)
            .megatag2XYStdev(0.06)
            .robotToRotationAxisTransform(t1)
            .rotationAxisToLensTransform(t2)
            .enableRewind(false)
            .build();

    assertEquals("moving-cam", config.key());
    assertEquals(CameraType.LIMELIGHT_3G, config.cameraType());
    assertEquals(80.0, config.horizontalFOV());
    assertEquals(50.0, config.verticalFOV());
    assertEquals(0.15, config.megatagXYStdev());
    assertEquals(0.25, config.metatagThetaStdev());
    assertEquals(0.06, config.megatag2XYStdev());
    assertEquals(t1, config.robotToRotationAxisTransform());
    assertEquals(t2, config.rotationAxisToLensTransform());
    assertFalse(config.enableRewind());

    // Record coverage
    assertNotNull(config.toString());
    assertEquals(config.hashCode(), config.hashCode());
    assertEquals(config, config);
  }

  @Test
  public void testGompeiVisionConfig() {
    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d pose = new Pose3d();
    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei-cam")
            .hardwareID("HW123")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(10.0)
            .gain(5.0)
            .width(640)
            .height(480)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .horizontalFOV(70.0)
            .verticalFOV(45.0)
            .singletagXYStdev(0.3)
            .thetaStdev(0.4)
            .multitagXYStdev(0.05)
            .robotRelativePose(pose)
            .build();

    assertEquals("gompei-cam", config.key());
    assertEquals("HW123", config.hardwareID());
    assertEquals(CameraType.THRIFTYCAM, config.cameraType());
    assertEquals(10.0, config.exposure());
    assertEquals(5.0, config.gain());
    assertEquals(640, config.width());
    assertEquals(480, config.height());
    assertEquals(cameraMatrix, config.cameraMatrix());
    assertEquals(distortion, config.distortionCoefficients());
    assertEquals(70.0, config.horizontalFOV());
    assertEquals(45.0, config.verticalFOV());
    assertEquals(0.3, config.singletagXYStdev());
    assertEquals(0.4, config.thetaStdev());
    assertEquals(0.05, config.multitagXYStdev());
    assertEquals(pose, config.robotRelativePose());

    // Record coverage
    assertNotNull(config.toString());
    assertEquals(config.hashCode(), config.hashCode());
    assertEquals(config, config);
  }
}
