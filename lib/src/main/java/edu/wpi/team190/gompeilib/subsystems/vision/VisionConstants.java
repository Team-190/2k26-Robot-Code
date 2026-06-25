package edu.wpi.team190.gompeilib.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import lombok.Builder;

public class VisionConstants {
  public static final double AMBIGUITY_THRESHOLD = 0.4;
  public static final double XY_STDEV_DISTANCE_EXPONENT = 1.2;
  public static final double XY_STDEV_TAG_COUNT_EXPONENT = 2.0;

  @Builder
  public record StaticLimelightConfig(
      String key,
      CameraType cameraType,
      double horizontalFOV,
      double verticalFOV,
      double megatagXYStdev,
      double metatagThetaStdev,
      double megatag2XYStdev,
      Transform3d robotToCameraTransform,
      Boolean enableRewind) {}

  @Builder
  public record MovingLimelightConfig(
      String key,
      CameraType cameraType,
      double horizontalFOV,
      double verticalFOV,
      double megatagXYStdev,
      double metatagThetaStdev,
      double megatag2XYStdev,
      Transform3d robotToRotationAxisTransform,
      Transform3d rotationAxisToLensTransform,
      Boolean enableRewind) {}

  @Builder
  public record GompeiVisionConfig(
      String key,
      String hardwareID,
      CameraType cameraType,
      double exposure,
      double gain,
      int width,
      int height,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N5, N1> distortionCoefficients,
      double horizontalFOV,
      double verticalFOV,
      double singletagXYStdev,
      double thetaStdev,
      double multitagXYStdev,
      Pose3d robotRelativePose) {}
}
