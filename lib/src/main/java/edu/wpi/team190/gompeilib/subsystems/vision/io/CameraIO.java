package edu.wpi.team190.gompeilib.subsystems.vision.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.Camera;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLog;

/**
 * An interface for the hardware implementation of a {@link Camera Camera}. Contains the methods
 * necessary for providing information to a camera, like its pipeline, and getting information out
 * of a camera, like its individual robot pose estimate.
 */
public interface CameraIO {

  @AutoLog
  public static class GompeiVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double captureFPS = 0;
    public double processingFPS = 0;
  }

  @AutoLog
  public static class LimelightIOInputs {
    public PoseEstimate mt1PoseEstimate = new PoseEstimate();
    public PoseEstimate mt2PoseEstimate = new PoseEstimate();
    public RawFiducial[] rawFiducials = {};
  }

  public default void updateInputs(GompeiVisionIOInputs inputs) {}

  public default void updateInputs(LimelightIOInputs inputs) {}

  public default String getName() {
    return "";
  }

  public record PoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      double latency,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      RawFiducial[] rawFiducials,
      boolean isMegaTag2) {
    public PoseEstimate() {
      this(Pose2d.kZero, 0, 0, 0, 0, 0, 0, new RawFiducial[] {}, false);
    }

    public PoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
      this(
          poseEstimate.pose,
          poseEstimate.timestampSeconds,
          poseEstimate.latency,
          poseEstimate.tagCount,
          poseEstimate.tagSpan,
          poseEstimate.avgTagDist,
          poseEstimate.avgTagArea,
          Arrays.stream(poseEstimate.rawFiducials)
              .map(RawFiducial::new)
              .toArray(RawFiducial[]::new),
          poseEstimate.isMegaTag2);
    }
  }

  public record RawFiducial(
      int id,
      double txnc,
      double tync,
      double ta,
      double distToCamera,
      double distToRobot,
      double ambiguity) {
    public RawFiducial() {
      this(0, 0, 0, 0, 0, 0, 0);
    }

    public RawFiducial(LimelightHelpers.RawFiducial rawFiducial) {
      this(
          rawFiducial.id,
          rawFiducial.txnc,
          rawFiducial.tync,
          rawFiducial.ta,
          rawFiducial.distToCamera,
          rawFiducial.distToRobot,
          rawFiducial.ambiguity);
    }
  }

  public record RawDetection(
      int classId,
      double txnc,
      double tync,
      double ta,
      double corner0_X,
      double corner0_Y,
      double corner1_X,
      double corner1_Y,
      double corner2_X,
      double corner2_Y,
      double corner3_X,
      double corner3_Y) {
    public RawDetection() {
      this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  }
}
