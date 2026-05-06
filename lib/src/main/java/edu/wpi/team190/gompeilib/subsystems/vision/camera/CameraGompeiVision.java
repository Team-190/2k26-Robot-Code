package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.GompeiVisionConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOGompeiVision;
import edu.wpi.team190.gompeilib.subsystems.vision.io.GompeiVisionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(GeometryUtil.class)
public class CameraGompeiVision extends Camera {
  private final GompeiVisionIOInputsAutoLogged inputs;
  private final CameraIOGompeiVision io;

  private final GompeiVisionConfig config;
  private final Supplier<AprilTagFieldLayout> aprilTagFieldLayoutSupplier;
  private final double fieldBorderMarginMeters;
  private final Supplier<Pose2d> currentRobotPoseSupplier;

  @Getter private final String name;

  @Getter private final List<Pose3d> allTagPoses;
  @Getter private Pose2d robotPose;

  public CameraGompeiVision(
      CameraIOGompeiVision io,
      GompeiVisionConfig config,
      Supplier<AprilTagFieldLayout> aprilTagFieldLayoutSupplier,
      double fieldBorderMarginMeters,
      Supplier<Pose2d> currentRobotPoseSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionMultiTxTyObservation>>> txtyObservers) {
    super(config.key(), poseObservers, txtyObservers, new ArrayList<>());

    inputs = new GompeiVisionIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.aprilTagFieldLayoutSupplier = aprilTagFieldLayoutSupplier;
    this.fieldBorderMarginMeters = fieldBorderMarginMeters;
    this.currentRobotPoseSupplier = currentRobotPoseSupplier;

    this.name = this.config.key();

    allTagPoses = new ArrayList<>();
    robotPose = Pose2d.kZero;

    currentCameraPose = config.robotRelativePose();
  }

  @Override
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();
    robotPose = Pose2d.kZero;

    for (int i = 0; i < inputs.frames.length; i++) {
      double timestamp = inputs.timestamps[i];
      int totalTargets;
      double averageDistance = 0.0;
      double[] values = inputs.frames[i];

      if (values.length == 0 || values[0] == 0) {
        continue;
      }

      Pose3d cameraPose = null;
      Pose2d robotPose = null;

      switch ((int) values[0]) {
        case 1:
          // One pose
          cameraPose =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
          robotPose =
              cameraPose
                  .toPose2d()
                  .transformBy(currentCameraPose.toPose2d().toTransform2d().inverse());
          break;

        case 2:
          // Multiple poses
          double error0 = values[1];
          double error1 = values[9];
          Pose3d cameraPose0 =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));

          Pose3d cameraPose1 =
              new Pose3d(
                  values[10],
                  values[11],
                  values[12],
                  new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
          Transform2d cameraToRobot = currentCameraPose.toPose2d().toTransform2d().inverse();
          Pose2d robotPose0 = cameraPose0.toPose2d().transformBy(cameraToRobot);
          Pose2d robotPose1 = cameraPose1.toPose2d().transformBy(cameraToRobot);

          if (error0 < error1 * VisionConstants.AMBIGUITY_THRESHOLD
              || error1 < error0 * VisionConstants.AMBIGUITY_THRESHOLD) {
            Rotation2d currentRotation = currentRobotPoseSupplier.get().getRotation();
            Rotation2d visionRotation0 = robotPose0.getRotation();
            Rotation2d visionRotation1 = robotPose1.getRotation();
            if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
              robotPose = robotPose0;
              cameraPose = cameraPose0;
            } else {
              robotPose = robotPose1;
              cameraPose = cameraPose1;
            }
          }

          break;
        default:
          DriverStation.reportWarning("FAILED TO CAPTURE FRAMES", false);
          continue;
      }

      if (cameraPose == null || robotPose == null) {
        continue;
      }

      // Exit if robot pose is off the field
      double fieldLength = aprilTagFieldLayoutSupplier.get().getFieldLength();
      double fieldWidth = aprilTagFieldLayoutSupplier.get().getFieldWidth();

      if (robotPose.getX() < -fieldBorderMarginMeters
          || robotPose.getX() > fieldLength + fieldBorderMarginMeters
          || robotPose.getY() < -fieldBorderMarginMeters
          || robotPose.getY() > fieldWidth + fieldBorderMarginMeters) {
        continue;
      }

      // Get tag poses and update last detection times
      List<Pose3d> tagPoses = new ArrayList<>();
      for (int j = (values[0] == 1 ? 9 : 17); j < values.length; j += 10) {
        int tagId = (int) values[j];
        Optional<Pose3d> tagPose = aprilTagFieldLayoutSupplier.get().getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Prepare arrays
      totalTargets = tagPoses.size();
      Set<Integer> tagIds = new HashSet<>();

      if (!tagPoses.isEmpty()) {
        // Calculate average distance
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(currentCameraPose.getTranslation());
        }
        averageDistance = totalDistance / tagPoses.size();

        // --- Parse tag angle + distance data ---
        int tagEstimationDataEndIndex =
            switch ((int) values[0]) {
              case 1 -> 8;
              case 2 -> 16;
              default -> 0;
            };

        int indexCounter = 0;

        // Step through each 10-value chunk safely
        for (int index = tagEstimationDataEndIndex + 1; index + 9 < values.length; index += 10) {
          int tagId = (int) values[index];
          double[] tx = new double[4];
          double[] ty = new double[4];

          // Read 4 corner pairs
          for (int j = 0; j < 4; j++) {
            tx[i] = values[index + 1 + (2 * i)];
            ty[i] = values[index + 1 + (2 * i) + 1];
          }

          double distance = values[index + 9];

          // Store data
          if (indexCounter < totalTargets) {
            indexCounter++;
            multiTxTyObservationList.add(
                new VisionMultiTxTyObservation(tagId, tx, ty, distance, timestamp, cameraPose));
            tagIds.add(tagId);
          } else {
            System.out.println("[WARN] More tag data than expected: indexCounter=" + indexCounter);
          }
        }

        // Optional debug check
        if ((values.length - (tagEstimationDataEndIndex + 1)) % 10 != 0) {
          System.out.println(
              "[WARN] Observation array not multiple of 10! Length="
                  + values.length
                  + " start="
                  + (tagEstimationDataEndIndex + 1));
        }
      }
      double xyStdevCoeff;
      double thetaStdev;

      if (totalTargets > 1) {
        xyStdevCoeff = config.multitagXYStdev();
        thetaStdev =
            config.thetaStdev() * Math.pow(averageDistance, 1.2) / Math.pow(totalTargets, 2.0);
      } else {
        xyStdevCoeff = config.singletagXYStdev();
        thetaStdev = Double.POSITIVE_INFINITY;
      }

      // Add observation to list
      double xyStdDev = xyStdevCoeff * Math.pow(averageDistance, 1.2) / Math.pow(totalTargets, 2.0);

      poseObservationList.add(
          new VisionPoseObservation(
              robotPose, tagIds, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    super.sendObservers();
  }
}
