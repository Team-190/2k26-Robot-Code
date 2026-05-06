package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.LimelightIOInputsAutoLogged;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CameraStaticLimelight extends Camera {
  private final LimelightIOInputsAutoLogged inputs;
  private final CameraIOLimelight io;

  private final StaticLimelightConfig config;
  @Getter private final String name;

  private final Supplier<Rotation2d> headingSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final LongSupplier timestampSupplier;
  private final DoubleArrayPublisher headingPublisher;

  @Getter private final List<Pose3d> allTagPoses;

  private boolean wasEnabled;
  private double enabledTimestamp;

  public CameraStaticLimelight(
      CameraIOLimelight io,
      StaticLimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      LongSupplier timestampSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(config.key(), poseObservers, new ArrayList<>(), singleTxTyObservers);

    inputs = new LimelightIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.name = "limelight-" + this.config.key();

    this.headingSupplier = headingSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.timestampSupplier = timestampSupplier;
    this.headingPublisher =
        NetworkTableInstance.getDefault()
            .getTable(this.name)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();

    allTagPoses = new ArrayList<>();

    currentCameraPose =
        new Pose3d(
            config.robotToCameraTransform().getTranslation(),
            config.robotToCameraTransform().getRotation());

    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        currentCameraPose.getX(),
        -currentCameraPose.getY(),
        currentCameraPose.getZ(),
        currentCameraPose.getRotation().getMeasureX().in(Degrees),
        currentCameraPose.getRotation().getMeasureY().in(Degrees),
        currentCameraPose.getRotation().getMeasureZ().in(Degrees));

    LimelightHelpers.SetIMUAssistAlpha(name, 0.0067);
    LimelightHelpers.setRewindEnabled(name, config.enableRewind());

    LimelightHelpers.SetIMUMode(name, 1);
    LimelightHelpers.SetThrottle(name, 190);

    wasEnabled = false;
    enabledTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();
    singleTxTyObservationList.clear();

    if (DriverStation.isEnabled()) {
      if (!wasEnabled) {
        enabledTimestamp = Timer.getFPGATimestamp();
        wasEnabled = true;
        LimelightHelpers.SetIMUMode(name, 0);
        LimelightHelpers.SetThrottle(name, 0);
      }

      if (Timer.getFPGATimestamp() - enabledTimestamp >= 165 && config.enableRewind()) {
        LimelightHelpers.triggerRewindCapture(name, 165);
        enabledTimestamp = Timer.getFPGATimestamp();
      }
    }

    if (DriverStation.isDisabled()) {
      if (wasEnabled) {
        if (config.enableRewind()) {
          LimelightHelpers.triggerRewindCapture(name, Timer.getFPGATimestamp() - enabledTimestamp);
        }
        wasEnabled = false;
        LimelightHelpers.SetIMUMode(name, 1);
        if (GompeiLib.isTuning()) {
          LimelightHelpers.SetThrottle(name, 190);
        } else {
          LimelightHelpers.SetIMUMode(name, 0);
        }
      }
    }

    headingPublisher.set(
        new double[] {headingSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0},
        timestampSupplier.getAsLong());

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();

    double xyStdDev = config.megatagXYStdev();
    double thetaStdev = config.metatagThetaStdev();

    if (inputs.mt1PoseEstimate.tagCount() != 0) {
      xyStdDev =
          config.megatagXYStdev()
              * Math.pow(
                  inputs.mt1PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt1PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
      thetaStdev =
          inputs.mt1PoseEstimate.tagCount() > 1
                  && Math.abs(chassisSpeedsSupplier.get().vxMetersPerSecond) <= 0.15
                  && Math.abs(chassisSpeedsSupplier.get().vyMetersPerSecond) <= 0.15
                  && Math.abs(chassisSpeedsSupplier.get().omegaRadiansPerSecond) <= 0.05
                  && Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                          .mapToDouble(CameraIO.RawFiducial::ambiguity)
                          .average()
                          .orElse(Double.MAX_VALUE)
                      < VisionConstants.AMBIGUITY_THRESHOLD
              ? config.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT)
              : Double.POSITIVE_INFINITY;

      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt1PoseEstimate.pose(),
              Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt1PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    if (inputs.mt2PoseEstimate.tagCount() != 0) {
      xyStdDev =
          config.megatag2XYStdev()
              * Math.pow(
                  inputs.mt2PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt2PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
      thetaStdev = Double.POSITIVE_INFINITY;

      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt2PoseEstimate.pose(),
              Arrays.stream(inputs.mt2PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt2PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    if (inputs.rawFiducials.length != 0) {
      Arrays.stream(inputs.rawFiducials)
          .forEach(
              fiducial ->
                  singleTxTyObservationList.add(
                      new VisionSingleTxTyObservation(
                          fiducial.id(),
                          fiducial.txnc(),
                          fiducial.tync(),
                          fiducial.distToCamera(),
                          inputs.mt1PoseEstimate.timestampSeconds(),
                          currentCameraPose)));
    }

    super.sendObservers();
  }
}
