package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import lombok.Getter;

public abstract class Camera {
  @Getter private final String name;
  List<Consumer<List<VisionPoseObservation>>> poseObservers;
  List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers;
  List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers;

  @Getter List<VisionPoseObservation> poseObservationList;
  @Getter List<VisionMultiTxTyObservation> multiTxTyObservationList;
  @Getter List<VisionSingleTxTyObservation> singleTxTyObservationList;

  @Getter protected Pose3d currentCameraPose;

  public Camera(
      String name,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    this.name = name;
    this.poseObservers = poseObservers;
    this.multiTxTyObservers = multiTxTyObservers;
    this.singleTxTyObservers = singleTxTyObservers;

    poseObservationList = new ArrayList<>();
    multiTxTyObservationList = new ArrayList<>();
    singleTxTyObservationList = new ArrayList<>();
  }

  @Trace
  public void periodic() {}

  public void setCameraPose(Pose3d cameraPose) {
    currentCameraPose = cameraPose;
  }

  public void sendObservers() {
    for (Consumer<List<VisionPoseObservation>> observer : poseObservers) {
      observer.accept(poseObservationList);
    }
    for (Consumer<List<VisionMultiTxTyObservation>> observer : multiTxTyObservers) {
      observer.accept(multiTxTyObservationList);
    }
    for (Consumer<List<VisionSingleTxTyObservation>> observer : singleTxTyObservers) {
      observer.accept(singleTxTyObservationList);
    }
  }
}
