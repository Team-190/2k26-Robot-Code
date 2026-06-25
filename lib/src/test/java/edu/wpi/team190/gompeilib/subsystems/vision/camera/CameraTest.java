package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

public class CameraTest {

  private static class TestCamera extends Camera {
    public TestCamera(
        String name,
        List<Consumer<List<VisionPoseObservation>>> poseObservers,
        List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers,
        List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
      super(name, poseObservers, multiTxTyObservers, singleTxTyObservers);
    }
  }

  @Test
  public void testCameraBaseMethods() {
    List<List<VisionPoseObservation>> receivedPose = new ArrayList<>();
    List<List<VisionMultiTxTyObservation>> receivedMulti = new ArrayList<>();
    List<List<VisionSingleTxTyObservation>> receivedSingle = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = receivedPose::add;
    Consumer<List<VisionMultiTxTyObservation>> multiObserver = receivedMulti::add;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = receivedSingle::add;

    TestCamera camera =
        new TestCamera(
            "TestCam", List.of(poseObserver), List.of(multiObserver), List.of(singleObserver));

    assertEquals("TestCam", camera.getName());
    assertNotNull(camera.getPoseObservationList());
    assertNotNull(camera.getMultiTxTyObservationList());
    assertNotNull(camera.getSingleTxTyObservationList());

    Pose3d pose = new Pose3d();
    camera.setCameraPose(pose);
    assertEquals(pose, camera.getCurrentCameraPose());

    // Call periodic (no-op in base class)
    camera.periodic();

    // Trigger observers when lists are empty
    camera.sendObservers();
    assertEquals(1, receivedPose.size());
    assertEquals(1, receivedMulti.size());
    assertEquals(1, receivedSingle.size());
    assertTrue(receivedPose.get(0).isEmpty());
    assertTrue(receivedMulti.get(0).isEmpty());
    assertTrue(receivedSingle.get(0).isEmpty());
  }
}
