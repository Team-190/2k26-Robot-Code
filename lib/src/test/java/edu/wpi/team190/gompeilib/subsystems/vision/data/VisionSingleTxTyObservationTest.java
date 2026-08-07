package edu.wpi.team190.gompeilib.subsystems.vision.data;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.Test;

public class VisionSingleTxTyObservationTest {

  @Test
  public void testRecord() {
    Pose3d cameraPose = new Pose3d();
    VisionSingleTxTyObservation obs =
        new VisionSingleTxTyObservation(15, 1.1, 2.2, 3.3, 4.4, cameraPose);

    assertEquals(15, obs.tagId());
    assertEquals(1.1, obs.tx());
    assertEquals(2.2, obs.ty());
    assertEquals(3.3, obs.distance());
    assertEquals(4.4, obs.timestamp());
    assertEquals(cameraPose, obs.cameraPose());

    assertNotNull(obs.toString());
    assertEquals(obs.hashCode(), obs.hashCode());
    assertEquals(obs, obs);
  }
}
