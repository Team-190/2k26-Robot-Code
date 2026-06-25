package edu.wpi.team190.gompeilib.subsystems.vision.data;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import org.junit.jupiter.api.Test;

public class VisionMultiTxTyObservationTest {

  @Test
  public void testRecord() {
    Pose3d cameraPose = new Pose3d();
    double[] tx = {1.0, 2.0};
    double[] ty = {3.0, 4.0};
    VisionMultiTxTyObservation obs =
        new VisionMultiTxTyObservation(42, tx, ty, 5.5, 10.0, cameraPose);

    assertEquals(42, obs.tagId());
    assertArrayEquals(tx, obs.tx());
    assertArrayEquals(ty, obs.ty());
    assertEquals(5.5, obs.distance());
    assertEquals(10.0, obs.timestamp());
    assertEquals(cameraPose, obs.cameraPose());

    assertNotNull(obs.toString());
    assertEquals(obs.hashCode(), obs.hashCode());
    assertEquals(obs, obs);
  }
}
