package edu.wpi.team190.gompeilib.subsystems.vision.data;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class VisionPoseObservationTest {

  @Test
  public void testRecord() {
    Pose2d pose = new Pose2d();
    Set<Integer> tagIds = Set.of(1, 2, 3);
    var stddevs = VecBuilder.fill(0.1, 0.1, 0.2);
    VisionPoseObservation obs = new VisionPoseObservation(pose, tagIds, 12.34, stddevs);

    assertEquals(pose, obs.pose());
    assertEquals(tagIds, obs.tagIds());
    assertEquals(12.34, obs.timestamp());
    assertEquals(stddevs, obs.stddevs());

    assertNotNull(obs.toString());
    assertEquals(obs.hashCode(), obs.hashCode());
    assertEquals(obs, obs);
  }
}
