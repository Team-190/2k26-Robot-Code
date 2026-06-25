package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class FieldZoneTest {
  @Test
  public void testFieldZone() {
    AprilTag tag = new AprilTag(1, new Pose3d());
    FieldZone zone = new FieldZone(Set.of(tag));
    assertEquals(Set.of(tag), zone.aprilTags());
  }
}
