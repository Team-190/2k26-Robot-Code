package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;

public class FieldZoneTest {
  @Test
  public void testFieldZone() {
    FieldTag tag = new FieldTag(1, new Pose3d());
    FieldZone zone = new FieldZone(Set.of(tag));
    assertEquals(Set.of(tag), zone.aprilTags());
  }
}
