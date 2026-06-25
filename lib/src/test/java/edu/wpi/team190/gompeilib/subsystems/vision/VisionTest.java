package edu.wpi.team190.gompeilib.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.Camera;
import java.util.List;
import org.junit.jupiter.api.Test;

public class VisionTest {

  @Test
  public void testVisionInitializationAndPeriodic() {
    AprilTag tag1 =
        new AprilTag(1, new Pose3d(1.0, 2.0, 3.0, new edu.wpi.first.math.geometry.Rotation3d()));
    AprilTagFieldLayout layout = new AprilTagFieldLayout(List.of(tag1), 16.0, 8.0);

    Camera mockCamera1 = mock(Camera.class);
    Camera mockCamera2 = mock(Camera.class);

    Vision vision = new Vision(() -> layout, mockCamera1, mockCamera2);

    assertSame(layout, vision.getFieldLayoutSupplier().get());
    assertEquals(2, vision.getCameras().length);
    assertSame(mockCamera1, vision.getCameras()[0]);
    assertSame(mockCamera2, vision.getCameras()[1]);

    // Check that tags are published to NetworkTables table "field"
    double[] tagData =
        NetworkTableInstance.getDefault()
            .getTable("field")
            .getDoubleArrayTopic("tag_1")
            .subscribe(new double[0])
            .get();

    assertEquals(7, tagData.length);
    assertEquals(1.0, tagData[0]);
    assertEquals(2.0, tagData[1]);
    assertEquals(3.0, tagData[2]);
    assertEquals(1.0, tagData[3]); // W of Quaternion (no rotation)
    assertEquals(0.0, tagData[4]); // X of Quaternion
    assertEquals(0.0, tagData[5]); // Y of Quaternion
    assertEquals(0.0, tagData[6]); // Z of Quaternion

    // Verify periodic
    vision.periodic();
    verify(mockCamera1, times(1)).periodic();
    verify(mockCamera2, times(1)).periodic();
  }
}
