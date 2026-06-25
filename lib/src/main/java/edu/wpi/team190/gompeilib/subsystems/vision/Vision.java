package edu.wpi.team190.gompeilib.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.team190.gompeilib.core.utility.VirtualSubsystem;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.Camera;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * A class to contain all the {@link Camera Camera}s for a robot and methods to interact with them.
 * A vision object publishes the data about the field to the robot, and runs the periodic method for
 * all of its cameras.
 */
public class Vision extends VirtualSubsystem {
  @Getter private final Camera[] cameras;
  @Getter private final Supplier<AprilTagFieldLayout> fieldLayoutSupplier;

  public Vision(Supplier<AprilTagFieldLayout> fieldLayoutSupplier, Camera... cameras) {
    this.cameras = cameras;
    this.fieldLayoutSupplier = fieldLayoutSupplier;

    NetworkTable fieldTable = NetworkTableInstance.getDefault().getTable("field");

    for (AprilTag tag : fieldLayoutSupplier.get().getTags()) {
      fieldTable
          .getDoubleArrayTopic("tag_" + tag.ID)
          .publish()
          .set(
              new double[] {
                tag.pose.getX(),
                tag.pose.getY(),
                tag.pose.getZ(),
                tag.pose.getRotation().getQuaternion().getW(),
                tag.pose.getRotation().getQuaternion().getX(),
                tag.pose.getRotation().getQuaternion().getY(),
                tag.pose.getRotation().getQuaternion().getZ()
              });
    }
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
    }
  }
}
