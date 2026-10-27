package edu.wpi.team190.gompeilib.subsystems.vision;

import edu.wpi.team190.gompeilib.core.utility.VirtualSubsystem;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.Camera;
import java.util.function.Supplier;
import lombok.Getter;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A class to contain all the {@link Camera Camera}s for a robot and methods to interact with them.
 * A vision object publishes the data about the field to the robot, and runs the periodic method for
 * all of its cameras.
 */
public class Vision extends VirtualSubsystem {
  @Getter private final Camera[] cameras;
  @Getter private final Supplier<Field> fieldLayoutSupplier;

  public Vision(Supplier<Field> fieldLayoutSupplier, Camera... cameras) {
    this.cameras = cameras;
    this.fieldLayoutSupplier = fieldLayoutSupplier;

    NetworkTable fieldTable = NetworkTableInstance.getDefault().getTable("field");

    for (FieldTag tag : fieldLayoutSupplier.get().getTags()) {
      fieldTable
          .getDoubleArrayTopic("tag_" + tag.getID())
          .publish()
          .set(
              new double[] {
                tag.getPose().getX(),
                tag.getPose().getY(),
                tag.getPose().getZ(),
                tag.getPose().getRotation().getQuaternion().getW(),
                tag.getPose().getRotation().getQuaternion().getX(),
                tag.getPose().getRotation().getQuaternion().getY(),
                tag.getPose().getRotation().getQuaternion().getZ()
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
