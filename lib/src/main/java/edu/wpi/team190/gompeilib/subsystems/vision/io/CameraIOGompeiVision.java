package edu.wpi.team190.gompeilib.subsystems.vision.io;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import java.util.*;
import lombok.Getter;

public class CameraIOGompeiVision implements CameraIO {
  @Getter private final String name;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber captureFPSAprilTagSubscriber;
  private final IntegerSubscriber processingFPSAprilTagSubscriber;

  public CameraIOGompeiVision(VisionConstants.GompeiVisionConfig config) {

    this.name = config.key();

    NetworkTable configTable =
        NetworkTableInstance.getDefault()
            .getTable("cameras")
            .getSubTable(this.name)
            .getSubTable("config");
    NetworkTable outputTable =
        NetworkTableInstance.getDefault()
            .getTable("cameras")
            .getSubTable(this.name)
            .getSubTable("output");

    String deviceId = config.key();

    configTable.getStringTopic("role").publish().set(name);
    configTable.getStringTopic("hardware_id").publish().set(deviceId);
    configTable.getDoubleArrayTopic("camera_matrix").publish().set(config.cameraMatrix().getData());
    configTable
        .getDoubleArrayTopic("distortion_coefficients")
        .publish()
        .set(config.distortionCoefficients().getData());
    configTable.getDoubleTopic("exposure").publish().set(config.exposure());
    configTable.getDoubleTopic("gain").publish().set(config.gain());
    configTable.getIntegerTopic("width").publish().set(config.width());
    configTable.getIntegerTopic("height").publish().set(config.height());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(Units.inchesToMeters(6.50));
    configTable.getBooleanTopic("setup_mode").publish().set(false);

    this.observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    this.captureFPSAprilTagSubscriber = outputTable.getIntegerTopic("capture_fps").subscribe(0);
    this.processingFPSAprilTagSubscriber =
        outputTable.getIntegerTopic("processing_fps").subscribe(0);
  }

  @Override
  public void updateInputs(GompeiVisionIOInputs inputs) {
    var aprilTagQueue = observationSubscriber.readQueue();
    inputs.timestamps = new double[aprilTagQueue.length];
    inputs.frames = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      inputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      inputs.frames[i] = aprilTagQueue[i].value;
    }
    inputs.captureFPS = captureFPSAprilTagSubscriber.get();
    inputs.processingFPS = processingFPSAprilTagSubscriber.get();
  }
}
