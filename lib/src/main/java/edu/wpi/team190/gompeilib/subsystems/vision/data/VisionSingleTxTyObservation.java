package edu.wpi.team190.gompeilib.subsystems.vision.data;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionSingleTxTyObservation(
    int tagId, double tx, double ty, double distance, double timestamp, Pose3d cameraPose) {}
