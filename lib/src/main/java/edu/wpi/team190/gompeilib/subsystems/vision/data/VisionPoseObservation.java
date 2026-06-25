package edu.wpi.team190.gompeilib.subsystems.vision.data;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Set;

public record VisionPoseObservation(
    Pose2d pose, Set<Integer> tagIds, double timestamp, Matrix<N3, N1> stddevs) {}
