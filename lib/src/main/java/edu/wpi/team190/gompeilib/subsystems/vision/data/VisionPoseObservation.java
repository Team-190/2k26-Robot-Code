package edu.wpi.team190.gompeilib.subsystems.vision.data;

import java.util.Set;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public record VisionPoseObservation(
    Pose2d pose, Set<Integer> tagIds, double timestamp, Matrix<N3, N1> stddevs) {}
