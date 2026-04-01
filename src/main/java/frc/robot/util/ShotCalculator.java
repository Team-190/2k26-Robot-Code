package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public interface ShotCalculator {
  double phaseDelay = 0.3; // TODO: Change this value based on testing

  /**
   * Calculates a corrected pose for a moving target based on the shooter's current velocity.
   *
   * @param initialPose The current pose of the shooter.
   * @param targetPose The pose of the target.
   * @param robotVelocityMetersPerSecond The shooter's velocity in meters per second.
   * @param distanceToTimeFunction A function that converts distance to time.
   * @return The corrected pose to aim at.
   */
  static Translation2d getAdjustedTargetPose(
      Pose2d initialPose,
      Pose2d targetPose,
      ChassisSpeeds robotVelocityMetersPerSecond,
      Function<Distance, Time> distanceToTimeFunction,
      Transform2d centerToShooterCenter) {

    // Adds phase delay to the initial pose based on robot velocity to account for
    // latency caused by
    // target pose calculation
    initialPose =
        initialPose
            .transformBy(centerToShooterCenter)
            .exp(
                new Twist2d(
                    robotVelocityMetersPerSecond.vxMetersPerSecond * phaseDelay,
                    robotVelocityMetersPerSecond.vyMetersPerSecond * phaseDelay,
                    robotVelocityMetersPerSecond.omegaRadiansPerSecond * phaseDelay));

    Pose2d shooterPose = initialPose.plus(centerToShooterCenter);
    Transform2d shooterToTarget = new Transform2d(shooterPose, targetPose);

    Translation2d shooterRobotFrameVelocityMetersPerSecond =
        new Translation2d(
                -centerToShooterCenter
                    .getTranslation()
                    .getAngle()
                    .plus(initialPose.getRotation())
                    .getSin(),
                centerToShooterCenter
                    .getTranslation()
                    .getAngle()
                    .plus(initialPose.getRotation())
                    .getCos())
            .times(robotVelocityMetersPerSecond.omegaRadiansPerSecond)
            .times(centerToShooterCenter.getTranslation().getNorm());

    Translation2d shooterFieldFrameVelocityMetersPerSecond =
        shooterRobotFrameVelocityMetersPerSecond.plus(
            new Translation2d(
                robotVelocityMetersPerSecond.vxMetersPerSecond,
                robotVelocityMetersPerSecond.vyMetersPerSecond));

    double deltaT =
        distanceToTimeFunction
            .apply(Meters.of(shooterToTarget.getTranslation().getNorm()))
            .in(Seconds);

    double correctedX =
        targetPose.getX() - shooterFieldFrameVelocityMetersPerSecond.getX() * deltaT;
    double correctedY =
        targetPose.getY() - shooterFieldFrameVelocityMetersPerSecond.getY() * deltaT;

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Target Pose", targetPose);
    Logger.recordOutput(
        NTPrefixes.POSE_DATA + "Corrected Pose",
        new Pose2d(correctedX, correctedY, new Rotation2d()));

    return new Translation2d(correctedX, correctedY);
  }
}
