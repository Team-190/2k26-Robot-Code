package frc.robot.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Function;

public interface MovingShotCorrection {
  /**
   * Calculates a corrected pose for a moving target based on the shooter's current velocity.
   *
   * @param initialPose The current pose of the shooter.
   * @param targetPose The pose of the target.
   * @param robotVelocityMetersPerSecond The shooter's velocity in meters per second.
   * @param distanceToTimeFunction A function that converts distance to time.
   * @return The corrected pose to aim at.
   */
  public default Translation2d getCorrection(
      Pose2d initialPose,
      Pose2d targetPose,
      ChassisSpeeds robotVelocityMetersPerSecond,
      Function<Double, Double> distanceToTimeFunction,
      Transform2d centerToShooterCenter) {

    Pose2d shooterPose = initialPose.plus(centerToShooterCenter);
    Transform2d shooterToTarget = new Transform2d(shooterPose, targetPose);

    Translation2d shooterVelocityMetersPerSecond =
        new Translation2d(-centerToShooterCenter.getRotation().getSin(),
                centerToShooterCenter.getRotation().getCos())
                .times(robotVelocityMetersPerSecond.omegaRadiansPerSecond)
                .times(centerToShooterCenter.getTranslation().getNorm());

    double xVelocityMetersPerSecond =
        shooterVelocityMetersPerSecond.getX() * initialPose.getRotation().getCos()
            - shooterVelocityMetersPerSecond.getY() * initialPose.getRotation().getSin()
            + robotVelocityMetersPerSecond.vxMetersPerSecond;
    double yVelocityMetersPerSecond =
        shooterVelocityMetersPerSecond.getY() * initialPose.getRotation().getCos()
            + shooterVelocityMetersPerSecond.getX() * initialPose.getRotation().getSin()
            + robotVelocityMetersPerSecond.vyMetersPerSecond;

    double deltaT = distanceToTimeFunction.apply(shooterToTarget.getTranslation().getNorm());

    double correctedX = targetPose.getX() - xVelocityMetersPerSecond * deltaT;
    double correctedY = targetPose.getY() - yVelocityMetersPerSecond * deltaT;

    return new Translation2d(correctedX, correctedY);
  }
}
