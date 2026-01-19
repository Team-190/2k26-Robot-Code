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

    Translation2d shooterRobotFrameVelocityMetersPerSecond =
        new Translation2d(-centerToShooterCenter.getRotation().getSin(),
                centerToShooterCenter.getRotation().getCos())
                .times(robotVelocityMetersPerSecond.omegaRadiansPerSecond)
                .times(centerToShooterCenter.getTranslation().getNorm());

    Translation2d velocityMetersPerSecond = new Translation2d(
        shooterRobotFrameVelocityMetersPerSecond.getX() * initialPose.getRotation().getCos()
            - shooterRobotFrameVelocityMetersPerSecond.getY() * initialPose.getRotation().getSin(),
        shooterRobotFrameVelocityMetersPerSecond.getY() * initialPose.getRotation().getCos()
            + shooterRobotFrameVelocityMetersPerSecond.getX() * initialPose.getRotation().getSin())
            .plus(new Translation2d(robotVelocityMetersPerSecond.vxMetersPerSecond,robotVelocityMetersPerSecond.vyMetersPerSecond));

    double deltaT = distanceToTimeFunction.apply(shooterToTarget.getTranslation().getNorm());

    double correctedX = targetPose.getX() - velocityMetersPerSecond.getX() * deltaT;
    double correctedY = targetPose.getY() - velocityMetersPerSecond.getY() * deltaT;

    return new Translation2d(correctedX, correctedY);
  }
}
