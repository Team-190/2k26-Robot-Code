package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Function;

public class MovingShotCalculator {
  public record MovingShotParameters(
      Pose2d robotPose,
      ChassisSpeeds robotFieldVelocity,
      Transform3d robotToShooter,
      Function<Double, Double> distanceToTimeFunction,
      Translation2d targetPosition,
      int iterations) {}

  public static Translation2d calculateMovingShotEffectiveTarget(MovingShotParameters params) {
    for (int i = 0; i < params.iterations(); i++) {
      double distanceToTarget =
          params.robotPose().getTranslation().getDistance(params.targetPosition());
      double timeToTarget = params.distanceToTimeFunction().apply(distanceToTarget);

      Translation2d shooterVelocityFieldRelative =
          new Translation2d(
                  params.robotFieldVelocity().vxMetersPerSecond,
                  params.robotFieldVelocity().vyMetersPerSecond) // robot linear velocity
              .plus(
                  new Translation2d(params.robotToShooter().getY(), params.robotToShooter().getX())
                      .rotateBy(params.robotPose().getRotation())
                      .times(
                          params.robotFieldVelocity()
                              .omegaRadiansPerSecond)); // tangential velocity

      Translation2d positionChange = shooterVelocityFieldRelative.times(timeToTarget);

      Translation2d newTarget =
          params
              .targetPosition()
              .minus(positionChange); // we change the target, instead of the robot
      // position. The math
      // works out the same as they are relative to eachother
      params =
          new MovingShotParameters(
              params.robotPose(),
              params.robotFieldVelocity(),
              params.robotToShooter(),
              params.distanceToTimeFunction(),
              newTarget,
              params.iterations() - 1);
    }
    return params.targetPosition();
  }
}
