package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import java.util.function.Function;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeometryUtil.class})
public class ShotCalculator {
  private static final LinearFilter chassisVelocityFilter = LinearFilter.movingAverage(5);
  private static final LinearFilter hoodVelocityFilter = LinearFilter.movingAverage(5);

  private static final double SETPOINT_TIME_CONSTANT_SECS = 0.06;
  private static final LinearFilter hoodSetpointFilter =
      LinearFilter.singlePoleIIR(SETPOINT_TIME_CONSTANT_SECS, GompeiLib.getLoopPeriod());
  private static Rotation2d smoothedChassisAngle;

  private static final double CONVERGENCE_THRESHOLD_METERS = 0.005;
  private static final int MAX_ITERATIONS = 20;
  private static final double MIN_ALIGN_DISTANCE_METERS = 0.5;

  private static Rotation2d lastChassisAngle;
  private static Rotation2d lastHoodAngle;

  public static ShotParameters getShotParameters(
      Pose2d robotPose,
      Translation2d targetPose,
      Transform2d robotToShooterTransform,
      ChassisSpeeds robotRelativeVelocity,
      Time phaseDelay,
      Function<Distance, Time> distanceToTimeFunction,
      Function<Distance, Rotation2d> distanceToHoodFunction,
      Function<Distance, AngularVelocity> distanceToFlywheelFunction) {

    Pose2d phaseDelayedPose =
        robotPose.exp(robotRelativeVelocity.toTwist2d(phaseDelay.in(Seconds)));
    Translation2d robotPosition = phaseDelayedPose.getTranslation();

    Translation2d lookaheadRobotPosition = robotPosition;
    double lookaheadDistance = targetPose.getDistance(robotPosition);

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double clampedDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);
      Time timeOfFlight = distanceToTimeFunction.apply(Meters.of(clampedDistance));

      double offsetX =
          ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation())
                  .vxMetersPerSecond
              * timeOfFlight.in(Seconds);
      double offsetY =
          ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation())
                  .vyMetersPerSecond
              * timeOfFlight.in(Seconds);

      lookaheadRobotPosition = robotPosition.plus(new Translation2d(offsetX, offsetY));

      double newLookaheadDistance = targetPose.getDistance(lookaheadRobotPosition);

      if (Math.abs(newLookaheadDistance - lookaheadDistance) < CONVERGENCE_THRESHOLD_METERS) {
        lookaheadDistance = newLookaheadDistance;
        break;
      }

      lookaheadDistance = newLookaheadDistance;
    }

    boolean isDistanceValid = lookaheadDistance >= MIN_ALIGN_DISTANCE_METERS;
    Rotation2d rawChassisAngle;

    if (!isDistanceValid) {
      rawChassisAngle = lastChassisAngle != null ? lastChassisAngle : robotPose.getRotation();
    } else {
      Rotation2d fieldToTargetAngle = targetPose.minus(lookaheadRobotPosition).getAngle();
      double shooterOffsetY = robotToShooterTransform.getTranslation().getY();

      Rotation2d lateralOffsetAngle =
          new Rotation2d(Math.asin(MathUtil.clamp(shooterOffsetY / lookaheadDistance, -1.0, 1.0)));

      rawChassisAngle =
          fieldToTargetAngle.minus(lateralOffsetAngle).minus(robotToShooterTransform.getRotation());
    }

    double clampedLookaheadDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);
    Rotation2d rawHoodAngle = distanceToHoodFunction.apply(Meters.of(clampedLookaheadDistance));

    Rotation2d currentHoodAngle =
        Rotation2d.fromRadians(hoodSetpointFilter.calculate(rawHoodAngle.getRadians()));

    Rotation2d currentChassisAngle;
    if (smoothedChassisAngle == null) {
      smoothedChassisAngle = rawChassisAngle;
      currentChassisAngle = rawChassisAngle;
    } else {
      double dt = GompeiLib.getLoopPeriod();
      double alpha = dt / (SETPOINT_TIME_CONSTANT_SECS + dt);
      double angleErrorRads =
          MathUtil.angleModulus(rawChassisAngle.minus(smoothedChassisAngle).getRadians());

      smoothedChassisAngle =
          smoothedChassisAngle.plus(Rotation2d.fromRadians(angleErrorRads * alpha));
      currentChassisAngle = smoothedChassisAngle;
    }

    if (lastChassisAngle == null) lastChassisAngle = currentChassisAngle;
    if (lastHoodAngle == null) lastHoodAngle = currentHoodAngle;

    AngularVelocity chassisVelocity =
        RadiansPerSecond.of(
            chassisVelocityFilter.calculate(
                currentChassisAngle.minus(lastChassisAngle).getRadians()
                    / GompeiLib.getLoopPeriod()));
    AngularVelocity hoodVelocity =
        RadiansPerSecond.of(
            hoodVelocityFilter.calculate(
                currentHoodAngle.minus(lastHoodAngle).getRadians() / GompeiLib.getLoopPeriod()));

    lastChassisAngle = currentChassisAngle;
    lastHoodAngle = currentHoodAngle;

    return new ShotParameters(
        isDistanceValid,
        new Pose2d(lookaheadRobotPosition, currentChassisAngle),
        currentChassisAngle,
        currentHoodAngle,
        chassisVelocity,
        hoodVelocity,
        distanceToFlywheelFunction.apply(Meters.of(clampedLookaheadDistance)));
  }

  public record ShotParameters(
      boolean isValid,
      Pose2d adjustedRobotPose,
      Rotation2d chassisAngle,
      Rotation2d hoodAngle,
      AngularVelocity chassisVelocity,
      AngularVelocity hoodVelocity,
      AngularVelocity flywheelSpeed) {}
}
