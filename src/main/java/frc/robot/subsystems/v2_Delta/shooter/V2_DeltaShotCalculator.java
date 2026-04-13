package frc.robot.subsystems.v2_Delta.shooter;

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
public class V2_DeltaShotCalculator {
  private static final LinearFilter hoodVelocityFilter = LinearFilter.movingAverage(5);

  private static final double SETPOINT_TIME_CONSTANT_SECS = 0.06;
  private static final LinearFilter hoodSetpointFilter =
      LinearFilter.singlePoleIIR(SETPOINT_TIME_CONSTANT_SECS, GompeiLib.getLoopPeriod());

  private static final double CONVERGENCE_THRESHOLD_METERS = 0.005;
  private static final int MAX_ITERATIONS = 20;
  private static final double MIN_ALIGN_DISTANCE_METERS = 0.5;

  private static Rotation2d lastHoodAngle;
  private static Rotation2d smoothedTurretAngle;

  public static void clear() {
    smoothedTurretAngle = null;
    lastHoodAngle = null;
    hoodVelocityFilter.reset();
    hoodSetpointFilter.reset();
  }

  public static ShotParameters getShotParameters(
      Pose2d robotPose,
      Translation2d targetPose,
      Transform2d robotToShooterTransform,
      ChassisSpeeds robotRelativeVelocity,
      Time phaseDelay,
      Function<Distance, Time> distanceToTimeFunction,
      Function<Distance, Rotation2d> distanceToHoodFunction,
      Function<Distance, AngularVelocity> distanceToFlywheelFunction) {
    boolean moving =
        Math.abs(robotRelativeVelocity.omegaRadiansPerSecond) >= 0.05
            && Math.abs(robotRelativeVelocity.vyMetersPerSecond) >= .02
            && Math.abs(robotRelativeVelocity.vxMetersPerSecond) >= 0.02;

    double lookaheadDistance;
    Translation2d lookaheadRobotPosition;
    Translation2d robotPosition;

    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, robotPose.getRotation());
    if (moving) { // Predict pose based on system latency
      Pose2d phaseDelayedPose =
          robotPose.exp(robotRelativeVelocity.toTwist2d(phaseDelay.in(Seconds)));
      robotPosition = phaseDelayedPose.getTranslation();

      lookaheadDistance = targetPose.getDistance(robotPosition);
      lookaheadRobotPosition = robotPosition;

      for (int i = 0; i < MAX_ITERATIONS; i++) {
        double clampedDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);
        Time timeOfFlight = distanceToTimeFunction.apply(Meters.of(clampedDistance));

        double offsetX = fieldVelocity.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double offsetY = fieldVelocity.vyMetersPerSecond * timeOfFlight.in(Seconds);

        Translation2d nextPosition = robotPosition.plus(new Translation2d(offsetX, offsetY));
        double newDistance = targetPose.getDistance(nextPosition);

        if (Math.abs(newDistance - lookaheadDistance) < CONVERGENCE_THRESHOLD_METERS) {
          lookaheadDistance = newDistance;
          lookaheadRobotPosition = nextPosition;
          break;
        }

        lookaheadDistance = newDistance;
        lookaheadRobotPosition = nextPosition;
      }
    } else {
      lookaheadDistance = targetPose.getDistance(robotPose.getTranslation());
      lookaheadRobotPosition = robotPose.getTranslation();
      robotPosition = robotPose.getTranslation();
    }
    boolean isDistanceValid = lookaheadDistance >= MIN_ALIGN_DISTANCE_METERS;
    Rotation2d rawTurretAngle;

    if (!isDistanceValid) {
      rawTurretAngle = robotPose.getRotation();
    } else {
      Rotation2d fieldToTargetAngle = targetPose.minus(lookaheadRobotPosition).getAngle();
      double shooterOffsetY = robotToShooterTransform.getTranslation().getY();

      Rotation2d lateralOffsetAngle =
          new Rotation2d(Math.asin(MathUtil.clamp(shooterOffsetY / lookaheadDistance, -1.0, 1.0)));

      rawTurretAngle =
          fieldToTargetAngle.minus(lateralOffsetAngle).minus(robotToShooterTransform.getRotation());
    }

    double clampedLookaheadDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);
    Rotation2d rawHoodAngle = distanceToHoodFunction.apply(Meters.of(clampedLookaheadDistance));
    Rotation2d currentHoodAngle =
        Rotation2d.fromRadians(hoodSetpointFilter.calculate(rawHoodAngle.getRadians()));

    if (smoothedTurretAngle == null) {
      smoothedTurretAngle = rawTurretAngle;
    } else {
      double dt = GompeiLib.getLoopPeriod();
      double alpha = dt / (SETPOINT_TIME_CONSTANT_SECS + dt);
      double error = MathUtil.angleModulus(rawTurretAngle.minus(smoothedTurretAngle).getRadians());
      smoothedTurretAngle = smoothedTurretAngle.plus(Rotation2d.fromRadians(error * alpha));
    }

    if (lastHoodAngle == null) lastHoodAngle = currentHoodAngle;

    // Calculate tangential component to cancel lag
    Translation2d toTarget = targetPose.minus(robotPosition);
    double distSq = toTarget.getNorm() * toTarget.getNorm();

    AngularVelocity turretVelocity;
    if (moving) {

      double angularComp =
          (fieldVelocity.vyMetersPerSecond * toTarget.getX()
                  - fieldVelocity.vxMetersPerSecond * toTarget.getY())
              / distSq;

      turretVelocity = RadiansPerSecond.of(angularComp);
    } else {
      turretVelocity = RadiansPerSecond.zero();
    }

    AngularVelocity hoodVelocity =
        RadiansPerSecond.of(
            hoodVelocityFilter.calculate(
                currentHoodAngle.minus(lastHoodAngle).getRadians() / GompeiLib.getLoopPeriod()));

    lastHoodAngle = currentHoodAngle;

    return new ShotParameters(
        isDistanceValid,
        new Pose2d(lookaheadRobotPosition, robotPose.getRotation()),
        currentHoodAngle,
        turretVelocity,
        hoodVelocity,
        distanceToFlywheelFunction.apply(Meters.of(clampedLookaheadDistance)));
  }

  public record ShotParameters(
      boolean isValid,
      Pose2d adjustedRobotPose,
      Rotation2d hoodAngle,
      AngularVelocity turretVelocity,
      AngularVelocity hoodVelocity,
      AngularVelocity flywheelSpeed) {}
}
