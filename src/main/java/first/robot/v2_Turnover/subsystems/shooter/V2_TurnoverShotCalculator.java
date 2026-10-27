package first.robot.v2_Turnover.subsystems.shooter;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import java.util.function.Function;
import lombok.experimental.ExtensionMethod;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

@ExtensionMethod({GeometryUtil.class})
public class V2_TurnoverShotCalculator {
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
      ChassisVelocities robotRelativeVelocity,
      Time phaseDelay,
      Function<Distance, Time> distanceToTimeFunction,
      Function<Distance, Rotation2d> distanceToHoodFunction,
      Function<Distance, AngularVelocity> distanceToFlywheelFunction) {

    ChassisVelocities fieldVelocity =
        robotRelativeVelocity.toFieldRelative(robotPose.getRotation());

    boolean moving =
        Math.hypot(fieldVelocity.vx, fieldVelocity.vy) > 0.02
            || Math.abs(fieldVelocity.omega) > 0.05;

    Pose2d basePose =
        moving
            ? robotPose.plus(robotRelativeVelocity.toTwist2d(phaseDelay.in(Seconds)).exp())
            : robotPose;

    Translation2d shooterPosition = basePose.transformBy(robotToShooterTransform).getTranslation();

    double lookaheadDistance = targetPose.getDistance(shooterPosition);

    Pose2d lookaheadPose = basePose;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double clampedDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);

      Time timeOfFlight = distanceToTimeFunction.apply(Meters.of(clampedDistance));
      double t = timeOfFlight.in(Seconds);

      Translation2d deltaTranslation =
          new Translation2d(fieldVelocity.vx * t, fieldVelocity.vy * t);

      Pose2d futurePose =
          new Pose2d(basePose.getTranslation().plus(deltaTranslation), basePose.getRotation());

      Translation2d nextShooterPosition =
          futurePose.transformBy(robotToShooterTransform).getTranslation();

      double newDistance = targetPose.getDistance(nextShooterPosition);

      if (Math.abs(newDistance - lookaheadDistance) < CONVERGENCE_THRESHOLD_METERS) {
        lookaheadPose = futurePose;
        shooterPosition = nextShooterPosition;
        lookaheadDistance = newDistance;
        break;
      }

      lookaheadPose = futurePose;
      shooterPosition = nextShooterPosition;
      lookaheadDistance = newDistance;
    }

    boolean isDistanceValid = lookaheadDistance >= MIN_ALIGN_DISTANCE_METERS;
    double clampedDistance = Math.max(lookaheadDistance, MIN_ALIGN_DISTANCE_METERS);

    Rotation2d rawTurretAngle;

    if (!isDistanceValid) {
      rawTurretAngle = lookaheadPose.getRotation(); // fallback
    } else {
      rawTurretAngle = targetPose.minus(shooterPosition).getAngle().orElse(Rotation2d.ZERO);
    }

    if (smoothedTurretAngle == null) {
      smoothedTurretAngle = rawTurretAngle;
    } else {
      double dt = GompeiLib.getLoopPeriod();
      double alpha = dt / (SETPOINT_TIME_CONSTANT_SECS + dt);
      double error = MathUtil.angleModulus(rawTurretAngle.minus(smoothedTurretAngle).getRadians());
      smoothedTurretAngle = smoothedTurretAngle.plus(Rotation2d.fromRadians(error * alpha));
    }

    Rotation2d rawHoodAngle = distanceToHoodFunction.apply(Meters.of(clampedDistance));

    Rotation2d currentHoodAngle =
        Rotation2d.fromRadians(hoodSetpointFilter.calculate(rawHoodAngle.getRadians()));

    if (lastHoodAngle == null) lastHoodAngle = currentHoodAngle;

    Translation2d toTarget = targetPose.minus(shooterPosition);
    double distSq = Math.max(toTarget.getNorm() * toTarget.getNorm(), 1e-4);

    AngularVelocity turretVelocity;

    if (moving) {
      double angularComp =
          (fieldVelocity.vy * toTarget.getX() - fieldVelocity.vx * toTarget.getY()) / distSq;

      turretVelocity = RadiansPerSecond.of(angularComp);
    } else {
      turretVelocity = RadiansPerSecond.zero();
    }

    AngularVelocity hoodVelocity =
        RadiansPerSecond.of(
            hoodVelocityFilter.calculate(
                currentHoodAngle.minus(lastHoodAngle).getRadians() / GompeiLib.getLoopPeriod()));

    lastHoodAngle = currentHoodAngle;

    AngularVelocity flywheelSpeed = distanceToFlywheelFunction.apply(Meters.of(clampedDistance));

    return new ShotParameters(
        isDistanceValid,
        lookaheadPose,
        currentHoodAngle,
        turretVelocity,
        hoodVelocity,
        flywheelSpeed);
  }

  public record ShotParameters(
      boolean isValid,
      Pose2d adjustedRobotPose,
      Rotation2d hoodAngle,
      AngularVelocity turretVelocity,
      AngularVelocity hoodVelocity,
      AngularVelocity flywheelSpeed) {}
}
