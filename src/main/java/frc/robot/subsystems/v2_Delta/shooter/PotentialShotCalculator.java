// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import java.util.function.Function;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeometryUtil.class})
public class PotentialShotCalculator {
  private static final LinearFilter turretAngleFilter = LinearFilter.movingAverage(5);
  private static final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(5);

  private static Rotation2d lastTurretAngle;
  private static Rotation2d lastHoodAngle;

  private static Rotation2d currentTurretAngle;
  private static Rotation2d currentHoodAngle;

  private static AngularVelocity turretVelocity;
  private static AngularVelocity hoodVelocity;

  private static ShotParameters latestShotParameters = null;

  private static Time phaseDelay;

  static {
    phaseDelay = Milliseconds.of(30);
  }

  public static ShotParameters getShotParameters(
      Pose2d robotPose,
      Translation2d targetPose,
      ChassisSpeeds robotVelocityMetersPerSecond,
      Function<Distance, Time> distanceToTimeFunction,
      Function<Distance, Rotation2d> distanceToHoodFunction,
      Function<Distance, AngularVelocity> distanceToFlywheelFunction,
      Pose2d robotToTurretTransform) {
    if (latestShotParameters != null) {
      return latestShotParameters;
    }

    Pose2d phaseDelayedPose =
        robotPose.exp(
            new Twist2d(
                robotVelocityMetersPerSecond.vxMetersPerSecond * phaseDelay.in(Seconds),
                robotVelocityMetersPerSecond.vyMetersPerSecond * phaseDelay.in(Seconds),
                robotVelocityMetersPerSecond.omegaRadiansPerSecond * phaseDelay.in(Seconds)));

    // Distance calculation
    Pose2d turretPosition = phaseDelayedPose.transformBy(robotToTurretTransform.toTransform2d());
    Distance turretToTargetDistance =
        Meters.of(targetPose.getDistance(turretPosition.getTranslation()));

    // Field relative turret velocity
    Rotation2d robotRotation = robotPose.getRotation();
    LinearVelocity turretVelocityX =
        MetersPerSecond.of(
            robotVelocityMetersPerSecond.vxMetersPerSecond
                + robotVelocityMetersPerSecond.omegaRadiansPerSecond
                    * (robotToTurretTransform.getY() * Math.cos(robotRotation.getRadians())
                        - robotToTurretTransform.getX() * Math.sin(robotRotation.getRadians())));
    LinearVelocity turretVelocityY =
        MetersPerSecond.of(
            robotVelocityMetersPerSecond.vyMetersPerSecond
                + robotVelocityMetersPerSecond.omegaRadiansPerSecond
                    * (robotToTurretTransform.getX() * Math.cos(robotRotation.getRadians())
                        - robotToTurretTransform.getY() * Math.sin(robotRotation.getRadians())));

    // Account for velocity imparted by robot
    Time timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    Distance lookaheadTurretToTargetDistance = turretToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight = distanceToTimeFunction.apply(lookaheadTurretToTargetDistance);
      Distance offsetX = turretVelocityX.times(timeOfFlight);
      Distance offsetY = turretVelocityY.times(timeOfFlight);
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance =
          Meters.of(targetPose.getDistance(lookaheadPose.getTranslation()));
    }

    // Calculate parameters accounted for imparted velocity
    currentTurretAngle = targetPose.minus(lookaheadPose.getTranslation()).getAngle();
    currentHoodAngle = distanceToHoodFunction.apply(lookaheadTurretToTargetDistance);
    if (lastTurretAngle == null) lastTurretAngle = currentTurretAngle;
    if (lastHoodAngle == null) lastHoodAngle = currentHoodAngle;
    turretVelocity =
        RadiansPerSecond.of(
            turretAngleFilter.calculate(
                currentTurretAngle.minus(lastTurretAngle).getRadians()
                    / GompeiLib.getLoopPeriod()));
    hoodVelocity =
        RadiansPerSecond.of(
            hoodAngleFilter.calculate(
                (currentHoodAngle.minus(lastHoodAngle)).getRadians() / GompeiLib.getLoopPeriod()));
    lastTurretAngle = currentTurretAngle;
    lastHoodAngle = currentHoodAngle;
    latestShotParameters =
        new ShotParameters(
            true,
            currentTurretAngle,
            currentHoodAngle,
            turretVelocity,
            hoodVelocity,
            distanceToFlywheelFunction.apply(lookaheadTurretToTargetDistance));

    return latestShotParameters;
  }

  public static void clearShotParameters() {
    latestShotParameters = null;
  }

  public record ShotParameters(
      boolean isValid,
      Rotation2d turretAngle,
      Rotation2d hoodAngle,
      AngularVelocity turretVelocity,
      AngularVelocity hoodVelocity,
      AngularVelocity flywheelSpeed) {}
}
