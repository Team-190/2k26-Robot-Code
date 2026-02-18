package frc.robot.subsystems.v1_DoomSpiral;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.NTPrefixes;
import java.util.HashSet;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotState {
  private static final AprilTagFieldLayout fieldLayout;

  private static final FieldZone globalZone;

  private static final Localization localization;

  @Getter private static Distance distanceToHub;
  @Getter private static Distance distanceToFeedTranslation;

  private static final InterpolatingTreeMap<Distance, Rotation2d> shootAngleTree;
  private static final InterpolatingTreeMap<Distance, AngularVelocity> shootSpeedTree;
  private static final InterpolatingTreeMap<Distance, Rotation2d> feedAngleTree;
  private static final InterpolatingTreeMap<Distance, AngularVelocity> feedSpeedTree;

  @Getter private static Rotation2d scoreAngle;
  @Getter private static double scoreVelocity;
  @Getter private static Rotation2d feedAngle;
  @Getter private static double feedVelocity;

  @Getter private static final ShooterOffsets shooterOffsets;
  @Getter private static final IntakeOffsets intakeOffsets;
  @Getter private static final SpindexerOffsets spindexerOffsets;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    globalZone = new FieldZone(new HashSet<>(fieldLayout.getTags()));

    localization =
        new Localization(
            List.of(globalZone),
            V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);

    distanceToHub =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);

    distanceToFeedTranslation =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.FEED_TRANSLATION))
                .getNorm(),
            Meters);

    shootAngleTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            Rotation2d::interpolate);
    shootSpeedTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                AngularVelocity.ofBaseUnits(
                    Interpolator.forDouble()
                        .interpolate(start.in(RadiansPerSecond), end.in(RadiansPerSecond), t),
                    RadiansPerSecond));
    feedAngleTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            Rotation2d::interpolate);
    feedSpeedTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                AngularVelocity.ofBaseUnits(
                    Interpolator.forDouble()
                        .interpolate(start.in(RadiansPerSecond), end.in(RadiansPerSecond), t),
                    RadiansPerSecond));

    shootAngleTree.put(Meters.of(1.34), Rotation2d.fromDegrees(5.0));
    shootAngleTree.put(Meters.of(1.78), Rotation2d.fromDegrees(7.0));
    shootAngleTree.put(Meters.of(2.17), Rotation2d.fromDegrees(7.0));
    shootAngleTree.put(Meters.of(2.81), Rotation2d.fromDegrees(9.0));
    shootAngleTree.put(Meters.of(3.82), Rotation2d.fromDegrees(10.0));
    shootAngleTree.put(Meters.of(4.09), Rotation2d.fromDegrees(13.0));
    shootAngleTree.put(Meters.of(4.40), Rotation2d.fromDegrees(14.0));
    shootAngleTree.put(Meters.of(4.77), Rotation2d.fromDegrees(16.0));
    shootAngleTree.put(Meters.of(5.57), Rotation2d.fromDegrees(17.0));
    shootAngleTree.put(Meters.of(5.60), Rotation2d.fromDegrees(20.0));

    shootSpeedTree.put(Meters.of(1.34), RadiansPerSecond.of(210.0));
    shootSpeedTree.put(Meters.of(1.78), RadiansPerSecond.of(220.0));
    shootSpeedTree.put(Meters.of(2.17), RadiansPerSecond.of(220.0));
    shootSpeedTree.put(Meters.of(2.81), RadiansPerSecond.of(230.0));
    shootSpeedTree.put(Meters.of(3.82), RadiansPerSecond.of(250.0));
    shootSpeedTree.put(Meters.of(4.09), RadiansPerSecond.of(255.0));
    shootSpeedTree.put(Meters.of(4.40), RadiansPerSecond.of(260.0));
    shootSpeedTree.put(Meters.of(4.77), RadiansPerSecond.of(265.0));
    shootSpeedTree.put(Meters.of(5.57), RadiansPerSecond.of(275.0));
    shootSpeedTree.put(Meters.of(5.60), RadiansPerSecond.of(290.0));

    feedAngleTree.put(
        Meters.of(0.0),
        Rotation2d.fromDegrees(V1_DoomSpiralShooterConstants.HOOD_CONSTANTS.maxAngle.getDegrees()));
    // feedHoodAngleTree.put(Meters.of(1.78), Rotation2d.fromDegrees(7.0));
    // feedHoodAngleTree.put(Meters.of(2.17), Rotation2d.fromDegrees(7.0));
    // feedHoodAngleTree.put(Meters.of(2.81), Rotation2d.fromDegrees(9.0));
    // feedHoodAngleTree.put(Meters.of(3.82), Rotation2d.fromDegrees(10.0));
    // feedHoodAngleTree.put(Meters.of(4.09), Rotation2d.fromDegrees(13.0));
    // feedHoodAngleTree.put(Meters.of(4.40), Rotation2d.fromDegrees(14.0));
    // feedHoodAngleTree.put(Meters.of(4.77), Rotation2d.fromDegrees(16.0));
    // feedHoodAngleTree.put(Meters.of(5.57), Rotation2d.fromDegrees(17.0));
    // feedHoodAngleTree.put(Meters.of(5.60), Rotation2d.fromDegrees(20.0));

    feedSpeedTree.put(Meters.of(0.0), RadiansPerSecond.of(400.0));
    // feedFlywheelSpeedTree.put(Meters.of(1.78), RadiansPerSecond.of(220.0));
    // feedFlywheelSpeedTree.put(Meters.of(2.17), RadiansPerSecond.of(220.0));
    // feedFlywheelSpeedTree.put(Meters.of(2.81), RadiansPerSecond.of(230.0));
    // feedFlywheelSpeedTree.put(Meters.of(3.82), RadiansPerSecond.of(250.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.09), RadiansPerSecond.of(255.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.40), RadiansPerSecond.of(260.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.77), RadiansPerSecond.of(265.0));
    // feedFlywheelSpeedTree.put(Meters.of(5.57), RadiansPerSecond.of(275.0));
    // feedFlywheelSpeedTree.put(Meters.of(5.60), RadiansPerSecond.of(290.0));

    scoreAngle = new Rotation2d();
    scoreVelocity = 0;
    feedAngle = new Rotation2d();
    feedVelocity = 0;

    shooterOffsets = new ShooterOffsets(0, new Rotation2d(0));
    intakeOffsets = new IntakeOffsets(new Rotation2d(), new Rotation2d(), new Rotation2d(), 0);
    spindexerOffsets = new SpindexerOffsets(0, 0, 0);
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions) {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Global Pose", getGlobalPose());

    distanceToHub =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);

    distanceToFeedTranslation =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.FEED_TRANSLATION))
                .getNorm(),
            Meters);

    scoreAngle = shootAngleTree.get(distanceToHub);
    scoreVelocity = shootSpeedTree.get(distanceToHub).in(RadiansPerSecond);
    feedAngle = feedAngleTree.get(distanceToFeedTranslation);
    feedVelocity = feedSpeedTree.get(distanceToFeedTranslation).in(RadiansPerSecond);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Hub", distanceToHub);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Hood/Score Angle", scoreAngle);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Hood/Feed Angle", feedAngle);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Shooter/Feed Velocity", feedVelocity);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Shooter/Score Velocity", scoreVelocity);
  }

  public static void addFieldLocalizerVisionMeasurement(List<VisionPoseObservation> observations) {
    localization.addPoseObservations(observations);
  }

  public static void resetPose(Pose2d pose) {
    localization.resetPose(pose);
  }

  public static Rotation2d getHeading() {
    return localization.getHeading();
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(globalZone);
  }

  @Data
  public static class SpindexerOffsets {
    private double spindexer;
    private double feeder;
    private double kicker;

    public SpindexerOffsets(double spindexer, double feeder, double kicker) {
      this.spindexer = spindexer;
      this.feeder = feeder;
      this.kicker = kicker;
    }
  }

  @Data
  public static class ShooterOffsets {
    private double flywheel;
    private Rotation2d hood;

    public ShooterOffsets(double flywheel, Rotation2d hood) {
      this.flywheel = flywheel;
      this.hood = hood;
    }
  }

  @Data
  @AllArgsConstructor
  public static class IntakeOffsets {
    private Rotation2d stowOffset;
    private Rotation2d bumpOffset;
    private Rotation2d collectOffset;
    private double rollerVoltsOffset;
  }

  public record FixedShotParameters(
      Rotation2d robotAngle, Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {}

  @RequiredArgsConstructor
  public enum FixedShots {
    LEFT_TRENCH(
        new FixedShotParameters(
            Rotation2d.fromDegrees(350.0),
            V1_DoomSpiralShooterConstants.TRENCH_SHOT_HOOD_ANGLE,
            V1_DoomSpiralShooterConstants.TRENCH_SHOT_FLYWHEEL_SPEED)),
    RIGHT_TRENCH(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-170.0),
            V1_DoomSpiralShooterConstants.TRENCH_SHOT_HOOD_ANGLE,
            V1_DoomSpiralShooterConstants.TRENCH_SHOT_FLYWHEEL_SPEED)),
    HUB(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-90.0),
            V1_DoomSpiralShooterConstants.HUB_SHOT_HOOD_ANGLE,
            V1_DoomSpiralShooterConstants.HUB_SHOT_FLYWHEEL_SPEED)),
    TOWER(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-90.0),
            V1_DoomSpiralShooterConstants.TOWER_SHOT_HOOD_ANGLE,
            V1_DoomSpiralShooterConstants.TOWER_SHOT_FLYWHEEL_SPEED));

    @Getter private final FixedShotParameters parameters;
  }
}
