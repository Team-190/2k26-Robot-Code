package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubActivePeriod;
import frc.robot.util.NTPrefixes;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import lombok.*;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaRobotState {
  private static final AprilTagFieldLayout fieldLayout;

  private static final Field2d field;

  private static double robotYawVelocity;

  private static final FieldZone globalZone;
  private static final FieldZone blueHubZone;
  private static final FieldZone redHubZone;
  private static final FieldZone blueTowerZone;
  private static final FieldZone redTowerZone;

  private static final Localization localization;

  @Getter private static Distance distanceToHub;
  @Getter private static Distance distanceToFeedTranslation;

  private static final InterpolatingTreeMap<Distance, Rotation2d> shootAngleTree;
  private static final InterpolatingTreeMap<Distance, AngularVelocity> shootSpeedTree;
  private static final InterpolatingTreeMap<Distance, Rotation2d> feedAngleTree;
  private static final InterpolatingTreeMap<Distance, AngularVelocity> feedSpeedTree;

  @Getter private static Rotation2d robotToHubAngle;
  @Getter private static Rotation2d scoreAngle;
  @Getter private static AngularVelocity scoreVelocity;
  @Getter private static Rotation2d feedAngle;
  @Getter private static AngularVelocity feedVelocity;

  @Getter private static final LEDStates ledStates;

  @Setter @Getter private static long headingUpdateTimestamp;

  @Getter private static boolean prohibitShot;
  @Getter private static boolean shouldHoodTuck;

  static {
    fieldLayout = FieldConstants.tagLayoutType.getLayout();

    field = new Field2d();

    globalZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.globalTags));
    blueHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueHubTags));
    redHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redHubTags));
    blueTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueTowerTags));
    redTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redTowerTags));

    localization =
        new Localization(
            List.of(globalZone, blueHubZone, redHubZone, blueTowerZone, redTowerZone),
            V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);

    distanceToHub =
        Distance.ofBaseUnits(
            getHubZonePose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);

    distanceToFeedTranslation =
        Distance.ofBaseUnits(
            getGlobalPose().getTranslation().minus(getFeedTranslation()).getNorm(), Meters);

    ledStates = new LEDStates(false, false, false, false, false, false);

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

    shootAngleTree.put(Meters.of(1.1038981214244048), Rotation2d.fromDegrees(5.0));
    shootSpeedTree.put(Meters.of(1.1038981214244048), RadiansPerSecond.of(340.0));

    shootAngleTree.put(Meters.of(1.491203295344588), Rotation2d.fromDegrees(6.0));
    shootSpeedTree.put(Meters.of(1.491203295344588), RadiansPerSecond.of(345.0));

    shootAngleTree.put(Meters.of(1.735737657075872), Rotation2d.fromDegrees(16.0));
    shootSpeedTree.put(Meters.of(1.735737657075872), RadiansPerSecond.of(345.0));

    shootAngleTree.put(Meters.of(2.0049492041827994), Rotation2d.fromDegrees(17.0));
    shootSpeedTree.put(Meters.of(2.0049492041827994), RadiansPerSecond.of(350.0));

    shootAngleTree.put(Meters.of(2.30668187255375), Rotation2d.fromDegrees(18.0));
    shootSpeedTree.put(Meters.of(2.30668187255375), RadiansPerSecond.of(355.0));

    shootAngleTree.put(Meters.of(2.572028569812878), Rotation2d.fromDegrees(19.0));
    shootSpeedTree.put(Meters.of(2.572028569812878), RadiansPerSecond.of(375.0));

    shootAngleTree.put(Meters.of(2.9670118924057562), Rotation2d.fromDegrees(20.0));
    shootSpeedTree.put(Meters.of(2.9670118924057562), RadiansPerSecond.of(388.0));

    shootAngleTree.put(Meters.of(3.297585897171101), Rotation2d.fromDegrees(20.0));
    shootSpeedTree.put(Meters.of(3.297585897171101), RadiansPerSecond.of(405.0));

    shootAngleTree.put(Meters.of(3.622009715844515), Rotation2d.fromDegrees(20.0));
    shootSpeedTree.put(Meters.of(3.622009715844515), RadiansPerSecond.of(415.0));

    shootAngleTree.put(Meters.of(3.8605317055005743), Rotation2d.fromDegrees(20.0));
    shootSpeedTree.put(Meters.of(3.8605317055005743), RadiansPerSecond.of(430.0));

    shootAngleTree.put(Meters.of(4.120246303911951), Rotation2d.fromDegrees(20.0));
    shootSpeedTree.put(Meters.of(4.120246303911951), RadiansPerSecond.of(457.0));

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

    feedSpeedTree.put(
        Meters.of(0.0), RadiansPerSecond.of(Units.rotationsPerMinuteToRadiansPerSecond(6300)));
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
    scoreVelocity = RadiansPerSecond.of(0.0);
    feedAngle = new Rotation2d();
    feedVelocity = RadiansPerSecond.of(0.0);

    field.setRobotPose(getGlobalPose());
    SmartDashboard.putData("Field", field);

    headingUpdateTimestamp = NetworkTablesJNI.now();
  }

  @Trace
  public static void periodic(
      Rotation2d robotHeading,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions,
      Rotation2d turretRotation,
      boolean isTurretWrapping) {
    V2_DeltaRobotState.robotYawVelocity = robotYawVelocity;

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Pose2d hubPose = getHubZonePose();

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Global Pose", getGlobalPose());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Hub Zone Pose", hubPose);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Tower Zone Pose", getTowerZonePose());

    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    Pose2d shooterPosition =
        hubPose.transformBy(
            new Transform2d(
                V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getX(),
                V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getY(),
                turretRotation));

    distanceToHub =
        Distance.ofBaseUnits(
            shooterPosition.getTranslation().minus(hubTranslation).getNorm(), Meters);

    Translation2d feedTranslation = getFeedTranslation();

    distanceToFeedTranslation =
        Distance.ofBaseUnits(
            getGlobalPose().getTranslation().minus(feedTranslation).getNorm(), Meters);
    robotToHubAngle =
        hubTranslation
            .minus(shooterPosition.getTranslation())
            .getAngle()
            .minus(V1_DoomSpiralShooterConstants.SHOOTER_POSE.getRotation());

    scoreAngle = shootAngleTree.get(distanceToHub);
    scoreVelocity = shootSpeedTree.get(distanceToHub);
    feedAngle = feedAngleTree.get(distanceToFeedTranslation);
    feedVelocity = feedSpeedTree.get(distanceToFeedTranslation);

    field.setRobotPose(getGlobalPose());

    shouldHoodTuck = GeometryUtil.contains(FieldConstants.Zones.HOOD_TUCK_ZONES, getGlobalPose());
    prohibitShot =
        isTurretWrapping
            || shouldHoodTuck
            || GeometryUtil.contains(FieldConstants.Zones.PROHIBIT_LAUNCH_ZONES, getHubZonePose());

    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Feed Translation", new Pose2d(feedTranslation, Rotation2d.kZero));

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Hub", distanceToHub);
    Logger.recordOutput(
        NTPrefixes.POSE_DATA + "Rotation to Hub",
        new Pose2d(hubPose.getTranslation(), robotToHubAngle));
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Hood/Score Angle", scoreAngle);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Hood/Feed Angle", feedAngle);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Shooter/Feed Velocity", feedVelocity);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Shooter/Score Velocity", scoreVelocity);

    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Shift Period/Active", HubActivePeriod.isHubActive());
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Shift Period/Time Remaining",
        (int) HubActivePeriod.getShiftTimeRemaining());
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Shift Period/Current Shift", HubActivePeriod.getCurrentShift());
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Trench Zones/Left Blue Trench",
        GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.BLUE_TRENCH));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Trench Zones/Left Red Trench",
        GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.RED_TRENCH));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Trench Zones/Right Blue Trench",
        GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.BLUE_TRENCH));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Trench Zones/Right Red Trench",
        GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.RED_TRENCH));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Tower Zones/Blue Tower Zone",
        GeometryUtil.rectanglePose2ds(FieldConstants.Tower.BLUE_TOWER));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Tower Zones/Red Tower Zone",
        GeometryUtil.rectanglePose2ds(FieldConstants.Tower.RED_TOWER));
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "No Feed Zone",
        GeometryUtil.rectanglePose2ds(FieldConstants.Hub.FEED_KEEPOUT));
  }

  public static void addLocalizerVisionMeasurement(List<VisionPoseObservation> observations) {
    if (Math.abs(robotYawVelocity) <= Units.degreesToRadians(20.0))
      localization.addPoseObservations(observations);
  }

  public static Translation2d getFeedTranslation() {
    Translation2d feedTranslation;
    if (getGlobalPose().getY() >= FieldConstants.fieldWidth / 2) {
      if (AllianceFlipUtil.shouldFlip()) {
        feedTranslation = FieldConstants.Outpost.RED_FEED_TRANSLATION;
      } else {
        feedTranslation = FieldConstants.Depot.BLUE_FEED_TRANSLATION;
      }
    } else {
      if (AllianceFlipUtil.shouldFlip()) {
        feedTranslation = FieldConstants.Depot.RED_FEED_TRANSLATION;
      } else {
        feedTranslation = FieldConstants.Outpost.BLUE_FEED_TRANSLATION;
      }
    }
    return feedTranslation;
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

  public static Pose2d getHubZonePose() {
    Pose2d hubZonePose;

    // use alliance pose if alliance is known, else fallback to global pose

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        hubZonePose = localization.getEstimatedPose(redHubZone);
      } else {
        hubZonePose = localization.getEstimatedPose(blueHubZone);
      }
    } else {
      // fall back to global pose if no alliance
      hubZonePose = getGlobalPose();
    }
    return hubZonePose;
  }

  public static Pose2d getTowerZonePose() {
    Pose2d towerZonePose;

    // use alliance pose if alliance is known, else fallback to global pose

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        towerZonePose = localization.getEstimatedPose(redTowerZone);
      } else {
        towerZonePose = localization.getEstimatedPose(blueTowerZone);
      }
    } else {
      // fall back to global pose if no alliance
      towerZonePose = getGlobalPose();
    }

    return towerZonePose;
  }

  public static void setAutoTrajectory(Pose2d... trajectory) {
    field.getObject("trajectory").setPoses(trajectory);
  }

  public static void setAutoTrajectory(AutoTrajectory... trajectories) {
    setAutoTrajectory(
        Arrays.stream(trajectories)
            .flatMap(traj -> Arrays.stream(traj.getRawTrajectory().getPoses()))
            .map(AllianceFlipUtil::apply)
            .toArray(Pose2d[]::new));
  }

  public static void setAutoTrajectory() {
    field.getObject("trajectory").setPoses();
  }

  public record FixedShotParameters(
      Rotation2d robotAngle, Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {}

  @RequiredArgsConstructor
  public enum FixedShots {
    LEFT_TRENCH(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-170.0 + 180),
            V2_DeltaShooterConstants.TRENCH_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.TRENCH_SHOT_FLYWHEEL_SPEED)),
    RIGHT_TRENCH(
        new FixedShotParameters(
            Rotation2d.fromDegrees(350.0 + 180),
            V2_DeltaShooterConstants.TRENCH_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.TRENCH_SHOT_FLYWHEEL_SPEED)),
    LEFT_CORNER( // TODO: Change to actual values.
        new FixedShotParameters(
            Rotation2d.fromDegrees(-170.0 + 180),
            V2_DeltaShooterConstants.CORNER_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.CORNER_SHOT_FLYWHEEL_SPEED)),
    RIGHT_CORNER( // TODO: Change to actual values.
        new FixedShotParameters(
            Rotation2d.fromDegrees(350.0 + 180),
            V2_DeltaShooterConstants.TRENCH_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.CORNER_SHOT_FLYWHEEL_SPEED)),
    HUB(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-90.0 + 180),
            V2_DeltaShooterConstants.HUB_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.HUB_SHOT_FLYWHEEL_SPEED)),
    TOWER(
        new FixedShotParameters(
            Rotation2d.fromDegrees(-90.0 + 180),
            V2_DeltaShooterConstants.TOWER_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.TOWER_SHOT_FLYWHEEL_SPEED));

    @Getter private final FixedShotParameters parameters;
  }

  @Data
  @AllArgsConstructor
  public static class LEDStates {
    boolean IntakeCollecting, IntakeIn, ShooterPrepping, ShooterShooting, Spitting, AutoClimbing;
  }
}
