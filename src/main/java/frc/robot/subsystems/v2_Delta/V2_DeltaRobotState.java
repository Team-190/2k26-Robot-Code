package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubActivePeriod;
import frc.robot.util.NTPrefixes;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import lombok.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(GeometryUtil.class)
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
  private static final InterpolatingTreeMap<Distance, Time> shootTimeOfFlightTree;
  private static final InterpolatingTreeMap<Distance, Rotation2d> feedAngleTree;
  private static final InterpolatingTreeMap<Distance, AngularVelocity> feedSpeedTree;
  private static final InterpolatingTreeMap<Distance, Time> feedTimeOfFlightTree;

  @Getter private static Rotation2d hoodAngle;
  @Getter private static Pose2d lookaheadPose;

  @Getter private static AngularVelocity turretVelocity;
  @Getter private static AngularVelocity flywheelVelocity;

  @Getter private static final LEDStates ledStates;

  @Setter @Getter private static long headingUpdateTimestamp;

  @Getter private static boolean prohibitShot;
  @Getter private static boolean shouldHoodTuck;
  @Getter private static boolean inAllianceZone;
  @Getter private static boolean intakeAtStow;

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
                .transformBy(V2_DeltaConstants.ROBOT_TO_SHOOTER_TRANSFORM_2D)
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

    shootTimeOfFlightTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                Time.ofBaseUnits(
                    Interpolator.forDouble().interpolate(start.in(Seconds), end.in(Seconds), t),
                    Seconds));
    feedTimeOfFlightTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                Time.ofBaseUnits(
                    Interpolator.forDouble().interpolate(start.in(Seconds), end.in(Seconds), t),
                    Seconds));

    shootAngleTree.put(Meters.of(1.074934), Rotation2d.fromRotations(0.0));
    shootAngleTree.put(Meters.of(1.490425), Rotation2d.fromRotations(0.0));
    shootAngleTree.put(Meters.of(1.988254), Rotation2d.fromRotations(0.008));
    shootAngleTree.put(Meters.of(2.481035), Rotation2d.fromRotations(0.012));
    shootAngleTree.put(Meters.of(3.002048), Rotation2d.fromRotations(0.018));
    shootAngleTree.put(Meters.of(3.519295), Rotation2d.fromRotations(0.034));
    shootAngleTree.put(Meters.of(3.972171), Rotation2d.fromRotations(0.040));
    shootAngleTree.put(Meters.of(4.551259), Rotation2d.fromRotations(0.045));
    shootAngleTree.put(Meters.of(4.894654), Rotation2d.fromRotations(0.05));
    shootAngleTree.put(Meters.of(5.336559), Rotation2d.fromRotations(0.06));

    shootSpeedTree.put(Meters.of(1.074934), RotationsPerSecond.of(22));
    shootSpeedTree.put(Meters.of(1.490425), RotationsPerSecond.of(24));
    shootSpeedTree.put(Meters.of(1.988254), RotationsPerSecond.of(26));
    shootSpeedTree.put(Meters.of(2.481035), RotationsPerSecond.of(27));
    shootSpeedTree.put(Meters.of(3.002048), RotationsPerSecond.of(28));
    shootSpeedTree.put(Meters.of(3.519295), RotationsPerSecond.of(29));
    shootSpeedTree.put(Meters.of(3.972171), RotationsPerSecond.of(30.5));
    shootSpeedTree.put(Meters.of(4.551259), RotationsPerSecond.of(33));
    shootSpeedTree.put(Meters.of(4.894654), RotationsPerSecond.of(35.5));
    shootSpeedTree.put(Meters.of(5.336559), RotationsPerSecond.of(37));

    feedAngleTree.put(Meters.of(2.834202), Rotation2d.fromDegrees(21.621094));
    feedSpeedTree.put(Meters.of(2.834202), RadiansPerSecond.of(109.297199));
    feedTimeOfFlightTree.put(Meters.of(2.834202), Seconds.of(.9));

    feedAngleTree.put(Meters.of(3.746701), Rotation2d.fromDegrees(21.621094));
    feedSpeedTree.put(Meters.of(3.746701), RadiansPerSecond.of(134.297199));
    feedTimeOfFlightTree.put(Meters.of(3.746701), Seconds.of(.95));

    feedAngleTree.put(Meters.of(4.776145), Rotation2d.fromDegrees(22.675781));
    feedSpeedTree.put(Meters.of(4.776145), RadiansPerSecond.of(161.830531));
    feedTimeOfFlightTree.put(Meters.of(4.776145), Seconds.of(1.2));

    feedAngleTree.put(Meters.of(5.605561), Rotation2d.fromDegrees(23.90625));
    feedSpeedTree.put(Meters.of(5.605561), RadiansPerSecond.of(184.328245));
    feedTimeOfFlightTree.put(Meters.of(5.605561), Seconds.of(1.3));

    feedAngleTree.put(Meters.of(6.658618), Rotation2d.fromDegrees(25.048828));
    feedSpeedTree.put(Meters.of(6.658618), RadiansPerSecond.of(227.428799));
    feedTimeOfFlightTree.put(Meters.of(6.658618), Seconds.of(1.45));

    feedAngleTree.put(Meters.of(7.82451), Rotation2d.fromDegrees(28.388672));
    feedSpeedTree.put(Meters.of(7.82451), RadiansPerSecond.of(252.807586));
    feedTimeOfFlightTree.put(Meters.of(7.82451), Seconds.of(1.5));

    // feedHoodAngleTree.put(Meters.of(1.78), Rotation2d.fromDegrees(7.0));
    // feedHoodAngleTree.put(Meters.of(2.17), Rotation2d.fromDegrees(7.0));
    // feedHoodAngleTree.put(Meters.of(2.81), Rotation2d.fromDegrees(9.0));
    // feedHoodAngleTree.put(Meters.of(3.82), Rotation2d.fromDegrees(10.0));
    // feedHoodAngleTree.put(Meters.of(4.09), Rotation2d.fromDegrees(13.0));
    // feedHoodAngleTree.put(Meters.of(4.40), Rotation2d.fromDegrees(14.0));
    // feedHoodAngleTree.put(Meters.of(4.77), Rotation2d.fromDegrees(16.0));
    // feedHoodAngleTree.put(Meters.of(5.57), Rotation2d.fromDegrees(17.0));
    // feedHoodAngleTree.put(Meters.of(5.60), Rotation2d.fromDegrees(20.0));

    // feedFlywheelSpeedTree.put(Meters.of(1.78), RadiansPerSecond.of(220.0));
    // feedFlywheelSpeedTree.put(Meters.of(2.17), RadiansPerSecond.of(220.0));
    // feedFlywheelSpeedTree.put(Meters.of(2.81), RadiansPerSecond.of(230.0));
    // feedFlywheelSpeedTree.put(Meters.of(3.82), RadiansPerSecond.of(250.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.09), RadiansPerSecond.of(255.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.40), RadiansPerSecond.of(260.0));
    // feedFlywheelSpeedTree.put(Meters.of(4.77), RadiansPerSecond.of(265.0));
    // feedFlywheelSpeedTree.put(Meters.of(5.57), RadiansPerSecond.of(275.0));
    // feedFlywheelSpeedTree.put(Meters.of(5.60), RadiansPerSecond.of(290.0));

    shootTimeOfFlightTree.put(Meters.of(1.525345), Seconds.of(4.381 / 4.0));
    shootTimeOfFlightTree.put(Meters.of(2.531303), Seconds.of(4.886 / 4.0));
    shootTimeOfFlightTree.put(Meters.of(3.518323), Seconds.of(4.899 / 4.0));

    lookaheadPose = new Pose2d();
    hoodAngle = new Rotation2d();
    flywheelVelocity = RadiansPerSecond.of(0.0);

    field.setRobotPose(getGlobalPose());
    SmartDashboard.putData("Field", field);

    shouldHoodTuck = false;
    prohibitShot = false;
    inAllianceZone = false;
    intakeAtStow = false;

    headingUpdateTimestamp = NetworkTablesJNI.now();
  }

  @Trace
  public static void periodic(
      Rotation2d robotHeading,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions,
      Rotation2d turretRotation,
      boolean isTurretWrapping,
      ChassisSpeeds robotVelocity,
      boolean intakeStowed) {
    V2_DeltaRobotState.robotYawVelocity = robotYawVelocity;

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Pose2d hubPose = getHubZonePose();

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Global Pose", getGlobalPose());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Hub Zone Pose", hubPose);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Tower Zone Pose", getTowerZonePose());

    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    Translation2d feedTranslation = getFeedTranslation();

    distanceToHub =
        Distance.ofBaseUnits(
            getHubZonePose()
                .transformBy(V2_DeltaConstants.ROBOT_TO_SHOOTER_TRANSFORM_2D)
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);

    distanceToFeedTranslation =
        Distance.ofBaseUnits(
            getGlobalPose().getTranslation().minus(feedTranslation).getNorm(), Meters);
    turretVelocity = RadiansPerSecond.zero();
    Rectangle2d allianceZone =
        new Rectangle2d(
            AllianceFlipUtil.apply(
                new Translation2d(
                    FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.fieldWidth)),
            AllianceFlipUtil.apply(new Translation2d(0, 0)));

    inAllianceZone = allianceZone.contains(getGlobalPose().getTranslation());

    V2_DeltaShotCalculator.ShotParameters shotParameters =
        V2_DeltaShotCalculator.getShotParameters(
            inAllianceZone ? hubPose : getGlobalPose(),
            inAllianceZone ? hubTranslation : feedTranslation,
            new Transform2d(
                V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getX(),
                V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getY(),
                turretRotation),
            robotVelocity,
            Seconds.of(0.03),
            d -> inAllianceZone ? shootTimeOfFlightTree.get(d) : feedTimeOfFlightTree.get(d),
            d -> inAllianceZone ? shootAngleTree.get(d) : feedAngleTree.get(d),
            d -> inAllianceZone ? shootSpeedTree.get(d) : feedSpeedTree.get(d));

    hoodAngle = shotParameters.hoodAngle();
    flywheelVelocity = shotParameters.flywheelSpeed();
    turretVelocity = shotParameters.turretVelocity();

    lookaheadPose = shotParameters.adjustedRobotPose();
    field.setRobotPose(getGlobalPose());

    shouldHoodTuck =
        GeometryUtil.intersects(
            FieldConstants.Zones.HOOD_TUCK_ZONES,
            getGlobalPose(),
            V2_DeltaConstants.DRIVE_CONFIG.bumperWidth(),
            V2_DeltaConstants.DRIVE_CONFIG.bumperLength());
    prohibitShot =
        isTurretWrapping
            || shouldHoodTuck
            || GeometryUtil.contains(FieldConstants.Zones.PROHIBIT_LAUNCH_ZONES, getGlobalPose())
            || !shotParameters.isValid();

    intakeAtStow = intakeStowed;

    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Feed Translation", new Pose2d(feedTranslation, Rotation2d.kZero));

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Hub", distanceToHub);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Feed", distanceToFeedTranslation);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Lookahead Pose", lookaheadPose);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Hood/Score Angle", hoodAngle);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Shooter/Score Velocity", flywheelVelocity);

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
    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Alliance Zone", GeometryUtil.rectanglePose2ds(allianceZone));
    Logger.recordOutput(
        NTPrefixes.SHOOTING_DATA + "Shot Parameters/Turret Velocity",
        shotParameters.turretVelocity());
    Logger.recordOutput(
        NTPrefixes.SHOOTING_DATA + "Shot Parameters/Valid", shotParameters.isValid());

    Logger.recordOutput(NTPrefixes.SHOOTING_DATA + "In Alliance Zone", inAllianceZone);
    Logger.recordOutput(NTPrefixes.SHOOTING_DATA + "Prohibit Shot", prohibitShot);
    Logger.recordOutput(NTPrefixes.SHOOTING_DATA + "Should Hood Tuck", shouldHoodTuck);

    Logger.recordOutput("Elastic/Drive/Slow Factor", DriveCommands.getSlowFactor());
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
      Translation2d robotPosition, Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {}

  @RequiredArgsConstructor
  public enum FixedShots {
    LEFT_TRENCH(
        new FixedShotParameters(
            V2_DeltaShooterConstants.LEFT_TRENCH_SHOT_LOCATION,
            V2_DeltaShooterConstants.LEFT_TRENCH_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.LEFT_TRENCH_SHOT_FLYWHEEL_SPEED)),
    RIGHT_TRENCH(
        new FixedShotParameters(
            V2_DeltaShooterConstants.RIGHT_TRENCH_SHOT_LOCATION,
            V2_DeltaShooterConstants.RIGHT_TRENCH_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.RIGHT_TRENCH_SHOT_FLYWHEEL_SPEED)),
    LEFT_CORNER(
        new FixedShotParameters(
            V2_DeltaShooterConstants.LEFT_CORNER_SHOT_LOCATION,
            V2_DeltaShooterConstants.LEFT_CORNER_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.LEFT_CORNER_SHOT_FLYWHEEL_SPEED)),
    RIGHT_CORNER(
        new FixedShotParameters(
            V2_DeltaShooterConstants.RIGHT_CORNER_SHOT_LOCATION,
            V2_DeltaShooterConstants.RIGHT_CORNER_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.RIGHT_CORNER_SHOT_FLYWHEEL_SPEED)),
    HUB(
        new FixedShotParameters(
            V2_DeltaShooterConstants.HUB_SHOT_LOCATION,
            V2_DeltaShooterConstants.HUB_SHOT_HOOD_ANGLE,
            V2_DeltaShooterConstants.HUB_SHOT_FLYWHEEL_SPEED)),
    TOWER(
        new FixedShotParameters(
            V2_DeltaShooterConstants.TOWER_SHOT_LOCATION,
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
