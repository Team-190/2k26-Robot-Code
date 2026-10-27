package first.robot.v2_Turnover;

import static org.wpilib.units.Units.*;

import choreo.auto.AutoTrajectory;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import first.robot.FieldConstants;
import first.robot.shared.commands.DriveCommands;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.HubActivePeriod;
import first.robot.shared.util.NTPrefixes;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooterConstants;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShotCalculator;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import lombok.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.*;
import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.math.interpolation.Interpolator;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

@ExtensionMethod(GeometryUtil.class)
public class V2_TurnoverRobotState {
  private static final Field fieldLayout;

  private static Pose2d[] autoTrajectory = new Pose2d[0];

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
  @Getter private static Pose2d shootLookaheadPose;
  @Getter private static Pose2d hoodTuckLookaheadPose;

  @Getter private static AngularVelocity turretVelocity;
  @Getter private static AngularVelocity flywheelVelocity;

  @Getter private static final LEDStates ledStates;

  @Setter @Getter private static long headingUpdateTimestamp;

  @Getter private static boolean prohibitShot;
  @Getter private static boolean shouldHoodTuck;
  @Getter private static boolean inAllianceZone;
  @Getter private static boolean intakeAtStow;

  private static final AngularVelocity
      UMAMI; // Colleen made me put this in, it represents a 15 radian/second offset

  static {
    fieldLayout = FieldConstants.tagLayoutType.getLayout();

    globalZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.globalTags));
    blueHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueHubTags));
    redHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redHubTags));
    blueTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueTowerTags));
    redTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redTowerTags));

    localization =
        new Localization(
            List.of(globalZone, blueHubZone, redHubZone, blueTowerZone, redTowerZone),
            V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);

    distanceToHub =
        Distance.ofBaseUnits(
            getHubZonePose()
                .transformBy(V2_TurnoverConstants.ROBOT_TO_SHOOTER_TRANSFORM_2D)
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

    shootAngleTree.put(Meters.of(1.216412), Rotation2d.fromRotations(0.0));
    shootAngleTree.put(Meters.of(1.516677), Rotation2d.fromRotations(0.01));
    shootAngleTree.put(Meters.of(2.012264), Rotation2d.fromRotations(0.02));
    shootAngleTree.put(Meters.of(2.522274), Rotation2d.fromRotations(0.03));
    shootAngleTree.put(Meters.of(2.984916), Rotation2d.fromRotations(0.04));
    shootAngleTree.put(Meters.of(3.582623), Rotation2d.fromRotations(0.05));
    shootAngleTree.put(Meters.of(4.034540), Rotation2d.fromRotations(0.06));
    shootAngleTree.put(Meters.of(4.497936), Rotation2d.fromRotations(0.07));
    shootAngleTree.put(Meters.of(5.015775), Rotation2d.fromRotations(0.07));
    shootAngleTree.put(Meters.of(5.336559), Rotation2d.fromRotations(0.07));

    shootSpeedTree.put(Meters.of(1.216412), RotationsPerSecond.of(23));
    shootSpeedTree.put(Meters.of(1.516677), RotationsPerSecond.of(23));
    shootSpeedTree.put(Meters.of(2.012264), RotationsPerSecond.of(23.5));
    shootSpeedTree.put(Meters.of(2.522274), RotationsPerSecond.of(24.5));
    shootSpeedTree.put(Meters.of(2.984916), RotationsPerSecond.of(26));
    shootSpeedTree.put(Meters.of(3.582623), RotationsPerSecond.of(28));
    shootSpeedTree.put(Meters.of(4.034540), RotationsPerSecond.of(29.5));
    shootSpeedTree.put(Meters.of(4.497936), RotationsPerSecond.of(31));
    shootSpeedTree.put(Meters.of(5.015775), RotationsPerSecond.of(33.5));
    shootSpeedTree.put(Meters.of(5.336559), RotationsPerSecond.of(35));

    shootTimeOfFlightTree.put(Meters.of(1.239737), Seconds.of(1.021));
    shootTimeOfFlightTree.put(Meters.of(2.006357), Seconds.of(1.040));
    shootTimeOfFlightTree.put(Meters.of(3.015107), Seconds.of(1.060));
    shootTimeOfFlightTree.put(Meters.of(4.003837), Seconds.of(1.072));
    shootTimeOfFlightTree.put(Meters.of(4.951601), Seconds.of(1.110));
    shootTimeOfFlightTree.put(Meters.of(5.201445), Seconds.of(1.181));

    shootTimeOfFlightTree.put(Meters.of(0.0), Seconds.of(0.0));

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

    feedAngleTree.put(Meters.of(11.275), Rotation2d.fromDegrees(26.156623));
    feedSpeedTree.put(Meter.of(11.275), RadiansPerSecond.of(377.807586));
    feedTimeOfFlightTree.put(Meters.of(11.275), Seconds.of(1.85));

    feedAngleTree.put(Meters.of(13.170), Rotation2d.fromDegrees(26.78));
    feedSpeedTree.put(Meter.of(13.170), RadiansPerSecond.of(457));
    feedTimeOfFlightTree.put(Meters.of(13.170), Seconds.of(2.8));

    shootLookaheadPose = new Pose2d();
    hoodTuckLookaheadPose = new Pose2d();

    hoodAngle = new Rotation2d();
    flywheelVelocity = RadiansPerSecond.of(0.0);

    shouldHoodTuck = false;
    prohibitShot = false;
    inAllianceZone = false;
    intakeAtStow = false;

    headingUpdateTimestamp = NetworkTablesJNI.now();

    UMAMI = RadiansPerSecond.of(5.0);
  }

  @Trace
  public static void periodic(
      Rotation2d robotHeading,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions,
      Rotation2d turretRotation,
      boolean isTurretWrapping,
      ChassisVelocities robotVelocity,
      boolean intakeStowed,
      boolean holding) {
    V2_TurnoverRobotState.robotYawVelocity = robotYawVelocity;

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Pose2d hubPose = getHubZonePose();

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Global Pose", getGlobalPose());
    Logger.recordOutput(
        NTPrefixes.POSE_DATA + "Hood Tuck Lookahead Pose", getHoodTuckLookaheadPose());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Hub Zone Pose", hubPose);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Tower Zone Pose", getTowerZonePose());

    Translation2d hubTranslation =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    Translation2d feedTranslation = getFeedTranslation();

    distanceToHub =
        Distance.ofBaseUnits(
            getHubZonePose()
                .transformBy(V2_TurnoverConstants.ROBOT_TO_SHOOTER_TRANSFORM_2D)
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
                    FieldConstants.LinesVertical.neutralZoneNear,
                    FieldConstants.fieldWidth + Units.inchesToMeters(15))),
            AllianceFlipUtil.apply(new Translation2d(0, -Units.inchesToMeters(-15))));

    inAllianceZone = allianceZone.contains(getGlobalPose().getTranslation());

    Translation2d targetPose = inAllianceZone ? hubTranslation : feedTranslation;

    V2_TurnoverShotCalculator.ShotParameters shotParameters =
        V2_TurnoverShotCalculator.getShotParameters(
            inAllianceZone ? hubPose : getGlobalPose(),
            targetPose,
            new Transform2d(
                V2_TurnoverShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getX(),
                V2_TurnoverShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getY(),
                turretRotation),
            robotVelocity,
            Seconds.of(0.03),
            d -> inAllianceZone ? shootTimeOfFlightTree.get(d) : feedTimeOfFlightTree.get(d),
            d -> inAllianceZone ? shootAngleTree.get(d) : feedAngleTree.get(d),
            d -> inAllianceZone ? shootSpeedTree.get(d) : feedSpeedTree.get(d));

    hoodAngle = shotParameters.hoodAngle();
    flywheelVelocity = shotParameters.flywheelSpeed().plus(UMAMI);
    turretVelocity = shotParameters.turretVelocity();

    shootLookaheadPose = shotParameters.adjustedRobotPose();
    hoodTuckLookaheadPose =
        getGlobalPose()
            .plus(
                new Transform2d(
                    // robotVelocity is robot-relative (drive.getMeasuredChassisVelocities()), so
                    // .plus() rotates it into the field frame
                    robotVelocity.vx * V2_TurnoverShooterConstants.HOOD_TUCK_TIME,
                    robotVelocity.vy * V2_TurnoverShooterConstants.HOOD_TUCK_TIME,
                    new Rotation2d()));
    Logger.recordOutput("Field/Robot", getGlobalPose());
    Logger.recordOutput("Field/trajectory", autoTrajectory);

    shouldHoodTuck =
        intersectsAlongPath(
            FieldConstants.Zones.HOOD_TUCK_ZONES,
            getGlobalPose(),
            getHoodTuckLookaheadPose(),
            V2_TurnoverConstants.DRIVE_CONFIG.bumperWidth(),
            V2_TurnoverConstants.DRIVE_CONFIG.bumperLength());

    prohibitShot =
        isTurretWrapping
            || shouldHoodTuck
            || GeometryUtil.contains(FieldConstants.Zones.PROHIBIT_LAUNCH_ZONES, getGlobalPose())
            || !shotParameters.isValid();

    intakeAtStow = intakeStowed;

    ledStates.setShooterShooting(!prohibitShot && inAllianceZone && !holding);
    ledStates.setShooterFeeding(!prohibitShot && !inAllianceZone && !holding);

    Logger.recordOutput(
        NTPrefixes.ROBOT_STATE + "Feed Translation", new Pose2d(feedTranslation, Rotation2d.ZERO));

    Logger.recordOutput(
        NTPrefixes.POSE_DATA + "Aiming Location", new Pose2d(targetPose, Rotation2d.ZERO));

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Hub", distanceToHub);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Distance To Feed", distanceToFeedTranslation);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Lookahead Pose", shootLookaheadPose);
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
    if (Math.abs(robotYawVelocity) <= Units.degreesToRadians(10.0))
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

    Optional<Alliance> alliance = MatchState.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.RED) {
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

    Optional<Alliance> alliance = MatchState.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.RED) {
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
    autoTrajectory = trajectory;
  }

  public static void setAutoTrajectory(AutoTrajectory... trajectories) {
    setAutoTrajectory(
        Arrays.stream(trajectories)
            .flatMap(traj -> Arrays.stream(traj.getRawTrajectory().getPoses()))
            .map(AllianceFlipUtil::apply)
            .toArray(Pose2d[]::new));
  }

  public static void setAutoTrajectory() {
    autoTrajectory = new Pose2d[0];
  }

  public record FixedShotParameters(
      Translation2d robotPosition, Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {}

  @RequiredArgsConstructor
  public enum FixedShots {
    LEFT_TRENCH(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.LEFT_TRENCH_SHOT_LOCATION,
            V2_TurnoverShooterConstants.LEFT_TRENCH_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.LEFT_TRENCH_SHOT_FLYWHEEL_SPEED)),
    RIGHT_TRENCH(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.RIGHT_TRENCH_SHOT_LOCATION,
            V2_TurnoverShooterConstants.RIGHT_TRENCH_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.RIGHT_TRENCH_SHOT_FLYWHEEL_SPEED)),
    LEFT_CORNER(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.LEFT_CORNER_SHOT_LOCATION,
            V2_TurnoverShooterConstants.LEFT_CORNER_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.LEFT_CORNER_SHOT_FLYWHEEL_SPEED)),
    RIGHT_CORNER(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.RIGHT_CORNER_SHOT_LOCATION,
            V2_TurnoverShooterConstants.RIGHT_CORNER_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.RIGHT_CORNER_SHOT_FLYWHEEL_SPEED)),
    HUB(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.HUB_SHOT_LOCATION,
            V2_TurnoverShooterConstants.HUB_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.HUB_SHOT_FLYWHEEL_SPEED)),
    TOWER(
        new FixedShotParameters(
            V2_TurnoverShooterConstants.TOWER_SHOT_LOCATION,
            V2_TurnoverShooterConstants.TOWER_SHOT_HOOD_ANGLE,
            V2_TurnoverShooterConstants.TOWER_SHOT_FLYWHEEL_SPEED));

    @Getter private final FixedShotParameters parameters;
  }

  @Data
  @AllArgsConstructor
  public static class LEDStates {
    boolean IntakeCollecting,
        IntakeIn,
        ShooterFeeding,
        ShooterShooting,
        IntakeSlowRolling,
        AutoClimbing;
  }

  /**
   * Checks whether the robot's footprint at either pose, or the straight path between the two
   * poses, touches any rectangle. Catches fast motion that would skip over a zone between loops.
   *
   * @param rectangle2ds Array of rectangles to check against
   * @param robotPose The current robot pose
   * @param lookaheadPose The predicted future robot pose
   * @param bumperWidth Full width of the robot footprint
   * @param bumperLength Full length of the robot footprint
   * @return Whether the robot touches any rectangle now, at the lookahead, or in between
   */
  private static boolean intersectsAlongPath(
      Rectangle2d[] rectangle2ds,
      Pose2d robotPose,
      Pose2d lookaheadPose,
      double bumperWidth,
      double bumperLength) {
    if (GeometryUtil.intersects(rectangle2ds, robotPose, bumperWidth, bumperLength)
        || GeometryUtil.intersects(rectangle2ds, lookaheadPose, bumperWidth, bumperLength)) {
      return true;
    }

    Translation2d pathStart = robotPose.getTranslation();
    Translation2d pathVector =
        lookaheadPose.getTranslation().minus(pathStart); // robot -> lookahead

    for (Rectangle2d zone : rectangle2ds) {
      Pose2d[] zonePoses = GeometryUtil.rectanglePose2ds(zone);
      for (int cornerIndex = 0; cornerIndex < 4; cornerIndex++) {
        Translation2d wallStart = zonePoses[cornerIndex].getTranslation();
        Translation2d wallEnd = zonePoses[(cornerIndex + 1) % 4].getTranslation();
        Translation2d wallVector = wallEnd.minus(wallStart); // corner -> next corner

        double crossOfDirections =
            pathVector.getX() * wallVector.getY() - pathVector.getY() * wallVector.getX();
        if (crossOfDirections == 0) {
          continue; // path is parallel to this wall, can't cross it
        }

        Translation2d pathStartToWallStart = wallStart.minus(pathStart);

        // 0 = at the robot, 1 = at the lookahead
        double fractionAlongPath =
            (pathStartToWallStart.getX() * wallVector.getY()
                    - pathStartToWallStart.getY() * wallVector.getX())
                / crossOfDirections;

        // 0 = at wallStart, 1 = at wallEnd
        double fractionAlongWall =
            (pathStartToWallStart.getX() * pathVector.getY()
                    - pathStartToWallStart.getY() * pathVector.getX())
                / crossOfDirections;

        boolean crossingIsOnPath = fractionAlongPath >= 0 && fractionAlongPath <= 1;
        boolean crossingIsOnWall = fractionAlongWall >= 0 && fractionAlongWall <= 1;

        if (crossingIsOnPath && crossingIsOnWall) {
          return true;
        }
      }
    }
    return false;
  }
}
