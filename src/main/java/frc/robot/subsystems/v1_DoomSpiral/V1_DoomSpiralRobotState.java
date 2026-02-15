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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.NTPrefixes;
import java.util.HashSet;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotState {
  private static final AprilTagFieldLayout fieldLayout;
  private static final Localization localization;

  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  @Getter private static final ShooterOffsets shooterOffsets;
  @Getter private static final IntakeOffsets intakeOffsets;
  @Getter private static final SpindexerOffsets spindexerOffsets;

  @Getter private static final InterpolatingTreeMap<Distance, Rotation2d> hoodAngleTree;
  @Getter private static final InterpolatingTreeMap<Distance, AngularVelocity> flywheelSpeedTree;

  @Getter private static Distance distanceToHub;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Score Angle")
  @Getter
  private static Rotation2d scoreAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Feed Angle")
  @Getter
  private static final Rotation2d feedAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Shooter/Feed Velocity")
  @Getter
  private static final double feedVelocity;

  private static final FieldZone globalZone;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    globalZone = new FieldZone(new HashSet<>(fieldLayout.getTags()));

    localization =
        new Localization(
            List.of(globalZone),
            V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);

    scoreAngle = new Rotation2d();

    feedAngle = new Rotation2d();
    feedVelocity = 0;

    shooterOffsets = new ShooterOffsets(0, new Rotation2d(0));
    intakeOffsets = new IntakeOffsets(new Rotation2d(), new Rotation2d(), new Rotation2d(), 0);
    spindexerOffsets = new SpindexerOffsets(0, 0, 0);
    hoodAngleTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            Rotation2d::interpolate);
    flywheelSpeedTree =
        new InterpolatingTreeMap<>(
            (start, end, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(start.in(Meters), end.in(Meters), q.in(Meters)),
            (start, end, t) ->
                AngularVelocity.ofBaseUnits(
                    Interpolator.forDouble()
                        .interpolate(start.in(RadiansPerSecond), end.in(RadiansPerSecond), t),
                    RadiansPerSecond));
    hoodAngleTree.put(Meters.of(1.34), Rotation2d.fromDegrees(19.0));
    hoodAngleTree.put(Meters.of(1.78), Rotation2d.fromDegrees(19.0));
    hoodAngleTree.put(Meters.of(2.17), Rotation2d.fromDegrees(24.0));
    hoodAngleTree.put(Meters.of(2.81), Rotation2d.fromDegrees(27.0));
    hoodAngleTree.put(Meters.of(3.82), Rotation2d.fromDegrees(29.0));
    hoodAngleTree.put(Meters.of(4.09), Rotation2d.fromDegrees(30.0));
    hoodAngleTree.put(Meters.of(4.40), Rotation2d.fromDegrees(31.0));
    hoodAngleTree.put(Meters.of(4.77), Rotation2d.fromDegrees(32.0));
    hoodAngleTree.put(Meters.of(5.57), Rotation2d.fromDegrees(32.0));
    hoodAngleTree.put(Meters.of(5.60), Rotation2d.fromDegrees(35.0));

    flywheelSpeedTree.put(Meters.of(1.34), RadiansPerSecond.of(210.0));
    flywheelSpeedTree.put(Meters.of(1.78), RadiansPerSecond.of(220.0));
    flywheelSpeedTree.put(Meters.of(2.17), RadiansPerSecond.of(220.0));
    flywheelSpeedTree.put(Meters.of(2.81), RadiansPerSecond.of(230.0));
    flywheelSpeedTree.put(Meters.of(3.82), RadiansPerSecond.of(250.0));
    flywheelSpeedTree.put(Meters.of(4.09), RadiansPerSecond.of(255.0));
    flywheelSpeedTree.put(Meters.of(4.40), RadiansPerSecond.of(260.0));
    flywheelSpeedTree.put(Meters.of(4.77), RadiansPerSecond.of(265.0));
    flywheelSpeedTree.put(Meters.of(5.57), RadiansPerSecond.of(275.0));
    flywheelSpeedTree.put(Meters.of(5.60), RadiansPerSecond.of(290.0));

    distanceToHub =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);
  }

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions) {
    InternalLoggedTracer.reset();
    V1_DoomSpiralRobotState.robotHeading = robotHeading;
    V1_DoomSpiralRobotState.modulePositions = modulePositions;
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    localization.addOdometryObservation(Timer.getTimestamp(), robotHeading, modulePositions);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "/GlobalPose", getGlobalPose());

    distanceToHub =
        Distance.ofBaseUnits(
            getGlobalPose()
                .getTranslation()
                .minus(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()))
                .getNorm(),
            Meters);
    Logger.recordOutput(NTPrefixes.POSE_DATA + "/Distance To Hub", distanceToHub);

    scoreAngle = hoodAngleTree.get(distanceToHub);
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
}
