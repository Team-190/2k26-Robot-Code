package first.robot.v0_Funky;

import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import first.robot.shared.util.NTPrefixes;
import java.util.List;
import java.util.stream.Collectors;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.system.Timer;

public class V0_FunkyRobotState {
  private static final Field fieldLayout;
  private static final Localization localization;
  @Getter private static ChassisVelocities chassisSpeeds;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Score Angle")
  @Getter
  private static final Rotation2d scoreAngle;

  @AutoLogOutput(key = NTPrefixes.ROBOT_STATE + "Hood/Feed Angle")
  @Getter
  private static final Rotation2d feedAngle;

  private static final FieldZone globalZone;

  @Setter @Getter private static long networktablesTimestamp;
  @Setter @Getter private static boolean turretWrapping;

  static {
    fieldLayout = Field.loadField(Fields.FRC_2026_REBUILT_ANDY_MARK);

    globalZone = new FieldZone(fieldLayout.getTags().stream().collect(Collectors.toSet()));

    localization =
        new Localization(
            List.of(globalZone), V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.kinematics(), 2);
    scoreAngle = Rotation2d.ZERO;
    feedAngle = Rotation2d.ZERO;

    chassisSpeeds = new ChassisVelocities();

    networktablesTimestamp = NetworkTablesJNI.now();
  }

  public static void periodic(
      Rotation2d heading, SwerveModulePosition[] modulePositions, ChassisVelocities chassisSpeeds) {

    localization.addOdometryObservation(Timer.getTimestamp(), heading, modulePositions);
    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Global Pose", getGlobalPose());
    V0_FunkyRobotState.chassisSpeeds = chassisSpeeds;
  }

  public static void addFieldLocalizerVisionMeasurement(List<VisionPoseObservation> observations) {
    if (Math.abs(chassisSpeeds.omega) <= Units.degreesToRadians(120.0))
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
}
