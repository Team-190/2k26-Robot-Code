package frc.robot.commands.shared;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.FieldConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NTPrefixes;
import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AdjustPathCommand extends Command {

  private final Supplier<Pose2d> currentPoseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final double goalEndVelocity;
  private final LinearConstraints linearConstraints;
  private final AngularPositionConstraints angularVelocityConstraints;
  private final SwerveDrive drive;
  private Command followCommand;
  private final BooleanSupplier useTrench;
  private final BooleanSupplier useBump;

  /**
   * Creates a command that generates and follows a corrective path to a target pose using
   * PathPlanner.
   *
   * @param currentPoseSupplier
   * @param targetPoseSupplier
   * @param translationGains
   * @param rotationGains
   * @param driveConstraints
   * @param maxAccelerationMPS2
   * @param maxAngularAccelerationRPS2
   * @param goalEndVelocity
   * @param drive
   */
  public AdjustPathCommand(
      Supplier<Pose2d> currentPoseSupplier,
      Supplier<Pose2d> targetPoseSupplier,
      Gains translationGains,
      Gains rotationGains,
      double goalEndVelocity,
      LinearConstraints linearConstraints,
      AngularPositionConstraints angularVelocityConstraints,
      BooleanSupplier useTrench,
      BooleanSupplier useBump,
      SwerveDrive drive) {

    this.currentPoseSupplier = currentPoseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.linearConstraints = linearConstraints;
    this.angularVelocityConstraints = angularVelocityConstraints;
    this.goalEndVelocity = goalEndVelocity;
    this.drive = drive;
    this.useTrench = useTrench;
    this.useBump = useBump;

    try {
      AutoBuilder.configure(
          currentPoseSupplier,
          pose -> {},
          drive::getMeasuredChassisSpeeds,
          (speeds, feedforwards) -> drive.runVelocity(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(
                  translationGains.getKP(), translationGains.getKI(), translationGains.getKD()),
              new PIDConstants(
                  rotationGains.getKP(), rotationGains.getKI(), rotationGains.getKD())),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drive);
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to load PathPlanner robot config", e);
    }
  }

  @Override
  public void initialize() {
    Pose2d intermediatePose;

    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d currentPose = currentPoseSupplier.get();

    List<Waypoint> waypoints;

    PathConstraints constraints =
        new PathConstraints(
            linearConstraints.maxVelocity().getRawValue(),
            Double.POSITIVE_INFINITY,
            angularVelocityConstraints.maxVelocity().getRawValue(),
            Double.POSITIVE_INFINITY);

    if (useTrench.getAsBoolean()) {
      Pose2d right = AllianceFlipUtil.apply(FieldConstants.RightTrench.BLUE_TRENCH.getCenter());
      Pose2d left = AllianceFlipUtil.apply(FieldConstants.LeftTrench.BLUE_TRENCH.getCenter());

      intermediatePose =
          right
                      .getTranslation()
                      .getDistance(V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
                  < left.getTranslation()
                      .getDistance(V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
              ? right
              : left;

      Logger.recordOutput(NTPrefixes.POSE_DATA + "Intermediate Pose", intermediatePose);

      followCommand =
          AutoBuilder.pathfindToPose(intermediatePose, constraints)
              .andThen(AutoBuilder.pathfindToPose(targetPose, constraints));

    } else if (useBump.getAsBoolean()) {
      Translation2d right = AllianceFlipUtil.apply(FieldConstants.RightBump.nearRightCorner);
      Translation2d left = AllianceFlipUtil.apply(FieldConstants.LeftBump.nearLeftCorner);

      intermediatePose =
          right.getDistance(V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
                  < left.getDistance(V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
              ? new Pose2d(right, targetPose.getRotation())
              : new Pose2d(left, targetPose.getRotation());

      Logger.recordOutput(NTPrefixes.POSE_DATA + "Intermediate Pose", intermediatePose);

      followCommand =
          AutoBuilder.pathfindToPose(intermediatePose, constraints)
              .andThen(AutoBuilder.pathfindToPose(targetPose, constraints));
    } else {
      followCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    followCommand.initialize();
  }

  @Override
  public void execute() {
    followCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    followCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }
}
