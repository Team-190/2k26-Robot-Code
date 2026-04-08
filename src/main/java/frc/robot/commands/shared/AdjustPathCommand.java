package frc.robot.commands.shared;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import java.io.IOException;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class AdjustPathCommand extends Command {
  private final Supplier<Pose2d> targetPoseSupplier;
  private double maxAccelerationMetersPerSecondSquared;
  private double maxAngularAccelerationMetersPerSecondSquared;
  private PathPlannerPath adjustedPath;
  private Command followCommand;
  private Consumer<Pose2d> resetPose;
  private final SwerveDrive drive;

  /*
   * Creates a command that generates and follows a corrective path back to the target path if the robot has deviated using PathPlanner
   * @param targetPoseSupplier A supplier that provides the target pose to which the robot should adjust
   * @param maxAccelerationMetersPerSecondSquared The maximum acceleration to use when generating the path
   * @param maxAngularAccelerationMetersPerSecondSquared The maximum angular acceleration to use when generating
   */
  public AdjustPathCommand(
      Supplier<Pose2d> targetPoseSupplier,
      double maxAccelerationMetersPerSecondSquared,
      double maxAngularAccelerationMetersPerSecondSquared,
      SwerveDrive drive)
      throws IOException, ParseException {
    this.targetPoseSupplier = targetPoseSupplier;
    this.maxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
    this.maxAngularAccelerationMetersPerSecondSquared =
        maxAngularAccelerationMetersPerSecondSquared;
    this.drive = drive;

    this.resetPose = pose -> V2_DeltaRobotState.resetPose(pose);

    try {
      AutoBuilder.configure(
          () -> V2_DeltaRobotState.getGlobalPose(),
          V2_DeltaRobotState::resetPose,
          drive::getMeasuredChassisSpeeds,
          (speeds, feedforwards) -> drive.runVelocity(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(
                  V2_DeltaConstants.TRANSLATION_AUTO_GAINS.getKP(),
                  V2_DeltaConstants.TRANSLATION_AUTO_GAINS.getKI(),
                  V2_DeltaConstants.TRANSLATION_AUTO_GAINS.getKD()),
              new PIDConstants(
                  V2_DeltaConstants.ROTATION_AUTO_GAINS.getKP(),
                  V2_DeltaConstants.ROTATION_AUTO_GAINS.getKI(),
                  V2_DeltaConstants.ROTATION_AUTO_GAINS.getKD())),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drive);
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to load PathPlanner robot config", e);
    }
  }

  @Override
  public void initialize() {
    Pose2d targetPose2d = targetPoseSupplier.get();
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(V2_DeltaRobotState.getGlobalPose(), targetPose2d);

    PathConstraints constraints =
        new PathConstraints(
            V2_DeltaConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond().doubleValue(),
            maxAccelerationMetersPerSecondSquared,
            V2_DeltaConstants.DRIVE_CONFIG.maxAngularVelocity().doubleValue(),
            maxAngularAccelerationMetersPerSecondSquared);

    PathPlannerPath adjustedPath =
        new PathPlannerPath(
            waypoints, constraints, null, new GoalEndState(5.0, targetPose2d.getRotation()));

    adjustedPath.preventFlipping = true;

    followCommand = AutoBuilder.followPath(adjustedPath);
    followCommand.initialize();
  }

  @Override
  public void execute() {
    followCommand.execute();
  }

  @Override
  public void end(boolean endUninterrupted) {
    followCommand.end(endUninterrupted);
  }

  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }
}
