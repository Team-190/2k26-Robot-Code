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
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class AdjustPathCommand extends Command {

  private final Supplier<Pose2d> currentPoseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final double maxAccelerationMPS2;
  private final double maxAngularAccelerationRPS2;
  private final double goalEndVelocity;
  private final LinearConstraints linearConstraints;
  private final AngularPositionConstraints angularVelocityConstraints;
  private final SwerveDrive drive;
  private Command followCommand;

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
      double maxAccelerationMPS2,
      double maxAngularAccelerationRPS2,
      double goalEndVelocity,
      LinearConstraints linearConstraints,
      AngularPositionConstraints angularVelocityConstraints,
      SwerveDrive drive) {

    this.currentPoseSupplier = currentPoseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
    this.linearConstraints = linearConstraints;
    this.angularVelocityConstraints = angularVelocityConstraints;
    this.maxAccelerationMPS2 = maxAccelerationMPS2;
    this.maxAngularAccelerationRPS2 = maxAngularAccelerationRPS2;
    this.goalEndVelocity = goalEndVelocity;
    this.drive = drive;

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
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d currentPose = currentPoseSupplier.get();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);

    PathConstraints constraints =
        new PathConstraints(
            linearConstraints.maxVelocity().getRawValue(),
            maxAccelerationMPS2,
            angularVelocityConstraints.maxVelocity().getRawValue(),
            maxAngularAccelerationRPS2);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(goalEndVelocity, targetPose.getRotation()));

    path.preventFlipping = true;

    followCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
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
