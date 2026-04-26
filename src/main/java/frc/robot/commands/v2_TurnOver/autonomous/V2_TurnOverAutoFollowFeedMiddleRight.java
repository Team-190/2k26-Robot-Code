package frc.robot.commands.v2_TurnOver.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.AdjustPathCommand;
import frc.robot.commands.v2_TurnOver.V2_TurnOverCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_TurnOver.V2_TurnOverRobotState;
import frc.robot.subsystems.v2_TurnOver.clopper.V2_TurnOverClopper;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;
import frc.robot.util.command.AutoCommandBuilder;
import java.util.function.Supplier;

public class V2_TurnOverAutoFollowFeedMiddleRight {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnOverClopper clopper,
      V2_TurnOverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    try {
      PathPlannerPath FOLLOW_1 = PathPlannerPath.fromPathFile("FOLLOW_FEED_1").mirrorPath();
      PathPlannerPath FOLLOW_2 = PathPlannerPath.fromPathFile("FOLLOW_FEED_MIDDLE_2").mirrorPath();
      AdjustPathCommand adjustPathCommand1 =
          new AdjustPathCommand(
              () -> FOLLOW_1.getPathPoses().get(FOLLOW_1.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);
      AdjustPathCommand adjustPathCommand2 =
          new AdjustPathCommand(
              () -> FOLLOW_2.getPathPoses().get(FOLLOW_2.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);

      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRollerOverride().alongWith(intake.deploy()).ignoringDisable(true));
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_TurnOverRobotState.resetPose(
                      AllianceFlipUtil.apply(FOLLOW_1.getStartingHolonomicPose().get()))),
          intake.deploy(),
          AutoCommandBuilder.sequence(
              AutoBuilder.followPath(FOLLOW_1), AutoBuilder.followPath(FOLLOW_2)),
          V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.runOnce(
              () ->
                  Elastic.sendNotification(
                      new Elastic.Notification(
                          Elastic.NotificationLevel.ERROR,
                          "Failed to load auto path",
                          e.getMessage())))
          .ignoringDisable(true);
    }
  }
}
