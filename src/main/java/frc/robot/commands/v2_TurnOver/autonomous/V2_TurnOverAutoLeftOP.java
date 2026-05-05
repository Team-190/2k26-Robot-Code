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
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;
import java.util.function.Supplier;

public class V2_TurnOverAutoLeftOP {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnOverClopper clopper,
      V2_TurnOverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    try {

      PathPlannerPath OP_1 = PathPlannerPath.fromPathFile("OP_1");
      PathPlannerPath OP_2 = PathPlannerPath.fromPathFile("OP_2");

      AdjustPathCommand followCommandOP_1 =
          new AdjustPathCommand(
              () -> OP_1.getPathPoses().get(OP_1.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);

      AdjustPathCommand followCommandOP_2 =
          new AdjustPathCommand(
              () -> OP_2.getPathPoses().get(OP_2.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);

      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRollerOverride().alongWith(intake.deploy()).ignoringDisable(true));

      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_TurnOverRobotState.resetPose(
                      AllianceFlipUtil.apply(OP_1.getStartingHolonomicPose().get()))),
          intake.deploy().alongWith(intake.setOverrideRollerVoltage(11)),
          AutoBuilder.followPath(OP_1),
          AutoBuilder.followPath(OP_2)
              .alongWith(
                  Commands.sequence(
                      Commands.waitSeconds(6.72),
                      shooter.setNonRequiringGoal(V2_TurnOverShooterConstants.ShooterGoal.STOW),
                      clopper.stopBallTunnel(),
                      clopper.stopRollerFloor())),
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
