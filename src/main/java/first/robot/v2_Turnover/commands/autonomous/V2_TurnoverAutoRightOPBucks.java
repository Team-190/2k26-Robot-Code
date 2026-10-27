package first.robot.v2_Turnover.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.core.utility.PathPlannerFiles;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.shared.commands.AdjustPathCommand;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.Elastic;
import first.robot.v2_Turnover.V2_TurnoverRobotState;
import first.robot.v2_Turnover.commands.V2_TurnoverCompositeCommands;
import first.robot.v2_Turnover.subsystems.clopper.V2_TurnoverClopper;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooter;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooterConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;

public class V2_TurnoverAutoRightOPBucks {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnoverClopper clopper,
      V2_TurnoverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    try {
      PathPlannerPath OP_1_CROSS = PathPlannerFiles.loadPath("OP_1_CROSS").mirrorPath();
      PathPlannerPath OP_2 = PathPlannerFiles.loadPath("OP_2").mirrorPath();

      AdjustPathCommand followCommandOP_1_CROSS =
          new AdjustPathCommand(
              () -> OP_1_CROSS.getPathPoses().get(OP_1_CROSS.getPathPoses().size() - 1),
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

      BooleanSupplier invertScoreLocation = () -> false;
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_TurnoverRobotState.resetPose(
                      AllianceFlipUtil.apply(OP_1_CROSS.getStartingHolonomicPose().get()))),
          intake.deploy().alongWith(intake.setOverrideRollerVoltage(11)),
          AutoBuilder.followPath(OP_1_CROSS),
          AutoBuilder.followPath(OP_2)
              .alongWith(
                  Commands.sequence(
                      Commands.waitSeconds(6.72),
                      shooter.setNonRequiringGoal(V2_TurnoverShooterConstants.ShooterGoal.STOW),
                      clopper.stopBallTunnel(),
                      clopper.stopRollerFloor())),
          V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation));
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
