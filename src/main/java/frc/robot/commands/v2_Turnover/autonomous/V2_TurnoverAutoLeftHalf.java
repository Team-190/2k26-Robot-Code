package frc.robot.commands.v2_Turnover.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.AdjustPathCommand;
import frc.robot.commands.v2_Turnover.V2_TurnoverCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopper;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;
import java.util.function.Supplier;

public class V2_TurnoverAutoLeftHalf {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnoverClopper clopper,
      V2_TurnoverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    try {
      PathPlannerPath HALF = PathPlannerPath.fromPathFile("LEFT_HALF_SWEEP");
      AdjustPathCommand followCommandHALF =
          new AdjustPathCommand(
              () -> HALF.getPathPoses().get(HALF.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);

      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRollerOverride().alongWith(intake.deploy()).ignoringDisable(true));
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_TurnoverRobotState.resetPose(
                      AllianceFlipUtil.apply(HALF.getStartingHolonomicPose().get()))),
          intake.deploy().alongWith(intake.setOverrideRollerVoltage(11)),
          AutoBuilder.followPath(HALF),
          V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper));

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
