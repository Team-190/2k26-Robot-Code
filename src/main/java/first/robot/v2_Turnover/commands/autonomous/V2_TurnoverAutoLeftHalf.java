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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;

public class V2_TurnoverAutoLeftHalf {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnoverClopper clopper,
      V2_TurnoverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    try {
      PathPlannerPath HALF = PathPlannerFiles.loadPath("LEFT_HALF_SWEEP");
      AdjustPathCommand followCommandHALF =
          new AdjustPathCommand(
              () -> HALF.getPathPoses().get(HALF.getPathPoses().size() - 1),
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
                      AllianceFlipUtil.apply(HALF.getStartingHolonomicPose().get()))),
          intake.deploy().alongWith(intake.setOverrideRollerVoltage(11)),
          AutoBuilder.followPath(HALF),
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
