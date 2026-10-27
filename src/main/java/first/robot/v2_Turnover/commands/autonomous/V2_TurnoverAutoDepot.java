package first.robot.v2_Turnover.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.core.utility.PathPlannerFiles;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.subsystems.intake.IntakeConstants;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.Elastic;
import first.robot.v2_Turnover.V2_TurnoverRobotState;
import first.robot.v2_Turnover.commands.V2_TurnoverCompositeCommands;
import first.robot.v2_Turnover.subsystems.clopper.V2_TurnoverClopper;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooter;
import first.robot.v2_Turnover.subsystems.shooter.V2_TurnoverShooterConstants.ShooterGoal;
import java.util.function.BooleanSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;

public class V2_TurnoverAutoDepot {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_TurnoverClopper clopper, V2_TurnoverShooter shooter) {

    PathPlannerPath DEPOT;
    try {
      DEPOT = PathPlannerFiles.loadPath("DEPOT");
      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRollerOverride().alongWith(intake.deploy()).ignoringDisable(true));

      BooleanSupplier invertScoreLocation = () -> false;
      return Commands.sequence(
          Commands.runOnce(
                  () ->
                      V2_TurnoverRobotState.resetPose(
                          AllianceFlipUtil.apply(DEPOT.getStartingHolonomicPose().get())))
              .alongWith(
                  intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE), intake.deploy()),
          Commands.deadline(
              AutoBuilder.followPath(DEPOT), V2_TurnoverCompositeCommands.hold(clopper, shooter)),
          drive.runOnce(drive::stop),
          Commands.parallel(
              V2_TurnoverCompositeCommands.scoreOrFeedCommand(
                  shooter, clopper, invertScoreLocation),
              Commands.sequence(
                  Commands.waitSeconds(4.0),
                  intake.stopRollerOverride(),
                  Commands.waitSeconds(.25),
                  intake
                      .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                      .alongWith(intake.stopRollerOverride()),
                  Commands.waitSeconds(1.5),
                  intake.deploy())),
          shooter.setGoal(ShooterGoal.IDLE));
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
