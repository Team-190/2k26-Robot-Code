package frc.robot.commands.v2_Turnover.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v2_Turnover.V2_TurnoverCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopper;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooterConstants.ShooterGoal;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;

public class V2_TurnoverAutoDepot {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_TurnoverClopper clopper, V2_TurnoverShooter shooter) {

    PathPlannerPath DEPOT;
    try {
      DEPOT = PathPlannerPath.fromPathFile("DEPOT");
      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRollerOverride().alongWith(intake.deploy()).ignoringDisable(true));

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
              V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper),
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
