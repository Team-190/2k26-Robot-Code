package frc.robot.commands.v2_Delta.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Elastic;

public class V2_DeltaAutoRightOP {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_DeltaClopper clopper, V2_DeltaShooter shooter) {

    try {
      PathPlannerPath OP_1 = PathPlannerPath.fromPathFile("OP_1").mirrorPath();
      PathPlannerPath OP_2 = PathPlannerPath.fromPathFile("OP_2").mirrorPath();

      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRoller().alongWith(intake.deploy()).ignoringDisable(true));

      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V2_DeltaRobotState.resetPose(
                      AllianceFlipUtil.apply(OP_1.getStartingHolonomicPose().get()))),
          Commands.deadline(
              AutoBuilder.followPath(OP_1),
              intake.deploy(),
              intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE),
              V2_DeltaCompositeCommands.hold(clopper, shooter)),
          intake.setOverrideRollerVoltage(0),
          Commands.deadline(
              AutoBuilder.followPath(OP_2),
              Commands.sequence(
                  V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper).withTimeout(3.8),
                  Commands.parallel(
                          V2_DeltaCompositeCommands.hold(clopper, shooter),
                          intake.deploy(),
                          intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE))
                      .withTimeout(4.5))),
          intake.setOverrideRollerVoltage(0),
          V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper));
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
