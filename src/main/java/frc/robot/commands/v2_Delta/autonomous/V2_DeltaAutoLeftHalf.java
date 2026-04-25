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

public class V2_DeltaAutoLeftHalf {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_DeltaClopper clopper, V2_DeltaShooter shooter) {

    try {
      PathPlannerPath HALF = PathPlannerPath.fromPathFile("LEFT_HALF_SWEEP");
      RobotModeTriggers.autonomous()
          .negate()
          .onTrue(intake.stopRoller().alongWith(intake.deploy()).ignoringDisable(true));
      return Commands.parallel(
          Commands.sequence(
              Commands.runOnce(
                  () ->
                      V2_DeltaRobotState.resetPose(
                          AllianceFlipUtil.apply(HALF.getStartingHolonomicPose().get()))),
              intake
                  .deploy()
                  .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
              AutoBuilder.followPath(HALF),
              drive.runOnce(drive::stop)),
          Commands.sequence(
              V2_DeltaCompositeCommands.hold(clopper, shooter).withTimeout(3),
              V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper).withTimeout(5.5),
              V2_DeltaCompositeCommands.hold(clopper, shooter).withTimeout(1.5),
              intake.stopRoller(),
              V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)));
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
