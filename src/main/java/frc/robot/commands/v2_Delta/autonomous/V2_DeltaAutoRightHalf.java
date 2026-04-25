package frc.robot.commands.v2_Delta.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.util.AllianceFlipUtil;

public class V2_DeltaAutoRightHalf {
  public static Command getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_DeltaClopper clopper, V2_DeltaShooter shooter) {

    PathPlannerPath RIGHT_HALF_1;
    PathPlannerPath RIGHT_HALF_2;
    try {
      RIGHT_HALF_1 = PathPlannerPath.fromPathFile("RIGHT_HALF_SWEEP_1_FLIP").mirrorPath();
      RIGHT_HALF_2 = PathPlannerPath.fromPathFile("RIGHT_HALF_SWEEP_2");
      return Commands.parallel(
          Commands.sequence(
              Commands.runOnce(
                  () ->
                      V2_DeltaRobotState.resetPose(
                          AllianceFlipUtil.apply(RIGHT_HALF_1.getStartingHolonomicPose().get()))),
              intake
                  .deploy()
                  .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
              AutoBuilder.followPath(RIGHT_HALF_1),
              AutoBuilder.followPath(RIGHT_HALF_2),
              drive.runOnce(drive::stop)),
          Commands.sequence(
              V2_DeltaCompositeCommands.hold(clopper, shooter).withTimeout(3),
              intake.stopRollerOverride(),
              V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper).withTimeout(5),
              intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE),
              V2_DeltaCompositeCommands.hold(clopper, shooter).withTimeout(3),
              intake.stopRollerOverride(),
              V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)));
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
