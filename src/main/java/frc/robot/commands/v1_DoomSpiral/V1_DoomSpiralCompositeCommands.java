package frc.robot.commands.v1_DoomSpiral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shared.hood.HoodConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;

public class V1_DoomSpiralCompositeCommands {

  public static Command feedCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return Commands.parallel(
            shooter.setGoal(HoodConstants.HoodGoal.FEED, V1_DoomSpiralRobotState::getFeedVelocity),
            shooter.waitUntilAtGoal())
        .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command collect(V1_DoomSpiralIntake intake) {
    return Commands.sequence(
        intake.deploy(), intake.setRollerVoltage(V1_DoomSpiralIntakeConstants.COLLECTING_VOLTAGE));
  }
}
