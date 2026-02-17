package frc.robot.commands.v1_DoomSpiral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shared.hood.HoodConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;

public class V1_DoomSpiralCompositeCommands {

  public static Command feedCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return shooter
        .setGoal(HoodConstants.HoodGoal.FEED, V1_DoomSpiralRobotState::getFeedVelocity)
        .until(shooter::atGoal)
        .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }
}
