package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.v1_Gamma.spindexer.V1_GammaSpindexer;

public class AutonomousCommands {

  public static void loadAutoTrajectories(SwerveDrive drive) {
    drive.getAutoFactory().cache().loadTrajectory("Trajectory Name");
  }

  public static final Command spindexerTest(V1_GammaSpindexer spindexer) {
    return spindexer.setVoltage(3.0);
  }
}
