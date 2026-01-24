package frc.robot.commands;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;

public class AutonomousCommands {

  public static void loadAutoTrajectories(SwerveDrive drive) {
    drive.getAutoFactory().cache().loadTrajectory("Trajectory Name");
  }
}
