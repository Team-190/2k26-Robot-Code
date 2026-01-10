package frc.robot.commands;

import frc.robot.subsystems.shared.drive.Drive;

public class AutonomousCommands {

  public static void loadAutoTrajectories(Drive drive) {
    drive.getAutoFactory().cache().loadTrajectory("Trajectory Name");
  }
}
