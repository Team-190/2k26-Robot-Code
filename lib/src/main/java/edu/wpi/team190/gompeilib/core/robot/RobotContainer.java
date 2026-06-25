package edu.wpi.team190.gompeilib.core.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface RobotContainer {

  public default void robotPeriodic() {}

  public default Command getAutonomousCommand() {
    return Commands.none();
  }
}
