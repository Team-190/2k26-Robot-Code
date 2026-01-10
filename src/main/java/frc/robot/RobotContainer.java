package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface RobotContainer {

  default void robotPeriodic() {}

  default Command getAutonomousCommand() {
    return Commands.none();
  }
}
