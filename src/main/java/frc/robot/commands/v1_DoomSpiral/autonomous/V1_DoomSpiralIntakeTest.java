package frc.robot.commands.v1_DoomSpiral.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;

public class V1_DoomSpiralIntakeTest {
  public static Command testIntake(V1_DoomSpiralIntake intake) {
    return Commands.sequence(
            intake.stow(), Commands.waitSeconds(5), intake.deploy(), Commands.waitSeconds(5))
        .repeatedly();
  }
}
