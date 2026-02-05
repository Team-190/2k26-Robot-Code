package frc.robot.commands.v1_Gamma.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntake;

public class V1_GammaIntakeTest {
  public static Command testIntake(V1_GammaIntake intake) {
    return Commands.sequence(
            intake.stow(), Commands.waitSeconds(5), intake.deploy(), Commands.waitSeconds(5))
        .repeatedly();
  }
}
