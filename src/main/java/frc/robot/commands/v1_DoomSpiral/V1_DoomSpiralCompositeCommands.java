package frc.robot.commands.v1_DoomSpiral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;

public class V1_DoomSpiralCompositeCommands {

    private static final double COLLECTING_VOLTAGE = 5;

    public static Command collect(V1_DoomSpiralIntake intake) {
        return Commands.sequence(
            Commands.runOnce(() -> intake.deploy()),
            intake.setRollerVoltage(COLLECTING_VOLTAGE)
        );
    }


}