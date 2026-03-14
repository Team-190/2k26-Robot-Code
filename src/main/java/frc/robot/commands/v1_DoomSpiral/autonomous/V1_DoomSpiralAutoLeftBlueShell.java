package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;

public class V1_DoomSpiralAutoLeftBlueShell {
  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("LEFT_BLUESHELL");

    AutoTrajectory LEFT_BLUESHELL =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.LEFT_BLUESHELL);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                LEFT_BLUESHELL.resetOdometry(),

                // Deploy the intake

                intake.deploy().alongWith(intake.setRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                LEFT_BLUESHELL.cmd(),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS))));

    routine
        .active()
        .negate()
        .onTrue(V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer));

    return routine;
  }
}
