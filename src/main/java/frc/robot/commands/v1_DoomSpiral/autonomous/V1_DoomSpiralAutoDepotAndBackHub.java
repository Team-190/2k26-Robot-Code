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

public class V1_DoomSpiralAutoDepotAndBackHub {
  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("DEPOT_AND_BACK_HUB");

    AutoTrajectory DEPOT_AND_BACK_HUB_PATH_1 =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.DEPOT_AND_XXX_PATH_1);
    AutoTrajectory DEPOT_AND_BACK_HUB_PATH_2 =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.DEPOT_AND_BACK_HUB_PATH_2);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                DEPOT_AND_BACK_HUB_PATH_1.resetOdometry(),

                // Deploy the intake

                intake.deploy().alongWith(intake.setRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                DEPOT_AND_BACK_HUB_PATH_1.cmd(),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS),
                        intake.agitate())
                    .withTimeout(9.0),
                intake
                    .deploy()
                    .alongWith(
                        V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer),
                        intake.setRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
                DEPOT_AND_BACK_HUB_PATH_2.cmd()));

    routine
        .active()
        .negate()
        .onTrue(V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer));

    return routine;
  }
}
