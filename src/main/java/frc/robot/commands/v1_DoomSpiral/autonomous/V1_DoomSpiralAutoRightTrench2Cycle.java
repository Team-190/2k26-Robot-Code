package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;

public class V1_DoomSpiralAutoRightTrench2Cycle {
  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("DEPOT_AND_BACK_HUB");

    AutoTrajectory RIGHT_TRENCH_2_CYCLE_PATH_1 =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.RIGHT_TRENCH_2_CYCLE_PATH_1);
    AutoTrajectory RIGHT_TRENCH_2_CYCLE_PATH_2 =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.RIGHT_TRENCH_2_CYCLE_PATH_2);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                RIGHT_TRENCH_2_CYCLE_PATH_1.resetOdometry(),

                // Deploy the intake

                intake
                    .deploy()
                    .alongWith(
                        intake.setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                RIGHT_TRENCH_2_CYCLE_PATH_1.cmd(),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS))
                    .withTimeout(5.0),
                intake
                    .deploy()
                    .alongWith(
                        intake.setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),
                RIGHT_TRENCH_2_CYCLE_PATH_2.cmd(),
                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS))
                    .withTimeout(5.0)));

    routine
        .active()
        .negate()
        .onTrue(V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer));

    return routine;
  }
}
