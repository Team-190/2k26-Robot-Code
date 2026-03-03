package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;

public class V1_DoomSpiralAutoClimb {
  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("CLIMB");

    AutoTrajectory CLIMB = routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.CLIMB);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                CLIMB.resetOdometry(),
                // V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                //     .withTimeout(2.5),

                // Follow the path

                CLIMB.cmd()));

    routine
        .active()
        .negate()
        .onTrue(V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer));

    return routine;
  }
}
