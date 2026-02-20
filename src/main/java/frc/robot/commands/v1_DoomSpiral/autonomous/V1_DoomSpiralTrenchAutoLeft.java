package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;

/** Autonomous Routine for gathering fuel from the neutral zone, scoring, then climbing */
public class V1_DoomSpiralTrenchAutoLeft {

  private static final double SHOOT_TIME = 4;

  /**
   * @param drive The Swerve Drive subsystem
   * @param intake The Intake subsystem
   * @param shooter The Shooter subsystem
   * @param spindexer The Spindexer subsystem
   * @param climber The Climber subsystem
   * @return Routine that returns the Autonomous Routine for gathering fuel from the neutral zone
   *     (from the left trench), going to the tower, scoring the fuel, then climbing to level 1
   */
  public static final AutoRoutine V1_DoomSpiralTrenchLeft(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("trenchAutoLeft");

    AutoTrajectory LEFT_TRENCH = routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.LEFT_TRENCH);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                LEFT_TRENCH.resetOdometry(),

                // Deploy the intake

                intake
                    .deploy()
                    .alongWith(
                        intake.setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),

                // Folow the path

                LEFT_TRENCH.cmd(),

                // Stop the intake and align the shooter in parallel

                Commands.parallel(
                        intake.stopRoller(),
                        shooter.setGoal(HoodGoal.SCORE, V1_DoomSpiralRobotState::getScoreVelocity))
                    .until(shooter::atGoal),

                // Start the spindexer

                spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE),

                // Wait for shooter to finish shooting

                Commands.waitSeconds(SHOOT_TIME),

                // Stop the spindexer, stow the hood, and align to tower in parallel

                Commands.parallel(spindexer.stopSpindexer(), shooter.setGoal(HoodGoal.STOW, 0))));

    return routine;
  }
}
