package frc.robot.commands.v1_Gamma.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import frc.robot.subsystems.v1_Gamma.V1_GammaRobotState;
import frc.robot.subsystems.v1_Gamma.climber.V1_GammaClimber;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntake;
import frc.robot.subsystems.v1_Gamma.shooter.V1_GammaShooter;
import frc.robot.subsystems.v1_Gamma.spindexer.V1_GammaSpindexer;

/** Autonomous Routine for gathering fuel from the neutral zone, scoring, then climbing */
public class V1_GammaTrenchAutoLeft {

  private static final double INTAKE_VOLTAGE = 3;
  private static final double SPINDEXER_VOLTAGE = 3;
  private static final double SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND = 400;
  private static final double WAIT_TIME = 4;
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
  public static final AutoRoutine V1_GammaTrenchLeft(
      SwerveDrive drive,
      V1_GammaIntake intake,
      V1_GammaShooter shooter,
      V1_GammaSpindexer spindexer,
      V1_GammaClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("trenchAutoLeft");

    AutoTrajectory LEFT_TRENCH = routine.trajectory(V1_GammaAutoTrajectoryCache.LEFT_TRENCH);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                LEFT_TRENCH.resetOdometry(),

                // Deploy the intake

                Commands.print("Deploy Intake"), // TODO: IMPL

                // Start the intake

                intake.setRollerVoltage(INTAKE_VOLTAGE),

                // Folow the path

                LEFT_TRENCH.cmd(),

                // Stop the intake and align the shooter in parallel

                Commands.parallel(
                    intake.stopRoller(),
                    shooter.setGoal(HoodGoal.SCORE, SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND)),

                // Wait to reach the goal

                Commands.waitSeconds(WAIT_TIME),

                // Start the spindexer

                spindexer.setVoltage(SPINDEXER_VOLTAGE),

                // Wait for shooter to finish shooting

                Commands.waitSeconds(SHOOT_TIME),

                // Stop the spindexer, stow the hood, and align to tower in parallel

                Commands.parallel(
                    spindexer.stopSpindexer(),
                    shooter.setGoal(HoodGoal.STOW, 0),
                    DriveCommands.autoAlignTowerCommand(
                        drive,
                        V1_GammaRobotState::getGlobalPose,
                        V1_GammaConstants.AUTO_ALIGN_NEAR_CONSTANTS)),

                // Climb to L1

                climber.climbAutoSequence()));

    return routine;
  }
}
