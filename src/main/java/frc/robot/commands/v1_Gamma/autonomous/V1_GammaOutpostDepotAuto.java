package frc.robot.commands.v1_Gamma.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import frc.robot.subsystems.v1_Gamma.V1_GammaRobotState;
import frc.robot.subsystems.v1_Gamma.climber.V1_GammaClimber;
import frc.robot.subsystems.v1_Gamma.climber.V1_GammaClimberConstants;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntake;
import frc.robot.subsystems.v1_Gamma.shooter.V1_GammaShooter;
import frc.robot.subsystems.v1_Gamma.spindexer.V1_GammaSpindexer;
import frc.robot.util.PathCache;

/**
 * Autonomous Routine for gathering fuel from the outpost, gathering fuel from depot, going to the
 * tower, scoring the fuel, then climbing to level 1
 */
public class V1_GammaOutpostDepotAuto {

  private static final double WAIT_TIME = 4;
  private static final double SHOOT_TIME = 4;
  private static final double INTAKE_VOLTAGE = 3;
  private static final double SPINDEXER_VOLTAGE = 3;
  private static final double SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND = 10;

  /**
   * Method that returns the Autonomous Routine for gathering fuel from the outpost, waiting, and
   * then running another autonomous routine for going to the depot, gathering fuel, going to the
   * tower, scoring the fuel, then climbing tolevel 1
   *
   * @param drive The Swerve Drive subsystem
   * @param intake The Intake subsystem
   * @param shooter The Shooter subsystem
   * @param spindexer The Spindexer subsystem
   * @param climber The Climber subsystem
   * @return the Routine for the entire Outpost + Depot Autonomous Routine
   */
  public static final AutoRoutine V1_GammaOutpostDepot(
      SwerveDrive drive,
      V1_GammaIntake intake,
      V1_GammaShooter shooter,
      V1_GammaSpindexer spindexer,
      V1_GammaClimber climber) {

    // Create the routine and the trajectory for the first part of the outpost auto

    AutoRoutine routine = drive.getAutoFactory().newRoutine("outpostDepotRoutine");

    AutoTrajectory OUTPOST_1 = routine.trajectory(PathCache.getTrajectory("OUTPOST_1"));

    // Create the trajectory for the second part of the outpost auto

    AutoTrajectory OUTPOST_DEPOT = routine.trajectory(PathCache.getTrajectory("OUTPOST_DEPOT"));

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set initial pose for the first path
                OUTPOST_1.resetOdometry(),

                // Run the first path
                OUTPOST_1.cmd(),

                // Wait until the fuel has been dumped into the robot
                Commands.waitSeconds(WAIT_TIME),

                // Follow the path, lower intake, and set the shooter goal in parallel
                Commands.parallel(
                    OUTPOST_DEPOT.cmd(),
                    Commands.print("Deploying intake"), // TODO: IMPL
                    intake.setVoltage(INTAKE_VOLTAGE),
                    shooter.setGoal(HoodGoal.SCORE, SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND)),

                // Stop the intake
                intake.setVoltage(0),

                // Start the spindexer
                spindexer.setVoltage(SPINDEXER_VOLTAGE),

                // Wait for shooting
                Commands.waitSeconds(SHOOT_TIME),

                // Stop shooter and spindexer, then auto-align to the tower
                Commands.parallel(
                    shooter.stopFlywheel(),
                    spindexer.setVoltage(0),
                    DriveCommands.autoAlignTowerCommand(
                        drive,
                        V1_GammaRobotState::getGlobalPose,
                        V1_GammaConstants.AUTO_ALIGN_NEAR_CONSTANTS)),

                // Climb to L1
                climber.setPositionGoal(
                    new Rotation2d(V1_GammaClimberConstants.levelOnePositionGoal))));

    return routine;
  }
}
