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
 * Autonomous Routine for gathering fuel from the depot, going to the tower, scoring the fuel, then
 * climbing to level 1
 */
public class V1_GammaDepotAuto {

  private static final double INTAKE_VOLTAGE = 3;
  private static final double SHOOT_TIME = 4;
  private static final double SPINDEXER_VOLTAGE = 3;

  private static final double SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND = 10;

  /**
   * Method that returns the Autonomous Routine for gathering fuel from the depot, going to the
   * tower, scoring the fuel, then climbing to level 1
   *
   * @param drive The Swerve Drive subsystem
   * @param intake The Intake subsystem
   * @param shooter The Shooter subsystem
   * @param spindexer The Spindexer subsystem
   * @param climber The Climber subsystem
   * @return the Routine for the Depot Autonomous Routine
   */
  public static final AutoRoutine V1_GammaDepot(
      SwerveDrive drive,
      V1_GammaIntake intake,
      V1_GammaShooter shooter,
      V1_GammaSpindexer spindexer,
      V1_GammaClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("depotRoutine");

    AutoTrajectory DEPOT = routine.trajectory(PathCache.getTrajectory("DEPOT"));

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set initial pose
                DEPOT.resetOdometry(),

                // Deploy Intake
                Commands.print("Deploying intake"), // TODO: IMPL

                // Start the intake
                intake.setVoltage(INTAKE_VOLTAGE),

                // Follow the path and set the shooter goal in parallel
                Commands.parallel(
                    DEPOT.cmd(),
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
