package frc.robot.commands.v1_Gamma.autonomous;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;

/** Autonomous Path for gathering fuel from the neutral zone, scoring, then climbing */
public class V1_GammaTrenchAutoLeft {
  private static Command TRENCH_LEFT_PATH_CMD;

  private static final double INTAKE_VOLTAGE = 3;
  private static final double SPINDEXER_VOLTAGE = 3;
  private static final double SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND = 400;
  private static final double WAIT_TIME = 4;

  /**
   * @param drive The Swerve Drive subsystem
   * @param intake The Intake subsystem
   * @param shooter The Shooter subsystem
   * @param spindexer The Spindexer subsystem
   * @param climber The Climber subsystem
   * @return Command that returns the Autonomous Path for gathering fuel from the depot, going to
   *     the tower, scoring the fuel, then climbing to level 1
   */
  public static final Command getTrenchLeft(
      SwerveDrive drive,
      V1_GammaIntake intake,
      V1_GammaShooter shooter,
      V1_GammaSpindexer spindexer,
      V1_GammaClimber climber) {

    drive.getAutoFactory().cache().loadTrajectory("TRENCH_LEFT_PATH");
    TRENCH_LEFT_PATH_CMD = drive.getAutoFactory().trajectoryCmd("TRENCH_LEFT_PATH");
    Optional<? extends Trajectory<?>> TRENCH_LEFT =
        drive.getAutoFactory().cache().loadTrajectory("TRENCH_LEFT");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                V1_GammaRobotState.resetPose(
                    TRENCH_LEFT.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.print("Deploy Intake"),
        intake.setVoltage(INTAKE_VOLTAGE),
        TRENCH_LEFT_PATH_CMD,
        Commands.parallel(
            intake.stop(),
            shooter.setGoal(HoodGoal.SCORE, SHOOTER_FLYWHEEL_VELOCITY_RADS_PER_SECOND)),
        Commands.waitSeconds(WAIT_TIME),
        spindexer.setVoltage(SPINDEXER_VOLTAGE),
        Commands.waitSeconds(WAIT_TIME),
        Commands.parallel(
            spindexer.stopSpindexer(),
            shooter.setGoal(HoodGoal.STOW, 0),
            DriveCommands.autoAlignTowerCommand(
                drive,
                V1_GammaRobotState::getGlobalPose,
                V1_GammaConstants.AUTO_ALIGN_NEAR_CONSTANTS)),
        climber.setPositionGoal(new Rotation2d(V1_GammaClimberConstants.levelOnePositionGoal)));
  }
}
