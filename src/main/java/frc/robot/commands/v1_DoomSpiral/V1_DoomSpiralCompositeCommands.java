package frc.robot.commands.v1_DoomSpiral;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState.FixedShotParameters;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;

public class V1_DoomSpiralCompositeCommands {

  public static Command feedCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return shooter
        .setGoal(HoodGoal.FEED, V1_DoomSpiralRobotState::getFeedVelocity)
        .until(shooter::atGoal)
        .andThen(Commands.print("shooter at goal (both)"))
        .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command scoreCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return shooter
        .setGoal(HoodGoal.SCORE, V1_DoomSpiralRobotState::getScoreVelocity)
        .until(shooter::atGoal)
        .andThen(
            spindexer.setSpindexerOnlyVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command stopShooterCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return Commands.parallel(
        shooter.setHoodGoal(HoodGoal.STOW), spindexer.setVoltage(0), shooter.stopFlywheel());
  }

  public static Command fixedShotCommand(
      SwerveDrive drive,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      FixedShotParameters shotParameters) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.rotateToAngle(
                drive,
                V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                V1_DoomSpiralRobotState::getHeading,
                shotParameters.robotAngle()),
            shooter.setFlywheelVelocity(shotParameters.flywheelSpeed().in(RadiansPerSecond)),
            shooter.setOverrideHoodGoal(shotParameters.hoodAngle())),
        spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }
}
