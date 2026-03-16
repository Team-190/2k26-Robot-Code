package frc.robot.commands.v1_DoomSpiral;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState.FixedShotParameters;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
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
      V1_DoomSpiralShooter shooter, V1_DoomSpiralIntake intake, V1_DoomSpiralSpindexer spindexer) {
    return Commands.parallel(
        intake.stopRoller(),
        shooter
            .setGoal(HoodGoal.SCORE, V1_DoomSpiralRobotState::getScoreVelocity)
            .until(
                () ->
                    (shooter.atGoal()
                        && DriveCommands.atAngle(V1_DoomSpiralRobotState.getRobotAngle())))
            .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE)));
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
                shooter.setOverrideHoodGoal(shotParameters.hoodAngle()))
            .until(shooter::atGoal),
        spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command deployClimber(V1_DoomSpiralIntake intake, V1_DoomSpiralClimber climber) {
    return Commands.sequence(
        intake.stow(),
        Commands.parallel(
            Commands.sequence(
                intake.setRollerVoltage(-12.0),
                intake.waitUntilIntakeAtGoal(),
                intake.stopRoller()),
            climber.setPositionGoal(ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ZERO)));
  }

  public static Command unClimbPostAuto(V1_DoomSpiralIntake intake, V1_DoomSpiralClimber climber) {
    return Commands.parallel(
        intake.stow(), climber.setPositionGoal(ClimberGoal.UNCLIMB.getPosition(), GainSlot.ZERO));
  }

  public static Command autoAllignL3(SwerveDrive drive) {
    return DriveCommands.autoAlignPoseCommand(
            drive,
            V1_DoomSpiralRobotState::getGlobalPose,
            new Pose2d(1.3, 4.67, Rotation2d.fromDegrees(90)),
            V1_DoomSpiralConstants.AUTO_ALIGN_CONSTANTS);
  }
}
