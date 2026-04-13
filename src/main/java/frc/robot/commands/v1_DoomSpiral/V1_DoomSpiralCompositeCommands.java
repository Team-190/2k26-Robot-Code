package frc.robot.commands.v1_DoomSpiral;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState.FixedShotParameters;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.command.ContinuousConditionalCommand;

public class V1_DoomSpiralCompositeCommands {

  public static Command feedCommand(
      V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
    return shooter
        .setGoal(HoodGoal.FEED, V1_DoomSpiralRobotState::getFeedVelocity)
        .until(shooter::atGoal)
        .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command scoreCommand(
      V1_DoomSpiralShooter shooter, Intake intake, V1_DoomSpiralSpindexer spindexer) {
    return Commands.parallel(
        intake.stopRoller(),
        shooter.setGoal(
            HoodGoal.SCORE,
            () ->
                V1_DoomSpiralRobotState.getShootingParameters()
                    .flywheelSpeed()
                    .in(RadiansPerSecond)),
        new ContinuousConditionalCommand(
            spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE),
            spindexer.agitateSpindexer(),
            () ->
                shooter.atGoal()
                    && DriveCommands.atAngle(
                        V1_DoomSpiralRobotState.getShootingParameters().chassisAngle())));
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
      Intake intake,
      FixedShotParameters shotParameters) {
    return Commands.sequence(
        intake.stopRoller(),
        Commands.parallel(
                DriveCommands.rotateToAngle(
                    drive,
                    V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                    V1_DoomSpiralRobotState::getHeading,
                    () -> AllianceFlipUtil.apply(shotParameters.robotAngle())),
                shooter.setFlywheelGoal(shotParameters.flywheelSpeed()),
                shooter.setOverrideHoodGoal(shotParameters.hoodAngle()),
                spindexer.agitateSpindexer())
            .until(shooter::atGoal),
        spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
  }

  public static Command deployClimber(Intake intake, Climber climber) {
    return Commands.sequence(
        intake.stow(),
        Commands.parallel(
            Commands.sequence(intake.stopCollect()),
            climber.setPositionGoal(ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ZERO)));
  }

  public static Command unClimbPostAuto(Intake intake, Climber climber) {
    return Commands.parallel(
        intake.stow(), climber.setPositionGoal(ClimberGoal.UNCLIMB.getPosition(), GainSlot.ZERO));
  }
}
