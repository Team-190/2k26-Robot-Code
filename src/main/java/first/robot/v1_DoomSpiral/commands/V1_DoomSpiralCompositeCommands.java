package first.robot.v1_DoomSpiral.commands;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.shared.commands.DriveCommands;
import first.robot.shared.subsystems.climber.Climber;
import first.robot.shared.subsystems.climber.ClimberConstants.ClimberGoal;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.command.ContinuousConditionalCommand;
import first.robot.v1_DoomSpiral.V1_DoomSpiralConstants;
import first.robot.v1_DoomSpiral.V1_DoomSpiralRobotState;
import first.robot.v1_DoomSpiral.V1_DoomSpiralRobotState.FixedShotParameters;
import first.robot.v1_DoomSpiral.subsystems.shooter.V1_DoomSpiralShooter;
import first.robot.v1_DoomSpiral.subsystems.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import first.robot.v1_DoomSpiral.subsystems.spindexer.V1_DoomSpiralSpindexer;
import first.robot.v1_DoomSpiral.subsystems.spindexer.V1_DoomSpiralSpindexerConstants;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

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
        intake.stopRollerOverride(),
        shooter.setGoal(HoodGoal.SCORE, () -> V1_DoomSpiralRobotState.getScoreVelocity()),
        new ContinuousConditionalCommand(
            spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE),
            spindexer.agitateSpindexer(),
            () ->
                shooter.atGoal()
                    && DriveCommands.atAngle(
                        V1_DoomSpiralRobotState.getHeading(),
                        V1_DoomSpiralRobotState.getRobotToHubAngle())));
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
        intake.stopRollerOverride(),
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
