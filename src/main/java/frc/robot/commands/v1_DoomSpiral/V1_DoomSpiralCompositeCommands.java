package frc.robot.commands.v1_DoomSpiral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState.FixedShotParameters;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.util.AllianceFlipUtil;

public class V1_DoomSpiralCompositeCommands {

    public static Command feedCommand(
            V1_DoomSpiralShooter shooter, V1_DoomSpiralSpindexer spindexer) {
        return shooter
                .setGoal(HoodGoal.FEED, V1_DoomSpiralRobotState::getFeedVelocity)
                .until(shooter::atGoal)
                .andThen(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE));
    }

    public static Command scoreCommand(
            V1_DoomSpiralShooter shooter, V1_DoomSpiralIntake intake, V1_DoomSpiralSpindexer spindexer) {
        return Commands.parallel(
                intake.stopRoller(),
                shooter.setGoal(HoodGoal.SCORE, V1_DoomSpiralRobotState::getScoreVelocity),
                Commands.sequence(
                        spindexer
                                .agitateSpindexer()
                                .until(
                                        () -> (shooter.atGoal()
                                                && DriveCommands.atAngle(
                                                        V1_DoomSpiralRobotState.getRobotToHubAngle()))),
                        spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE)));
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
            V1_DoomSpiralIntake intake,FixedShotParameters shotParameters) {
        return Commands.sequence(
                intake.stopRoller(),Commands.parallel(
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

    public static Command autoAlignL3(SwerveDrive drive, V1_DoomSpiralClimber climber) {
        Pose2d hubPose = new Pose2d(1.576, 4.54, Rotation2d.fromDegrees(90));
        ChassisSpeeds autoAlignSpeed = new ChassisSpeeds(-0.04, 0.15, 0);

        return Commands.sequence(
                DriveCommands.autoAlignPoseCommand(
                        drive,
                        V1_DoomSpiralRobotState::getTowerZonePose,
                        AllianceFlipUtil.apply(hubPose),
                        V1_DoomSpiralConstants.AUTO_ALIGN_CONSTANTS)
                        .withTimeout(4),
                Commands.run(() -> drive.runVelocity(autoAlignSpeed)).withTimeout(1));
    }
}
