package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.util.AllianceFlipUtil;

public class V1_DoomSpiralAutoClimb {
  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      Climber climber) {
    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("CLIMB");

    AutoTrajectory CLIMB = routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.CLIMB);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                CLIMB.resetOdometry(),
                // V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                //     .withTimeout(5),
                // V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer),

                // Follow the path

                CLIMB.cmd(),
                climber.setPositionGoal(ClimberGoal.L1_AUTO_POSITION_GOAL),
                DriveCommands.autoAlignPoseCommand(
                        drive,
                        V1_DoomSpiralRobotState::getGlobalPose,
                        AllianceFlipUtil.apply(
                            new Pose2d(1.055, 3.589, Rotation2d.fromDegrees(90.0))),
                        V1_DoomSpiralConstants.AUTO_ALIGN_CONSTANTS)
                    .withTimeout(5.0),
                intake.deploy(),
                Commands.waitSeconds(1.0),
                climber.setVoltage(-3.0),
                Commands.waitUntil(
                    () ->
                        climber.getArmPosition().getRadians()
                            <= ClimberGoal.L1_AUTO_POSITION_GOAL_CLIMBED
                                .getPosition()
                                .getRadians()),
                climber.stop()));

    routine
        .active()
        .negate()
        .onTrue(V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer));

    return routine;
  }
}
