package frc.robot.commands.v1_DoomSpiral.autonomous;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.util.AllianceFlipUtil;

public class V1_DoomSpiralAutoClimb {
  private static final SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
      SwerveDriveConstants.AutoAlignConstants.builder()
          .withXGains(
              Gains.fromDoubles()
                  .withPrefix("Drive/Auto Align/Climb/X")
                  .withKP(1.5)
                  .withKD(0.05)
                  .build())
          .withXConstraints(V1_DoomSpiralConstants.AUTO_ALIGN_X_CONSTRAINTS)
          .withYGains(
              Gains.fromDoubles()
                  .withPrefix("Drive/Auto Align/Climb/Y")
                  .withKP(1.5)
                  .withKD(0.05)
                  .build())
          .withYConstraints(V1_DoomSpiralConstants.AUTO_ALIGN_Y_CONSTRAINTS)
          .withRotationGains(
              Gains.fromDoubles()
                  .withPrefix("Drive/Auto Align/Climb/Rotation")
                  .withKP(0.5)
                  .withKD(0.01)
                  .build())
          .withRotationConstraints(V1_DoomSpiralConstants.AUTO_ALIGN_THETA_CONSTRAINTS)
          .withAngularThreshold(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Climb/AngularThreshold", Degrees.of(0.25)))
          .withLinearThreshold(
              new LoggedTunableMeasure<>("Drive/Auto Align/Climb/LinearThreshold", Meters.of(0.02)))
          .build();

  public static final AutoRoutine getAutoRoutine(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {
    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("CLIMB");

    AutoTrajectory CLIMB = routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.CLIMB);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                CLIMB.resetOdometry(),
                Commands.parallel(
                        intake.stopRoller(),
                        shooter.setGoal(
                            V1_DoomSpiralShooterConstants.HoodGoal.SCORE,
                            V1_DoomSpiralRobotState::getScoreVelocity),
                        Commands.sequence(
                            spindexer.agitateSpindexer().until(shooter::atGoal),
                            spindexer.setVoltage(
                                V1_DoomSpiralSpindexerConstants.SPINDEXER_VOLTAGE)))
                    .withTimeout(3.5),

                // Follow the path

                CLIMB
                    .cmd()
                    .alongWith(
                        V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)),
                climber.setPositionGoal(ClimberGoal.L1_AUTO_POSITION_GOAL),
                DriveCommands.autoAlignPoseCommand(
                    drive,
                    V1_DoomSpiralRobotState::getGlobalPose,
                    AllianceFlipUtil.apply(new Pose2d(1.055, 3.589, Rotation2d.fromDegrees(90.0))),
                    autoAlignConstants),
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

    RobotModeTriggers.autonomous()
        .negate()
        .onTrue(
            Commands.parallel(
                    V1_DoomSpiralCompositeCommands.unClimbPostAuto(intake, climber),
                    intake.stopRoller())
                .ignoringDisable(true));

    return routine;
  }
}
