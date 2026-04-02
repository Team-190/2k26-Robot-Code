package frc.robot.commands.v1_DoomSpiral.autonomous;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
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
import frc.robot.util.BetterAutoChooser;

public class V1_DoomSpiralAutoClimb {

  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
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
                Commands.parallel(
                        CLIMB.resetOdometry(),
                        intake.stopRoller(),
                        shooter.setGoal(
                            V1_DoomSpiralShooterConstants.HoodGoal.SCORE,
                            () ->
                                V1_DoomSpiralRobotState.getShootingParameters()
                                    .flywheelSpeed()
                                    .in(RadiansPerSecond)),
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
                        V1_DoomSpiralRobotState::getTowerZonePose,
                        AllianceFlipUtil.apply(
                            new Pose2d(1.055, 3.589, Rotation2d.fromDegrees(90.0))),
                        V1_DoomSpiralConstants.AUTO_ALIGN_CONSTANTS)
                    .withTimeout(3.5),
                intake.deploy(),
                Commands.waitSeconds(0.45),
                climber.setVoltage(-3.5),
                Commands.waitUntil(
                    () ->
                        climber.getArmPosition().getRadians()
                            <= ClimberGoal.L1_AUTO_POSITION_GOAL_CLIMBED
                                .getPosition()
                                .getRadians()),
                climber.stop()));

    return new BetterAutoChooser.AutoRoutineConfiguration(
        () -> routine,
        () -> CLIMB.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      Gains.builder()
                          .withKP(new LoggedTunableNumber("Drive/Auto/Climb/Translation Kp", 3.5))
                          .withKD(new LoggedTunableNumber("Drive/Auto/Climb/Translation Kd", 0.0))
                          .build(),
                      Gains.builder()
                          .withKP(new LoggedTunableNumber("Drive/Auto/Climb/Rotation Kp", 2.0))
                          .withKD(new LoggedTunableNumber("Drive/Auto/Climb/Rotation Kd", 0.0))
                          .build());
                  V1_DoomSpiralRobotState.setAutoTrajectory(CLIMB);
                }));
  }
}
