package first.robot.v1_DoomSpiral.commands.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import first.robot.shared.commands.AdjustPathCommand;
import first.robot.shared.commands.DriveCommands;
import first.robot.shared.subsystems.climber.Climber;
import first.robot.shared.subsystems.climber.ClimberConstants.ClimberGoal;
import first.robot.shared.subsystems.intake.Intake;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.BetterAutoChooser;
import first.robot.shared.util.ChoreoSwerveDrive;
import first.robot.v1_DoomSpiral.V1_DoomSpiralConstants;
import first.robot.v1_DoomSpiral.V1_DoomSpiralRobotState;
import first.robot.v1_DoomSpiral.commands.V1_DoomSpiralCompositeCommands;
import first.robot.v1_DoomSpiral.subsystems.shooter.V1_DoomSpiralShooter;
import first.robot.v1_DoomSpiral.subsystems.shooter.V1_DoomSpiralShooterConstants;
import first.robot.v1_DoomSpiral.subsystems.spindexer.V1_DoomSpiralSpindexer;
import first.robot.v1_DoomSpiral.subsystems.spindexer.V1_DoomSpiralSpindexerConstants;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class V1_DoomSpiralAutoClimb {

  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
      ChoreoSwerveDrive drive,
      Intake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      Climber climber) {
    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("CLIMB");

    AutoTrajectory CLIMB = routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.CLIMB);

    AdjustPathCommand followCommand = new AdjustPathCommand(() -> CLIMB.getFinalPose().get());

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                        CLIMB.resetOdometry(),
                        intake.stopRollerOverride(),
                        shooter.setGoal(
                            V1_DoomSpiralShooterConstants.HoodGoal.SCORE,
                            () -> V1_DoomSpiralRobotState.getScoreVelocity()),
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
                followCommand
                    .onlyWhile(
                        () -> {
                          Pose2d currentPose = V1_DoomSpiralRobotState.getGlobalPose();
                          Pose2d targetPose = CLIMB.getFinalPose().get();
                          double distanceToTarget =
                              currentPose.getTranslation().getDistance(targetPose.getTranslation());
                          boolean isFinished =
                              distanceToTarget
                                  < V1_DoomSpiralConstants.AUTO_CORRECTION_THRESHOLD_METERS;
                          return !isFinished;
                        })
                    .withTimeout(3.5),
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
                          .withPrefix("Drive/Auto/Climb/Translation")
                          .withKP(3.5)
                          .withKD(0.0)
                          .build(),
                      Gains.builder()
                          .withPrefix("Drive/Auto/Climb/Rotation")
                          .withKP(2.0)
                          .withKD(0.0)
                          .build());
                  V1_DoomSpiralRobotState.setAutoTrajectory(CLIMB);
                }));
  }
}
