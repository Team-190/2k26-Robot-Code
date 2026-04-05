package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.util.BetterAutoChooser;

public class V1_DoomSpiralAutoRightTrenchAntiBucksCrosses {
  private static boolean RETURN_TO_MID = false;

  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("RIGHT_TRENCH_ANTI_BUCKS_CROSSES");

    AutoTrajectory RIGHT_TRENCH_ANTI_BUCKS_CROSSES =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.RIGHT_TRENCH_ANTI_BUCKS_CROSSES);
    AutoTrajectory RIGHT_RETURN =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.RIGHT_RETURN_TO_MID);
    V1_DoomSpiralAutoTrajectoryCache.GO_BACK_TRIGGER.onTrue(
        Commands.runOnce(() -> RETURN_TO_MID = true));

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                RIGHT_TRENCH_ANTI_BUCKS_CROSSES.resetOdometry(),

                // Deploy the intake

                intake
                    .deploy()
                    .alongWith(
                        intake.setOverrideRollerVoltage(
                            V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                RIGHT_TRENCH_ANTI_BUCKS_CROSSES.cmd(),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS),
                        Commands.sequence(Commands.waitSeconds(3.0), intake.agitate()))
                    .until(() -> RETURN_TO_MID),
                RIGHT_RETURN
                    .cmd()
                    .alongWith(
                        V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer),
                        intake.collect())));
    return new BetterAutoChooser.AutoRoutineConfiguration(
        () -> routine,
        () -> RIGHT_TRENCH_ANTI_BUCKS_CROSSES.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      V1_DoomSpiralConstants.TRANSLATION_AUTO_GAINS,
                      V1_DoomSpiralConstants.ROTATION_AUTO_GAINS);
                  V1_DoomSpiralRobotState.setAutoTrajectory(
                      RIGHT_TRENCH_ANTI_BUCKS_CROSSES, RIGHT_RETURN);
                }));
  }
}
