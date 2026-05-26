package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.AdjustPathCommand;
import frc.robot.commands.shared.AdjustPathCommand.PathAdjustmentMode;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.util.BetterAutoChooser;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class V1_DoomSpiralAutoLeftTrenchSimple {
  private static boolean RETURN_TO_MID = false;

  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      LoggedNetworkBoolean returnToMid,
      Supplier<PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("LEFT_TRENCH_SIMPLE");

    AutoTrajectory LEFT_TRENCH_SIMPLE =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.LEFT_TRENCH_SIMPLE);
    AutoTrajectory LEFT_RETURN =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.LEFT_RETURN_TO_MID);

    V1_DoomSpiralAutoTrajectoryCache.GO_BACK_TRIGGER.onTrue(
        Commands.runOnce(() -> RETURN_TO_MID = returnToMid.get()));

    AdjustPathCommand followCommand =
        new AdjustPathCommand(
            () -> LEFT_TRENCH_SIMPLE.getFinalPose().get(), 0, pathAdjustmentModeSupplier);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                LEFT_TRENCH_SIMPLE.resetOdometry(),

                // Deploy the intake

                intake
                    .deploy()
                    .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                LEFT_TRENCH_SIMPLE.cmd(),
                followCommand.onlyWhile(
                    () -> {
                      Pose2d currentPose = V1_DoomSpiralRobotState.getGlobalPose();
                      Pose2d targetPose = LEFT_TRENCH_SIMPLE.getFinalPose().get();
                      double distanceToTarget =
                          currentPose.getTranslation().getDistance(targetPose.getTranslation());
                      boolean isFinished =
                          distanceToTarget
                              < V1_DoomSpiralConstants.AUTO_CORRECTION_THRESHOLD_METERS;
                      return !isFinished;
                    }),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS),
                        Commands.sequence(Commands.waitSeconds(3.0), intake.agitate()))
                    .until(() -> RETURN_TO_MID),
                LEFT_RETURN
                    .cmd()
                    .alongWith(
                        V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer),
                        intake.collect()),
                        DriveCommands.stop(drive)));

    return new BetterAutoChooser.AutoRoutineConfiguration(
        () -> routine,
        () -> LEFT_TRENCH_SIMPLE.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      V1_DoomSpiralConstants.TRANSLATION_AUTO_GAINS,
                      V1_DoomSpiralConstants.ROTATION_AUTO_GAINS);
                  V1_DoomSpiralRobotState.setAutoTrajectory(LEFT_TRENCH_SIMPLE, LEFT_RETURN);
                }));
  }
}
