package frc.robot.commands.v1_DoomSpiral.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

public class V1_DoomSpiralAutoLeftTrenchAntiBucks {
  public static final BetterAutoChooser.AutoRoutineConfiguration getAutoRoutine(
      SwerveDrive drive,
      V1_DoomSpiralIntake intake,
      V1_DoomSpiralShooter shooter,
      V1_DoomSpiralSpindexer spindexer,
      V1_DoomSpiralClimber climber) {

    // Create the routine and the trajectory

    AutoRoutine routine = drive.getAutoFactory().newRoutine("LEFT_TRENCH_ANTI_BUCKS");

    AutoTrajectory LEFT_TRENCH_ANTI_BUCKS =
        routine.trajectory(V1_DoomSpiralAutoTrajectoryCache.LEFT_TRENCH_ANTI_BUCKS);

    routine
        .active()
        .onTrue(
            Commands.sequence(

                // Set the inital pose

                LEFT_TRENCH_ANTI_BUCKS.resetOdometry(),

                // Deploy the intake

                intake
                    .deploy()
                    .alongWith(
                        intake.setOverrideRollerVoltage(
                            V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)),

                // Follow the path

                LEFT_TRENCH_ANTI_BUCKS.cmd(),

                // Stop drive

                Commands.runOnce(() -> drive.stop()),

                // Stop the intake and align the shooter in parallel

                V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                    .alongWith(
                        DriveCommands.aimAtHub(drive, V1_DoomSpiralConstants.DRIVE_CONSTANTS),
                        Commands.sequence(Commands.waitSeconds(3.0), intake.agitate()))));

    RobotModeTriggers.autonomous()
        .negate()
        .onTrue(
            Commands.parallel(
                    V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer),
                    intake.stopRoller(),
                    intake.deploy())
                .ignoringDisable(true));

    return new BetterAutoChooser.AutoRoutineConfiguration(
        () -> routine,
        () -> LEFT_TRENCH_ANTI_BUCKS.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      V1_DoomSpiralConstants.TRANSLATION_AUTO_GAINS,
                      V1_DoomSpiralConstants.ROTATION_AUTO_GAINS);
                  V1_DoomSpiralRobotState.setAutoTrajectory(LEFT_TRENCH_ANTI_BUCKS);
                }));
  }
}
