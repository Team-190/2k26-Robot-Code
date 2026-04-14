package frc.robot.commands.v2_Delta.autonomous;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.util.BetterAutoChooser.AutoRoutineConfiguration;

public class V2_DeltaAutoLeftOP {
  public static AutoRoutineConfiguration getAutoRoutine(
      SwerveDrive drive, Intake intake, V2_DeltaClopper clopper, V2_DeltaShooter shooter) {
    AutoRoutine routine = drive.getAutoFactory().newRoutine("LEFT_OP");

    AutoTrajectory OP_SAFE_1 = routine.trajectory(V2_DeltaAutoTrajectoryCache.LEFT_OP_SAFE_1);
    AutoTrajectory OP_2 = routine.trajectory(V2_DeltaAutoTrajectoryCache.LEFT_OP_2);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                OP_SAFE_1.resetOdometry(),
                intake
                    .deploy()
                    .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
                OP_SAFE_1.cmd(),
                drive.runOnce(drive::stop),
                V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper).withTimeout(5.0),
                OP_2.cmd()
                    .alongWith(intake.deploy(), V2_DeltaCompositeCommands.hold(clopper, shooter)),
                Commands.runOnce(drive::stop),
                V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)));

    return new AutoRoutineConfiguration(
        () -> routine,
        () -> OP_SAFE_1.getInitialPose().orElse(new Pose2d()),
        () ->
            Commands.runOnce(
                () -> {
                  drive.setAutoControllers(
                      V2_DeltaConstants.TRANSLATION_AUTO_GAINS,
                      V2_DeltaConstants.ROTATION_AUTO_GAINS);
                  V2_DeltaRobotState.setAutoTrajectory(OP_SAFE_1, OP_2);
                }));
  }
}
