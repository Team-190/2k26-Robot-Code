package frc.robot.commands.v2_Turnover.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import frc.robot.commands.shared.AdjustPathCommand;
import frc.robot.commands.v2_Turnover.V2_TurnoverCompositeCommands;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverConstants;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopper;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class V2_TurnoverAutoRightHalf {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      Intake intake,
      V2_TurnoverClopper clopper,
      V2_TurnoverShooter shooter,
      Supplier<AdjustPathCommand.PathAdjustmentMode[]> pathAdjustmentModeSupplier) {

    PathPlannerPath RIGHT_HALF_1;
    PathPlannerPath RIGHT_HALF_2;
    try {
      RIGHT_HALF_1 = PathPlannerPath.fromPathFile("RIGHT_HALF_SWEEP_1_FLIP").mirrorPath();
      RIGHT_HALF_2 = PathPlannerPath.fromPathFile("RIGHT_HALF_SWEEP_2");

      AdjustPathCommand followCommandRIGHT_HALF_1 =
          new AdjustPathCommand(
              () -> RIGHT_HALF_1.getPathPoses().get(RIGHT_HALF_1.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);
      AdjustPathCommand followCommandRIGHT_HALF_2 =
          new AdjustPathCommand(
              () -> RIGHT_HALF_2.getPathPoses().get(RIGHT_HALF_2.getPathPoses().size() - 1),
              0,
              pathAdjustmentModeSupplier);
      BooleanSupplier invertScoreLocation = () -> false;
      return Commands.parallel(
          Commands.sequence(
              Commands.runOnce(
                  () ->
                      V2_TurnoverRobotState.resetPose(
                          AllianceFlipUtil.apply(RIGHT_HALF_1.getStartingHolonomicPose().get()))),
              intake
                  .deploy()
                  .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)),
              AutoBuilder.followPath(RIGHT_HALF_1),
              followCommandRIGHT_HALF_1.onlyWhile(
                  () -> {
                    Pose2d currentPose = V2_TurnoverRobotState.getGlobalPose();
                    Pose2d targetPose =
                        RIGHT_HALF_1.getPathPoses().get(RIGHT_HALF_1.getPathPoses().size() - 1);
                    double distanceToTarget =
                        currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    boolean isFinished =
                        distanceToTarget < V2_TurnoverConstants.AUTO_CORRECTION_THRESHOLD_METERS;
                    return !isFinished;
                  }),
              AutoBuilder.followPath(RIGHT_HALF_2),
              followCommandRIGHT_HALF_2.onlyWhile(
                  () -> {
                    Pose2d currentPose = V2_TurnoverRobotState.getGlobalPose();
                    Pose2d targetPose =
                        RIGHT_HALF_2.getPathPoses().get(RIGHT_HALF_2.getPathPoses().size() - 1);
                    double distanceToTarget =
                        currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    boolean isFinished =
                        distanceToTarget < V2_TurnoverConstants.AUTO_CORRECTION_THRESHOLD_METERS;
                    return !isFinished;
                  }),
              drive.runOnce(drive::stop)),
          Commands.sequence(
              V2_TurnoverCompositeCommands.hold(clopper, shooter).withTimeout(3),
              intake.stopRollerOverride(),
              V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                  .withTimeout(5),
              intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE),
              V2_TurnoverCompositeCommands.hold(clopper, shooter).withTimeout(3),
              intake.stopRollerOverride(),
              V2_TurnoverCompositeCommands.scoreOrFeedCommand(
                  shooter, clopper, invertScoreLocation)));
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
