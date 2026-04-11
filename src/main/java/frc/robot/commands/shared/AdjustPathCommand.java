package frc.robot.commands.shared;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import frc.robot.FieldConstants;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class AdjustPathCommand extends Command {

  private final Supplier<Pose2d> targetPoseSupplier;
  private final Supplier<PathAdjustmentMode> adjustmentModeSupplier;
  private final double goalEndVelocity;
  private PathAdjustmentMode adjustmentMode;
  private Pose2d targetPose;
  private Command followCommand;

  /**
   * Creates a command that generates and follows a corrective path to a target pose using
   * PathPlanner.
   *
   * @param targetPoseSupplier
   * @param goalEndVelocity
   */
  public AdjustPathCommand(
      Supplier<Pose2d> targetPoseSupplier,
      double goalEndVelocity,
      Supplier<PathAdjustmentMode> adjustmentModeSupplier) {
    this.targetPoseSupplier = targetPoseSupplier;
    this.adjustmentModeSupplier = adjustmentModeSupplier;
    this.adjustmentMode = adjustmentModeSupplier.get();
    this.goalEndVelocity = goalEndVelocity;

    Pathfinding.ensureInitialized();
    followCommand = Commands.none();
  }

  public AdjustPathCommand(Supplier<Pose2d> targetPoseSupplier) {
    this(targetPoseSupplier, 0.0, () -> PathAdjustmentMode.USE_ANY_AVAILABLE);
  }

  public AdjustPathCommand(Supplier<Pose2d> targetPoseSupplier, Supplier<PathAdjustmentMode> mode) {
    this(targetPoseSupplier, 0.0, mode);
  }

  @Override
  public void initialize() {
    followCommand =
        AutoBuilder.pathfindToPose(
            targetPoseSupplier.get(), PathConstraints.unlimitedConstraints(12), goalEndVelocity);

    adjustmentMode = adjustmentModeSupplier.get();
    targetPose = targetPoseSupplier.get();
    Pathfinding.setDynamicObstacles(
        adjustmentMode.getObstacles(), AutoBuilder.getCurrentPose().getTranslation());

    followCommand.initialize();
  }

  @Override
  public void execute() {
    if (adjustmentMode != adjustmentModeSupplier.get()) {
      adjustmentMode = adjustmentModeSupplier.get();
      Pathfinding.setDynamicObstacles(
          adjustmentMode.getObstacles(), AutoBuilder.getCurrentPose().getTranslation());
    }

    if (!targetPose.equals(targetPoseSupplier.get())) {
      targetPose = targetPoseSupplier.get();
      followCommand.end(true);
      followCommand =
          AutoBuilder.pathfindToPose(
              targetPose, PathConstraints.unlimitedConstraints(12), goalEndVelocity);
      followCommand.initialize();
    }
    followCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    followCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }

  @RequiredArgsConstructor
  public enum PathAdjustmentMode {
    ALWAYS_USE_TRENCH(
        List.of(
            Pair.of(FieldConstants.LeftBump.nearRightCorner, FieldConstants.LeftBump.farLeftCorner),
            Pair.of(
                FieldConstants.RightBump.nearRightCorner, FieldConstants.RightBump.farLeftCorner),
            Pair.of(
                FieldConstants.LeftBump.oppNearRightCorner,
                FieldConstants.LeftBump.oppFarLeftCorner),
            Pair.of(
                FieldConstants.RightBump.oppNearRightCorner,
                FieldConstants.RightBump.oppFarLeftCorner))),

    NEVER_USE_TRENCH(
        List.of(
            // Top right and bottom left corners of each Rectangle2d
            Pair.of(
                GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.BLUE_TRENCH)[0]
                    .getTranslation(),
                GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.BLUE_TRENCH)[2]
                    .getTranslation()),
            Pair.of(
                GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.RED_TRENCH)[0]
                    .getTranslation(),
                GeometryUtil.rectanglePose2ds(FieldConstants.LeftTrench.RED_TRENCH)[2]
                    .getTranslation()),
            Pair.of(
                GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.BLUE_TRENCH)[0]
                    .getTranslation(),
                GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.BLUE_TRENCH)[2]
                    .getTranslation()),
            Pair.of(
                GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.RED_TRENCH)[0]
                    .getTranslation(),
                GeometryUtil.rectanglePose2ds(FieldConstants.RightTrench.RED_TRENCH)[2]
                    .getTranslation()))),
    USE_ANY_AVAILABLE(List.of()),
    ;

    @Getter private final List<Pair<Translation2d, Translation2d>> obstacles;
  }
}
