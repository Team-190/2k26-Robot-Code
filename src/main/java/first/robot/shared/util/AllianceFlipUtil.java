package first.robot.shared.util;

import first.robot.FieldConstants;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class AllianceFlipUtil {
  public static double fieldWidth = FieldConstants.fieldWidth;
  public static double fieldLength = FieldConstants.fieldLength;

  public static double applyX(double x) {
    return shouldFlip() ? fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.PI) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static double overrideApplyX(double x) {
    return fieldLength - x;
  }

  public static double overrideApplyY(double y) {
    return fieldWidth - y;
  }

  public static Translation2d overrideApply(Translation2d translation) {
    return new Translation2d(
        overrideApplyX(translation.getX()), overrideApplyY(translation.getY()));
  }

  public static Rotation2d overrideApply(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.PI);
  }

  public static Pose2d overrideApply(Pose2d pose) {
    return new Pose2d(overrideApply(pose.getTranslation()), overrideApply(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED;
  }
}
