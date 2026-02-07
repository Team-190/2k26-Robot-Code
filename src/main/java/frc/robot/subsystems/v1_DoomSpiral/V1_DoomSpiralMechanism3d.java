package frc.robot.subsystems.v1_DoomSpiral;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import java.util.List;

public class V1_DoomSpiralMechanism3d {
  private static final Translation3d spindexerTranslation =
      new Translation3d(-0.009525, 0, 0.067589);
  private static final Translation3d climberTranslation =
      new Translation3d(-0.202788, 0.090048, 0.477077);
  private static final Translation3d staticIntakeTranslation = new Translation3d(0.0, 0.0, 0.0);
  private static final Translation3d intakeCrankTranslation =
      new Translation3d(0.142476, 0, 0.278075);
  private static final Translation3d intakeFollowerTranslation =
      new Translation3d(0.278238, 0, 0.196629);
  private static final Pose3d hopperWallOffset =
      new Pose3d(new Translation3d(-0.009856, 0, 0.304569), new Rotation3d());

  private static final Rotation2d crankOffset = Rotation2d.fromDegrees(-171);
  private static final Rotation2d couplerOffset = Rotation2d.fromDegrees(-18.88);
  private static final Rotation2d followerOffset = Rotation2d.fromDegrees(-60.909742);

  public static Pose3d[] getPoses(
      Rotation2d spindexerPosition, Rotation2d climberPosition, V1_DoomSpiralIntake intake) {

    List<LinkageState> linkageStates = intake.linkage.getLinkagePoses();

    Pose3d spindexerPose = new Pose3d(spindexerTranslation, new Rotation3d(spindexerPosition));
    Pose3d climberPose =
        new Pose3d(climberTranslation, new Rotation3d(climberPosition.getRadians(), 0.0, 0.0));
    Pose3d staticIntakePose = new Pose3d(staticIntakeTranslation, new Rotation3d());
    Pose3d crankPose =
        new Pose3d(
            intakeCrankTranslation,
            new Rotation3d(
                0.0,
                linkageStates.get(0).rotation().plus(crankOffset).unaryMinus().getRadians(),
                0.0));
    Pose3d couplerPose =
        new Pose3d(
            intakeCrankTranslation.plus(linkageStates.get(1).pose().getTranslation()),
            new Rotation3d(
                0.0,
                linkageStates.get(1).rotation().plus(couplerOffset).unaryMinus().getRadians(),
                0.0));
    Pose3d followerPose =
        new Pose3d(
            intakeFollowerTranslation,
            new Rotation3d(
                0.0,
                linkageStates.get(2).rotation().minus(followerOffset).unaryMinus().getRadians(),
                0.0));

    Pose3d hopperWallPose = hopperWallOffset.transformBy(intake.getHopperWallTransform());

    return new Pose3d[] {
      spindexerPose,
      climberPose,
      staticIntakePose,
      crankPose,
      couplerPose,
      followerPose,
      hopperWallPose
    };
  }
}
