package frc.robot.subsystems.v1_Gamma;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntake;
import java.util.List;

public class V1_GammaMechanism3d {
  private static final Translation3d spindexerTranslation =
      new Translation3d(-0.009525, 0, 0.067589);
  private static final Translation3d climberTranslation =
      new Translation3d(-0.202788, 0.090048, 0.477077);
  private static final Translation3d staticIntakeTranslation = new Translation3d(0.0, 0.0, 0.0);
  private static final Translation3d intakeCrankTranslation = new Translation3d(0.142, 0, 0.278075);

  public static Pose3d[] getPoses(
      Rotation2d spindexerPosition, Rotation2d climberPosition, V1_GammaIntake intake) {

    // Get all four-bar linkage poses with the crank mounted at intakeCrankTranslation
    List<LinkageState> linkageStates = intake.linkage.getLinkagePoses();

    Pose3d spindexerPose = new Pose3d(spindexerTranslation, new Rotation3d(spindexerPosition));
    Pose3d climberPose =
        new Pose3d(climberTranslation, new Rotation3d(climberPosition.getRadians(), 0.0, 0.0));
    Pose3d staticIntakePose = new Pose3d(staticIntakeTranslation, new Rotation3d());
    Pose3d crankPose = linkageStates.get(0).pose();
    Pose3d couplerPose = linkageStates.get(1).pose();
    Pose3d followerPose = linkageStates.get(2).pose();

    return new Pose3d[] {
      spindexerPose, climberPose, staticIntakePose, crankPose, couplerPose, followerPose
    };
  }
}
