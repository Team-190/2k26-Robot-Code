package frc.robot.subsystems.v2_Turnover;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import java.util.List;

public class V2_TurnoverMechanism3d {
  // Shooter
  private static final Translation3d baseTurretTranslation = // robot centric
      new Translation3d(-0.017463, -0.163513, 0.371475);
  private static final Translation3d baseHoodTranslation =
      new Translation3d(0.043168, -0.1635125, 0.474975); // robot centric
  private static final Transform3d turretToHoodTransform =
      new Transform3d(baseHoodTranslation.minus(baseTurretTranslation), Rotation3d.kZero);

  // Climber
  private static final Translation3d climberTranslation =
      new Translation3d(-0.317482, 0.090043, 0.477114);

  // Intake
  private static final Translation3d intakeCrankTranslation =
      new Translation3d(0.139700, 0, 0.254000);
  private static final Translation3d intakeFollowerTranslation =
      new Translation3d(0.292100, 0, 0.171450);

  private static final Rotation2d crankOffset = Rotation2d.fromDegrees(-180 + 30.838927);
  private static final Rotation2d couplerOffset = Rotation2d.fromDegrees(0);
  private static final Rotation2d followerOffset = Rotation2d.fromDegrees(-70.753060);

  public static Pose3d[] getPoses(
      Rotation2d climberPosition, Intake intake, V2_TurnoverShooter shooter) {
    // Shooter
    Pose3d turretPose =
        new Pose3d(
            baseTurretTranslation,
            new Rotation3d(Rotation2d.k180deg.plus(shooter.getTurretRotation())));
    Pose3d hoodPose =
        turretPose.transformBy(
            new Transform3d(
                turretToHoodTransform.getTranslation(),
                new Rotation3d(0.0, shooter.getHoodAngle().getRadians(), 0.0)));

    // Climber
    Pose3d climberPose =
        new Pose3d(climberTranslation, new Rotation3d(-climberPosition.getRadians(), 0.0, 0.0));

    // Intake
    List<LinkageState> linkageStates = intake.getLinkage().getLinkagePoses();
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

    return new Pose3d[] {
      turretPose, hoodPose, climberPose, Pose3d.kZero, crankPose, couplerPose, followerPose
    };
  }
}
