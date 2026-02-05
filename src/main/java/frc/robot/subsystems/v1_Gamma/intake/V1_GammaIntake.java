package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.linkage.FourBarLinkage;
import frc.robot.subsystems.shared.linkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.shared.linkage.FourBarLinkageIO;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_GammaIntake extends SubsystemBase {
  public GenericRoller topRoller;
  public GenericRoller bottomRoller;
  public FourBarLinkage linkage;

  public V1_GammaIntake(
      GenericRollerIO topIO, GenericRollerIO bottomIO, FourBarLinkageIO linkageIO) {
    topRoller = new GenericRoller(topIO, this, "IntakeTopFlywheel");
    bottomRoller = new GenericRoller(bottomIO, this, "IntakeBottomFlywheel");
    linkage = new FourBarLinkage(linkageIO, V1_GammaIntakeConstants.LINKAGE_CONSTANTS, this, 0);
  }

  @Override
  public void periodic() {
    topRoller.periodic();
    bottomRoller.periodic();
    linkage.periodic();

    List<Pose3d> intakeGlobalPose =
        linkage.getLinkagePoses().stream().map((LinkageState state) -> state.POSE()).toList();

    for (int i = 0; i < intakeGlobalPose.size(); i++) {
      Logger.recordOutput("Intake/Linkage/Pose " + i, intakeGlobalPose.get(i));
    }

    Pose3d hopperPosition = linkage.getHopperWallPose();
    Logger.recordOutput("Intake/Linkage/HopperWallPose", hopperPosition);
  }

  public Command setRollerVoltage(double voltage) {
    return Commands.parallel(topRoller.setVoltage(voltage), bottomRoller.setVoltage(voltage));
  }

  public Command stopRoller() {
    return Commands.parallel(topRoller.setVoltage(0), bottomRoller.setVoltage(0));
  }

  public Command deploy() {
    return linkage.setPositionGoal(V1_GammaIntakeConstants.DEPLOY_ANGLE);
  }

  public Command stow() {
    return linkage.setPositionGoal(V1_GammaIntakeConstants.STOW_ANGLE);
  }

  public Command testIntake() {
    return Commands.sequence(stow(), Commands.waitSeconds(5), deploy(), Commands.waitSeconds(5));
  }
}
