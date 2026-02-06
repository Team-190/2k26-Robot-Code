package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
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
        linkage.getLinkagePoses().stream().map(LinkageState::pose).toList();

    for (int i = 0; i < intakeGlobalPose.size(); i++) {
      Logger.recordOutput("Intake/Linkage/Pose " + i, intakeGlobalPose.get(i));
    }

    Pose3d hopperPosition = getHopperWallPose();
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

  public Pose3d getHopperWallPose() {

    final double yPos = linkage.getPosition().getSin() * V1_GammaIntakeConstants.PIN_LENGTH;
    final double x0 = linkage.getPosition().getCos() * V1_GammaIntakeConstants.PIN_LENGTH;

    double xOff = 0;

    final double Y_MIN = V1_GammaIntakeConstants.LINK_BOUNDS.MIN(); // 0.810921
    final double Y_PHASE_1 = V1_GammaIntakeConstants.LINK_BOUNDS.PHASE_1(); // 2.86545
    final double Y_PHASE_2 = V1_GammaIntakeConstants.LINK_BOUNDS.PHASE_2(); // 4.752162
    final double Y_MAX = V1_GammaIntakeConstants.LINK_BOUNDS.MAX(); // 6.46545

    final double RADIUS_1 = V1_GammaIntakeConstants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = V1_GammaIntakeConstants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = V1_GammaIntakeConstants.LINK_CONST.CENTER_OFFSET();

    if (yPos <= Y_PHASE_1 && yPos > Y_MIN) {
      xOff = Math.sqrt(Math.pow(RADIUS_1, 2) - Math.pow(yPos, 2)) - CENTER_OFFSET;
    } else if (yPos <= Y_PHASE_2 && yPos > Y_PHASE_1) {
      xOff = 0;
    } else if (yPos <= Y_MAX && yPos > Y_PHASE_2) {
      xOff = Math.sqrt(Math.pow(RADIUS_2, 2) - Math.pow(yPos - Y_PHASE_2, 2)) - RADIUS_2;
    }

    return new Pose3d(-(x0 + xOff), 0, 0, new Rotation3d(0, 0, 0));
  }
}
