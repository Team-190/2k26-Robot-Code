package frc.robot.subsystems.v1_DoomSpiral.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkage;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageConstants.LinkageState;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralIntake extends SubsystemBase {
  public GenericRoller topRoller;
  public GenericRoller bottomRoller;
  public FourBarLinkage linkage;

  public V1_DoomSpiralRobotState robotState;

  public V1_DoomSpiralIntake(
      GenericRollerIO topIO, GenericRollerIO bottomIO, FourBarLinkageIO linkageIO) {
    topRoller = new GenericRoller(topIO, this, "IntakeTopFlywheel");
    bottomRoller = new GenericRoller(bottomIO, this, "IntakeBottomFlywheel");
    linkage =
        new FourBarLinkage(linkageIO, V1_DoomSpiralIntakeConstants.LINKAGE_CONSTANTS, this, 0);
  }

  @Override
  public void periodic() {
    topRoller.periodic();
    bottomRoller.periodic();
    linkage.periodic();

    List<LinkageState> intakeGlobalPose = linkage.getLinkagePoses();

    for (int i = 0; i < intakeGlobalPose.size(); i++) {
      Logger.recordOutput("Intake/Linkage/Pose " + i, intakeGlobalPose.get(i).pose());
    }

    for (int i = 0; i < intakeGlobalPose.size(); i++) {
      Logger.recordOutput("Intake/Linkage/Rotation " + i, intakeGlobalPose.get(i).rotation());
    }
  }

  public Command setRollerVoltage(double voltage) {
    return Commands.parallel(topRoller.setVoltage(voltage), bottomRoller.setVoltage(voltage));
  }

  public Command stopRoller() {
    return Commands.parallel(topRoller.setVoltage(0), bottomRoller.setVoltage(0));
  }

  public Command deploy() {
    return linkage.setPositionGoal(V1_DoomSpiralIntakeConstants.DEPLOY_ANGLE);
  }

  public Command stow() {
    return linkage.setPositionGoal(V1_DoomSpiralIntakeConstants.STOW_ANGLE);
  }

  public Transform3d getHopperWallTransform() {
    // 1. Calculate Current Pose
    final double currentY =
        linkage.getPosition().getSin() * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double currentX0 =
        linkage.getPosition().getCos() * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double currentXOff = calculateXOffset(currentY);
    Pose3d currentPose = new Pose3d(-(currentX0 + currentXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 2. Calculate "Zero" Pose (at Y_MIN)
    final double zeroY = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MIN();
    final double zeroAngle = Math.asin(zeroY / V1_DoomSpiralIntakeConstants.PIN_LENGTH);
    final double zeroX0 = Math.cos(zeroAngle) * V1_DoomSpiralIntakeConstants.PIN_LENGTH;
    final double zeroXOff = calculateXOffset(zeroY);
    Pose3d zeroPose = new Pose3d(-(zeroX0 + zeroXOff), 0, 0, new Rotation3d(0, 0, 0));

    // 3. Return the Transform (Zero -> Current)
    return currentPose.minus(zeroPose);
  }

  /** Piecewise logic for the linkage offset */
  public double calculateXOffset(double yPos) {
    final double Y_MIN = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MIN();
    final double Y_PHASE_1 = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.PHASE_1();
    final double Y_PHASE_2 = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.PHASE_2();
    final double Y_MAX = V1_DoomSpiralIntakeConstants.LINK_BOUNDS.MAX();

    final double RADIUS_1 = V1_DoomSpiralIntakeConstants.LINK_CONST.RADIUS_1();
    final double RADIUS_2 = V1_DoomSpiralIntakeConstants.LINK_CONST.RADIUS_2();
    final double CENTER_OFFSET = V1_DoomSpiralIntakeConstants.LINK_CONST.CENTER_OFFSET();

    if (yPos <= Y_PHASE_1 && yPos >= Y_MIN) {
      return Math.sqrt(Math.pow(RADIUS_1, 2) - Math.pow(yPos, 2)) - CENTER_OFFSET;
    } else if (yPos <= Y_PHASE_2 && yPos > Y_PHASE_1) {
      return 0;
    } else if (yPos <= Y_MAX && yPos > Y_PHASE_2) {
      return Math.sqrt(Math.pow(RADIUS_2, 2) - Math.pow(yPos - Y_PHASE_2, 2)) - RADIUS_2;
    }
    return 0;
  }

  public Command subtractStowOffset() {
        return Commands.runOnce(
          () -> robotState.stowOffset -= 0.5);
    }

    public Command addStowOffset() {
      return Commands.runOnce(
          () -> robotState.stowOffset += 0.5);
    }

    public Command subtractDepotOffset(){
        return Commands.runOnce(() -> robotState.depotOffset -= 0.5);
    }
    
    public Command addDepotOffset(){
        return Commands.runOnce(() -> robotState.depotOffset += 0.5);
    }
    
}
