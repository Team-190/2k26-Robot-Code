package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.shared.linkage.Linkage;
import frc.robot.subsystems.shared.linkage.LinkageIO;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_GammaIntake extends SubsystemBase {
  public GenericRoller topRoller;
  public GenericRoller bottomRoller;
  public Linkage pivot;

  public V1_GammaIntake(GenericRollerIO topIO, GenericRollerIO bottomIO, LinkageIO pivotIO) {
    topRoller = new GenericRoller(topIO, this, "IntakeTopFlywheel");
    bottomRoller = new GenericRoller(bottomIO, this, "IntakeBottomFlywheel");
    pivot = new Linkage(pivotIO, this, 0);
  }

  @Override
  public void periodic() {
    topRoller.periodic();
    bottomRoller.periodic();
    pivot.periodic();

    List<Pose3d> intakeGlobalPose = pivot.getLinkagePoses();

    // get poses from linkage
    // get robot pose from v1_gammaRobotState
    // turn pose2d's into pose 3d's based on offset in linkage constants and robot pose
    // log pose 3ds under intake topics

    for (int i = 0; i < intakeGlobalPose.size(); i++) {
      Logger.recordOutput("Intake/Linkage/Pose" + i, intakeGlobalPose.get(i));
    }

    Pose3d hopperPosition = pivot.getHopperWallPose();
    Logger.recordOutput("Intake/Linkage/HopperWallPose", hopperPosition);
  }

  public Command setVoltage(double voltage) {
    return runOnce(
        () -> {
          topRoller.setVoltage(voltage);
          bottomRoller.setVoltage(voltage);
        });
  }

  public Command stop() {
    return runOnce(
        () -> {
          topRoller.setVoltage(0);
          bottomRoller.setVoltage(0);
        });
  }

  public Command deploy() {
    return pivot.setPositionGoal(V1_GammaIntakeConstants.DEPLOY_ANGLE);
  }

  public Command stow() {
    return pivot.setPositionGoal(V1_GammaIntakeConstants.STOW_ANGLE);
  }

  public Command testIntake() {
    return Commands.sequence(deploy(), Commands.waitSeconds(10), stow(), Commands.waitSeconds(10))
        .repeatedly();

    // return pivot.setPositionGoal(Rotation2d.kZero);
  }
}
