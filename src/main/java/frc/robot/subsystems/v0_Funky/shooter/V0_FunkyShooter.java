package frc.robot.subsystems.v0_Funky.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;

public class V0_FunkyShooter extends SubsystemBase {
  private final GenericFlywheel flywheel;
  private final Turret turret;

  public V0_FunkyShooter(GenericFlywheelIO io, TurretIO turretIO) {
    flywheel =
        new GenericFlywheel(io, this, V0_FunkyShooterConstants.SHOOT_CONSTANTS, "Flywheel 1");
    turret =
        new Turret(
            turretIO,
            this,
            "",
            V0_FunkyRobotState::getGlobalPose,
            V0_FunkyShooterConstants.TURRET_CONSTANTS);
  }

  @Override
  public void periodic() {
    flywheel.periodic();
    turret.periodic();
  }

  // public void setVoltage(double volts) {
  //   flywheel.setVoltage(volts);
  // }

  // public Command setTurretVoltage(double volts) {
  //   return turret.setTurretVoltage(volts);
  // }

  // public Command setTurretGoal(Rotation2d goal) {
  //   return turret.setGoal(goal);
  // }

  // public Command waitUntilAtGoal() {
  //   return turret.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  // }

  public Command runTurretSysID() {
    return turret.runSysId();
  }
}
