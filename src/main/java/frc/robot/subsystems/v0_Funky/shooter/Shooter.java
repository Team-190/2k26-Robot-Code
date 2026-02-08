package frc.robot.subsystems.v0_Funky.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;

public class Shooter extends SubsystemBase {
  private GenericFlywheel flywheel;
  private Turret turret;

  public Shooter(GenericFlywheelIO io, TurretIO turretIO) {
    flywheel = new GenericFlywheel(io, this, "Flywheel 1");
    turret =
        new Turret(
            turretIO,
            this,
            0,
            V0_FunkyRobotState::getGlobalPose,
            ShooterConstants.TURRET_CONSTANTS);
  }

  @Override
  public void periodic() {
    flywheel.periodic();
    turret.periodic();
  }

  public void setVoltage(double volts) {
    flywheel.setVoltage(volts);
  }

  public Command setTurretVoltage(double volts) {
    return turret.setTurretVoltage(volts);
  }

  public Command setTurretGoal(Rotation2d goal) {
    return turret.setTurretGoal(goal);
  }

  public Command runTurretSysID() {
    return turret.runSysId();
  }
}
