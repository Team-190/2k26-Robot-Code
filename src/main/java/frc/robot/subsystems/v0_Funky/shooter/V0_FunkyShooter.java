package frc.robot.subsystems.v0_Funky.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurret;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurretIO;
import frc.robot.util.InternalLoggedTracer;

public class V0_FunkyShooter extends SubsystemBase {

  private final V0_FunkyTurret turret;

  public V0_FunkyShooter(V0_FunkyTurretIO io, int index) {
    turret = new V0_FunkyTurret(io, this, index);
  }

  @Override
  public void periodic() {
    InternalLoggedTracer.reset();
    turret.periodic();
    InternalLoggedTracer.record("Shooter Periodic", "Intake Periodic");
  }

  public Command setVoltageCommand(double voltage) {
    return turret.setTurretVoltage(voltage);
  }

  public void setTurretGoal(Rotation2d turretGoal) {
    turret.setTurretGoal(turretGoal);
  }
}
