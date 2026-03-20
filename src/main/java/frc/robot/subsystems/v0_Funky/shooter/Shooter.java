package frc.robot.subsystems.v0_Funky.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private final GenericFlywheel flywheel;
  private final Turret turret;

  public Shooter(GenericFlywheelIO io, TurretIO turretIO) {
    flywheel = new GenericFlywheel(io, this, ShooterConstants.SHOOT_CONSTANTS, "Flywheel 1");
    turret =
        new Turret(
            turretIO,
            this,
            "",
            V0_FunkyRobotState::getGlobalPose,
            V0_FunkyRobotState::getChassisSpeeds,
            ShooterConstants.TURRET_CONSTANTS);
  }

  @Override
  public void periodic() {
    flywheel.periodic();
    turret.periodic();
  }

  public void setVoltage(double volts) {
    flywheel.setVoltageGoal(Volts.of(volts));
  }

  public Command setTurretVoltage(Voltage volts) {
    return Commands.run(() -> turret.setVoltageGoal(volts));
  }

  public Command setTurretGoal(Rotation2d goal) {
    return Commands.runOnce(() -> turret.setPositionGoal(goal));
  }

  public Command setTurretGoal(Supplier<Rotation2d> goal) {
    return Commands.run(() -> turret.setPositionGoal(goal.get()));
  }

  public Command setTurretGoalPose(Supplier<Translation2d> goal) {
    return Commands.run(() -> turret.setFieldRelativeGoal(goal.get()));
  }

  public Command waitUntilAtGoal() {
    return turret.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command runTurretSysID() {
    return turret.runSysId();
  }
}
