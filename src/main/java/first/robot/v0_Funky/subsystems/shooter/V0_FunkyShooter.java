package first.robot.v0_Funky.subsystems.shooter;

import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import first.robot.shared.subsystems.turret.Turret;
import first.robot.shared.subsystems.turret.TurretIO;
import first.robot.v0_Funky.V0_FunkyRobotState;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Voltage;

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
            V0_FunkyRobotState::getChassisSpeeds,
            V0_FunkyShooterConstants.TURRET_CONSTANTS);
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
    return this.run(() -> turret.setVoltageGoal(volts));
  }

  public Command setTurretGoal(Rotation2d goal) {
    return this.run(() -> turret.setPositionGoal(goal));
  }

  public Command setTurretGoal(Supplier<Rotation2d> goal) {
    return this.run(() -> turret.setPositionGoal(goal.get()));
  }

  public Command setTurretGoalPose(Supplier<Translation2d> goal) {
    return this.run(() -> turret.setFieldRelativeGoal(goal.get()));
  }

  public Command waitUntilAtGoal() {
    return turret.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command runTurretSysID() {
    return turret.runSysIdRoutine();
  }
}
