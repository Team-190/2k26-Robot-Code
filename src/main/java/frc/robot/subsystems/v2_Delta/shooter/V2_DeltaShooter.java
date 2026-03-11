package frc.robot.subsystems.v2_Delta.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import java.util.function.DoubleSupplier;

public class V2_DeltaShooter extends SubsystemBase {

  private final Hood hood;

  private final GenericFlywheel flywheel;

  private final Turret turret;

  public V2_DeltaShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO, TurretIO turretIO) {
    setName("Shooter");

    flywheel = new GenericFlywheel(flywheelIO, this, V2_DeltaShooterConstants.SHOOT_CONSTANTS, "");
    hood =
        new Hood(
            hoodIO,
            V2_DeltaShooterConstants.HOOD_CONSTANTS,
            this,
            "",
            V1_DoomSpiralRobotState::getScoreAngle,
            V1_DoomSpiralRobotState::getFeedAngle);
    turret =
        new Turret(
            turretIO,
            this,
            "",
            V1_DoomSpiralRobotState::getGlobalPose,
            V2_DeltaShooterConstants.TURRET_CONSTANTS);
  }

  @Override
  public void periodic() {
    hood.periodic();
    flywheel.periodic();
    turret.periodic();

    //    if (hood.getHoodGoal().equals(HoodGoal.SCORE) || hood.getHoodGoal().equals(HoodGoal.FEED))
    // {
    //      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(true);
    //      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(atGoal());
    //    } else {
    //      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(false);
    //      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(false);
    //    }
  }

  public Command setHoodGoal(HoodGoal goal) {
    return Commands.runOnce(() -> hood.setGoal(goal));
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond);
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond, double feedforward) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond, feedforward);
  }

  public Command setFlywheelGoal(DoubleSupplier velocityRadiansPerSecond) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond);
  }

  public Command setFlywheelGoal(
      DoubleSupplier velocityRadiansPerSecond, DoubleSupplier feedforward) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond, feedforward);
  }

  public Command setHoodGoal(Rotation2d goal) {
    return Commands.runOnce(
        () -> {
          hood.setGoal(HoodGoal.OVERRIDE);
          hood.setOverridePosition(goal);
        });
  }

  public Command setTurretGoal(Rotation2d goal) {
    return Commands.runOnce(() -> turret.setGoal(goal));
  }

  public Command setTurretGoal(Translation2d goal) {
    return Commands.runOnce(() -> turret.setFieldRelativeGoal(goal));
  }

  public Command setShooterGoal(
      DoubleSupplier velocityRadiansPerSecond,
      DoubleSupplier feedforward,
      Rotation2d hoodGoal,
      Translation2d turretGoal) {
    return Commands.parallel(
        setHoodGoal(hoodGoal),
        setFlywheelGoal(velocityRadiansPerSecond, feedforward),
        setTurretGoal(turretGoal));
  }

  public Command setShooterGoal(
      DoubleSupplier velocityRadiansPerSecond,
      DoubleSupplier feedforward,
      HoodGoal hoodGoal,
      Translation2d turretGoal) {
    return Commands.parallel(
        setHoodGoal(hoodGoal),
        setFlywheelGoal(velocityRadiansPerSecond, feedforward),
        setTurretGoal(turretGoal));
  }

  public boolean atGoal() {
    return flywheel.atGoal() && hood.atPositionGoal() && turret.atPositionGoal();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  public Command runTurretSysId() {
    return turret.runSysId();
  }

  public Command hoodSysId() {
    return hood.runSysId();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineTorque();
  }
}
