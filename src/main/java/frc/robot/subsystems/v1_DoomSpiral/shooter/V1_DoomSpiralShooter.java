package frc.robot.subsystems.v1_DoomSpiral.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import java.util.function.DoubleSupplier;

public class V1_DoomSpiralShooter extends SubsystemBase {

  private final Hood hood;

  private final GenericFlywheel flywheel;

  public V1_DoomSpiralShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO) {

    flywheel = new GenericFlywheel(flywheelIO, this, "1");
    hood =
        new Hood(
            hoodIO,
            this,
            1,
            V1_DoomSpiralRobotState::getScoreAngle,
            V1_DoomSpiralRobotState::getFeedAngle);
  }

  @Override
  public void periodic() {
    hood.periodic();
    flywheel.periodic();
  }

  public Command setHoodGoal(HoodGoal goal) {
    return hood.setGoal(goal);
  }

  public Command setOverrideHoodGoal(Rotation2d position) {
    return Commands.runOnce(
            () ->
                hood.setOverridePosition(
                    position.plus(
                        V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getHoodAngleOffsetRotations())))
        .andThen(hood.setGoal(HoodGoal.OVERRIDE));
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.SHOOTER_OFFSETS.setHoodAngleOffsetRotations(
                (V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getHoodAngleOffsetRotations())
                    .plus(V1_DoomSpiralShooterConstants.hoodAngleIncrementRotations)));
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.SHOOTER_OFFSETS.setHoodAngleOffsetRotations(
                (V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getHoodAngleOffsetRotations())
                    .minus(V1_DoomSpiralShooterConstants.hoodAngleIncrementRotations)));
  }

  public Command setHoodVoltage(double volts) {
    return hood.setVoltage(volts);
  }

  public Command stopHood() {
    return hood.setVoltage(0);
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond, boolean useTorqueControl) {
    return flywheel.setGoal(
        velocityRadiansPerSecond
            + V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getFlywheelVelocityOffsetRPS(),
        useTorqueControl);
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.SHOOTER_OFFSETS.setFlywheelVelocityOffsetRPS(
                V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getFlywheelVelocityOffsetRPS()
                    + V1_DoomSpiralShooterConstants.flywheelVelocityIncrementRPS));
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.SHOOTER_OFFSETS.setFlywheelVelocityOffsetRPS(
                V1_DoomSpiralRobotState.SHOOTER_OFFSETS.getFlywheelVelocityOffsetRPS()
                    - V1_DoomSpiralShooterConstants.flywheelVelocityIncrementRPS));
  }

  public Command setFlywheelVoltage(double volts) {
    return flywheel.setVoltage(volts);
  }

  public Command stopFlywheel() {
    return flywheel.setVoltage(0);
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), setFlywheelGoal(velocityRadiansPerSecond, true));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), flywheel.setGoal(velocityRadiansPerSecond.getAsDouble(), true));
  }

  public boolean atGoal() {
    return hood.atGoal() && flywheel.atGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilHoodAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }
}
