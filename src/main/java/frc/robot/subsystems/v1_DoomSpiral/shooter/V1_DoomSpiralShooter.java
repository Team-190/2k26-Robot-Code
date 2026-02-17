package frc.robot.subsystems.v1_DoomSpiral.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
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
    setName("Shooter");

    flywheel =
        new GenericFlywheel(
            flywheelIO, this, V1_DoomSpiralRobotState.getShooterOffsets()::getFlywheel, "");
    hood =
        new Hood(
            hoodIO,
            V1_DoomSpiralShooterConstants.HOOD_CONSTANTS,
            this,
            "",
            V1_DoomSpiralRobotState::getScoreAngle,
            V1_DoomSpiralRobotState::getFeedAngle,
            V1_DoomSpiralRobotState.getShooterOffsets()::getHood);
  }

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          flywheel.setPID(
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kP().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kD().get());

          flywheel.setFeedForward(
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kS().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kV().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kA().get());
        },
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kP(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kD(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kS(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kV(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.gains.kA());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            flywheel.setProfile(
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .maxAccelerationRadiansPerSecondSquared()
                    .get(),
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .cruisingVelocityRadiansPerSecond()
                    .get(),
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .goalToleranceRadiansPerSecond()
                    .get()),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints
            .maxAccelerationRadiansPerSecondSquared(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints
            .cruisingVelocityRadiansPerSecond(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints.goalToleranceRadiansPerSecond());

    hood.periodic();
    flywheel.periodic();
  }

  public Command setHoodGoal(HoodGoal goal) {
    return hood.setGoal(goal);
  }

  public Command setOverrideHoodGoal(Rotation2d position) {
    return Commands.runOnce(() -> hood.setOverridePosition(position))
        .andThen(hood.setGoal(HoodGoal.OVERRIDE));
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.getShooterOffsets()
                .setHood(
                    (V1_DoomSpiralRobotState.getShooterOffsets().getHood())
                        .plus(V1_DoomSpiralShooterConstants.HOOD_ANGLE_INCREMENT_ROTATIONS)));
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.getShooterOffsets()
                .setHood(
                    (V1_DoomSpiralRobotState.getShooterOffsets().getHood())
                        .minus(V1_DoomSpiralShooterConstants.HOOD_ANGLE_INCREMENT_ROTATIONS)));
  }

  public Command setHoodVoltage(double volts) {
    return hood.setVoltage(volts);
  }

  public Command stopHood() {
    return hood.setVoltage(0);
  }

  public Command zeroHood() {
    return hood.resetHoodZero();
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond, boolean useTorqueControl) {
    return flywheel.setGoal(velocityRadiansPerSecond, useTorqueControl);
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.getShooterOffsets()
                .setFlywheel(
                    V1_DoomSpiralRobotState.getShooterOffsets().getFlywheel()
                        + V1_DoomSpiralShooterConstants.FLYWHEEL_VELOCITY_INCREMENT_RPS));
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(
        () ->
            V1_DoomSpiralRobotState.getShooterOffsets()
                .setFlywheel(
                    V1_DoomSpiralRobotState.getShooterOffsets().getFlywheel()
                        - V1_DoomSpiralShooterConstants.FLYWHEEL_VELOCITY_INCREMENT_RPS));
  }

  public Command setFlywheelVoltage(double volts) {
    return flywheel.setVoltage(volts);
  }

  public Command setFlywheelVelocity(double velocityRadiansPerSecond) {
    return flywheel.setGoal(velocityRadiansPerSecond, false);
  }

  public Command stopFlywheel() {
    return flywheel.stop();
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) { //TODO: Figure out why it doesnt work
    return Commands.parallel(
        setHoodGoal(hoodGoal), setFlywheelGoal(velocityRadiansPerSecond, false));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), flywheel.setGoal(velocityRadiansPerSecond, false));
  }

  public boolean atGoal() {
    return hood.atGoal() && flywheel.atGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilHoodAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command hoodSysId() {
    return hood.runSysId();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineVoltage();
  }
}
