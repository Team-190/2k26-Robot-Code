package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
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
        new GenericFlywheel(flywheelIO, this, V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS, "");
    flywheel.getVelocityGoalRadiansPerSecond().increment(RadiansPerSecond.of(20.0));
    hood =
        new Hood(
            hoodIO,
            V1_DoomSpiralShooterConstants.HOOD_CONSTANTS,
            this,
            "",
            V1_DoomSpiralRobotState::getScoreAngle,
            V1_DoomSpiralRobotState::getFeedAngle);
  }

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          flywheel.setPID(
              GainSlot.ZERO,
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kP().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kD().get());

          flywheel.setFeedForward(
              GainSlot.ZERO,
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kS().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kV().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kA().get());
        },
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kP(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kD(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kS(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kV(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.voltageGains.kA());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          flywheel.setPID(
              GainSlot.ONE,
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kP().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kD().get());

          flywheel.setFeedForward(
              GainSlot.ONE,
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kS().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kV().get(),
              V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kA().get());
        },
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kP(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kD(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kS(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kV(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.torqueGains.kA());

    LoggedTunableMeasure.ifChanged(
        hashCode(),
        () ->
            flywheel.setProfile(
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .maxAcceleration()
                    .get()
                    .in(RadiansPerSecondPerSecond),
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .maxVelocity()
                    .get()
                    .in(RadiansPerSecond),
                V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS
                    .constraints
                    .goalTolerance()
                    .get()
                    .in(Radians)),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints.maxAcceleration(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints.maxVelocity(),
        V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS.constraints.goalTolerance());

    hood.periodic();
    flywheel.periodic();

    if (hood.getPositionGoal().equals(HoodGoal.SCORE)
        || hood.getPositionGoal().equals(HoodGoal.FEED)) {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(true);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(atGoal());
    } else {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(false);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(false);
    }
  }

  public Command setHoodGoal(HoodGoal goal) {
    return hood.setGoal(goal);
  }

  public Command setOverrideHoodGoal(Rotation2d position) {
    return Commands.runOnce(() -> hood.setOverridePosition(position))
        .andThen(hood.setGoal(HoodGoal.OVERRIDE));
  }

  public Command incrementHoodAngle() {
    return hood.incrementOffset();
  }

  public Command decrementHoodAngle() {
    return hood.decrementOffset();
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

  public Command setFlywheelGoal(double velocityRadiansPerSecond) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond);
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond, double feedforward) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond, feedforward);
  }

  public Command incrementFlywheelVelocity() {
    return flywheel.incrementVelocityOffset();
  }

  public Command decrementFlywheelVelocity() {
    return flywheel.decrementVelocityOffset();
  }

  public Command setFlywheelVoltage(double volts) {
    return flywheel.setVoltage(volts);
  }

  public Command setFlywheelVelocity(double velocityRadiansPerSecond) {
    return flywheel.setVelocityGoal(velocityRadiansPerSecond);
  }

  public Command stopFlywheel() {
    return flywheel.stop();
  }

  public Command setGoal(
      HoodGoal hoodGoal, double velocityRadiansPerSecond) { // TODO: Figure out why it doesnt work
    return Commands.parallel(setHoodGoal(hoodGoal), setFlywheelGoal(velocityRadiansPerSecond));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), flywheel.setVelocityGoal(velocityRadiansPerSecond));
  }

  public Command setGoal(
      HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond, DoubleSupplier feedforward) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), flywheel.setVelocityGoal(velocityRadiansPerSecond, feedforward));
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
    return flywheel.sysIdRoutineTorque();
  }
}
