package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import java.util.function.DoubleSupplier;

public class V1_DoomSpiralShooter extends SubsystemBase {

  private final Hood hood;

  private HoodGoal hoodGoal;

  private final GenericFlywheel flywheel;

  public V1_DoomSpiralShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO) {
    setName("Shooter");

    flywheel =
        new GenericFlywheel(flywheelIO, this, V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V1_DoomSpiralShooterConstants.HOOD_CONSTANTS, this, "");

    hoodGoal = HoodGoal.STOW;
  }

  @Override
  public void periodic() {
    hood.periodic();
    flywheel.periodic();

    if (hoodGoal.equals(HoodGoal.SCORE) || hoodGoal.equals(HoodGoal.FEED)) {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(true);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(atGoal());
    } else {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(false);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(false);
    }
  }

  public Command setHoodGoal(HoodGoal goal) {
    return Commands.runEnd(
        () -> {
          hood.setPositionGoal(getHoodGoal(goal));
          hoodGoal = goal;
        },
        () -> hood.setPositionGoal(Rotation2d.kZero));
  }

  private Rotation2d getHoodGoal(HoodGoal goal) {
    return switch (hoodGoal) {
      case SCORE -> V1_DoomSpiralRobotState.getScoreAngle();
      case FEED -> V1_DoomSpiralRobotState.getFeedAngle();
      default -> Rotation2d.kZero;
    };
  }

  public Command setOverrideHoodGoal(Rotation2d position) {
    return Commands.runOnce(() -> hood.setPositionGoal(position))
        .andThen(Commands.runOnce(() -> hoodGoal = HoodGoal.OVERRIDE));
  }

  public Command setHoodVoltage(double volts) {
    return Commands.runOnce(() -> hood.setVoltageGoal(Volts.of(volts)));
  }

  public Command stopHood() {
    return Commands.runOnce(() -> hood.setVoltageGoal(Volts.zero()));
  }

  public Command zeroHood() {
    return hood.resetHoodZero();
  }

  public Command setFlywheelGoal(AngularVelocity velocityGoal) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal));
  }

  public Command setFlywheelGoal(AngularVelocity velocityGoal, Current feedforward) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal, feedforward));
  }

  public Command setFlywheelVoltage(double volts) {
    return Commands.runOnce(() -> flywheel.setVoltageGoal(Volts.of(volts)));
  }

  public Command stopFlywheel() {
    return Commands.runOnce(flywheel::stop);
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), setFlywheelGoal(RadiansPerSecond.of(velocityRadiansPerSecond)));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
    return Commands.parallel(
        setHoodGoal(hoodGoal),
        Commands.run(
            () ->
                flywheel.setVelocityGoal(
                    RadiansPerSecond.of(velocityRadiansPerSecond.getAsDouble()))));
  }

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atVelocityGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
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

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::decrement);
  }
}
