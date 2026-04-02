package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;

public class V1_DoomSpiralShooter extends SubsystemBase {

  @Getter private final Hood hood;

  private HoodGoal hoodGoal;

  private final GenericFlywheel flywheel;

  public V1_DoomSpiralShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO) {
    setName("Shooter");

    flywheel =
        new GenericFlywheel(flywheelIO, this, V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V1_DoomSpiralShooterConstants.HOOD_CONSTANTS, this, "");

    hoodGoal = HoodGoal.STOW;

    flywheel.getVelocityGoalRadiansPerSecond().decrement(RadiansPerSecond.of(30));
  }

  @Trace
  public void periodic() {
    hood.periodic();
    flywheel.periodic();

    Logger.recordOutput("Shooter/Hood/Goal", hoodGoal);

    // Logger.recordOutput(
    //     "Shooter/Hood/Goal Degrees",
    //     String.format("%.1f", hood.getInputs().position.getDegrees()));
    // Logger.recordOutput(
    //     "Shooter/Hood/Offset Degrees",
    //     String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Offset",
        flywheel.getVelocityGoalRadiansPerSecond().getOffset().in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Magnitude",
        (int) Math.abs(flywheel.getVelocityGoalRadiansPerSecond().getSetpoint().in(RadiansPerSecond)));

    if (hoodGoal.equals(HoodGoal.SCORE) || hoodGoal.equals(HoodGoal.FEED)) {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(true);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(atGoal());
    } else {
      V1_DoomSpiralRobotState.getLedStates().setShooterPrepping(false);
      V1_DoomSpiralRobotState.getLedStates().setShooterShooting(false);
    }
  }

  public Command setHoodGoal(HoodGoal goal) {
    return Commands.runOnce(
        () -> {
          hoodGoal = goal;
          hood.setPositionGoal(getHoodGoal(goal));
        });
  }

  private Rotation2d getHoodGoal(HoodGoal goal) {
    Rotation2d rotation =
        switch (goal) {
          case SCORE -> V1_DoomSpiralRobotState.getScoreAngle();
          case FEED -> V1_DoomSpiralRobotState.getFeedAngle();
          default -> Rotation2d.kZero;
        };
    Logger.recordOutput("Shooter/Hood/DebugGoal", rotation);
    Logger.recordOutput("Shooter/Hood/debugGoal1", goal);
    return rotation;
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

  public Command setFlywheelGoal(DoubleSupplier velocityGoal) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal));
  }

  public Command setFlywheelGoal(double velocityGoal, double feedforward) {
    return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal, feedforward));
  }

  public Command setFlywheelVoltage(double volts) {
    return Commands.runOnce(() -> flywheel.setVoltage(volts));
  }

  public Command stopFlywheel() {
    return Commands.runOnce(flywheel::stop);
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond, double feedforward) {
    return Commands.parallel(
        setHoodGoal(hoodGoal), setFlywheelGoal(velocityRadiansPerSecond, feedforward));
  }

  public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond, double feedforward) {
    return Commands.parallel(
        Commands.run(
            () -> {
              this.hoodGoal = hoodGoal;
              hood.setPositionGoal(getHoodGoal(hoodGoal));
            }),
        Commands.run(
            () ->
                flywheel.setVelocityGoal(
                    velocityRadiansPerSecond.getAsDouble(), feedforward)));
  }

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command hoodSysId() {
    return hood.runSysIdRoutine();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineTorque();
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoalRadiansPerSecond()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoalRadiansPerSecond()::decrement);
  }

  // public Command incrementHoodAngle() {
  //   return Commands.runOnce(hood.getPositionGoal()::increment);
  // }

  // public Command decrementHoodAngle() {
  //   return Commands.runOnce(hood.getPositionGoal()::decrement);
  // }
}
