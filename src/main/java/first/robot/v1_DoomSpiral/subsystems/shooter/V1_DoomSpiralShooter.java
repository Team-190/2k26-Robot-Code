package first.robot.v1_DoomSpiral.subsystems.shooter;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import first.robot.shared.subsystems.hood.Hood;
import first.robot.shared.subsystems.hood.HoodIO;
import first.robot.v1_DoomSpiral.V1_DoomSpiralRobotState;
import first.robot.v1_DoomSpiral.subsystems.shooter.V1_DoomSpiralShooterConstants.HoodGoal;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

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

    flywheel.getVelocityGoal().decrement(RadiansPerSecond.of(20));
  }

  @Trace
  public void periodic() {
    hood.periodic();
    flywheel.periodic();

    Logger.recordOutput("Shooter/Hood/Goal", hoodGoal);

    Logger.recordOutput(
        "Shooter/Hood/Goal Degrees",
        String.format("%.1f", hood.getPositionGoal().getSetpoint().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Hood/Offset Degrees",
        String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Offset",
        flywheel.getVelocityGoal().getOffset().in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Magnitude",
        (int) Math.abs(flywheel.getVelocityGoal().getSetpoint().in(RadiansPerSecond)));

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
          default -> Rotation2d.ZERO;
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
        Commands.run(
            () -> {
              this.hoodGoal = hoodGoal;
              hood.setPositionGoal(getHoodGoal(hoodGoal));
            }),
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
    return hood.runSysIdRoutine();
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
