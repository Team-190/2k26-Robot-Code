package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface LinkageIO {
  @AutoLog
  public static class LinkageIOInputs {

    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(LinkageIOInputs inputs) {
  }

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {
  }

  /** Sets motor closed loop position. */
  public default void setPosition(Rotation2d position) {
  }

  /** Sets motor closed loop position setpoint. */
  public default void setPositionGoal(Rotation2d position) {
  }

  /** Sets motor PID gains. */
  public default void setPID(double kp, double ki, double kd) {
  }

  /** Sets feedforward gains */
  public default void setFeedforward(double ks, double kv, double ka) {
  }

  /** Sets motor profile. */
  public default void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
  }

  public default boolean atGoal() {
    return false;
  }

}