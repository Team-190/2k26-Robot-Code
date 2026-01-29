package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface LinkageIO {
  @AutoLog
  public static class LinkageIOInputs {
    public Rotation2d leftPosition = new Rotation2d();
    public AngularVelocity leftVelocity = RadiansPerSecond.zero();
    public Voltage leftAppliedVolts = Volts.zero();
    public Current leftSupplyCurrent = Amps.zero();
    public Current leftTorqueCurrent = Amps.zero();
    public Temperature leftTemperature = Celsius.zero();
    public Rotation2d leftPositionGoal = new Rotation2d();
    public Rotation2d leftPositionSetpoint = new Rotation2d();
    public Rotation2d leftPositionError = new Rotation2d();

    public Rotation2d rightPosition = new Rotation2d();
    public AngularVelocity rightVelocity = RadiansPerSecond.zero();
    public Voltage rightAppliedVolts = Volts.zero();
    public Current rightSupplyCurrent = Amps.zero();
    public Current rightTorqueCurrent = Amps.zero();
    public Temperature rightTemperature = Celsius.zero();
    public Rotation2d rightPositionGoal = new Rotation2d();
    public Rotation2d rightPositionSetpoint = new Rotation2d();
    public Rotation2d rightPositionError = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(LinkageIOInputs inputs) {
  }

  /** Sets left motor voltage. */
  public default void setVoltageLeft(double volts) {
  }

  /** Sets right motor voltage. */
  public default void setVoltageRight(double volts) {
  }

  /** Sets left motor closed loop position. */
  public default void setPositionLeft(Rotation2d position) {
  }

  /** Sets right motor closed loop position. */
  public default void setPositionRight(Rotation2d position) {
  }

  /** Sets left motor closed loop position setpoint. */
  public default void setPositionGoalLeft(Rotation2d position) {
  }

  /** Sets right motor closed loop position setpoint. */
  public default void setPositionGoalRight(Rotation2d position) {
  }

  /** Sets left motor PID gains. */
  public default void setPIDLeft(double kp, double ki, double kd) {
  }

  /** Sets right motor PID gains. */
  public default void setPIDRight(double kp, double ki, double kd) {
  }

  /** Sets right feedforward gains */
  public default void setFeedforwardRight(double ks, double kv, double ka) {
  }

  /** Sets left feedforward gains */
  public default void setFeedforwardLeft(double ks, double kv, double ka) {
  }

  /** Sets left motor profile. */
  public default void setProfileLeft(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
  }

  /** Sets right motor profile. */
  public default void setProfileRight(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
  }

  /** Checks if the linkage is within tolerance */
  public default boolean atGoalRight() {
    return false;
  }

  public default boolean atGoalLeft() {
    return false;
  }

}