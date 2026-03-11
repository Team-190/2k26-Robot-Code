package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface LinearExtensionIO {
  @AutoLog
  class LinearExtensionIOInputs {

    public double position = 0.0;
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public double positionGoal = 0.0;
    public double positionSetpoint = 0.0;
    public double positionError = 0.0;

    public Rotation2d canCoderAbsolutePosition = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  default void updateInputs(LinearExtensionIOInputs inputs) {}

  /** Sets motor voltage. */
  default void setVoltage(double volts) {}

  /** Sets motor closed loop position. */
  default void setPosition(double position) {}

  /** Sets motor closed loop position setpoint. */
  default void setPositionGoal(double position) {}

  /** Sets motor PID gains. */
  default void setPID(double kp, double ki, double kd) {}

  /** Sets feedforward gains */
  default void setFeedforward(double ks, double kv, double kg, double ka) {}

  /** Sets motor profile. */
  default void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {}

  /** Checks if the linkage is at the goal position. */
  default boolean atGoal() {
    return false;
  }
}
