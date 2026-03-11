package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearConstraints;

import org.littletonrobotics.junction.AutoLog;

public interface LinearExtensionIO {
  @AutoLog
  class LinearExtensionIOInputs {

    public Distance position;
    public LinearVelocity velocity = MetersPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Distance positionGoal;
    public Distance positionSetpoint;
    public Distance positionError;

    public Rotation2d canCoderAbsolutePosition = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  default void updateInputs(LinearExtensionIOInputs inputs) {}

  /** Sets motor voltage. */
  default void setVoltage(Voltage volts) {}

  /** Sets motor closed loop position. */
  default void setPosition(Distance position) {}

  /** Sets motor closed loop position setpoint. */
  default void setPositionGoal(Distance position) {}

  /** Sets motor PID gains. */
  default void setPID(Gains gains) {}

  /** Sets motor profile. */
  default void setProfile(
      LinearConstraints constraints) {}

  default void updateGains(Gains gains) {}

  default void updateConstraints(double maxAcceleration, double maxVelocity) {}

  default void resetExtension() {}

  /** Checks if the linkage is at the goal position. */
  default boolean atGoal() {
    return false;
  }
}
