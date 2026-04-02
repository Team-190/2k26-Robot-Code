package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Funky's hood subsystem. */
public interface HoodIO {

  /**
   * Inputs for Funky's hood subsystem. Positions and velocities are of the output shaft, not the
   * motor shaft.
   */
  @AutoLog
  class HoodIOInputs {
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
  default void updateInputs(HoodIOInputs inputs) {}

  /** Sets motor voltage. */
  default void setVoltage(Voltage volts) {}

  /** Sets motor closed loop position setpoint. */
  default void setPositionGoal(Rotation2d positionGoal) {}

  default void setPosition(Rotation2d position) {}

  default void setGains(Gains gains) {}

  default void setProfile(AngularPositionConstraints constraints) {}

  /** Checks if the hood is within tolerance */
  default boolean atPositionGoal(Rotation2d positionReference) {
    return false;
  }

  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }
}
