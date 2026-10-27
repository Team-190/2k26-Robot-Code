package edu.wpi.team190.gompeilib.subsystems.generic.hood;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/** Interface for 's hood subsystem. */
public interface GenericHoodIO {

  /**
   * Inputs for hood subsystem. Positions and velocities are of the output shaft, not the motor
   * shaft.
   */
  @AutoLog
  class GenericHoodIOInputs {
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
  default void updateInputs(GenericHoodIOInputs inputs) {}

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
