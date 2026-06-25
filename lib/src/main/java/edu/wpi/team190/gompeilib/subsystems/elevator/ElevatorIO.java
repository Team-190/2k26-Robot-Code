package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public Distance position = Meters.of(0.0);
    public LinearVelocity velocity = MetersPerSecond.of(0.0);
    public LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public Distance positionGoalMeters = Meters.of(0.0);
    public Distance positionSetpointMeters = Meters.of(0.0);
    public Distance positionErrorMeters = Meters.of(0.0);

    public GainSlot gainSlot;
  }

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs to update.
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param voltageGoal The voltage of the elevator in volts.
   */
  default void setVoltageGoal(Voltage voltageGoal) {}

  /**
   * Sets the position goal of the elevator.
   *
   * @param positionGoal The position goal of the elevator in meters.
   */
  default void setPositionGoal(Distance positionGoal) {}

  /**
   * Checks if the voltage of the elevator matches the volts argument
   *
   * @param voltageReference The voltage to check against
   * @return True if the voltage matches, false otherwise
   */
  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }

  /**
   * Checks if the position of the subsystem matches the positionGoal argument
   *
   * @param positionReference the position to check against
   * @return True if the position matches, false otherwise
   */
  default boolean atPositionGoal(Distance positionReference) {
    return false;
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The position to set.
   */
  default void setPosition(Distance position) {}

  /**
   * @param gainSlot The CTRE gain slot to set the elevator to.
   */
  default void setGainSlot(GainSlot gainSlot) {}

  /**
   * Sets the gains for the elevator.
   *
   * @param gains the gains to update
   */
  default void updateGains(Gains gains, GainSlot gainSlot) {}

  /**
   * Sets the constraints for the elevator.
   *
   * @param constraints the constraints to update
   */
  default void updateConstraints(LinearConstraints constraints) {}
}
