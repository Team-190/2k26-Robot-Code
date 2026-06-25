package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();

    public GainSlot gainSlot = GainSlot.ZERO;
  }

  /**
   * Updates the inputs for the arm.
   *
   * @param inputs The inputs to update.
   */
  default void updateInputs(ArmIOInputs inputs) {}

  /**
   * Sets the voltage for the arm.
   *
   * @param voltageGoal The voltage of the arm in volts
   */
  default void setVoltageGoal(Voltage voltageGoal) {}

  /**
   * Sets the position goal of the arm
   *
   * @param positionGoal the position goal of the arm
   */
  default void setPositionGoal(Rotation2d positionGoal) {}

  /**
   * Checks if the voltage of the arm matches the volts argument
   *
   * @param voltageReference The voltage to check against
   * @return True if the voltage matches, false otherwise
   */
  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }

  /**
   * Checks if the position of the arm matches the positionGoal argument
   *
   * @param positionReference the position to check against
   * @return True if the position matches, false otherwise
   */
  default boolean atPositionGoal(Rotation2d positionReference) {
    return false;
  }

  /**
   * Sets the position of the arm.
   *
   * @param position The position to set.
   */
  default void setPosition(Rotation2d position) {}

  /**
   * @param gainSlot The CTRE gain slot to set the arm to.
   */
  default void setGainSlot(GainSlot gainSlot) {}

  /**
   * Sets the gains for the arm.
   *
   * @param gains the gains to update
   */
  default void updateGains(Gains gains, GainSlot gainSlot) {}

  /**
   * Sets the constraints for the arm.
   *
   * @param constraints the constraints to update
   */
  default void updateConstraints(AngularPositionConstraints constraints) {}
}
