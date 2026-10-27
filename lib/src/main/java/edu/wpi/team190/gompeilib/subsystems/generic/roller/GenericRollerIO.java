package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public interface GenericRollerIO {
  @AutoLog
  public static class GenericRollerIOInputs {
    public Angle position = Radians.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};
  }

  default void updateInputs(GenericRollerIOInputs inputs) {}

  default void setVoltageGoal(Voltage volts) {}

  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }
}
