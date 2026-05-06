package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerIO {
  @AutoLog
  public static class GenericRollerIOInputs {
    public Rotation2d position = new Rotation2d();
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
