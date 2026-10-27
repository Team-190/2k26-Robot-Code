package edu.wpi.team190.gompeilib.core.utility;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class ExtensionMethods {
  @SuppressWarnings("unchecked")
  public static <M extends Measure<?>> M abs(M measure) {
    return measure.baseUnitMagnitude() < 0 ? (M) measure.unaryMinus() : measure;
  }

  public static <U extends Unit> boolean matchesSetpoint(Measure<U> measure, Setpoint<U> setpoint) {
    return measure.isEquivalent(setpoint.getSetpoint());
  }
}
