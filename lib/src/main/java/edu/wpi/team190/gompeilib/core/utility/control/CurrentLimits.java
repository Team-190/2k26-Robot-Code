package edu.wpi.team190.gompeilib.core.utility.control;

import lombok.Builder;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Current;

@Builder(setterPrefix = "with")
public record CurrentLimits(Current supplyCurrentLimit, Current statorCurrentLimit) {
  @Builder(
      setterPrefix = "with",
      builderClassName = "FromDoubles",
      builderMethodName = "fromDoubles")
  public CurrentLimits(double supplyCurrentLimit, double statorCurrentLimit) {
    this(
        Current.ofBaseUnits(supplyCurrentLimit, Units.Amps),
        Current.ofBaseUnits(statorCurrentLimit, Units.Amps));
  }
}
