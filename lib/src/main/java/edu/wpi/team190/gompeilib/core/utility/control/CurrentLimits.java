package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import lombok.Builder;

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
