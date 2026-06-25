package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import lombok.Getter;

public class Setpoint<U extends Unit> {
  @Getter private Measure<U> setpoint;
  @Getter private Measure<U> newSetpoint;
  @Getter private Measure<U> offset;
  private final Measure<U> step;
  public Measure<U> min;
  public Measure<U> max;

  public Setpoint(Measure<U> setpoint, Measure<U> step, Measure<U> min, Measure<U> max) {
    this.setpoint = setpoint;
    this.offset = setpoint.times(0);
    this.step = step;
    this.min = min.minus(setpoint);
    this.max = max.minus(setpoint);
    this.newSetpoint = setpoint;
  }

  public Setpoint(Measure<U> setpoint, Measure<U> step) {
    this.setpoint = setpoint;
    this.offset = setpoint.times(0);
    this.step = step;
    this.min = step.times(Double.NEGATIVE_INFINITY);
    this.max = step.times(Double.POSITIVE_INFINITY);
    this.newSetpoint = setpoint;
  }

  public void setSetpoint(Measure<U> setpoint) {
    this.min = min.plus(this.setpoint).minus(setpoint);
    this.max = max.plus(this.setpoint).minus(setpoint);
    this.setpoint = setpoint;

    //    offset = clamp(offset);
    newSetpoint = calculateSetpoint();
  }

  @Trace
  private Measure<U> calculateSetpoint() {
    double sign = Math.signum(setpoint.magnitude());

    // Apply offset in the direction of the setpoint's sign
    // positive offset always increases magnitude, negative always decreases
    var newMagnitude = setpoint.plus(offset.times(sign));

    // Clamp to zero if we've crossed zero (don't flip sign)
    if (Math.signum(newMagnitude.magnitude()) != sign && sign != 0) {
      return setpoint.times(0); // return zero in same unit
    }

    return newMagnitude;
  }

  public void increment() {
    increment(step);
  }

  public void decrement() {
    decrement(step);
  }

  public void increment(Measure<U> step) {
    Measure<U> next = offset.plus(step);
    if (next.gt(max)) {
      offset = max;
    } else {
      offset = next;
    }
    newSetpoint = calculateSetpoint();
  }

  public void decrement(Measure<U> step) {
    Measure<U> next = offset.minus(step);
    if (next.lt(min)) {
      offset = min;
    } else {
      offset = next;
    }
    newSetpoint = calculateSetpoint();
  }

  public void reset() {
    offset = offset.times(0);
    newSetpoint = setpoint;
  }
}
