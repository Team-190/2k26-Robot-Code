package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import java.util.function.Consumer;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record AngularVelocityConstraints(
    LoggedTunableMeasure<AngularVelocityUnit> goalTolerance,
    LoggedTunableMeasure<AngularVelocityUnit> maxVelocity,
    LoggedTunableMeasure<AngularAccelerationUnit> maxAcceleration)
    implements Constraints<AngularVelocityConstraints> {

  @Builder(
      setterPrefix = "with",
      builderClassName = "FromMeasures",
      builderMethodName = "fromMeasures")
  public AngularVelocityConstraints(
      @NonNull String prefix,
      Measure<AngularVelocityUnit> goalTolerance,
      Measure<AngularVelocityUnit> maxVelocity,
      Measure<AngularAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(String.format("%s/Goal Tolerance", prefix), goalTolerance),
        new LoggedTunableMeasure<>(String.format("%s/Max Velocity", prefix), maxVelocity),
        new LoggedTunableMeasure<>(String.format("%s/Max Acceleration", prefix), maxAcceleration));
  }

  public void update(int id, Consumer<AngularVelocityConstraints> consumer) {
    LoggedTunableMeasure.ifChanged(
        id, () -> consumer.accept(this), goalTolerance, maxVelocity, maxAcceleration);
  }
}
