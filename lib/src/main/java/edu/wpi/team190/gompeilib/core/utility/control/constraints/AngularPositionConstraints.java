package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import edu.wpi.first.units.*;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import java.util.function.Consumer;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record AngularPositionConstraints(
    LoggedTunableMeasure<AngleUnit> goalTolerance,
    LoggedTunableMeasure<AngularVelocityUnit> maxVelocity,
    LoggedTunableMeasure<AngularAccelerationUnit> maxAcceleration)
    implements Constraints<AngularPositionConstraints>,
        Constraints.PositionConstraints<AngularPositionConstraints> {

  @Builder(
      setterPrefix = "with",
      builderClassName = "FromMeasures",
      builderMethodName = "fromMeasures")
  public AngularPositionConstraints(
      @NonNull String prefix,
      Measure<AngleUnit> goalTolerance,
      Measure<AngularVelocityUnit> maxVelocity,
      Measure<AngularAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(String.format("%s/Goal Tolerance", prefix), goalTolerance),
        new LoggedTunableMeasure<>(String.format("%s/Max Velocity", prefix), maxVelocity),
        new LoggedTunableMeasure<>(String.format("%s/Max Acceleration", prefix), maxAcceleration));
  }

  public double getGoalTolerance(AngleUnit unit) {
    return goalTolerance.get(unit);
  }

  public double getMaxVelocity(AngularVelocityUnit unit) {
    return maxVelocity.get(unit);
  }

  public double getMaxAcceleration(AngularAccelerationUnit unit) {
    return maxAcceleration.get(unit);
  }

  public void update(int id, Consumer<AngularPositionConstraints> consumer) {
    LoggedTunableMeasure.ifChanged(
        id, () -> consumer.accept(this), goalTolerance, maxVelocity, maxAcceleration);
  }
}
