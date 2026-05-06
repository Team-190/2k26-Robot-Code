package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import edu.wpi.first.units.*;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import java.util.function.Consumer;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record LinearConstraints(
    LoggedTunableMeasure<DistanceUnit> goalTolerance,
    LoggedTunableMeasure<LinearVelocityUnit> maxVelocity,
    LoggedTunableMeasure<LinearAccelerationUnit> maxAcceleration)
    implements Constraints<LinearConstraints>, Constraints.PositionConstraints<LinearConstraints> {

  @Builder(
      setterPrefix = "with",
      builderMethodName = "fromMeasures",
      builderClassName = "FromMeasures")
  public LinearConstraints(
      @NonNull String prefix,
      Measure<DistanceUnit> goalTolerance,
      Measure<LinearVelocityUnit> maxVelocity,
      Measure<LinearAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(String.format("%s/Goal Tolerance", prefix), goalTolerance),
        new LoggedTunableMeasure<>(String.format("%s/Max Velocity", prefix), maxVelocity),
        new LoggedTunableMeasure<>(String.format("%s/Max Acceleration", prefix), maxAcceleration));
  }

  public double getGoalToleranceMeters(DistanceUnit unit) {
    return goalTolerance.get(unit);
  }

  public double getMaxVelocityMetersPerSecond(LinearVelocityUnit unit) {
    return maxVelocity.get(unit);
  }

  public double getMaxAccelerationMetersPerSecondSquared(LinearAccelerationUnit unit) {
    return maxAcceleration.get(unit);
  }

  public void update(int id, Consumer<LinearConstraints> consumer) {
    LoggedTunableMeasure.ifChanged(
        id, () -> consumer.accept(this), goalTolerance, maxVelocity, maxAcceleration);
  }
}
