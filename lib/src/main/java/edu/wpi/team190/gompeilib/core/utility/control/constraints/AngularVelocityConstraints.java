package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.team190.gompeilib.core.utility.tunable.GompeiTunables;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import lombok.Builder;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Tunable angular velocity constraints, published under {@code prefix} (for example {@code
 * /Tunables/<prefix>/Goal Tolerance}). Constraints built without a prefix are not published. Unset
 * values default to zero.
 */
public final class AngularVelocityConstraints implements Constraints<AngularVelocityConstraints> {
  private final Tunable<AngularVelocity> goalTolerance;
  private final Tunable<AngularVelocity> maxVelocity;
  private final Tunable<AngularAcceleration> maxAcceleration;

  private final List<Consumer<AngularVelocityConstraints>> listeners = new ArrayList<>();
  private List<Double> lastNotifiedValues;

  @Builder(setterPrefix = "with")
  public AngularVelocityConstraints(
      String prefix,
      AngularVelocity goalTolerance,
      AngularVelocity maxVelocity,
      AngularAcceleration maxAcceleration) {
    TunableTable table = GompeiTunables.table(prefix);
    Runnable onTune = this::notifyListeners;
    this.goalTolerance =
        GompeiTunables.value(
            table,
            "Goal Tolerance",
            Objects.requireNonNullElse(goalTolerance, RadiansPerSecond.of(0)),
            onTune);
    this.maxVelocity =
        GompeiTunables.value(
            table,
            "Max Velocity",
            Objects.requireNonNullElse(maxVelocity, RadiansPerSecond.of(0)),
            onTune);
    this.maxAcceleration =
        GompeiTunables.value(
            table,
            "Max Acceleration",
            Objects.requireNonNullElse(maxAcceleration, RadiansPerSecondPerSecond.of(0)),
            onTune);
    lastNotifiedValues = values();
  }

  public Tunable<AngularVelocity> goalTolerance() {
    return goalTolerance;
  }

  public Tunable<AngularVelocity> maxVelocity() {
    return maxVelocity;
  }

  public Tunable<AngularAcceleration> maxAcceleration() {
    return maxAcceleration;
  }

  public double getGoalTolerance(AngularVelocityUnit unit) {
    return goalTolerance.get().in(unit);
  }

  public double getMaxVelocity(AngularVelocityUnit unit) {
    return maxVelocity.get().in(unit);
  }

  public double getMaxAcceleration(AngularAccelerationUnit unit) {
    return maxAcceleration.get().in(unit);
  }

  @Override
  public void onChange(Consumer<AngularVelocityConstraints> listener) {
    listeners.add(listener);
  }

  private List<Double> values() {
    return List.of(
        goalTolerance.get().baseUnitMagnitude(),
        maxVelocity.get().baseUnitMagnitude(),
        maxAcceleration.get().baseUnitMagnitude());
  }

  private void notifyListeners() {
    // Every edited constraint triggers this; only notify once per distinct set of values.
    List<Double> values = values();
    if (values.equals(lastNotifiedValues)) {
      return;
    }
    lastNotifiedValues = values;
    listeners.forEach(listener -> listener.accept(this));
  }
}
