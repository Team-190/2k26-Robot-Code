package edu.wpi.team190.gompeilib.core.utility.tunable;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.Constraints;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Consumer;

/**
 * Central registry for tunable objects that need to be polled for runtime updates.
 *
 * <p>Each registry maps a tunable object or group of tunable objects to the action that should run
 * when a value changes. The {@link #periodic()} method should be called regularly to detect changes
 * and invoke the corresponding update callbacks.
 */
public class TunableUpdaterRegistry {

  private TunableUpdaterRegistry() {}

  /** Registered gain sets and their update callbacks. */
  private static final HashMap<Gains, Consumer<Gains>> GAINS_UPDATER = new HashMap<>();

  /** Registered constraint sets and their update callbacks. */
  private static final HashMap<Constraints<?>, Consumer> CONSTRAINTS_UPDATER = new HashMap<>();

  /** Registered tunable number groups and their update callbacks. */
  private static final HashMap<LoggedTunableNumber[], Consumer<double[]>> NUMBER_UPDATER =
      new HashMap<>();

  /** Registered tunable measure groups and their update callbacks. */
  private static final HashMap<LoggedTunableMeasure<?>[], Runnable> MEASURE_UPDATER =
      new HashMap<>();

  /**
   * Checks all registered tunables for changes and runs any associated update callbacks.
   *
   * <p>This method is intended to be called periodically, once per robot loop, so runtime tuning
   * changes are detected and applied.
   */
  public static void periodic() {
    GAINS_UPDATER.forEach((k, v) -> k.update(k.hashCode(), v));
    CONSTRAINTS_UPDATER.forEach((k, v) -> k.update(k.hashCode(), v));
    NUMBER_UPDATER.forEach((k, v) -> LoggedTunableNumber.ifChanged(Arrays.hashCode(k), v, k));
    MEASURE_UPDATER.forEach((k, v) -> LoggedTunableMeasure.ifChanged(Arrays.hashCode(k), v, k));
  }

  public static void registerGains(Gains g, Consumer<Gains> c) {
    GAINS_UPDATER.putIfAbsent(g, c);
  }

  public static void registerConstraints(
      Constraints<?> constraints, Consumer<Constraints<?>> consumer) {
    CONSTRAINTS_UPDATER.putIfAbsent(constraints, consumer);
  }

  public static void registerNumber(LoggedTunableNumber[] numbers, Consumer<double[]> consumer) {
    NUMBER_UPDATER.putIfAbsent(numbers, consumer);
  }

  public static void registerMeasure(LoggedTunableMeasure<?>[] measures, Runnable consumer) {
    MEASURE_UPDATER.putIfAbsent(measures, consumer);
  }
}
