package edu.wpi.team190.gompeilib.core.utility.tunable;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;

/**
 * Helpers for creating WPILib tunables with GompeiLib conventions.
 *
 * <p>Tunables are always published so their values are visible, but are only editable from a
 * dashboard when {@link GompeiLib#isTuning()} is true.
 */
public final class GompeiTunables {
  private GompeiTunables() {}

  /**
   * Creates the standard config for a GompeiLib tunable.
   *
   * @param onTune callback run when the value is edited remotely, or null for none
   * @return tunable config
   */
  public static TunableConfig config(Runnable onTune) {
    TunableConfig config =
        TunableConfig.of(GompeiLib.isTuning() ? TunableOption.MUTABLE : TunableOption.IMMUTABLE);
    return onTune == null ? config : config.withOnTune(onTune);
  }

  /**
   * Gets the table to publish a group of tunables under.
   *
   * @param prefix table path, or null to leave the tunables unpublished
   * @return table, or null if unpublished
   */
  public static TunableTable table(String prefix) {
    return prefix == null ? null : Tunables.getTable(prefix);
  }

  /**
   * Creates and publishes a tunable value. Measures keep their static type, so passing a {@code
   * Distance} returns a {@code Tunable<Distance>}.
   *
   * @param table table to publish in, or null to leave unpublished
   * @param name name within the table
   * @param initial initial value
   * @param onTune callback run when the value is edited remotely, or null for none
   * @return tunable
   */
  public static <T> Tunable<T> value(TunableTable table, String name, T initial, Runnable onTune) {
    Tunable<T> tunable = Tunable.createConfig(initial, config(onTune));
    if (table != null) {
      table.publish(name, tunable);
    }
    return tunable;
  }

  /**
   * Creates and publishes a standalone tunable value that is read directly rather than listened to.
   *
   * @param path full path under {@code /Tunables}
   * @param initial initial value
   * @return tunable
   */
  public static <T> Tunable<T> value(String path, T initial) {
    return value(Tunables.getTable(), path, initial, null);
  }

  /**
   * Creates and publishes a tunable double.
   *
   * @param table table to publish in, or null to leave unpublished
   * @param name name within the table
   * @param initial initial value
   * @param onTune callback run when the value is edited remotely, or null for none
   * @return tunable
   */
  public static TunableDouble number(
      TunableTable table, String name, double initial, Runnable onTune) {
    TunableDouble tunable = TunableDouble.createConfig(initial, config(onTune));
    if (table != null) {
      table.publish(name, tunable);
    }
    return tunable;
  }
}
