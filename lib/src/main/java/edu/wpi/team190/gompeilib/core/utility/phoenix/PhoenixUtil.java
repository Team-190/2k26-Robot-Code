package edu.wpi.team190.gompeilib.core.utility.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh, grouped by the CAN bus they live on. */
  private static final Map<CANBus, BaseStatusSignal[]> signalsByBus = new LinkedHashMap<>();

  /**
   * Registers a set of signals for synchronized refresh. Signals are grouped per CAN bus, since
   * {@link BaseStatusSignal#refreshAll} requires every signal in a call to share a bus. This
   * supports any number of SystemCore CAN ports and CANivores.
   */
  public static void registerSignals(CANBus canBus, BaseStatusSignal... signals) {
    signalsByBus.merge(
        canBus,
        signals,
        (existing, added) -> {
          BaseStatusSignal[] newSignals = new BaseStatusSignal[existing.length + added.length];
          System.arraycopy(existing, 0, newSignals, 0, existing.length);
          System.arraycopy(added, 0, newSignals, existing.length, added.length);
          return newSignals;
        });
  }

  /** Refresh all registered signals, one synchronized refresh per CAN bus. */
  public static void refreshAll() {
    for (BaseStatusSignal[] signals : signalsByBus.values()) {
      if (signals.length > 0) {
        BaseStatusSignal.refreshAll(signals);
      }
    }
  }
}
