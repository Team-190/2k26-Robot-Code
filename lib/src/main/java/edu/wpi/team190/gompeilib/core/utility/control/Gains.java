package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.team190.gompeilib.core.utility.tunable.GompeiTunables;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import lombok.Builder;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;

/**
 * A set of tunable feedback and feedforward gains, published under {@code prefix} (for example
 * {@code /Tunables/<prefix>/kP}). Gains built without a prefix are not published.
 */
public final class Gains {
  private final TunableDouble kP;
  private final TunableDouble kI;
  private final TunableDouble kD;
  private final TunableDouble kS;
  private final TunableDouble kV;
  private final TunableDouble kA;
  private final TunableDouble kG;

  private final List<Consumer<Gains>> listeners = new ArrayList<>();
  private List<Double> lastNotifiedValues;

  @Builder(setterPrefix = "with")
  public Gains(
      String prefix, double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    TunableTable table = GompeiTunables.table(prefix);
    Runnable onTune = this::notifyListeners;
    this.kP = GompeiTunables.number(table, "kP", kP, onTune);
    this.kI = GompeiTunables.number(table, "kI", kI, onTune);
    this.kD = GompeiTunables.number(table, "kD", kD, onTune);
    this.kS = GompeiTunables.number(table, "kS", kS, onTune);
    this.kV = GompeiTunables.number(table, "kV", kV, onTune);
    this.kA = GompeiTunables.number(table, "kA", kA, onTune);
    this.kG = GompeiTunables.number(table, "kG", kG, onTune);
    lastNotifiedValues = values();
  }

  public TunableDouble kP() {
    return kP;
  }

  public TunableDouble kI() {
    return kI;
  }

  public TunableDouble kD() {
    return kD;
  }

  public TunableDouble kS() {
    return kS;
  }

  public TunableDouble kV() {
    return kV;
  }

  public TunableDouble kA() {
    return kA;
  }

  public TunableDouble kG() {
    return kG;
  }

  public double getKP() {
    return kP.get();
  }

  public double getKI() {
    return kI.get();
  }

  public double getKD() {
    return kD.get();
  }

  public double getKS() {
    return kS.get();
  }

  public double getKV() {
    return kV.get();
  }

  public double getKA() {
    return kA.get();
  }

  public double getKG() {
    return kG.get();
  }

  /**
   * Registers a callback to run when any gain is edited from a dashboard. The callback runs once
   * per batch of edits, after all edits in that batch have been applied.
   *
   * @param listener callback receiving these gains
   */
  public void onChange(Consumer<Gains> listener) {
    listeners.add(listener);
  }

  private List<Double> values() {
    return List.of(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
  }

  private void notifyListeners() {
    // Every edited gain triggers this; only notify once per distinct set of values.
    List<Double> values = values();
    if (values.equals(lastNotifiedValues)) {
      return;
    }
    lastNotifiedValues = values;
    listeners.forEach(listener -> listener.accept(this));
  }
}
