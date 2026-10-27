package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import java.util.function.Consumer;

public interface Constraints<T extends Constraints<T>> {
  /**
   * Registers a callback to run when any constraint is edited from a dashboard. The callback runs
   * once per batch of edits, after all edits in that batch have been applied.
   *
   * @param listener callback receiving these constraints
   */
  public void onChange(Consumer<T> listener);

  public sealed interface PositionConstraints<T extends PositionConstraints<T>>
      extends Constraints<T> permits AngularPositionConstraints, LinearConstraints {}
}
