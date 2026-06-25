package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import java.util.function.Consumer;

public interface Constraints<T extends Constraints<T>> {
  public void update(int id, Consumer<T> consumer);

  public sealed interface PositionConstraints<T extends PositionConstraints<T>>
      extends Constraints<T> permits AngularPositionConstraints, LinearConstraints {}
}
