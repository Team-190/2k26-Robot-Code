package frc.robot.subsystems.shared.linkage;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LinkageState {
  OPEN_LOOP_VOLTAGE_CONTROL,
  CLOSED_LOOP_POSITION_CONTROL,
  IDLE;

  public Output set(double leftVolts, double rightVolts) {
    switch (this) {
      case OPEN_LOOP_VOLTAGE_CONTROL, IDLE -> {
        return new Output.Voltage(leftVolts, rightVolts);
      }
      case CLOSED_LOOP_POSITION_CONTROL -> throw new IllegalStateException(
          "Not voltage controlled");
    }
    throw new AssertionError();
  }

  // Construct Output for position-controlled states
  public Output set(Rotation2d left, Rotation2d right) {
    switch (this) {
      case CLOSED_LOOP_POSITION_CONTROL -> {
        return new Output.Position(left, right);
      }
      default -> throw new IllegalStateException("Not position controlled");
    }
  }

  public sealed interface Output permits Output.Voltage, Output.Position {

    <T> T match(
        java.util.function.Function<Voltage, T> voltage,
        java.util.function.Function<Position, T> position);

    default double leftVolts() {
      return match(
          Voltage::leftVolts,
          p -> {
            throw new IllegalStateException("Not voltage output");
          });
    }

    default double rightVolts() {
      return match(
          Voltage::rightVolts,
          p -> {
            throw new IllegalStateException("Not voltage output");
          });
    }

    default Rotation2d leftPosition() {
      return match(
          v -> {
            throw new IllegalStateException("Not position output");
          },
          Position::left);
    }

    default Rotation2d rightPosition() {
      return match(
          v -> {
            throw new IllegalStateException("Not position output");
          },
          Position::right);
    }

    record Voltage(double leftVolts, double rightVolts) implements Output {

      @Override
      public <T> T match(
          java.util.function.Function<Voltage, T> voltage,
          java.util.function.Function<Position, T> position) {
        return voltage.apply(this);
      }
    }

    record Position(Rotation2d left, Rotation2d right) implements Output {

      @Override
      public <T> T match(
          java.util.function.Function<Voltage, T> voltage,
          java.util.function.Function<Position, T> position) {
        return position.apply(this);
      }
    }
  }
}
