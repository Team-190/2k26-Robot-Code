package frc.robot.subsystems.shared.fourbarlinkage;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FourBarLinkageState {
  OPEN_LOOP_VOLTAGE_CONTROL,
  CLOSED_LOOP_POSITION_CONTROL,
  IDLE;

  public Output set(edu.wpi.first.units.measure.Voltage volts) {
    switch (this) {
      case OPEN_LOOP_VOLTAGE_CONTROL, IDLE -> {
        return new Output.Voltage(volts);
      }
      case CLOSED_LOOP_POSITION_CONTROL -> throw new IllegalStateException(
          "Not voltage controlled");
    }
    throw new AssertionError();
  }

  // Construct Output for position-controlled states
  public Output set(Rotation2d position) {
    switch (this) {
      case CLOSED_LOOP_POSITION_CONTROL -> {
        return new Output.Position(position);
      }
      default -> throw new IllegalStateException("Not position controlled");
    }
  }

  public sealed interface Output permits Output.Voltage, Output.Position {

    <T> T match(
        java.util.function.Function<Voltage, T> voltage,
        java.util.function.Function<Position, T> position);

    default edu.wpi.first.units.measure.Voltage volts() {
      return match(
          Voltage::volts,
          p -> {
            throw new IllegalStateException("Not voltage output");
          });
    }

    default Rotation2d position() {
      return match(
          v -> {
            throw new IllegalStateException("Not position output");
          },
          Position::position);
    }

    record Voltage(edu.wpi.first.units.measure.Voltage volts) implements Output {

      @Override
      public <T> T match(
          java.util.function.Function<Voltage, T> voltage,
          java.util.function.Function<Position, T> position) {
        return voltage.apply(this);
      }
    }

    record Position(Rotation2d position) implements Output {

      @Override
      public <T> T match(
          java.util.function.Function<Voltage, T> voltage,
          java.util.function.Function<Position, T> position) {
        return position.apply(this);
      }
    }
  }
}
