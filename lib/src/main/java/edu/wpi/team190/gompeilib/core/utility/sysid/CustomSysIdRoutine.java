// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.team190.gompeilib.core.utility.sysid;

import static org.wpilib.units.Units.*;

import java.util.function.Consumer;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.units.*;
import org.wpilib.units.measure.Per;

/**
 * A generic SysId characterization routine. Subclass this for specific units.
 *
 * @param <U> The unit type for the output (e.g., VoltageUnit, CurrentUnit)
 */
public class CustomSysIdRoutine<U extends Unit> extends SysIdRoutineLog {
  private final Config<U> config;
  private final Mechanism<U> mechanism;
  private Measure<U> outputValue;
  private final Consumer<State> recordState;

  /**
   * Create a new SysId characterization routine. * @param config Configuration with strongly typed
   * measures.
   *
   * @param mechanism Mechanism interface.
   */
  public CustomSysIdRoutine(Config<U> config, Mechanism<U> mechanism) {
    super(mechanism.name);
    this.config = config;
    this.mechanism = mechanism;
    outputValue = of(config.outputUnit, 0);
    recordState = config.recordState != null ? config.recordState : this::recordState;
  }

  // Unit#of(double) only returns Measure<?> since U can't be reified here; WPILib measures are
  // immutable now (no more MutableMeasure), so every value has to be rebuilt through this cast.
  @SuppressWarnings("unchecked")
  private static <U extends Unit> Measure<U> of(U unit, double magnitude) {
    return (Measure<U>) unit.of(magnitude);
  }

  /**
   * @param rampRate We use Measure<Unit> for ramp rate because it is U/Time.
   */
  public record Config<U extends Unit>(
      Per<U, TimeUnit> rampRate,
      Measure<U> stepOutput,
      Measure<TimeUnit> timeout,
      Consumer<State> recordState,
      U outputUnit) {
    public Config(
        Per<U, TimeUnit> rampRate,
        Measure<U> stepOutput,
        Measure<TimeUnit> timeout,
        Consumer<State> recordState,
        U outputUnit) {
      this.rampRate = rampRate;
      this.stepOutput = stepOutput;
      this.timeout = timeout != null ? timeout : Seconds.of(10);
      this.recordState = recordState;
      this.outputUnit = outputUnit;
    }
  }

  public static class Mechanism<U extends Unit> {
    public final Consumer<Measure<U>> drive;
    public final Consumer<SysIdRoutineLog> log;
    public final Subsystem subsystem;
    public final String name;

    public Mechanism(
        Consumer<Measure<U>> drive,
        Consumer<SysIdRoutineLog> log,
        Subsystem subsystem,
        String name) {
      this.drive = drive;
      this.log = log != null ? log : l -> {};
      this.subsystem = subsystem;
      this.name = name != null ? name : subsystem.getName();
    }

    public Mechanism(Consumer<Measure<U>> drive, Subsystem subsystem) {
      this(drive, null, subsystem, null);
    }
  }

  public enum Direction {
    kForward,
    kReverse
  }

  public Command quasistatic(Direction direction) {
    State state =
        (direction == Direction.kForward) ? State.QUASISTATIC_FORWARD : State.QUASISTATIC_REVERSE;

    double outputSign = (direction == Direction.kForward) ? 1.0 : -1.0;
    Timer timer = new Timer();

    double rampRateUnitsPerSec = config.rampRate.magnitude();

    return mechanism
        .subsystem
        .runOnce(timer::restart)
        .andThen(
            mechanism.subsystem.run(
                () -> {
                  mechanism.drive.accept(
                      of(config.outputUnit, outputSign * timer.get() * rampRateUnitsPerSec));

                  mechanism.log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.drive.accept(of(config.outputUnit, 0));
              recordState.accept(State.NONE);
              timer.stop();
            })
        .withName("sysid-" + state + "-" + mechanism.name)
        .withTimeout(config.timeout.in(Seconds));
  }

  public Command dynamic(Direction direction) {
    double outputSign = (direction == Direction.kForward) ? 1.0 : -1.0;
    State state = (direction == Direction.kForward) ? State.DYNAMIC_FORWARD : State.DYNAMIC_REVERSE;

    // OPTIMIZED: Pre-calculate step magnitude safely
    double stepMagnitude = config.stepOutput.in(config.outputUnit);

    return mechanism
        .subsystem
        .runOnce(() -> outputValue = of(config.outputUnit, stepMagnitude * outputSign))
        .andThen(
            mechanism.subsystem.run(
                () -> {
                  mechanism.drive.accept(outputValue);
                  mechanism.log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.drive.accept(of(config.outputUnit, 0));
              recordState.accept(State.NONE);
            })
        .withName("sysid-" + state.toString() + "-" + mechanism.name)
        .withTimeout(config.timeout.in(Seconds));
  }
}
