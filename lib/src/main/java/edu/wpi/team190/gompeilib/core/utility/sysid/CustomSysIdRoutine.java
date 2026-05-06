// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.team190.gompeilib.core.utility.sysid;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/**
 * A generic SysId characterization routine. Subclass this for specific units.
 *
 * @param <U> The unit type for the output (e.g., VoltageUnit, CurrentUnit)
 */
public class CustomSysIdRoutine<U extends Unit> extends SysIdRoutineLog {
  private final Config<U> config;
  private final Mechanism<U> mechanism;
  private final MutableMeasure<U, ?, ?> outputValue;
  private final Consumer<State> recordState;

  /**
   * Create a new SysId characterization routine. * @param config Configuration with strongly typed
   * measures.
   *
   * @param mechanism Mechanism interface.
   * @param initialMutable The mutable measure instance (created by the subclass).
   */
  public CustomSysIdRoutine(
      Config<U> config, Mechanism<U> mechanism, MutableMeasure<U, ?, ?> initialMutable) {
    super(mechanism.name);
    this.config = config;
    this.mechanism = mechanism;
    outputValue = initialMutable;
    recordState = config.recordState != null ? config.recordState : this::recordState;
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
        (direction == Direction.kForward) ? State.kQuasistaticForward : State.kQuasistaticReverse;

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
                      outputValue.mut_replace(
                          outputSign * timer.get() * rampRateUnitsPerSec, config.outputUnit));

                  mechanism.log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.drive.accept(outputValue.mut_replace(0, config.outputUnit));
              recordState.accept(State.kNone);
              timer.stop();
            })
        .withName("sysid-" + state + "-" + mechanism.name)
        .withTimeout(config.timeout.in(Seconds));
  }

  public Command dynamic(Direction direction) {
    double outputSign = (direction == Direction.kForward) ? 1.0 : -1.0;
    State state = (direction == Direction.kForward) ? State.kDynamicForward : State.kDynamicReverse;

    // OPTIMIZED: Pre-calculate step magnitude safely
    double stepMagnitude = config.stepOutput.in(config.outputUnit);

    return mechanism
        .subsystem
        .runOnce(() -> outputValue.mut_replace(stepMagnitude * outputSign, config.outputUnit))
        .andThen(
            mechanism.subsystem.run(
                () -> {
                  mechanism.drive.accept(outputValue);
                  mechanism.log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.drive.accept(outputValue.mut_replace(0, config.outputUnit));
              recordState.accept(State.kNone);
            })
        .withName("sysid-" + state.toString() + "-" + mechanism.name)
        .withTimeout(config.timeout.in(Seconds));
  }
}
