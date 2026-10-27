package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomUnits;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.*;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final String aKitTopic;

  @Getter private GenericFlywheelState currentState;

  @Getter private Setpoint<AngularVelocityUnit> velocityGoal;
  @Getter private Setpoint<VoltageUnit> voltageGoal;
  @Getter private Current currentGoal;

  private final CustomSysIdRoutine<VoltageUnit> voltageCharacterizationRoutine;
  private final CustomSysIdRoutine<CurrentUnit> torqueCharacterizationRoutine;

  // Torque current has no back-EMF to cap speed, so torque tests end here before the current loop
  // saturates near free speed and corrupts the data.
  private final AngularVelocity torqueSysIdVelocityLimit;
  private final boolean torqueSysIdReverseEnabled;

  public GenericFlywheel(
      GenericFlywheelIO io,
      Subsystem subsystem,
      GenericFlywheelConstants constants,
      String name,
      Setpoint<AngularVelocityUnit> velocityGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/" + "Flywheel" + name;

    currentState = GenericFlywheelState.IDLE;

    this.velocityGoal = velocityGoal;
    this.voltageGoal = voltageGoal;
    currentGoal = Amps.of(0.0);

    voltageCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<VoltageUnit>(
                CustomUnits.voltsPerSecond.ofNative(0.5),
                Volts.of(8),
                Seconds.of(24),
                (state) ->
                    Logger.recordOutput(aKitTopic + "/Voltage SysID State", state.toString()),
                Volts),
            new CustomSysIdRoutine.Mechanism<>(
                (volts) -> io.setVoltageGoal(Volts.of(volts.in(Volts))), subsystem));

    torqueCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<CurrentUnit>(
                CustomUnits.ampsPerSecond.ofNative(2.0),
                Amps.of(20),
                Seconds.of(30),
                (state) ->
                    Logger.recordOutput(
                        aKitTopic + "/Torque Current SysID State", state.toString()),
                Amp),
            new CustomSysIdRoutine.Mechanism<>(
                (amps) -> io.setCurrentGoal(Amps.of(amps.in(Amps))), subsystem));

    torqueSysIdVelocityLimit =
        RadiansPerSecond.of(0.8 * constants.motorConfig.freeSpeed / constants.gearRatio);
    torqueSysIdReverseEnabled =
        constants.peakReverseTorqueCurrent == null
            || constants.peakReverseTorqueCurrent.lt(Amps.zero());

    constants.voltageGains.onChange(gains -> io.updateGains(gains, GainSlot.ZERO));
    constants.torqueGains.onChange(gains -> io.updateGains(gains, GainSlot.ONE));
    constants.constraints.onChange(io::updateConstraints);
  }

  public GenericFlywheel(
      GenericFlywheelIO io, Subsystem subsystem, GenericFlywheelConstants constants, String name) {
    this(
        io,
        subsystem,
        constants,
        name,
        new Setpoint<>(
            RadiansPerSecond.of(0),
            constants.velocityOffsetStep,
            RadiansPerSecond.of(-constants.gearRatio * constants.motorConfig.freeSpeed),
            RadiansPerSecond.of(constants.gearRatio * constants.motorConfig.freeSpeed)),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public GenericFlywheel(
      GenericFlywheelIO io,
      Subsystem subsystem,
      GenericFlywheelConstants constants,
      String name,
      Setpoint<AngularVelocityUnit> velocityGoal) {
    this(
        io,
        subsystem,
        constants,
        name,
        velocityGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Velocity Goal", velocityGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Velocity Offset", velocityGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/Current Goal", currentGoal);
    Logger.recordOutput(aKitTopic + "/At Velocity Goal", atVelocityGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Current Goal", atCurrentGoal());

    switch (currentState) {
      case VELOCITY_VOLTAGE_CONTROL:
        io.setVelocityGoal((AngularVelocity) velocityGoal.getNewSetpoint());
        break;
      case VELOCITY_TORQUE_CONTROL:
        io.setVelocityGoal((AngularVelocity) velocityGoal.getNewSetpoint(), currentGoal);
        break;
      case VOLTAGE_CONTROL:
        io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
        break;
      case STOP:
        io.setNeutralControl();
        break;
      case IDLE:
        break;
    }
  }

  public AngularVelocity getFlywheelVelocity() {
    return inputs.velocity;
  }

  public Angle getFlywheelPosition() {
    return inputs.position;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = GenericFlywheelState.VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = GenericFlywheelState.VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setVelocityGoal(AngularVelocity velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal);
  }

  public void setVelocityGoal(Setpoint<AngularVelocityUnit> velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal = velocityGoal;
  }

  public void setVelocityGoal(Supplier<AngularVelocity> velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal.get());
  }

  public void setVelocityGoal(AngularVelocity velocityGoal, Current currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal);
    this.currentGoal = currentGoal;
  }

  public void setVelocityGoal(Setpoint<AngularVelocityUnit> velocityGoal, Current currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal = velocityGoal;
    this.currentGoal = currentGoal;
  }

  public void setVelocityGoal(
      Supplier<AngularVelocity> velocityGoal, Supplier<Current> currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal.get());
    this.currentGoal = currentGoal.get();
  }

  public void stop() {
    currentState = GenericFlywheelState.STOP;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return io.atVelocityGoal(velocityReference);
  }

  public boolean atCurrentGoal(Current currentReference) {
    return io.atCurrentGoal(currentReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVelocityGoal() {
    return atVelocityGoal((AngularVelocity) velocityGoal.getNewSetpoint());
  }

  public boolean atCurrentGoal() {
    return atCurrentGoal(currentGoal);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atVelocityGoal);
  }

  public void updateGains(Gains gains, GainSlot gainSlot) {
    io.updateGains(gains, gainSlot);
  }

  public void updateConstraints(AngularVelocityConstraints constraints) {
    io.updateConstraints(constraints);
  }

  public Command sysIdRoutineVoltage() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
        voltageCharacterizationRoutine
            .dynamic(CustomSysIdRoutine.Direction.kForward)
            .withTimeout(10),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine
            .dynamic(CustomSysIdRoutine.Direction.kReverse)
            .withTimeout(10),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }

  public Command sysIdRoutineTorque() {
    Command routine =
        Commands.sequence(
            Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
            torqueSysIdTest(
                torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kForward)),
            torqueSysIdTest(
                torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward)));
    // A zero peak reverse torque current clamps reverse requests to 0 A, so skip those tests
    if (torqueSysIdReverseEnabled) {
      routine =
          routine.andThen(
              torqueSysIdTest(
                  torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kReverse)),
              torqueSysIdTest(
                  torqueCharacterizationRoutine.quasistatic(
                      CustomSysIdRoutine.Direction.kReverse)));
    }
    return routine;
  }

  /** Runs a torque test until the velocity limit, then waits for the flywheel to coast down. */
  private Command torqueSysIdTest(Command test) {
    return test.until(
            () ->
                Math.abs(inputs.velocity.in(RadiansPerSecond))
                    >= torqueSysIdVelocityLimit.in(RadiansPerSecond))
        .andThen(
            Commands.waitUntil(() -> Math.abs(inputs.velocity.in(RadiansPerSecond)) < 1.0)
                .withTimeout(30.0));
  }
}
