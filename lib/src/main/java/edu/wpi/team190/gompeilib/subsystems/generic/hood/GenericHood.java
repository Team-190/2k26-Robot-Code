package edu.wpi.team190.gompeilib.subsystems.generic.hood;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class GenericHood {
  private final GenericHoodIO io;
  private final String aKitTopic;
  private final GenericHoodIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  private GenericHoodState currentState;

  @Getter private Setpoint<AngleUnit> positionGoal;
  @Getter private Setpoint<VoltageUnit> voltageGoal;

  private final GenericHoodConstants constants;

  /**
   * Constructor for the Funky GenericHood subsystem. Makes a routine that sets the voltage passed
   * into the GenericHood and adds a default GenericHood goal.
   *
   * @param io the IO implementation
   * @param subsystem the parent subsystem
   * @param name the name of the GenericHood (for logging purposes)
   */
  public GenericHood(
      GenericHoodIO io,
      Subsystem subsystem,
      GenericHoodConstants constants,
      String name,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    inputs = new GenericHoodIOInputsAutoLogged();
    this.io = io;

    this.currentState = GenericHoodState.IDLE;

    aKitTopic = subsystem.getName() + "/Hood" + name;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.75).per(Seconds),
                Volts.of(4.5),
                Seconds.of(4),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, subsystem));

    this.positionGoal = positionGoal;
    this.voltageGoal = voltageGoal;

    this.constants = constants;

    constants.gains.onChange(io::setGains);
    constants.constraints.onChange(io::setProfile);
  }

  public GenericHood(
      GenericHoodIO io,
      Subsystem subsystem,
      GenericHoodConstants constants,
      String name,
      Setpoint<AngleUnit> positionGoal) {
    this(
        io,
        subsystem,
        constants,
        name,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageStep, Volts.of(-12), Volts.of(12)));
  }

  public GenericHood(
      GenericHoodIO io, Subsystem subsystem, GenericHoodConstants constants, String name) {
    this(
        io,
        subsystem,
        constants,
        name,
        new Setpoint<>(
            Radians.zero(),
            constants.offsetStep,
            constants.minAngle.getMeasure(),
            constants.maxAngle.getMeasure()));
  }

  /** Periodic method for the hood subsystem. Updates inputs and sets position if in closed loop. */
  @Trace
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    switch (currentState) {
      case CLOSED_LOOP_POSITION_CONTROL:
        io.setPositionGoal(new Rotation2d(positionGoal.getNewSetpoint().baseUnitMagnitude()));
        break;
      case OPEN_LOOP_VOLTAGE_CONTROL:
        io.setVoltage((Voltage) voltageGoal.getNewSetpoint());
        break;
      case IDLE:
        break;
    }

    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/State", currentState);
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutput(
        aKitTopic + "/Position Goal", new Rotation2d((Angle) positionGoal.getSetpoint()));
    Logger.recordOutputMeasure(aKitTopic + "/Position Offset", positionGoal.getOffset());
  }

  /**
   * Sets the voltage being passed into the hood subsystem.
   *
   * @param voltageGoal the voltage passed into the hood.
   * @return A command that sets the specified voltage.
   */
  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = GenericHoodState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = GenericHoodState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    currentState = GenericHoodState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal.getMeasure());
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    currentState = GenericHoodState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  /**
   * Checks if the hood is at the goal position.
   *
   * @return If the hood is within tolerance of the goal (true) or not (false).
   */
  public boolean atPositionGoal() {
    return io.atPositionGoal(new Rotation2d(positionGoal.getNewSetpoint().baseUnitMagnitude()));
  }

  public boolean atPositionGoal(Rotation2d position) {
    return io.atPositionGoal(position);
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVoltageGoal(Voltage voltage) {
    return io.atVoltageGoal(voltage);
  }

  /**
   * Waits until hood at goal/in tolerance.
   *
   * @return A command that waits until the hood is at the goal.
   */
  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atPositionGoal));
  }

  /**
   * Updates the PID values for the hood.
   *
   * @param gains The proportional gain, derivative gain, and feedforward gains.
   */
  public void setGains(Gains gains) {
    io.setGains(gains);
  }

  /**
   * Resets the hood to the zero position, while ensuring that the motor is not producing any
   * torque.
   *
   * <p>This command sequence is as follows:
   *
   * <ol>
   *   <li>Move the hood to its maximum angle.
   *   <li>Set the hood state to idle.
   *   <li>Apply a negative voltage to the hood motor until the torque current is near zero.
   *   <li>Set the hood position to zero.
   * </ol>
   *
   * @return A command sequence that resets the hood to the zero position
   */
  public Command resetHoodZero() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericHoodState.IDLE),
        Commands.run(() -> io.setVoltage(constants.zeroVoltage.times(-1)))
            .until(
                () ->
                    inputs.torqueCurrent.isNear(
                        constants.zeroCurrentThreshold, constants.zeroCurrentEpsilon)),
        Commands.runOnce(() -> io.setPosition(Rotation2d.ZERO)));
  }

  /**
   * Updates the profile constraints.
   *
   * @param constraints The profile constraints.
   */
  public void setProfile(AngularPositionConstraints constraints) {
    io.setProfile(constraints);
  }

  private boolean anyAtEndStop(SysIdRoutine.Direction direction) {
    double position = getAngle().getMeasure().in(Radians);
    double tolerance = constants.constraints.goalTolerance().get().in(Radians);
    if (direction == SysIdRoutine.Direction.FORWARD
        ? position >= constants.maxAngle.getMeasure().in(Radians) - tolerance
        : position <= constants.minAngle.getMeasure().in(Radians) + tolerance) {
      return true;
    }
    return false;
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericHoodState.OPEN_LOOP_VOLTAGE_CONTROL),
        characterizationRoutine
            .quasistatic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD)),
        characterizationRoutine
            .quasistatic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE)),
        characterizationRoutine
            .dynamic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD)),
        characterizationRoutine
            .dynamic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE)));
  }

  public Rotation2d getAngle() {
    return inputs.position;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }
}
