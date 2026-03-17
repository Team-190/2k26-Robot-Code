package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private final HoodIO io;
  private final String aKitTopic;
  private final HoodIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  private HoodState currentState;

  @Getter private Setpoint<AngleUnit> positionGoal;
  @Getter private Setpoint<VoltageUnit> voltageGoal;

  private final HoodConstants constants;

  /**
   * Constructor for the Funky hood subsystem. Makes a routine that sets the voltage passed into the
   * hood and adds a default hood goal.
   *
   * @param io the IO implementation
   * @param subsystem the parent subsystem
   * @param name the name of the hood (for logging purposes)
   */
  public Hood(HoodIO io, HoodConstants constants, Subsystem subsystem, String name) {
    inputs = new HoodIOInputsAutoLogged();
    this.io = io;

    this.currentState = HoodState.IDLE;

    aKitTopic = subsystem.getName() + "/Hood" + name;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.75).per(Seconds),
                Volts.of(4.5),
                Seconds.of(4),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, subsystem));

    positionGoal =
        new Setpoint<>(
            Radians.zero(),
            constants.offsetStep,
            constants.minAngle.getMeasure(),
            constants.maxAngle.getMeasure());
    voltageGoal = new Setpoint<>(Volts.of(0), constants.voltageStep, Volts.of(-12), Volts.of(12));

    this.constants = constants;
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
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutput(
        aKitTopic + "/Position Goal", new Rotation2d((Angle) positionGoal.getSetpoint()));
    Logger.recordOutput(aKitTopic + "/Position Offset", positionGoal.getOffset());
  }

  /**
   * Sets the voltage being passed into the hood subsystem.
   *
   * @param voltageGoal the voltage passed into the hood.
   * @return A command that sets the specified voltage.
   */
  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = HoodState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = HoodState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    currentState = HoodState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal.getMeasure());
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    currentState = HoodState.CLOSED_LOOP_POSITION_CONTROL;
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
        Commands.runOnce(() -> io.setPosition(constants.maxAngle)),
        Commands.runOnce(() -> currentState = HoodState.IDLE),
        Commands.run(() -> io.setVoltage(constants.zeroVoltage.times(-1)))
            .until(
                () ->
                    inputs.torqueCurrent.isNear(
                        constants.zeroCurrentThreshold, constants.zeroCurrentEpsilon)),
        Commands.runOnce(() -> io.setPosition(Rotation2d.kZero)));
  }

  /**
   * Updates the profile constraints.
   *
   * @param constraints The profile constraints.
   */
  public void setProfile(AngularPositionConstraints constraints) {
    io.setProfile(constraints);
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = HoodState.IDLE),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
