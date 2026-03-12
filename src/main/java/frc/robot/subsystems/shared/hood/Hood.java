package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private final HoodIO io;
  private final String aKitTopic;
  private final HoodIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  private HoodState currentState;

  @Getter private HoodGoal hoodGoal;
  private Rotation2d positionGoal;
  private Voltage voltageGoal;

  @Getter @Setter private Rotation2d overridePosition;

  private final Supplier<Rotation2d> scoreRotationSupplier;
  private final Supplier<Rotation2d> feedRotationSupplier;

  private final HoodConstants constants;

  /**
   * Constructor for the Funky hood subsystem. Makes a routine that sets the voltage passed into the
   * hood and adds a default hood goal.
   *
   * @param io the IO implementation
   * @param subsystem the parent subsystem
   * @param name the name of the hood (for logging purposes)
   */
  public Hood(
      HoodIO io,
      HoodConstants constants,
      Subsystem subsystem,
      String name,
      Supplier<Rotation2d> scoreRotationSupplier,
      Supplier<Rotation2d> feedRotationSupplier) {
    inputs = new HoodIOInputsAutoLogged();
    this.io = io;

    this.currentState = HoodState.IDLE;
    this.hoodGoal = HoodGoal.STOW;

    aKitTopic = subsystem.getName() + "/Hood" + name;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.75).per(Seconds),
                Volts.of(4.5),
                Seconds.of(4),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, subsystem));

    this.scoreRotationSupplier = scoreRotationSupplier;
    this.feedRotationSupplier = feedRotationSupplier;

    positionGoal = Rotation2d.kZero;

    this.constants = constants;
  }

  /** Periodic method for the hood subsystem. Updates inputs and sets position if in closed loop. */
  @Trace
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    switch (currentState) {
      case CLOSED_LOOP_POSITION_CONTROL:
        Rotation2d position =
            switch (hoodGoal) {
              case SCORE -> scoreRotationSupplier.get();
              case FEED -> feedRotationSupplier.get();
              case OVERRIDE -> overridePosition;
              default -> Rotation2d.kZero;
            };
        positionGoal = position;
        io.setPositionGoal(position);
        break;
      case OPEN_LOOP_VOLTAGE_CONTROL:
        io.setVoltage(voltageGoal);
        break;
      case IDLE:
        break;
    }

    Logger.recordOutput(aKitTopic + "/Override Position", overridePosition);
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/State", currentState);
    Logger.recordOutput(aKitTopic + "/Position Goal", hoodGoal);
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal);
  }

  /**
   * Tells the hood what position it should be in.
   *
   * @param goal The position that the robot should be in.
   * @return The command that moves the robot towards the goal state.
   */
  public void setGoal(HoodGoal goal) {
    currentState = HoodState.CLOSED_LOOP_POSITION_CONTROL;
    this.hoodGoal = goal;
  }

  /**
   * Sets the voltage being passed into the hood subsystem.
   *
   * @param volts the voltage passed into the hood.
   * @return A command that sets the specified voltage.
   */
  public void setVoltage(Voltage volts) {
    currentState = HoodState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = volts;
  }

  /**
   * Checks if the hood is at the goal position.
   *
   * @return If the hood is within tolerance of the goal (true) or not (false).
   */
  public boolean atPositionGoal() {
    return io.atPositionGoal(positionGoal);
  }

  public boolean atPositionGoal(Rotation2d position) {
    return io.atPositionGoal(position);
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal(voltageGoal);
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
