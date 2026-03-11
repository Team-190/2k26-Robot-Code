package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LinearExtension {
  private final LinearExtensionIO io;
  private final String aKitTopic;
  private final LinearExtensionIOInputsAutoLogged inputs;

  private Voltage voltageGoal;

  private final LinearExtensionConstants constants;

  private final SysIdRoutine characterizationRoutine;

  private LinearExtensionState currentState;

  /**
   * Creates a linear extension object with four bars. Handles the calculations of position in 3D
   * space.
   *
   * @param io the IO object for the linear extension. Either {@link
   *     frc.robot.subsystems.shared.fourbarlinear extension.FourBarlinear extensionIOSim} or {@link
   *     frc.robot.subsystems.shared.fourbarlinear extension.FourBarlinear extensionIOTalonFX}.
   * @param constants The constants file for FourBarlinear extension.{@link
   *     frc.robot.subsystems.shared.fourbarlinear extension.FourBarlinear extensionConstants}
   * @param subsystem The subsystem this linear extension belongs to.
   * @param index The index of multiple linear extensions in the same subsystem.
   */
  public LinearExtension(
      LinearExtensionIO io, LinearExtensionConstants constants, Subsystem subsystem, int index) {

    inputs = new LinearExtensionIOInputsAutoLogged();
    this.io = io;
    this.constants = constants;
    aKitTopic = subsystem.getName() + "/LinearExtension" + index;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(Volts.of(6)), null, subsystem));

    currentState = LinearExtensionState.CLOSED_LOOP_POSITION_CONTROL;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs(aKitTopic, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          io.setPID(constants.GAINS);
        },
        constants.GAINS.kP(),
        constants.GAINS.kD(),
        constants.GAINS.kS(),
        constants.GAINS.kG(),
        constants.GAINS.kV(),
        constants.GAINS.kA());

    LoggedTunableMeasure.ifChanged(
        hashCode(),
        () ->
            io.setProfile(
                constants.CONSTRAINTS),
        constants.CONSTRAINTS.maxAcceleration(),
        constants.CONSTRAINTS.maxVelocity(),
        constants.CONSTRAINTS.goalTolerance());

    Logger.recordOutput(aKitTopic + "/velocityRotationsPerSec", inputs.velocity.in(MetersPerSecond));
    Logger.recordOutput(aKitTopic + "/At Goal", atGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> {
        io.setVoltage(voltageGoal);
      }
      case CLOSED_LOOP_POSITION_CONTROL -> {
        io.setPositionGoal(inputs.position);
      }
      default -> {}
    }
  }

  /**
   * Gets the current position of the linear extension.
   *
   * @return The current position of the linear extension as a Rotation2d.
   */
  public Distance getPosition() {
    return inputs.position;
  }

  public Command setIdle() {
    return Commands.runOnce(
        () -> {
          currentState = LinearExtensionState.IDLE;
        });
  }

  /**
   * Sets the voltage being passed into the linear extension subsystem.
   *
   * @param volts the voltage passed into the linear extension.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = LinearExtensionState.OPEN_LOOP_VOLTAGE_CONTROL;
        });
  }

  /**
   * Set the goal for the linear extension.
   *
   * @param position the goal
   * @return A command to set the goal to the specified value.
   */
  public Command setPositionGoal(Rotation2d position, Supplier<Rotation2d> positionOffset) {
    return Commands.runOnce(
        () -> {
          currentState = LinearExtensionState.CLOSED_LOOP_POSITION_CONTROL;
        });
  }

  /**
   * Set the goal for the linear extension.
   *
   * @param position the goal
   * @return A command to set the goal to the specified value.
   */
  public Command setPositionGoal(
      Supplier<Rotation2d> position, Supplier<Rotation2d> positionOffset) {
    return Commands.runOnce(
        () -> {
          currentState = LinearExtensionState.CLOSED_LOOP_POSITION_CONTROL;
        });
  }

  public Command setPosition(Distance position) {
    return Commands.runOnce(() -> io.setPosition(position));
  }

  /**
   * Checks if the linear extension is at the goal position.
   *
   * @return If the linear extension is within tolerance of the goal (true) or not (false).
   */
  public boolean atGoal() {
    return io.atGoal();
  }

  /**
   * Checks if the linear extension is at the goal position.
   *
   * @param position The state to check goal against.
   * @return If the linear extension is within tolerance of the goal (true) or not (false).
   */
  public boolean atGoal(double position) {
    return Math.abs(inputs.position.baseUnitMagnitude() - position)
        <= constants.CONSTRAINTS.goalTolerance().get().in(Meters);
  }

  /**
   * Waits until linear extension at goal/in tolerance.
   *
   * @return A command that waits until the linear extension is at the goal.
   */
  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  /**
   * Updates the PID values for the linear extension.
   *
   * @param kp the proportional gain
   * @param kd the derivative gain
   */
  public void setPID(double kp, double kd) {
    Gains gains = new Gains(
      new LoggedTunableNumber(aKitTopic, kp), 
      new LoggedTunableNumber(aKitTopic, 0), 
      new LoggedTunableNumber(aKitTopic, kd),
      new LoggedTunableNumber(aKitTopic, 0), 
      new LoggedTunableNumber(aKitTopic, 0), 
      new LoggedTunableNumber(aKitTopic, 0), 
      new LoggedTunableNumber(aKitTopic, 0));
    io.setPID(gains);
  }

  /**
   * Updates the profile constraints.
   *
   * @param maxVelocityRadiansPerSecond Maximum velocity (rad/sec)
   * @param maxAccelerationRadiansPerSecondSquared Maximum acceleration (rad/sec^2)
   * @param goalToleranceRadians Tolerance (rad)
   */
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfile(
        constants.CONSTRAINTS);
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = LinearExtensionState.IDLE),
        Commands.print("Sys Id being run"),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
