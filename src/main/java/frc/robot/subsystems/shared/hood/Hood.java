package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import frc.robot.subsystems.shared.hood.GenericHoodState.HoodState;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private final HoodIO io;
  private final String aKitTopic;
  private final HoodIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  private HoodState currentState;

  private HoodGoal positionGoal;
  private double voltageGoal;

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
   * @param index the index of the hood in the subsystem
   */
  public Hood(
      HoodIO io,
      HoodConstants constants,
      Subsystem subsystem,
      int index,
      Supplier<Rotation2d> scoreRotationSupplier,
      Supplier<Rotation2d> feedRotationSupplier) {
    inputs = new HoodIOInputsAutoLogged();
    this.io = io;

    this.currentState = HoodState.IDLE;

    aKitTopic = subsystem.getName() + "/Hood" + index;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));

    this.scoreRotationSupplier = scoreRotationSupplier;
    this.feedRotationSupplier = feedRotationSupplier;

    this.constants = constants;
  }

  /** Periodic method for the hood subsystem. Updates inputs and sets position if in closed loop. */
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    Logger.recordOutput(aKitTopic + "/Override Position", overridePosition);
    switch (currentState) {
      case CLOSED_LOOP_POSITION_CONTROL:
        Rotation2d position;
        switch (positionGoal) {
          case SCORE:
            position = scoreRotationSupplier.get();
            break;
          case FEED:
            position = feedRotationSupplier.get();
            break;
          case OVERRIDE:
            position = overridePosition;
            break;
          case STOW:
          default:
            position = Rotation2d.kZero;
            break;
        }
        io.setPosition(position);
        break;
      case OPEN_LOOP_VOLTAGE_CONTROL:
        io.setVoltage(voltageGoal);
        break;
      case IDLE:
        break;
    }
  }

  /**
   * Tells the hood what position it should be in.
   *
   * @param goal The position that the robot should be in.
   * @return The command that moves the robot towards the goal state.
   */
  public Command setGoal(HoodGoal goal) {
    return Commands.runOnce(
        () -> {
          currentState = HoodState.CLOSED_LOOP_POSITION_CONTROL;
          this.positionGoal = goal;
        });
  }

  /**
   * Sets the voltage being passed into the hood subsystem.
   *
   * @param volts the voltage passed into the hood.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = HoodState.OPEN_LOOP_VOLTAGE_CONTROL;
          this.voltageGoal = volts;
        });
  }

  /**
   * Checks if the hood is at the goal position.
   *
   * @return If the hood is within tolerance of the goal (true) or not (false).
   */
  @AutoLogOutput(key = "Hood/At Goal")
  public boolean atGoal() {
    return io.atGoal();
  }

  /**
   * Waits until hood at goal/in tolerance.
   *
   * @return A command that waits until the hood is at the goal.
   */
  public Command waitUntilHoodAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoal));
  }

  /**
   * Updates the PID values for the hood.
   *
   * @param kp the proportional gain
   * @param kd the derivative gain
   */
  public void setPID(double kp, double kd) {
    io.setPID(kp, 0.0, kd);
  }

  /**
   * Updates the feedforward gains for the hood.
   *
   * @param ks the static friction
   * @param kv the velocity
   * @param ka the acceleration
   */
  public void setFeedforward(double ks, double kv, double ka) {
    io.setFeedforward(ks, kv, ka);
  }

  /**
   * Resets the hood to the zero position, while ensuring that the motor is not producing any torque.
   *
   * <p>This command sequence is as follows:</p>
   * <ol>
   *   <li>Move the hood to its maximum angle.</li>
   *   <li>Set the hood state to idle.</li>
   *   <li>Apply a negative voltage to the hood motor until the torque current is near zero.</li>
   *   <li>Set the hood position to zero.</li>
   * </ol>
   * 
   * @return A command sequence that resets the hood to the zero position
   */
  public Command resetHoodZero() {
    return Commands.sequence(
        Commands.runOnce(() -> io.setPosition(constants.maxAngle)),
        Commands.runOnce(() -> currentState = HoodState.IDLE),
        Commands.run(() -> io.setVoltage(-constants.zeroVoltage.in(Volts)))
            .until(
                () ->
                    inputs.torqueCurrent.isNear(
                        constants.zeroCurrentThreshold, constants.zeroCurrentEpsilon)),
        Commands.runOnce(() -> io.setPosition(Rotation2d.kZero)));
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
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
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
