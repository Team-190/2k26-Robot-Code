package frc.robot.subsystems.v0_Funky.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import frc.robot.subsystems.v0_Funky.hood.V0_FunkyHoodConstants.HoodGoal;
import frc.robot.subsystems.v0_Funky.hood.V0_FunkyHoodState.HoodState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyHood {
  private final V0_FunkyHoodIO io;
  private final String aKitTopic;
  private final V0_FunkyHoodIOInputsAutoLogged inputs;

  private SysIdRoutine characterizationRoutine;

  private HoodState currentState;

  private HoodGoal positionGoal;
  private double voltageGoal;

  /**
   * Constructor for the Funky hood subsystem. Makes a routine that sets the voltage passed into the
   * hood and adds a default hood goal.
   *
   * @param io the IO implementation
   * @param subsystem the parent subsystem
   * @param index the index of the hood in the subsystem
   */
  public V0_FunkyHood(V0_FunkyHoodIO io, Subsystem subsystem, int index) {
    inputs = new V0_FunkyHoodIOInputsAutoLogged();
    this.io = io;

    this.currentState = HoodState.IDLE;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Arm/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));

    aKitTopic = subsystem.getName() + "/Hood" + index;
  }
  /** Periodic method for the hood subsystem. Updates inputs and sets position if in closed loop. */
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    switch (currentState) {
      case CLOSED_LOOP_POSITION_CONTROL:
        io.setPosition(positionGoal.getAngle());
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
          io.setVoltage(volts);
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
