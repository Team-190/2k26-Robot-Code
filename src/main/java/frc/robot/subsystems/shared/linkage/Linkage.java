package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import frc.robot.subsystems.shared.linkage.LinkageConstants.LinkageGoal;
import frc.robot.subsystems.shared.linkage.LinkageConstants.LinkageState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Linkage {
  public final LinkageIO io;
  public final String aKitTopic;
  private final LinkageIOInputsAutoLogged inputs;
  private LinkageGoal positionGoalRight;
  private LinkageGoal positionGoalLeft;

  private LinkageState currentState;

  private double voltageGoalRight;
  private double voltageGoalLeft;

  private SysIdRoutine characterizationRoutineLeft;
  private SysIdRoutine characterizationRoutineRight;

  public Linkage(LinkageIO io, Subsystem subsystem, int index) {

    inputs = new LinkageIOInputsAutoLogged();
    this.io = io;

    aKitTopic = subsystem.getName() + "/Linkage" + index;

    characterizationRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDStateLeft", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltageLeft(voltage.in(Volts)), null, subsystem));

    characterizationRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/sysIDStateRight", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltageRight(voltage.in(Volts)), null, subsystem));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    // Will add more later
  }

  /**
   * Tells the linkage what position it should be in.
   *
   * @param goal The position that the robot should be in.
   * @return The command that moves the robot towards the goal state.
   */
  public Command setGoalRight(LinkageGoal goal) {
    return Commands.runOnce(
        () -> {
            currentState = LinkageState
          this.positionGoalRight = goal;
        });
  }

  /**
   * Tells the linkage what position it should be in.
   *
   * @param goal The position that the robot should be in.
   * @return The command that moves the robot towards the goal state.
   */
  public Command setGoalLeft(LinkageGoal goal) {
    return Commands.runOnce(
        () -> {
          this.positionGoalLeft = goal;
        });
  }

  /**
   * Sets the voltage being passed into the linkage subsystem.
   *
   * @param volts the voltage passed into the linkage.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltageRight(double volts) {
    return Commands.runOnce(
        () -> {
          this.voltageGoalRight = volts;
        });
  }

  /**
   * Sets the voltage being passed into the linkage subsystem.
   *
   * @param volts the voltage passed into the linkage.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltageLeft(double volts) {
    return Commands.runOnce(
        () -> {
          this.voltageGoalLeft = volts;
        });
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  @AutoLogOutput(key = "linkage/At Goal")
  public boolean atGoalRight() {
    return io.atGoalRight();
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  @AutoLogOutput(key = "linkage/At Goal")
  public boolean atGoalLeft() {
    return io.atGoalLeft();
  }

  /**
   * Waits until left linkage at goal/in tolerance.
   *
   * @return A command that waits until the linkage is at the goal.
   */
  public Command waitUntilLinkageLeftAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoalLeft));
  }

  /**
   * Waits until right linkage at goal/in tolerance.
   *
   * @return A command that waits until the linkage is at the goal.
   */
  public Command waitUntilLinkageRightAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoalRight));
  }

  /**
   * Updates the PID values for the linkage.
   *
   * @param kp the proportional gain
   * @param kd the derivative gain
   */
  public void setPIDRight(double kp, double kd) {
    io.setPIDRight(kp, 0.0, kd);
  }

  /**
   * Updates the PID values for the linkage.
   *
   * @param kp the proportional gain
   * @param kd the derivative gain
   */
  public void setPIDLeft(double kp, double kd) {
    io.setPIDLeft(kp, 0.0, kd);
  }

  /**
   * Updates the profile constraints.
   *
   * @param maxVelocityRadiansPerSecond Maximum velocity (rad/sec)
   * @param maxAccelerationRadiansPerSecondSquared Maximum acceleration (rad/sec^2)
   * @param goalToleranceRadians Tolerance (rad)
   */
  public void setProfileRight(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfileRight(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  /**
   * Updates the profile constraints.
   *
   * @param maxVelocityRadiansPerSecond Maximum velocity (rad/sec)
   * @param maxAccelerationRadiansPerSecondSquared Maximum acceleration (rad/sec^2)
   * @param goalToleranceRadians Tolerance (rad)
   */
  public void setProfileLeft(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    io.setProfileLeft(
        maxVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared, goalToleranceRadians);
  }

  /**
   * Runs the system ID
   *
   * @return runs a sysID charecterization routine command
   */
  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = LinkageState.IDLE),
        characterizationRoutineRight.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutineRight.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutineRight.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutineRight.dynamic(Direction.kReverse),
        characterizationRoutineLeft.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutineLeft.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutineLeft.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutineLeft.dynamic(Direction.kReverse));
  }
}
