package frc.robot.subsystems.shared.linkage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.littletonrobotics.junction.Logger;

public class Linkage {
  public final LinkageIO io;
  public final String aKitTopic;
  private final LinkageIOInputsAutoLogged inputs;

  private LinkageState currentState;
  private LinkageState.Output currentOutput;

  private final SysIdRoutine characterizationRoutineLeft;
  private final SysIdRoutine characterizationRoutineRight;

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

    Logger.recordOutput(aKitTopic + "/At Goal Right", atGoalRight());
    Logger.recordOutput(aKitTopic + "/At Goal Left", atGoalLeft());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> {
        io.setVoltageLeft(currentOutput.leftVolts());
        io.setVoltageRight(currentOutput.rightVolts());
      }
      case CLOSED_LOOP_POSITION_CONTROL -> {
        io.setPositionGoalLeft(currentOutput.leftPosition());
        io.setPositionGoalRight(currentOutput.rightPosition());
      }
      default -> {}
    }
  }

  /**
   * Sets the voltage being passed into the linkage subsystem.
   *
   * @param volts the voltage passed into the linkage.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double leftVolts, double rightVolts) {
    return Commands.runOnce(
        () -> {
          currentState = LinkageState.OPEN_LOOP_VOLTAGE_CONTROL;
          currentState.set(leftVolts, rightVolts);
        });
  }

  public Command setPositionGoal(Rotation2d leftPosition, Rotation2d rightPosition) {
    return Commands.runOnce(
        () -> {
          currentState = LinkageState.CLOSED_LOOP_POSITION_CONTROL;
          currentState.set(leftPosition, rightPosition);
        });
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
  public boolean atGoalRight() {
    return io.atGoalRight();
  }

  /**
   * Checks if the linkage is at the goal position.
   *
   * @return If the linkage is within tolerance of the goal (true) or not (false).
   */
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
