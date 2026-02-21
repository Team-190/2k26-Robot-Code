package frc.robot.subsystems.v1_DoomSpiral.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralClimber extends SubsystemBase {
  private final Arm arm;
  private final Supplier<Angle> rollSupplier;

  private Rotation2d goal;

  private ClimberGoal state;
  private final ProfiledPIDController controller;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed; // TODO: Update this

  public V1_DoomSpiralClimber(ArmIO io, Supplier<Angle> rollSupplier) {
    setName("Climber");
    arm = new Arm(io, this, 1);
    this.rollSupplier = rollSupplier;

    goal = Rotation2d.kZero;

    state = ClimberGoal.DEFAULT;
    controller =
        new ProfiledPIDController(
            V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.kP(),
            0,
            V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.kD(),
            new TrapezoidProfile.Constraints(
                V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.maxVelocity(),
                V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.maxAcceleration()));
    CommandScheduler.getInstance().schedule(arm.setPosition(Rotation2d.kZero));
  }

  @Override
  public void periodic() {
    arm.periodic();

    Logger.recordOutput(getName() + "/Goal", goal);
    Logger.recordOutput(getName() + "/At Goal Real", atGoal());
  }

  public Rotation2d getArmPosition() {
    return arm.inputs.position;
  }

  /**
   * Sets the voltage being passed into the arm subsystem.
   *
   * @param voltage the voltage passed into the arm.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double voltage) {
    return arm.setVoltage(voltage);
  }

  /**
   * Resets position of the arm subsystem to the given position.
   *
   * @param position the position to reset the arm to.
   * @return a command that resets the arm to the given position.
   */
  public Command setPosition(Rotation2d position) {
    return arm.setPosition(position);
  }

  /**
   * Sets the goal for the arm subsystem to the given position.
   *
   * @param positionGoal the Rotation2d to set the arm to.
   * @return A command that sets the arm to the given position and waits until the arm is at the
   *     goal.
   */
  public Command setPositionGoal(Rotation2d positionGoal) {
    return Commands.runOnce(() -> goal = positionGoal)
        .andThen(
            Commands.either(
                arm.setVoltage(12),
                arm.setVoltage(-12),
                () -> positionGoal.minus(arm.getArmPosition()).getRadians() > 0),
            waitUntilPosition())
        .andThen(stop());
  }

  /**
   * Sets the goal for the arm subsystem to the given position.
   *
   * @param climberGoal the ClimberGoal to set the arm to.
   * @return A command that sets the arm to the given position and waits until the arm is at the
   *     goal.
   */
  public Command setPositionGoal(ClimberGoal climberGoal) {
    return Commands.runOnce(() -> goal = climberGoal.getPosition())
        .andThen(arm.setVoltage(12), waitUntilPosition())
        .andThen(stop());
  }

  public Command waitUntilPosition() {
    return Commands.waitUntil(this::atGoal);
  }

  public boolean atGoal() {
    return Math.abs(arm.getArmPosition().getRadians() - goal.getRadians())
        < V1_DoomSpiralClimberConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  public Command stop() {
    return setVoltage(0);
  }

  /**
   * A command that either climbs to level 3, or if the robot is already at level 2, then it will
   * climb to level 3. If the robot is already at level 1, then it will climb to level 2, and then
   * climb to level 3. If the robot is already at level 3, then it will do nothing.
   *
   * @return A command that will climb to level 3 if the robot is at level 1, or if the robot is at
   *     level 2, then it will climb to level 3. If the robot is already at level 3, then it will do
   *     nothing.
   */
  public Command climbSequenceL3() {
    // return Commands.either(
    //     Commands.sequence(
    //         Commands.runOnce(() -> state = ClimberGoal.L1_POSITION_GOAL),
    //         setVoltage(12).until(this::atGoal),
    //         Commands.runOnce(() -> state = ClimberGoal.L2_FLIP_GOAL),
    //         setVoltage(12).until(this::atGoal),
    //         Commands.runOnce(() -> state = ClimberGoal.L2_POSITION_GOAL),
    //         setVoltage(controller.calculate(rollSupplier.get().in(Radians), Math.PI))
    //             .until(() -> rollSupplier.get().isNear(Radians.of(Math.PI), Degrees.of(1.0)))),
    //     Commands.either(
    //         Commands.sequence(
    //             Commands.runOnce(() -> state = ClimberGoal.L2_FLIP_GOAL),
    //             setVoltage(12).until(this::atGoal),
    //             Commands.runOnce(() -> state = ClimberGoal.L2_POSITION_GOAL),
    //             setVoltage(controller.calculate(rollSupplier.get().in(Radians), Math.PI))
    //                 .until(() -> rollSupplier.get().isNear(Radians.of(Math.PI),
    // Degrees.of(1.0)))),
    //         Commands.sequence(
    //             Commands.runOnce(() -> state = ClimberGoal.L2_POSITION_GOAL),
    //             setVoltage(controller.calculate(rollSupplier.get().in(Radians), Math.PI))
    //                 .until(() -> rollSupplier.get().isNear(Radians.of(Math.PI),
    // Degrees.of(1.0)))),
    //         () -> state.equals(ClimberGoal.L2_FLIP_GOAL)),
    //     () -> state.equals(ClimberGoal.L1_POSITION_GOAL));

    return setPositionGoal(
        ClimberGoal
            .L2_FLIP_GOAL); // .onlyWhile(()->arm.getArmPosition().getRadians()<goal.getRadians()).andThen(stop());
  }

  /**
   * A command that sets the arm to the auto position for level 1, and then waits until the arm is
   * at the goal.
   *
   * @return A command that sets the arm to the auto position for level 1, and then waits until the
   *     arm is at the goal.
   */
  public Command climbAutoSequence() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  state = ClimberGoal.L1_AUTO_POSITION_GOAL;
                  V1_DoomSpiralRobotState.getLedStates().setAutoClimbing(true);
                }),
            waitUntilPosition())
        .finallyDo(() -> V1_DoomSpiralRobotState.getLedStates().setAutoClimbing(false));
  }

  public Command runZeroSequence() { // add actual zeroing later
    return Commands.sequence(setPosition(Rotation2d.kZero), setPositionGoal(Rotation2d.kZero));
  }

  public Command resetClimberZero() {
    return Commands.sequence(setPosition(Rotation2d.kZero), setPositionGoal(Rotation2d.kZero));
  }

  public Command clockwiseSlow() {
    return setVoltage(-V1_DoomSpiralClimberConstants.SLOW_VOLTAGE);
  }

  public Command counterClockwiseSlow() {
    return setVoltage(V1_DoomSpiralClimberConstants.SLOW_VOLTAGE);
  }

  public Command setPositionDefault() {
    return setPositionGoal(V1_DoomSpiralClimberConstants.ClimberGoal.DEFAULT.getPosition());
  }

  public Command setPositionL1() {
    return setPositionGoal(
        V1_DoomSpiralClimberConstants.ClimberGoal.L1_POSITION_GOAL.getPosition());
  }

  public Command runSysId() {
    return arm.sysIdRoutine();
  }
}
