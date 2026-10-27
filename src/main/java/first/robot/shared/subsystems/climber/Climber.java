package first.robot.shared.subsystems.climber;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import first.robot.shared.subsystems.climber.ClimberConstants.ClimberGoal;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;

public class Climber extends SubsystemBase {
  private final Arm arm;
  private final Supplier<Angle> rollSupplier;

  private Rotation2d goal;

  private ClimberGoal state;
  private final ProfiledPIDController controller;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed; // TODO: Update this

  public Climber(ArmIO io, Supplier<Angle> rollSupplier) {
    setName("Climber");
    arm = new Arm(io, this, 1, ClimberConstants.CLIMBER_CONSTANTS);
    this.rollSupplier = rollSupplier;

    goal = Rotation2d.ZERO;

    state = ClimberGoal.DEFAULT;
    controller =
        new ProfiledPIDController(
            ClimberConstants.SLOT_2_GAINS.kP().get(),
            0,
            ClimberConstants.SLOT_2_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                ClimberConstants.ROLL_PID_CONSTANTS.maxVelocity(),
                ClimberConstants.ROLL_PID_CONSTANTS.maxAcceleration()));
    arm.setPosition(Rotation2d.ZERO);
  }

  @Override
  public void periodic() {
    arm.periodic();

    Logger.recordOutput(getName() + "/Goal", goal);
    Logger.recordOutput(getName() + "/At Goal Real", atGoal());
    Logger.recordOutput(getName() + "/State", state);
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
    return Commands.runOnce(() -> arm.setVoltageGoal(Volts.of(voltage)));
  }

  /**
   * Resets position of the arm subsystem to the given position.
   *
   * @param position the position to reset the arm to.
   * @return a command that resets the arm to the given position.
   */
  public Command setPosition(Rotation2d position) {
    return Commands.runOnce(() -> arm.setPosition(position));
  }

  /**
   * Sets the goal for the arm subsystem to the given position.
   *
   * @param positionGoal the Rotation2d to set the arm to.
   * @return A command that sets the arm to the given position and waits until the arm is at the
   *     goal.
   */
  public Command setPositionGoal(Rotation2d positionGoal, GainSlot gainSlot) {
    //    return Commands.runOnce(() -> goal = positionGoal)
    //        .andThen(
    //            Commands.either(
    //                arm.setVoltage(12),
    //                arm.setVoltage(-12),
    //                () -> positionGoal.minus(arm.getArmPosition()).getRadians() > 0),
    //            waitUntilPosition())
    //        .andThen(stop());

    return this.runOnce(
            () -> {
              goal = positionGoal;
              arm.setGainSlot(gainSlot);
            })
        .andThen(Commands.runOnce(() -> arm.setPositionGoal(positionGoal)), arm.waitUntilAtGoal())
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
    //    return Commands.runOnce(() -> goal = climberGoal.getPosition())
    //        .andThen(arm.setVoltage(12), waitUntilPosition())
    //        .andThen(setPositionGoal(climberGoal.getPosition(), GainSlot.ONE))
    //        .andThen(stop());
    return setPositionGoal(climberGoal.getPosition(), GainSlot.ONE);
  }

  public Command waitUntilPosition() {
    return Commands.waitUntil(this::atGoal);
  }

  public boolean atGoal() {
    return Math.abs(arm.getArmPosition().getRadians() - goal.getRadians())
        < ClimberConstants.CONSTRAINTS.goalTolerance().get().in(Radians);
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
        Commands.runOnce(() -> state = ClimberGoal.L1_AUTO_POSITION_GOAL), waitUntilPosition());
  }

  public Command runZeroSequence() { // add actual zeroing later
    return Commands.sequence(
        setPosition(Rotation2d.ZERO), setPositionGoal(Rotation2d.ZERO, GainSlot.ZERO));
  }

  public Command resetClimberZero() {
    return Commands.sequence(setPosition(Rotation2d.ZERO));
  }

  public Command clockwiseSlow() {
    return setVoltage(-ClimberConstants.SLOW_VOLTAGE);
  }

  public Command counterClockwiseSlow() {
    return setVoltage(ClimberConstants.SLOW_VOLTAGE);
  }

  public Command setPositionDefault() {
    return setPositionGoal(ClimberConstants.ClimberGoal.DEFAULT.getPosition(), GainSlot.ZERO);
  }

  public Command setPositionL1() {
    return setPositionGoal(
        ClimberConstants.ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ZERO);
  }

  public Command runSysId() {
    return arm.sysIdRoutine();
  }
}
