package frc.robot.subsystems.shared.climber;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import frc.robot.subsystems.shared.climber.ClimberConstants.ClimberGoal;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
    arm = new Arm(io, this, 1);
    this.rollSupplier = rollSupplier;

    goal = Rotation2d.kZero;

    state = ClimberGoal.DEFAULT;
    controller =
        new ProfiledPIDController(
            ClimberConstants.SLOT_2_GAINS.kP().get(),
            0,
            ClimberConstants.SLOT_2_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                ClimberConstants.CONSTRAINTS.maxVelocity().get(RadiansPerSecond),
                ClimberConstants.CONSTRAINTS.maxAcceleration().get(RadiansPerSecondPerSecond)));
    CommandScheduler.getInstance().schedule(arm.setPosition(Rotation2d.kZero));
  }

  @Override
  public void periodic() {
    arm.periodic();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            arm.updateGains(
                ClimberConstants.SLOT_0_GAINS.kP().get(),
                ClimberConstants.SLOT_0_GAINS.kD().get(),
                ClimberConstants.SLOT_0_GAINS.kS().get(),
                ClimberConstants.SLOT_0_GAINS.kV().get(),
                ClimberConstants.SLOT_0_GAINS.kA().get(),
                ClimberConstants.SLOT_0_GAINS.kG().get(),
                GainSlot.ZERO),
        ClimberConstants.SLOT_0_GAINS.kP(),
        ClimberConstants.SLOT_0_GAINS.kD(),
        ClimberConstants.SLOT_0_GAINS.kS(),
        ClimberConstants.SLOT_0_GAINS.kV(),
        ClimberConstants.SLOT_0_GAINS.kA(),
        ClimberConstants.SLOT_0_GAINS.kG());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            arm.updateGains(
                ClimberConstants.SLOT_1_GAINS.kP().get(),
                ClimberConstants.SLOT_1_GAINS.kD().get(),
                ClimberConstants.SLOT_1_GAINS.kS().get(),
                ClimberConstants.SLOT_1_GAINS.kV().get(),
                ClimberConstants.SLOT_1_GAINS.kA().get(),
                ClimberConstants.SLOT_1_GAINS.kG().get(),
                GainSlot.ONE),
        ClimberConstants.SLOT_1_GAINS.kP(),
        ClimberConstants.SLOT_1_GAINS.kD(),
        ClimberConstants.SLOT_1_GAINS.kS(),
        ClimberConstants.SLOT_1_GAINS.kV(),
        ClimberConstants.SLOT_1_GAINS.kA(),
        ClimberConstants.SLOT_1_GAINS.kG());

    LoggedTunableMeasure.ifChanged(
        hashCode(),
        () ->
            arm.updateConstraints(
                ClimberConstants.CONSTRAINTS.maxAcceleration().get(RadiansPerSecondPerSecond),
                ClimberConstants.CONSTRAINTS.maxVelocity().get(RadiansPerSecond),
                ClimberConstants.CONSTRAINTS.goalTolerance().get(Radians)),
        ClimberConstants.CONSTRAINTS.maxAcceleration(),
        ClimberConstants.CONSTRAINTS.maxVelocity(),
        ClimberConstants.CONSTRAINTS.goalTolerance());

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
              arm.setSlot(gainSlot);
            })
        .andThen(arm.setPositionGoal(positionGoal), arm.waitUntilAtGoal())
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
        setPosition(Rotation2d.kZero), setPositionGoal(Rotation2d.kZero, GainSlot.ZERO));
  }

  public Command resetClimberZero() {
    return Commands.sequence(setPosition(Rotation2d.kZero));
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
        ClimberConstants.ClimberGoal.L1_POSITION_GOAL.getPosition(), GainSlot.ONE);
  }

  public Command runSysId() {
    return arm.sysIdRoutine();
  }
}
