package frc.robot.subsystems.v1_DoomSpiral.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class V1_DoomSpiralClimber extends SubsystemBase {
  private final Arm arm;
  private final Supplier<Angle> rollSupplier;

  private ClimberGoal state;
  private final ProfiledPIDController controller;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed; // TODO: Update this

  public V1_DoomSpiralClimber(ArmIO io, Supplier<Angle> rollSupplier) {
    arm = new Arm(io, this, 1);
    this.rollSupplier = rollSupplier;

    state = ClimberGoal.DEFAULT;
    controller =
        new ProfiledPIDController(
            V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.kP(),
            0,
            V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.kD(),
            new TrapezoidProfile.Constraints(
                V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.maxVelocity(),
                V1_DoomSpiralClimberConstants.ROLL_PID_CONSTANTS.maxAcceleration()));

    setDefaultCommand(setPositionGoal(state.getPosition()));
  }

  @Override
  public void periodic() {
    arm.periodic();
  }

  public Rotation2d getArmPosition() {
    return arm.inputs.position;
  }

  public Command setVoltage(double voltage) {
    return runOnce(() -> arm.setVoltage(voltage));
  }

  public Command setPosition(Rotation2d position) {
    return Commands.runOnce(() -> arm.setPosition(position));
  }

  public Command setPositionGoal(Rotation2d positionGoal) {
    return runOnce(() -> arm.setPositionGoal(positionGoal));
  }

  public Command waitUntilPosition() {
    return Commands.waitUntil(this::atGoal);
  }

  public boolean atGoal() {
    return arm.atGoal();
  }

  public Command stop() {
    return Commands.runOnce(() -> arm.setVoltage(0));
  }

  public Command climbSequenceL3() {
    return Commands.sequence(
        Commands.runOnce(() -> state = ClimberGoal.L1_POSITION_GOAL),
        setVoltage(12).until(this::atGoal),
        Commands.runOnce(() -> state = ClimberGoal.L2_FLIP_GOAL),
        setVoltage(-12).until(this::atGoal),
        setVoltage(controller.calculate(rollSupplier.get().in(Radians), Math.PI))
            .until(() -> rollSupplier.get().isNear(Radians.of(Math.PI), Degrees.of(1.0))));
  }

  public Command climbAutoSequence() {
    return Commands.sequence(
        Commands.runOnce(() -> state = ClimberGoal.L1_AUTO_POSITION_GOAL), waitUntilPosition());
  }

  public Command runZeroSequence() { // add actual zeroing later
    return Commands.sequence(setPosition(Rotation2d.kZero), setPositionGoal(Rotation2d.kZero));
  }

  public Command resetClimberZero() {
    return Commands.sequence(setPosition(Rotation2d.kZero), setPositionGoal(Rotation2d.kZero));
  }

  public Command clockwiseSlow() {
    return Commands.runOnce(() -> setVoltage(-V1_DoomSpiralClimberConstants.SLOW_VOLTAGE));
  }

  public Command counterClockwiseSlow() {
    return Commands.runOnce(() -> setVoltage(V1_DoomSpiralClimberConstants.SLOW_VOLTAGE));
  }

  public Command setPositionDefault() {
    return Commands.runOnce(
        () -> setPositionGoal(V1_DoomSpiralClimberConstants.ClimberGoal.DEFAULT.getPosition()));
  }

  public Command setPositionL1() {
    return Commands.runOnce(
        () ->
            setPositionGoal(
                V1_DoomSpiralClimberConstants.ClimberGoal.L1_POSITION_GOAL.getPosition()));
  }
}
