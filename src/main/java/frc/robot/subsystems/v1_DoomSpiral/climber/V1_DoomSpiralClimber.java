package frc.robot.subsystems.v1_DoomSpiral.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants.ClimberGoal;
import org.littletonrobotics.junction.AutoLogOutput;

public class V1_DoomSpiralClimber extends SubsystemBase {
  private final Arm arm;
  private ClimberGoal state;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed; // TODO: Update this

  public V1_DoomSpiralClimber(ArmIO io) {
    state = ClimberGoal.DEFAULT;
    arm = new Arm(io, this, 1);
    arm.setPositionGoal(state.getPosition());
  }

  @Override
  public void periodic() {
    arm.periodic();
  }

  public Rotation2d getArmPosition() {
    return arm.inputs.position;
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(() -> arm.setVoltage(voltage));
  }

  public Command setPosition(Rotation2d position) {
    return Commands.runOnce(() -> arm.setPosition(position));
  }

  public Command setPositionGoal(Rotation2d positionGoal) {
    return Commands.runOnce(() -> arm.setPositionGoal(positionGoal));
  }

  public Command waitUntilPosition() {
    return Commands.waitUntil(this::atGoal);
  }

  public boolean atGoal() {
    return Math.abs(arm.inputs.position.getRadians() - arm.inputs.positionGoal.getRadians())
        < V1_DoomSpiralClimberConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  public Command stop() {
    return Commands.runOnce(() -> arm.setVoltage(0));
  }

  public Command climbSequenceL3() {
    return Commands.sequence(
        Commands.runOnce(() -> state = ClimberGoal.L1_POSITION_GOAL),
        waitUntilPosition(),
        Commands.runOnce(() -> state = ClimberGoal.L2_FLIP_GOAL),
        waitUntilPosition(),
        Commands.runOnce(() -> state = ClimberGoal.L2_POSITION_GOAL),
        waitUntilPosition());
  }

  public Command climbAutoSequence() {
    return Commands.sequence(
        Commands.runOnce(() -> state = ClimberGoal.L1_AUTO_POSITION_GOAL), waitUntilPosition());
  }

  public Command runZeroSequence(){
    return
        Commands.runOnce(() -> setPosition(new Rotation2d(0))).andThen(Commands.runOnce(() -> setPositionGoal(new Rotation2d(0))));
  }
}
