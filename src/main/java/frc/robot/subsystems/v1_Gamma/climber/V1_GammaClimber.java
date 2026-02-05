package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import org.littletonrobotics.junction.AutoLogOutput;

public class V1_GammaClimber extends SubsystemBase {
  private Arm arm;
  private V1_GammaClimberConstants.ClimberGoal state;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public V1_GammaClimber(ArmIO io) {
    state = V1_GammaClimberConstants.ClimberGoal.DEFAULT;
    arm = new Arm(io, this, 1);
    arm.setPositionGoal(state.getPosition());
  }

  @Override
  public void periodic() {
    arm.setPositionGoal(state.getPosition());
    arm.periodic();
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
    return Commands.waitUntil(
        () ->
            Math.abs(arm.inputs.position.getRadians() - arm.inputs.positionGoal.getRadians())
                < V1_GammaClimberConstants.positionToleranceRadians);
  }

  public Command stop() {
    return Commands.runOnce(() -> arm.setVoltage(0));
  }

  public Command climbSequenceL3() {
    return Commands.sequence(
        Commands.runOnce(() -> state = V1_GammaClimberConstants.ClimberGoal.L1_POSITION_GOAL),
        waitUntilPosition(),
        Commands.runOnce(() -> state = V1_GammaClimberConstants.ClimberGoal.L2_FLIP_GOAL),
        waitUntilPosition(),
        Commands.runOnce(() -> state = V1_GammaClimberConstants.ClimberGoal.L2_POSITION_GOAL),
        waitUntilPosition());
  }

  public Command climbAutoSequence() {
    return Commands.sequence(
        Commands.runOnce(() -> state = V1_GammaClimberConstants.ClimberGoal.L1_AUTO_POSITION_GOAL),
        waitUntilPosition());
  }
}
