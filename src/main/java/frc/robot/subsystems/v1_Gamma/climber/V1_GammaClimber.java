package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_GammaClimber extends SubsystemBase {
  private final V1_GammaClimberIO io;
  private final V1_GammaClimberIOInputsAutoLogged inputs;
  private final String aKitTopic;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public V1_GammaClimber(V1_GammaClimberIO io) {
    this.io = io;
    this.inputs = new V1_GammaClimberIOInputsAutoLogged();

    aKitTopic = this.getName();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
  }

  public void runVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public Command runVoltageCommand(double voltage) {
    return runOnce(() -> io.setVoltage(voltage));
  }

  public Command stopCommand() {
    return runOnce(() -> io.setVoltage(0));
  }
}
