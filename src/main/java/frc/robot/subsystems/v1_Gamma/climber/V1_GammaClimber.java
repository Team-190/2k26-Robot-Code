package frc.robot.subsystems.v1_Gamma.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_GammaClimber {
  private final V1_GammaClimberIO io;
  private final V1_GammaClimberIOInputsAutoLogged inputs;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public V1_GammaClimber(V1_GammaClimberIO io) {
    this.io = io;
    this.inputs = new V1_GammaClimberIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command runVoltageCommand(double voltage) {
    return Commands.runOnce(() -> io.setVoltage(voltage));
  }

  public Command stopCommand() {
    return Commands.runOnce(() -> io.setVoltage(0));
  }
}
