package frc.robot.subsystems.v1_Gamma.swank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class V1_GammaSwank extends SubsystemBase {
  private final V1_GammaSwankIO io;
  private final V1_GammaSwankIOInputsAutoLogged inputs;

  private double voltageGoal;

  public V1_GammaSwank(V1_GammaSwankIO io) {
    this.io = io;
    this.inputs = new V1_GammaSwankIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swank", inputs);
    io.setVoltage(voltageGoal);
  }

  public Command setVoltage(double voltage) {
    return runOnce(() -> voltageGoal = voltage);
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0));
  }
}
