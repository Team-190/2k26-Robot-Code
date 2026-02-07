package frc.robot.subsystems.v1_DoomSpiral.swank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralSwank extends SubsystemBase {
  private final V1_DoomSpiralSwankIO io;
  private final V1_DoomSpiralSwankIOInputsAutoLogged inputs;

  private double voltageGoal;

  public V1_DoomSpiralSwank(V1_DoomSpiralSwankIO io) {
    this.io = io;
    this.inputs = new V1_DoomSpiralSwankIOInputsAutoLogged();
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
    return runOnce(() -> voltageGoal = 0);
  }
}
