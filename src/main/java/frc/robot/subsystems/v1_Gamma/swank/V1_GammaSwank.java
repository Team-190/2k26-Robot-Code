package frc.robot.subsystems.v1_Gamma.swank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class V1_GammaSwank extends SubsystemBase {
  private final V1_GammaSwankIO io;
  private final V1_GammaSwankIOInputsAutoLogged inputs;

  public V1_GammaSwank(V1_GammaSwankIO io) {
    this.io = io;
    this.inputs = new V1_GammaSwankIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swank", inputs);
  }

  /**
   * Command that sets robot voltage
   * @param voltage
   * @return
   */
  public Command runVoltageCommand(double voltage) {
    return runOnce(() -> io.setVoltage(voltage));
  }

  /**
   * Command that stops robot by setting its voltage to 0
   * @return
   */
  public Command stopCommand() {
    return runOnce(() -> io.setVoltage(0));
  }
}
