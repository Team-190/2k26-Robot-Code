package frc.robot.subsystems.v1_Gamma.swank;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;

public class V1_GammaSwank extends SubsystemBase {
    private final V1_GammaSwankIO io;
    private final V1_GammaSwankIOInputsAutoLogged inputs;
    private final String aKitTopic;
    
    public V1_GammaSwank(
      V1_GammaSwankIO io, int index) {
        this.io = io;
        this.inputs = new V1_GammaSwankIOInputsAutoLogged();    
        
        aKitTopic = this.getName() + "/Swank" + index; 
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
