package frc.robot.subsystems.v1_Gamma.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class V1_GammaSpindexer extends SubsystemBase {
    private final V1_GammaSpindexerIO io;
    private final V1_GammaSpindexerIOInputsAutoLogged inputs;

    public V1_GammaSpindexer(V1_GammaSpindexerIO io) {
        this.io = io;
        inputs = new V1_GammaSpindexerIOInputsAutoLogged();
    }

    @Override
    @Trace
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
    }

    public void stopSpindexer(){
        io.setVoltage(0);
    }

}
