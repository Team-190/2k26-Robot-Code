package frc.robot.subsystems.v1_Gamma.spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;



public class V1_GammaSpindexer extends SubsystemBase {
    private final V1_GammaSpindexerIO io;
    private final V1_GammaSpindexerIOInputsAutoLogged inputs;
    private double voltageGoal;

    /**
     * Constructor for the Gamma Spindexer subsystem. 
     * @param io the IO implementation
     */
    public V1_GammaSpindexer(V1_GammaSpindexerIO io) {
        this.io = io;
        inputs = new V1_GammaSpindexerIOInputsAutoLogged();
    }

    /** Periodic method for the Spindexer subsystem. Updates inputs periodically.*/
    @Override
    @Trace
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        Commands.runOnce(
            () -> {
            io.setVoltage(voltageGoal);
            });
    }

      /**
       * Sets the voltage being passed into the spindexer subsystem.
       * @param volts the voltage passed into the spindexer.
       * @return A command that sets the specified voltage.
       */
    public Command setVoltage(double volts) {
        return Commands.runOnce(
            () -> {voltageGoal = volts;
            });
    }

    /**
     * Stops the spindexer.
     * @return A command that sets the voltage to zero.
     */
  public Command stopSpindexer() {
    return Commands.runOnce(
        () -> {
          io.setVoltage(0);
        });
  }

}
