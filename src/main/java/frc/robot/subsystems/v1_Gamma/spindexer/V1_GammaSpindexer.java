package frc.robot.subsystems.v1_Gamma.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;

import org.littletonrobotics.junction.Logger;

public class V1_GammaSpindexer extends SubsystemBase {
  private final V1_GammaSpindexerIO io;
  private final V1_GammaSpindexerIOInputsAutoLogged inputs;
  private double voltageGoal;
  private GenericRoller kicker;
  private int index;

  /**
   * Constructor for the Gamma Spindexer subsystem.
   *
   * @param io the IO implementation
   */
  public V1_GammaSpindexer(V1_GammaSpindexerIO io, GenericRollerIO kickerIO, int index) {
    this.io = io;
    inputs = new V1_GammaSpindexerIOInputsAutoLogged();
    kicker = new GenericRoller(kickerIO, this, index);
  }

  /** Periodic method for the Spindexer subsystem. Updates inputs periodically. */
  @Override
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    io.setVoltage(voltageGoal);
    kicker.periodic();
  }

  /**
   * Sets the voltage being passed into the spindexer subsystem.
   *
   * @param volts the voltage passed into the spindexer.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          voltageGoal = volts;
        });
  }

  /**
   * Stops the spindexer.
   *
   * @return A command that sets the voltage to zero.
   */
  public Command stopSpindexer() {
    return Commands.runOnce(
        () -> {
          voltageGoal = 0;
        });
  }
  
  public void setKicker(GenericRoller kicker) {
    this.kicker = kicker;
  }
    
}
