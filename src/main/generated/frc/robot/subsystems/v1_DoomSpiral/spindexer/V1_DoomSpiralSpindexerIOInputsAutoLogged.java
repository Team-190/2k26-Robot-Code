package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class V1_DoomSpiralSpindexerIOInputsAutoLogged
    extends V1_DoomSpiralSpindexerIO.V1_DoomSpiralSpindexerIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrent", supplyCurrent);
    table.put("TorqueCurrent", torqueCurrent);
    table.put("Temperature", temperature);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
    torqueCurrent = table.get("TorqueCurrent", torqueCurrent);
    temperature = table.get("Temperature", temperature);
  }

  public V1_DoomSpiralSpindexerIOInputsAutoLogged clone() {
    V1_DoomSpiralSpindexerIOInputsAutoLogged copy = new V1_DoomSpiralSpindexerIOInputsAutoLogged();
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrent = this.supplyCurrent;
    copy.torqueCurrent = this.torqueCurrent;
    copy.temperature = this.temperature;
    return copy;
  }
}
