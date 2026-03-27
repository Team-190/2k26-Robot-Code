package frc.robot.subsystems.shared.hood;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodIOInputsAutoLogged extends HoodIO.HoodIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrent", supplyCurrent);
    table.put("TorqueCurrent", torqueCurrent);
    table.put("Temperature", temperature);
    table.put("PositionGoal", positionGoal);
    table.put("PositionSetpoint", positionSetpoint);
    table.put("PositionError", positionError);
  }

  @Override
  public void fromLog(LogTable table) {
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
    torqueCurrent = table.get("TorqueCurrent", torqueCurrent);
    temperature = table.get("Temperature", temperature);
    positionGoal = table.get("PositionGoal", positionGoal);
    positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
    positionError = table.get("PositionError", positionError);
  }

  public HoodIOInputsAutoLogged clone() {
    HoodIOInputsAutoLogged copy = new HoodIOInputsAutoLogged();
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrent = this.supplyCurrent;
    copy.torqueCurrent = this.torqueCurrent;
    copy.temperature = this.temperature;
    copy.positionGoal = this.positionGoal;
    copy.positionSetpoint = this.positionSetpoint;
    copy.positionError = this.positionError;
    return copy;
  }
}
