package frc.robot.subsystems.shared.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TurretIOInputsAutoLogged extends TurretIO.TurretIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Angle", angle);
    table.put("Velocity", velocity);
    table.put("AppliedVoltage", appliedVoltage);
    table.put("SupplyCurrent", supplyCurrent);
    table.put("TorqueCurrent", torqueCurrent);
    table.put("Temperature", temperature);
    table.put("PositionGoal", positionGoal);
    table.put("PositionSetpoint", positionSetpoint);
    table.put("PositionError", positionError);
    table.put("Encoder1Position", encoder1Position);
    table.put("Encoder2Position", encoder2Position);
  }

  @Override
  public void fromLog(LogTable table) {
    angle = table.get("Angle", angle);
    velocity = table.get("Velocity", velocity);
    appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
    supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
    torqueCurrent = table.get("TorqueCurrent", torqueCurrent);
    temperature = table.get("Temperature", temperature);
    positionGoal = table.get("PositionGoal", positionGoal);
    positionSetpoint = table.get("PositionSetpoint", positionSetpoint);
    positionError = table.get("PositionError", positionError);
    encoder1Position = table.get("Encoder1Position", encoder1Position);
    encoder2Position = table.get("Encoder2Position", encoder2Position);
  }

  public TurretIOInputsAutoLogged clone() {
    TurretIOInputsAutoLogged copy = new TurretIOInputsAutoLogged();
    copy.angle = this.angle;
    copy.velocity = this.velocity;
    copy.appliedVoltage = this.appliedVoltage;
    copy.supplyCurrent = this.supplyCurrent;
    copy.torqueCurrent = this.torqueCurrent;
    copy.temperature = this.temperature;
    copy.positionGoal = this.positionGoal;
    copy.positionSetpoint = this.positionSetpoint;
    copy.positionError = this.positionError;
    copy.encoder1Position = this.encoder1Position;
    copy.encoder2Position = this.encoder2Position;
    return copy;
  }
}
