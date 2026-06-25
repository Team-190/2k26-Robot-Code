package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final GenericRollerIOInputsAutoLogged inputs;

  private final String aKitTopic;

  @Getter private Setpoint<VoltageUnit> voltageGoal;

  public GenericRoller(
      GenericRollerIO io,
      Subsystem subsystem,
      GenericRollerConstants constants,
      String name,
      Setpoint<VoltageUnit> voltageGoal) {
    this.io = io;
    inputs = new GenericRollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Roller" + name;
    this.voltageGoal = voltageGoal;
  }

  public GenericRoller(
      GenericRollerIO io, Subsystem subsystem, GenericRollerConstants constants, String name) {
    this(
        io,
        subsystem,
        constants,
        name,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());

    io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public void setVoltageGoal(Voltage voltage) {
    voltageGoal.setSetpoint(voltage);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltage) {
    voltageGoal = voltage;
  }

  public double[] getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
