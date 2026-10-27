package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

public class Arm {
  public ArmIO io;
  public ArmIOInputsAutoLogged inputs;

  private final String aKitTopic;

  @Getter private ArmState currentState;

  @Getter private Setpoint<VoltageUnit> voltageGoal;
  @Getter private Setpoint<AngleUnit> positionGoal;

  private final SysIdRoutine characterizationRoutine;

  public Arm(
      ArmIO io,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Arms" + index;

    currentState = ArmState.IDLE;

    this.positionGoal = positionGoal;
    this.voltageGoal = voltageGoal;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(9),
                Seconds.of(12),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    constants.slot0Gains.onChange(gains -> io.updateGains(gains, GainSlot.ZERO));
    constants.slot1Gains.onChange(gains -> io.updateGains(gains, GainSlot.ONE));
    constants.slot2Gains.onChange(gains -> io.updateGains(gains, GainSlot.TWO));
    constants.constraints.onChange(io::updateConstraints);
  }

  public Arm(ArmIO io, Subsystem subsystem, int index, ArmConstants constants) {
    this(
        io,
        subsystem,
        index,
        constants,
        new Setpoint<>(
            Rotation2d.ZERO.getMeasure(),
            constants.positionOffsetStep.getMeasure(),
            constants.armParameters.maxAngle().getMeasure(),
            constants.armParameters.minAngle().getMeasure()),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public Arm(
      ArmIO io,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal) {
    this(
        io,
        subsystem,
        index,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(
        aKitTopic + "/Position Goal", new Rotation2d((Angle) positionGoal.getSetpoint()));
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Position Offset", positionGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal(new Rotation2d((Angle) positionGoal.getNewSetpoint()));
      case IDLE -> {}
    }
  }

  public Rotation2d getArmPosition() {
    return inputs.position;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal.getMeasure());
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atPositionGoal(Rotation2d positionReference) {
    return io.atPositionGoal(positionReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atPositionGoal() {
    return atPositionGoal(new Rotation2d((Angle) positionGoal.getNewSetpoint()));
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public void setGainSlot(GainSlot slot) {
    io.setGainSlot(slot);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(Gains gains, GainSlot slot) {
    io.updateGains(gains, slot);
  }

  public void updateConstraints(AngularPositionConstraints constraints) {
    io.updateConstraints(constraints);
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ArmState.IDLE),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.REVERSE),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.REVERSE));
  }
}
