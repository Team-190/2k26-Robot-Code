package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralSpindexer extends SubsystemBase {
  private final V1_DoomSpiralSpindexerIO io;
  private final V1_DoomSpiralSpindexerIOInputsAutoLogged inputs;

  private double voltageGoal;
  private double kickerVoltageGoal;
  private double feederVoltageGoal;

  private final GenericRoller kicker;
  private final GenericRoller feeder;

  /**
   * Constructor for the DoomSpiral Spindexer subsystem.
   *
   * @param io the IO implementation
   */
  public V1_DoomSpiralSpindexer(
      V1_DoomSpiralSpindexerIO io,
      GenericRollerIO kickerIO,
      GenericRollerIO feederIO,
      String kickerName,
      String feederName) {
    this.io = io;
    inputs = new V1_DoomSpiralSpindexerIOInputsAutoLogged();
    kicker = new GenericRoller(kickerIO, this, kickerName);
    feeder = new GenericRoller(feederIO, this, feederName);

    voltageGoal = 0;
    kickerVoltageGoal = 0;
    feederVoltageGoal = 0;
  }

  /** Periodic method for the Spindexer subsystem. Updates inputs periodically. */
  @Override
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    io.setVoltage(voltageGoal);
    kicker.setVoltage(kickerVoltageGoal);
    feeder.setVoltage(feederVoltageGoal);

    kicker.periodic();
    feeder.periodic();
  }

  public Rotation2d getSpindexerPosition() {
    return inputs.position;
  }

  /**
   * Sets the voltage being passed into the spindexer subsystem.
   *
   * @param volts the voltage passed into the spindexer.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.parallel(
        setSpindexerVoltage(volts), setKickerVoltage(volts), setFeederVoltage(volts));
  }

  public Command setKickerVoltage(double volts) {
    return Commands.runOnce(() -> kickerVoltageGoal = volts);
  }

  public Command setFeederVoltage(double volts) {
    return Commands.runOnce(() -> feederVoltageGoal = volts);
  }

  public Command setSpindexerVoltage(double volts) {
    return Commands.runOnce(() -> voltageGoal = volts);
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
          kickerVoltageGoal = 0;
          feederVoltageGoal = 0;
        });
  }
}
