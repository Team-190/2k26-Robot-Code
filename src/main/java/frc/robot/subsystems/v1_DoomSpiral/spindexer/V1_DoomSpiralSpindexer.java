package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;

import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralSpindexer extends SubsystemBase {
  private final V1_DoomSpiralSpindexerIO io;
  private final V1_DoomSpiralSpindexerIOInputsAutoLogged inputs;
  private double voltageGoal;
  private GenericRoller kicker;
  private GenericRoller feeder;
  private V1_DoomSpiralRobotState robotState;

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
  }

  /** Periodic method for the Spindexer subsystem. Updates inputs periodically. */
  @Override
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    io.setVoltage(voltageGoal);
    kicker.setVoltage(voltageGoal);
    feeder.setVoltage(voltageGoal);

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

  public Command increaseSpindexerSpeed() {
    voltageGoal += 0.25;
    return Commands.runOnce(() -> this.setVoltage(voltageGoal));
  }

  public Command decreaseSpindexerSpeed() {
    voltageGoal -= 0.25;
    return Commands.runOnce(() -> this.setVoltage(voltageGoal));
  }

  public Command decreaseFeederSpeed() {
    voltageGoal -= 0.25;
    return Commands.runOnce(() -> feeder.setVoltage(voltageGoal));
  }

  public Command increaseFeederSpeed() {
    voltageGoal += 0.25;
    return Commands.runOnce(() -> feeder.setVoltage(voltageGoal));
  }
}
