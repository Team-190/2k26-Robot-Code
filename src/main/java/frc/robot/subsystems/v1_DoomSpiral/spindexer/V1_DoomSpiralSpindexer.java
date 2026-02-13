package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import static frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerState.*;

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

  private V1_DoomSpiralSpindexerState state;
  private double feederVoltageGoal;
  private final double kickerVoltageGoal;
  private double voltageGoal;

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

    state = STOP;
    feederVoltageGoal = 0;
    kickerVoltageGoal = 0;
    voltageGoal = 0;
  }

  /** Periodic method for the Spindexer subsystem. Updates inputs periodically. */
  @Override
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    switch (state) {
      case STOP -> {
        io.setVoltage(0);
      }
      case OPEN_LOOP_VOLTAGE -> {
        io.setVoltage(voltageGoal);
        feederVoltageGoal = feeder.getVoltageGoalVolts();
        kickerVoltageGoal = kicker.getVoltageGoalVolts();
      }
      case SPINDEXER_ONLY_VOLTAGE -> io.setVoltage(voltageGoal);
    }

    kicker.periodic();
    feeder.periodic();
  }

  public Rotation2d getSpindexerPosition() {
    return inputs.position;
  }

  /**
   * Sets the voltage being passed into the spindexer subsystem.
   *
   * @param spindexerVolts the voltage passed into the spindexer.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double spindexerVolts, double feederVolts, double kickerVolts) {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              state = OPEN_LOOP_VOLTAGE;
              voltageGoal = spindexerVolts;
            }),
        kicker.setVoltage(kickerVolts),
        feeder.setVoltage(feederVolts));
  }

  public Command setVoltage(double allVolts) {
    return setVoltage(allVolts, allVolts, allVolts);
  }

  public Command setSpindexerVoltage(double volts) {
    return Commands.runOnce(
            () -> {
              state = SPINDEXER_ONLY_VOLTAGE;
              voltageGoal = volts;
            })
        .alongWith(kicker.setVoltage(0), feeder.setVoltage(0));
  }

  /**
   * Stops the spindexer.
   *
   * @return A command that sets the voltage to zero.
   */
  public Command stopSpindexer() {
    return Commands.runOnce(() -> state = STOP);
  }

  public Command increaseSpindexerVoltage() {
    return Commands.runOnce(
        () -> voltageGoal += V1_DoomSpiralSpindexerConstants.SPINDEXER_INCREMENT_VOLTAGE);
  }

  public Command decreaseSpindexerVoltage() {
    return Commands.runOnce(
        () -> voltageGoal -= V1_DoomSpiralSpindexerConstants.SPINDEXER_INCREMENT_VOLTAGE);
  }

  public Command decreaseFeederVoltage() {
    return Commands.runOnce(
        () -> feederVoltageGoal -= V1_DoomSpiralSpindexerConstants.SPINDEXER_INCREMENT_VOLTAGE);
  }

  public Command increaseFeederVoltage() {
    return Commands.runOnce(
        () -> feederVoltageGoal += V1_DoomSpiralSpindexerConstants.SPINDEXER_INCREMENT_VOLTAGE);
  }
}
