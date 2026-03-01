package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerState.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Offset;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralSpindexer extends SubsystemBase {
  private final V1_DoomSpiralSpindexerIO io;
  private final V1_DoomSpiralSpindexerIOInputsAutoLogged inputs;

  private V1_DoomSpiralSpindexerState state;
  private final Offset<VoltageUnit> voltageGoal;

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
    setName("Spindexer");
    this.io = io;
    inputs = new V1_DoomSpiralSpindexerIOInputsAutoLogged();
    kicker =
        new GenericRoller(
            kickerIO, this, V1_DoomSpiralSpindexerConstants.KICKER_ROLLER_CONSTANTS, kickerName);
    feeder =
        new GenericRoller(
            feederIO, this, V1_DoomSpiralSpindexerConstants.FEEDER_ROLLER_CONSTANTS, feederName);

    state = STOP;
    voltageGoal =
        new Offset<>(
            Volts.zero(),
            V1_DoomSpiralSpindexerConstants.SPINDEXER_INCREMENT_VOLTAGE,
            Volts.of(-12),
            Volts.of(12));
  }

  /** Periodic method for the Spindexer subsystem. Updates inputs periodically. */
  @Override
  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    Logger.recordOutput(getName() + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(getName() + "/Voltage Offset", voltageGoal.getOffset());

    io.setVoltage(
        switch (state) {
          case STOP -> 0.00;
          case OPEN_LOOP_VOLTAGE, SPINDEXER_ONLY_VOLTAGE -> voltageGoal.getNewSetpoint().in(Volts);
        });

    kicker.periodic();
    feeder.periodic();
  }

  /**
   * Gets the position of the spindexer subsystem.
   *
   * @return The position of the spindexer subsystem.
   */
  public Rotation2d getSpindexerPosition() {
    return inputs.position;
  }

  /**
   * Sets the voltage being passed into the spindexer, feeder, and kicker subsystems.
   *
   * @param spindexerVolts the voltage passed into the spindexer subsystem.
   * @param feederVolts the voltage passed into the feeder subsystem.
   * @param kickerVolts the voltage passed into the kicker subsystem.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double spindexerVolts, double feederVolts, double kickerVolts) {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              state = OPEN_LOOP_VOLTAGE;
              voltageGoal.setSetpoint(Volts.of(spindexerVolts));
            }),
        kicker.setVoltage(kickerVolts),
        feeder.setVoltage(feederVolts));
  }

  /**
   * Sets the voltage being passed into the spindexer, feeder, and kicker subsystems.
   *
   * @param allVolts the voltage passed into the spindexer, feeder, and kicker.
   * @return A command that sets the specified voltage.
   */
  public Command setVoltage(double allVolts) {
    return setVoltage(allVolts, allVolts, allVolts);
  }

  /**
   * Sets the voltage being passed into the spindexer subsystem only.
   *
   * @param volts the voltage passed into the spindexer.
   * @return A command that sets the specified voltage.
   */
  public Command setSpindexerOnlyVoltage(double volts) {
    return Commands.runOnce(
            () -> {
              state = SPINDEXER_ONLY_VOLTAGE;
              voltageGoal.setSetpoint(Volts.of(volts));
            })
        .alongWith(kicker.setVoltage(0), feeder.setVoltage(0));
  }

  public Command stopSpindexer() {
    return Commands.parallel(
        Commands.runOnce(() -> state = STOP), kicker.setVoltage(0), feeder.setVoltage(0.0));
  }

  public Command increaseSpindexerVoltage() {
    return Commands.runOnce(voltageGoal::increment);
  }

  public Command decreaseSpindexerVoltage() {
    return Commands.runOnce(voltageGoal::decrement);
  }

  public Command decreaseFeederVoltage() {
    return feeder.decrementVoltageOffset();
  }

  public Command increaseFeederVoltage() {
    return feeder.incrementVoltageOffset();
  }

  public Command increaseKickerVoltage() {
    return kicker.incrementVoltageOffset();
  }

  public Command decreaseKickerVoltage() {
    return kicker.decrementVoltageOffset();
  }
}
