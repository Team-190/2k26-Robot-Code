package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.command.SuppliedWaitCommand;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoCommandBuilder {

  private static final LoggedNetworkNumber[] timeouts = {
    new LoggedNetworkNumber("/Auto/timeout1", 0.0),
    new LoggedNetworkNumber("/Auto/timeout2", 0.0),
    new LoggedNetworkNumber("/Auto/timeout3", 0.0),
    new LoggedNetworkNumber("/Auto/timeout4", 0.0),
    new LoggedNetworkNumber("/Auto/timeout5", 0.0),
    new LoggedNetworkNumber("/Auto/timeout6", 0.0),
  };

  public static Command getCommand(Command... commands) {
    Command sequence = Commands.none();

    for (int i = 0; i < commands.length; i++) {

      if (i >= timeouts.length) {
        // Append remaining commands without delays
        Command remaining =
            Commands.sequence(java.util.Arrays.copyOfRange(commands, i, commands.length));
        sequence = sequence.andThen(remaining);
        break;
      }

      sequence = sequence.andThen(new SuppliedWaitCommand(timeouts[i]).andThen(commands[i]));
    }

    return sequence;
  }
}
