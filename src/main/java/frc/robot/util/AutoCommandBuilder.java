package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.command.SuppliedWaitCommand;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Utility class for building autonomous command sequences with configurable timeout delays between
 * commands.
 *
 * <p>This class provides functionality to create command sequences where each command can be
 * preceded by a configurable delay/timeout. The delays are managed through NetworkTables and can be
 * tuned during runtime through logging frameworks like AdvantageScope.
 *
 * <p>The builder supports up to 6 configurable timeout values. Commands beyond the 6th position
 * will be executed without delays.
 *
 * @see SuppliedWaitCommand
 * @see LoggedNetworkNumber
 * @since 2026
 */
public class AutoCommandBuilder {

  private AutoCommandBuilder() {
    throw new IllegalStateException("Utility class must not be instantiated!");
  }

  /**
   * Array of logged network numbers representing configurable timeout/delay values.
   *
   * <p>Each timeout corresponds to a delay that can be inserted before the respective command in
   * the sequence. The timeouts are exposed through NetworkTables at:
   *
   * <ul>
   *   <li>/Auto/timeout1 - Delay before the 1st command
   *   <li>/Auto/timeout2 - Delay before the 2nd command
   *   <li>/Auto/timeout3 - Delay before the 3rd command
   *   <li>/Auto/timeout4 - Delay before the 4th command
   *   <li>/Auto/timeout5 - Delay before the 5th command
   *   <li>/Auto/timeout6 - Delay before the 6th command
   * </ul>
   *
   * <p>These values default to 0.0 seconds and can be modified at runtime through NetworkTables or
   * logging frameworks like AdvantageScope for autonomous tuning.
   */
  private static final LoggedNetworkNumber[] timeouts = {
    new LoggedNetworkNumber("/Auto/timeout1", 0.0),
    new LoggedNetworkNumber("/Auto/timeout2", 0.0),
    new LoggedNetworkNumber("/Auto/timeout3", 0.0),
    new LoggedNetworkNumber("/Auto/timeout4", 0.0),
    new LoggedNetworkNumber("/Auto/timeout5", 0.0),
    new LoggedNetworkNumber("/Auto/timeout6", 0.0),
  };

  /**
   * Builds a sequential command with configurable delays between individual commands.
   *
   * <p>Creates a command sequence where each command (up to the first 6) can be preceded by a
   * configurable delay. The delays are controlled by NetworkTables values that can be modified at
   * runtime for autonomous path tuning.
   *
   * <p>For commands beyond the 6th position, they will be appended to the sequence without any
   * preceding delays.
   *
   * <p><b>Example Usage:</b>
   *
   * <pre>{@code
   * // Create a sequence with three commands and configurable delays
   * Command auto = AutoCommandBuilder.sequence(
   *     driveForward(),
   *     shootNote(),
   *     driveBack()
   * );
   *
   * // The sequence will execute:
   * // 1. Wait for /Auto/timeout1 seconds (default: 0.0)
   * // 2. Execute driveForward()
   * // 3. Wait for /Auto/timeout2 seconds (default: 0.0)
   * // 4. Execute shootNote()
   * // 5. Wait for /Auto/timeout3 seconds (default: 0.0)
   * // 6. Execute driveBack()
   * }</pre>
   *
   * @param commands The commands to sequence together. Can be any number of commands.
   * @return A sequential command that executes all provided commands with configurable delays
   *     before the first 6 commands.
   * @throws NullPointerException if commands array is null or contains null commands
   * @see SuppliedWaitCommand
   * @see Commands#sequence(Command...)
   */
  public static Command sequence(Command... commands) {
    // Start with an empty command as the base of our sequence
    Command sequence = Commands.none();

    // Iterate through each provided command
    for (int i = 0; i < commands.length; i++) {

      // Check if we've exceeded the number of available timeout configurations
      if (i >= timeouts.length) {
        // For commands beyond the configurable limit, append them
        // without delays
        // This creates a standard WPILib sequence for the remaining commands
        Command remaining =
            Commands.sequence(java.util.Arrays.copyOfRange(commands, i, commands.length));
        sequence = sequence.andThen(remaining);
        break;
      }

      sequence = sequence.andThen(new SuppliedWaitCommand(timeouts[i]).andThen(commands[i]));
    }

    return sequence;
  }

  /**
   * Sets all timeout values to zero, effectively disabling all delays.
   *
   * <p>This method can be useful for testing or when you want to quickly disable all autonomous
   * delays without modifying NetworkTables values.
   */
  public static void resetAllTimeouts() {
    for (LoggedNetworkNumber timeout : timeouts) {
      timeout.set(0.0);
    }
  }
}
