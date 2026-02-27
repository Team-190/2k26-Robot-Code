package frc.robot.util.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * A command that does nothing but takes a specified amount of time to finish. Unlike the built-in
 * WaitCommand, this class accepts a supplier for the duration (especially useful when combined with
 * LoggedTunableNumber).
 */
public class SuppliedWaitCommand extends Command {
  protected Timer timer = new Timer();
  private final DoubleSupplier m_duration;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds A supplier for the time to wait, in seconds
   */
  public SuppliedWaitCommand(DoubleSupplier seconds) {
    m_duration = seconds;
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_duration.getAsDouble());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
