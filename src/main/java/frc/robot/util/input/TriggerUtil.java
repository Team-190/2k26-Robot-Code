package frc.robot.util.input;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;

public class TriggerUtil {
  private TriggerUtil() {}

  public static Trigger toggleBetweenCommandsOnTrue(
      Trigger trigger, final Command first, final Command second) {
    AtomicBoolean useFirst = new AtomicBoolean(true);

    trigger.onTrue(
        Commands.runOnce(
            () -> {
              if (useFirst.get()) {
                CommandScheduler.getInstance().cancel(second);
                CommandScheduler.getInstance().schedule(first);
              } else {
                CommandScheduler.getInstance().cancel(first);
                CommandScheduler.getInstance().schedule(second);
              }
              useFirst.set(!useFirst.get());
            }));

    return trigger;
  }
}
