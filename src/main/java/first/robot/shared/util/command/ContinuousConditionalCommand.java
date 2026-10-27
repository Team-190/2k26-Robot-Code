// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.shared.util.command;

import static org.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.function.BooleanSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.telemetry.TelemetryTable;

/**
 * A command composition that runs one of two commands, depending on the value of the given
 * condition.
 *
 * <p>Unlike the standard ConditionalCommand, this version evaluates the condition continuously. If
 * the condition changes during execution, the currently running command is interrupted (ended) and
 * the other command is initialized.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 */
public class ContinuousConditionalCommand extends Command {
  private final Command onTrue;
  private final Command onFalse;
  private final BooleanSupplier condition;
  private Command selectedCommand;

  /**
   * Creates a new ContinuousConditionalCommand.
   *
   * @param onTrue The command to run while the condition is true
   * @param onFalse The command to run while the condition is false
   * @param condition The condition to determine which command to run
   */
  public ContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
    this.onTrue = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
    this.onFalse = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
    this.condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

    CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);

    addRequirements(onTrue.getRequirements());
    addRequirements(onFalse.getRequirements());
  }

  @Override
  public void initialize() {
    if (condition.getAsBoolean()) {
      selectedCommand = onTrue;
    } else {
      selectedCommand = onFalse;
    }
    selectedCommand.initialize();
  }

  @Override
  public void execute() {
    // Check condition to see if we need to swap commands
    boolean conditionState = condition.getAsBoolean();
    Command shouldBeRunning = conditionState ? onTrue : onFalse;

    if (selectedCommand != shouldBeRunning) {
      // Interrupt the current command
      selectedCommand.end(true);

      // Swap to the new command
      selectedCommand = shouldBeRunning;
      selectedCommand.initialize();
    }

    selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return onTrue.runsWhenDisabled() && onFalse.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    if (onTrue.getInterruptionBehavior() == InterruptionBehavior.CANCEL_SELF
        || onFalse.getInterruptionBehavior() == InterruptionBehavior.CANCEL_SELF) {
      return InterruptionBehavior.CANCEL_SELF;
    } else {
      return InterruptionBehavior.CANCEL_INCOMING;
    }
  }

  @Override
  public void logTo(TelemetryTable table) {
    super.logTo(table);
    table.log("onTrue", onTrue.getName());
    table.log("onFalse", onFalse.getName());
    table.log("selected", selectedCommand == null ? "null" : selectedCommand.getName());
  }
}
