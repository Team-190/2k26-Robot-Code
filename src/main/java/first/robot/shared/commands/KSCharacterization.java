package first.robot.shared.commands;

import edu.wpi.team190.gompeilib.core.utility.tunable.GompeiTunables;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunable;

public class KSCharacterization extends Command {
  private static final Tunable<Double> currentRampFactor =
      GompeiTunables.value("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final Tunable<Double> minVelocity =
      GompeiTunables.value("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public KSCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("********** FF Characterization Results **********\n\n\n\n\n\n\n\n\n");
    System.out.println(
        "Static Characterization output: " + currentInput + " amps\n\n\n\n\n\n\n\n\n\n");
    Logger.recordOutput("KS output", currentInput);
    inputConsumer.accept(0);
  }
}
