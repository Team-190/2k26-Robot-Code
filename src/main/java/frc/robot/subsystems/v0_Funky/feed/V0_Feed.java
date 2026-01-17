package frc.robot.subsystems.v0_Funky.feed;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.Supplier;

public class V0_Feed extends SubsystemBase {
  private final GenericRoller feedRoller;

  public V0_Feed(GenericRollerIO io) {
    feedRoller = new GenericRoller(io, this, 0);
  }

  @Override
  public void periodic() {
    InternalLoggedTracer.reset();
    feedRoller.periodic();
    InternalLoggedTracer.record("Roller Periodic", "Intake/Periodic");
  }

  public void setVoltage(double volts) {
    feedRoller.setVoltage(volts);
  }

  public Command setVoltageCommand(Supplier<Double> volts) {
    return Commands.run(() -> setVoltage(volts.get()), this);
  }
}
