package frc.robot.subsystems.v0_Funky.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.Supplier;

public class V0_Shoot extends SubsystemBase {
  private final GenericFlywheel shootFlywheel;

  public V0_Shoot(GenericFlywheelIO io) {
    shootFlywheel = new GenericFlywheel(io, this, 0);
  }

  @Override
  public void periodic() {
    InternalLoggedTracer.reset();
    shootFlywheel.periodic();
    InternalLoggedTracer.record("Shooter Periodic", "Shoot/Periodic");
  }

  public void setVoltage(double volts) {
    shootFlywheel.setVoltage(volts);
  }

  public Command setVoltageCommand(Supplier<Double> volts) {
    return Commands.run(() -> setVoltage(volts.get()), this);
  }

  public void setGoal(double goal) {
    shootFlywheel.setGoal(goal);
  }

  public Command setGoalCommand(Supplier<Double> goal) {
    return Commands.run(() -> setGoal(goal.get()), this);
  }

  public Command runSysID() {
    SysIdRoutine characterizationRoutine = shootFlywheel.getCharacterization(1, 7, 10, this);

    return Commands.sequence(
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
