package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;

public class Feeder extends SubsystemBase {
  private GenericRoller roller;

  public Feeder(GenericRollerIO io) {
    roller = new GenericRoller(io, this, 0);
  }

  public void setVoltage(double voltage) {
    roller.setVoltage(voltage);
  }
}
