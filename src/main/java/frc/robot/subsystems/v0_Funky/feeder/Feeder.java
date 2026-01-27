package frc.robot.subsystems.v0_Funky.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;

public class Feeder extends SubsystemBase {
  private final GenericRoller roller;

  public Feeder(GenericRollerIO io) {
    roller = new GenericRoller(io, this, "Roller 1");
  }

  @Override
  public void periodic() {
    roller.periodic();
  }

  public void setVoltage(double voltage) {
    roller.setVoltage(voltage);
  }
}
