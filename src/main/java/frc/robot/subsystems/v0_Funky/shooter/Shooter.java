package frc.robot.subsystems.v0_Funky.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;

public class Shooter extends SubsystemBase {
  private GenericFlywheel flywheel;

  public Shooter(GenericFlywheelIO io, int index) {
    flywheel = new GenericFlywheel(io, this, "Flywheel " + index);
  }

  @Override
  public void periodic() {
    flywheel.periodic();
  }

  public void setVoltage(double volts) {
    flywheel.setVoltage(volts);
  }
}
