package frc.robot.subsystems.v1_Gamma.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.v1_Gamma.hood.V1_GammaHood;
import frc.robot.subsystems.v1_Gamma.hood.V1_GammaHoodConstants.HoodGoal;
import frc.robot.subsystems.v1_Gamma.hood.V1_GammaHoodIO;

public class V1_GammaShooter extends SubsystemBase {

  private V1_GammaHood hood;

  private GenericFlywheel flywheel;

  public V1_GammaShooter(
      GenericFlywheelIO flywheelIO, V1_GammaHoodIO hoodIO) {
  

    flywheel = new GenericFlywheel(flywheelIO, this, "Flywheel 1");
    hood = new V1_GammaHood(hoodIO, this, 1);
  }

  @Override
  public void periodic() {
    hood.periodic();
    flywheel.periodic();
  }

  public Command setHoodGoal(HoodGoal goal) {
    return hood.setGoal(goal);
  }

  public Command setHoodVoltage(double volts) {
    return hood.setVoltage(volts);
  }

  public Command stopHood() {
    return hood.setVoltage(0);
  }

  public Command setFlywheelGoal(double velocityRadiansPerSecond) {
    return flywheel.setGoal(velocityRadiansPerSecond);
  }

  public Command setFlywheelVoltage(double volts) {
    return flywheel.setVoltage(volts);
  }

  public Command stopFlywheel() {
    return flywheel.setVoltage(0);
  }

  public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) {
    return Commands.parallel(setHoodGoal(hoodGoal), setFlywheelGoal(velocityRadiansPerSecond));
  }
}
