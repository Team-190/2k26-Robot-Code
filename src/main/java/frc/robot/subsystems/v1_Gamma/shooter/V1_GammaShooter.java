package frc.robot.subsystems.v1_Gamma.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodConstants.HoodGoal;
import frc.robot.subsystems.shared.hood.HoodIO;


public class V1_GammaShooter extends SubsystemBase {

  private Hood hood;

  private GenericFlywheel flywheel;

  public V1_GammaShooter(GenericFlywheelIO flywheelIO, HoodIO hoodIO) {

    flywheel = new GenericFlywheel(flywheelIO, this, "Flywheel 1");
    hood = new Hood(hoodIO, this, 1);
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
