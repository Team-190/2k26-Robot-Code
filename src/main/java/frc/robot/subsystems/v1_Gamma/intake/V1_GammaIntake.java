package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;

public class V1_GammaIntake extends SubsystemBase {
  public GenericFlywheel topRoller;
  public GenericFlywheel bottomRoller;

  public V1_GammaIntake(GenericFlywheelIO topIO, GenericFlywheelIO bottomIO) {
    topRoller = new GenericFlywheel(topIO, this, "IntakeTopFlywheel");
    bottomRoller = new GenericFlywheel(bottomIO, this, "IntakeBottomFlywheel");
  }

  @Override
  public void periodic() {
    topRoller.periodic();
    bottomRoller.periodic();
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          topRoller.setVoltage(voltage);
          bottomRoller.setVoltage(voltage);
        });
  }

  public Command setVelocityGoal(double velocityGoal) {
    return Commands.runOnce(
        () -> {
          topRoller.setGoal(velocityGoal);
          bottomRoller.setGoal(velocityGoal);
        });
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          topRoller.setVoltage(0);
          bottomRoller.setVoltage(0);
          topRoller.setGoal(0);
          bottomRoller.setGoal(0);
        });
  }
}
