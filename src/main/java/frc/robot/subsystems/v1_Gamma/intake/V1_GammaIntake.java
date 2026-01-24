package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;

public class V1_GammaIntake extends SubsystemBase {
  public GenericFlywheel topFlywheel;
  public GenericFlywheel bottomFlywheel;

  public V1_GammaIntake(GenericFlywheelIO topIO, GenericFlywheelIO bottomIO) {
    topFlywheel = new GenericFlywheel(topIO, this, "IntakeTopFlywheel");
    bottomFlywheel = new GenericFlywheel(bottomIO, this, "IntakeBottomFlywheel");
  }

  @Override
  public void periodic() {
    topFlywheel.periodic();
    bottomFlywheel.periodic();
  }

  public Command setVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          topFlywheel.setVoltage(voltage);
          bottomFlywheel.setVoltage(voltage);
        });
  }

  public Command setVelocityGoal(double velocityGoal) {
    return Commands.runOnce(
        () -> {
          topFlywheel.setGoal(velocityGoal);
          bottomFlywheel.setGoal(velocityGoal);
        });
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          topFlywheel.setVoltage(0);
          bottomFlywheel.setVoltage(0);
          topFlywheel.setGoal(0);
          bottomFlywheel.setGoal(0);
        });
  }
}
