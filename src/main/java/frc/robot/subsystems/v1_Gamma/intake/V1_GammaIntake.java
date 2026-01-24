package frc.robot.subsystems.v1_Gamma.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;

public class V1_GammaIntake extends SubsystemBase {
  public GenericRoller topRoller;
  public GenericRoller bottomRoller;

  public V1_GammaIntake(GenericRollerIO topIO, GenericRollerIO bottomIO) {
    topRoller = new GenericRoller(topIO, this, "IntakeTopFlywheel");
    bottomRoller = new GenericRoller(bottomIO, this, "IntakeBottomFlywheel");
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

  public Command stop() {
    return Commands.runOnce(
        () -> {
          topRoller.setVoltage(0);
          bottomRoller.setVoltage(0);
        });
  }
}
