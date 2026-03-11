package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.LinearConstraints;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOInputsAutoLogged;
import frc.robot.subsystems.shared.linearextension.LinearExtension;
import frc.robot.subsystems.shared.linearextension.LinearExtensionConstants;
import frc.robot.subsystems.shared.linearextension.LinearExtensionIO;
import frc.robot.subsystems.shared.linearextension.LinearExtensionIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaIntake extends SubsystemBase {
  private LinearExtension extension;
  private LinearExtensionIO extensionIO;
  private LinearExtensionConstants extensionConstants;

  private GenericRoller roller;
  private GenericRollerIO rollerIO;
  private GenericRollerConstants rollerConstants;

  private LinearExtensionIOInputsAutoLogged extensionInputs;
  private GenericRollerIOInputsAutoLogged rollerInputs;

  private boolean isClosedLoop;

  public V2_DeltaIntake(GenericRollerIO rollerIO, LinearExtensionIO extensionIO) {
    this.rollerIO = rollerIO;
    this.extensionIO = extensionIO;

    isClosedLoop = true;
    roller = new GenericRoller(rollerIO, this, rollerConstants, "Intake Roller");
    extension = new LinearExtension(extensionIO, extensionConstants, this, 0);
  }

  @Trace
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    extensionIO.updateInputs(extensionInputs);
    Logger.processInputs("Roller", rollerInputs);
    Logger.processInputs("Extension", extensionInputs);
  }

  public boolean atGoal() {
    return extensionIO.atGoal();
  }

  public double getDistance() {
    return extensionInputs.position.baseUnitMagnitude();
  }

  public void updateGains(Gains gains) {
    extensionIO.setPID(gains);
  }

  public void updateConstraints(LinearConstraints constraints) {
    extensionIO.setProfile(constraints);
  }

  public Command setIntakeVoltage(double volts) {
    return Commands.runOnce(() -> extensionIO.setVoltage(Voltage.ofBaseUnits(volts, Volts)));
  }

  public Command setRollerVoltage(double volts) {
    return Commands.runOnce(() -> rollerIO.setVoltage(volts));
  }

  public Command homingSequence() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isClosedLoop = false;
              extensionIO.atGoal();
            }),
        setIntakeVoltage(-6)
            .until(() -> Math.abs(extensionInputs.torqueCurrent.baseUnitMagnitude()) > 45),
        setIntakeVoltage(0),
        Commands.runOnce(() -> extensionIO.resetExtension()));
  }

  public Command waitUntilExtensionAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }
}
