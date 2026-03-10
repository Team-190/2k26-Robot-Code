package frc.robot.subsystems.v2_Delta;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import frc.robot.util.InternalLoggedTracer;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaIntake extends SubsystemBase {
  private final V2_DeltaIntakeIO io;
  private final V2_DeltaIntakeIOInputsAutoLogged inputs;
  private GenericRoller roller;
  private GenericRollerIO rollerIO;
  private GenericRollerConstants constants;

  private boolean isClosedLoop;

  public V2_DeltaIntake(V2_DeltaIntakeIO io) {
    this.io = io;
    inputs = new V2_DeltaIntakeIOInputsAutoLogged();

    isClosedLoop = true;
    roller = new GenericRoller(rollerIO, this, constants, "Intake Roller");
  }

  public void periodic() {
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Intake/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Intake", inputs);
    InternalLoggedTracer.record("Process Inputs", "Intake/Periodic");
  }

  public double getExtension() {
    return inputs.extensionPositionMeters;
  }

  public boolean atGoal() {
    return io.atExtensionPositionGoal();
  }

  public double getDistance() {
    return inputs.extensionPositionMeters;
  }

  public void updateGains(double kP, double kD, double kS, double kA, double kV) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }

  public Command setIntakeVoltage(double volts) {
    return Commands.runOnce(() -> io.setExtensionVoltage(volts));
  }

  public Command setRollerVoltage(double volts) {
    return Commands.runOnce(() -> io.setRollerVoltage(volts));
  }

  public Command homingSequence() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isClosedLoop = false;
              io.maxExt();
            }),
        setIntakeVoltage(-6).until(() -> Math.abs(inputs.extensionTorqueCurrentAmps) > 45),
        setIntakeVoltage(0),
        Commands.runOnce(() -> io.resetExtension()));
  }

  public Command waitUntilExtensionAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }
}
