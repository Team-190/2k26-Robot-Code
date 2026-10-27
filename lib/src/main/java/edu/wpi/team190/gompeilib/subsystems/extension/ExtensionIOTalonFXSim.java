package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class ExtensionIOTalonFXSim extends ExtensionIOTalonFX {
  private final ElevatorSim elevatorSim;

  private final TalonFXSimState extensionController;

  public ExtensionIOTalonFXSim(ExtensionConstants constants) {
    super(constants);
    elevatorSim =
        new ElevatorSim(
            Models.elevatorFromPhysicalConstants(
                constants.extensionParameters.EXTENSION_MOTOR_CONFIG(),
                constants.extensionParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.extensionGearRatio),
            constants.extensionParameters.EXTENSION_MOTOR_CONFIG(),
            constants.extensionParameters.MIN_LENGTH().in(Meters),
            constants.extensionParameters.MAX_LENGTH().in(Meters),
            true,
            constants.extensionParameters.MIN_LENGTH().in(Meters));

    extensionController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(ExtensionIOInputs inputs) {
    extensionController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double extensionVoltage = extensionController.getMotorVoltage();

    elevatorSim.setInputVoltage(extensionVoltage);

    elevatorSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            elevatorSim.getPosition() * constants.extensionGearRatio * constants.drumRadius,
            Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            elevatorSim.getVelocity() * constants.extensionGearRatio * constants.drumRadius,
            RadiansPerSecond);
    extensionController.setRawRotorPosition(rotorPosition);
    extensionController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
