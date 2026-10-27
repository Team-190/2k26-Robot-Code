package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class GenericFlywheelIOTalonFXSim extends GenericFlywheelIOTalonFX {
  private final FlywheelSim flywheelSim;

  private final TalonFXSimState flywheelController;

  public GenericFlywheelIOTalonFXSim(GenericFlywheelConstants constants) {
    super(constants);
    flywheelSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    flywheelController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    flywheelController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double flywheelVoltage = flywheelController.getMotorVoltage();

    flywheelSim.setInputVoltage(flywheelVoltage);

    flywheelSim.update(GompeiLib.getLoopPeriod());

    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            flywheelSim.getAngularVelocity() * constants.gearRatio, RadiansPerSecond);
    AngularAcceleration rotorAcceleration =
        AngularAcceleration.ofBaseUnits(
            flywheelSim.getAngularAcceleration() * constants.gearRatio, RadiansPerSecondPerSecond);
    flywheelController.setRotorVelocity(rotorVelocity);
    flywheelController.setRotorAcceleration(rotorAcceleration);

    super.updateInputs(inputs);
  }
}
