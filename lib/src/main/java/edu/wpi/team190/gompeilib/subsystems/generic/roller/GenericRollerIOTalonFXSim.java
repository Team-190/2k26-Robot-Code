package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class GenericRollerIOTalonFXSim extends GenericRollerIOTalonFX {
  private final DCMotorSim rollerSim;

  private final TalonFXSimState rollerController;

  public GenericRollerIOTalonFXSim(GenericRollerConstants constants) {
    super(constants);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.rollerGearbox,
                constants.momentOfInertia.baseUnitMagnitude(),
                constants.rollerMotorGearRatio),
            constants.rollerGearbox);

    rollerController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(GenericRollerIOInputs inputs) {
    rollerController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double rollerVoltage = rollerController.getMotorVoltage();

    rollerSim.setInputVoltage(rollerVoltage);

    rollerSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            rollerSim.getAngularPositionRad() * constants.rollerMotorGearRatio, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            rollerSim.getAngularVelocityRadPerSec() * constants.rollerMotorGearRatio,
            RadiansPerSecond);
    rollerController.setRawRotorPosition(rotorPosition);
    rollerController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
