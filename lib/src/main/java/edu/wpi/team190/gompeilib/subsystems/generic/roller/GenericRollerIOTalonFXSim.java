package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class GenericRollerIOTalonFXSim extends GenericRollerIOTalonFX {
  private final DCMotorSim rollerSim;

  private final TalonFXSimState rollerController;

  public GenericRollerIOTalonFXSim(GenericRollerConstants constants) {
    super(constants);
    rollerSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
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
        Angle.ofBaseUnits(rollerSim.getAngularPosition() * constants.rollerMotorGearRatio, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            rollerSim.getAngularVelocity() * constants.rollerMotorGearRatio, RadiansPerSecond);
    rollerController.setRawRotorPosition(rotorPosition);
    rollerController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
