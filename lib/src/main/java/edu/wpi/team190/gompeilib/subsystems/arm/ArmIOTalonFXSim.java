package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class ArmIOTalonFXSim extends ArmIOTalonFX {
  private final SingleJointedArmSim armSim;

  private final TalonFXSimState armController;

  public ArmIOTalonFXSim(ArmConstants constants) {
    super(constants);
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                constants.armParameters.motorConfig(),
                constants.armParameters.momentOfInertia(),
                constants.armParameters.gearRatio()),
            constants.armParameters.motorConfig(),
            constants.armParameters.gearRatio(),
            constants.armParameters.lengthMeters(),
            constants.armParameters.minAngle().getRadians(),
            constants.armParameters.maxAngle().getRadians(),
            true,
            constants.armParameters.minAngle().getRadians());

    armController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(ArmIOInputs inputs) {
    armController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double armVoltage = armController.getMotorVoltage();

    armSim.setInputVoltage(armVoltage);

    armSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(armSim.getAngleRads() * constants.armParameters.gearRatio(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            armSim.getVelocityRadPerSec() * constants.armParameters.gearRatio(), RadiansPerSecond);
    armController.setRawRotorPosition(rotorPosition);
    armController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
