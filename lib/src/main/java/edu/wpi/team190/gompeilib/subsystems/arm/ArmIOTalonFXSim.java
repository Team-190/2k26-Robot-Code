package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class ArmIOTalonFXSim extends ArmIOTalonFX {
  private final SingleJointedArmSim armSim;

  private final TalonFXSimState armController;

  public ArmIOTalonFXSim(ArmConstants constants) {
    super(constants);
    armSim =
        new SingleJointedArmSim(
            Models.singleJointedArmFromPhysicalConstants(
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
        Angle.ofBaseUnits(armSim.getAngle() * constants.armParameters.gearRatio(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            armSim.getVelocity() * constants.armParameters.gearRatio(), RadiansPerSecond);
    armController.setRawRotorPosition(rotorPosition);
    armController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
