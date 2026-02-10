package frc.robot.subsystems.shared.hood;

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

public class HoodIOTalonFXSim extends HoodIOTalonFX {
  private final SingleJointedArmSim hoodSim;

  private final TalonFXSimState hoodController;

  public HoodIOTalonFXSim(HoodConstants constants) {
    super(constants);

    hoodSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig,
            constants.gearRatio,
            constants.lengthMeters,
            constants.minAngle,
            constants.maxAngle,
            true,
            constants.minAngle);

    hoodController = super.hoodMotor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(HoodIOInputs inputs) {
    hoodController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double hoodVoltage = hoodController.getMotorVoltage();

    hoodSim.setInputVoltage(hoodVoltage);

    hoodSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition = Angle.ofBaseUnits(hoodSim.getAngleRads() * constants.gearRatio, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            hoodSim.getVelocityRadPerSec() * constants.gearRatio, RadiansPerSecond);
    hoodController.setRawRotorPosition(rotorPosition);
    hoodController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
