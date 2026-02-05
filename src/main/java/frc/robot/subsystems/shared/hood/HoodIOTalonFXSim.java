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

  private TalonFXSimState hoodController;

  public HoodIOTalonFXSim() {
    super();

    hoodSim =
        new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                HoodConstants.MOTOR_CONFIG,
                HoodConstants.MOMENT_OF_INERTIA,
                HoodConstants.GEAR_RATIO),
            HoodConstants.MOTOR_CONFIG,
            HoodConstants.GEAR_RATIO,
            HoodConstants.LENGTH_METERS,
            HoodConstants.MIN_ANGLE,
            HoodConstants.MAX_ANGLE,
            true,
            HoodConstants.MIN_ANGLE);
  }

  @Override
  @Trace
  public void updateInputs(HoodIOInputs inputs) {
    hoodController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double hoodVoltage = hoodController.getMotorVoltage();

    hoodSim.setInputVoltage(hoodVoltage);

    hoodSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(hoodSim.getAngleRads() * HoodConstants.GEAR_RATIO, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            hoodSim.getVelocityRadPerSec() * HoodConstants.GEAR_RATIO, RadiansPerSecond);
    hoodController.setRawRotorPosition(rotorPosition);
    hoodController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
