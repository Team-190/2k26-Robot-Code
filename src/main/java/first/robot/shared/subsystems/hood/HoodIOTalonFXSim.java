package first.robot.shared.subsystems.hood;

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

public class HoodIOTalonFXSim extends HoodIOTalonFX {
  private final SingleJointedArmSim hoodSim;

  private final TalonFXSimState hoodController;

  public HoodIOTalonFXSim(HoodConstants constants) {
    super(constants);

    hoodSim =
        new SingleJointedArmSim(
            Models.singleJointedArmFromPhysicalConstants(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig,
            constants.gearRatio,
            constants.lengthMeters,
            constants.minAngle.getRadians(),
            constants.maxAngle.getRadians(),
            true,
            constants.minAngle.getRadians());

    hoodController = super.hoodMotor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(HoodIOInputs inputs) {
    hoodController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double hoodVoltage = hoodController.getMotorVoltage();

    hoodSim.setInputVoltage(hoodVoltage);

    hoodSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition = Angle.ofBaseUnits(hoodSim.getAngle() * constants.gearRatio, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(hoodSim.getVelocity() * constants.gearRatio, RadiansPerSecond);
    hoodController.setRawRotorPosition(rotorPosition);
    hoodController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
