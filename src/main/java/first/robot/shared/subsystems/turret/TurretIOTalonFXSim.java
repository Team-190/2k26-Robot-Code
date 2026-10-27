package first.robot.shared.subsystems.turret;

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

public class TurretIOTalonFXSim extends TurretIOTalonFX {
  private final DCMotorSim turretSim;

  private final TalonFXSimState turretController;

  public TurretIOTalonFXSim(TurretConstants constants) {
    super(constants);

    turretSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    turretController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(TurretIOInputs inputs) {
    turretController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double turretVoltage = turretController.getMotorVoltage();

    turretSim.setInputVoltage(turretVoltage);

    turretSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(turretSim.getAngularPosition() * turretSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            turretSim.getAngularVelocity() * turretSim.getGearing(), RadiansPerSecond);
    turretController.setRawRotorPosition(rotorPosition);
    turretController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
