package frc.robot.subsystems.shared.turret;

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

public class TurretIOTalonFXSim extends TurretIOTalonFX {
  private final DCMotorSim turretSim;

  private TalonFXSimState turretController;

  public TurretIOTalonFXSim() {
    super();

    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.MOTOR_CONFIG,
                TurretConstants.MOMENT_OF_INERTIA,
                TurretConstants.GEAR_RATIO),
            TurretConstants.MOTOR_CONFIG);
  }

  @Override
  @Trace
  public void updateInputs(TurretIOInputs inputs) {
    turretController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double turretVoltage = turretController.getMotorVoltage();

    turretSim.setInputVoltage(turretVoltage);

    turretSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(turretSim.getAngularPositionRad() * turretSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            turretSim.getAngularVelocityRadPerSec() * turretSim.getGearing(), RadiansPerSecond);
    turretController.setRawRotorPosition(rotorPosition);
    turretController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
