package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class HoodIOTalonFXSim extends HoodIOTalonFX {

  private final DCMotorSim motorSim;
  private TalonFXSimState talonFXSim;
  private double motorVoltage;

  public HoodIOTalonFXSim() {
    talonFXSim = hoodMotor.getSimState();
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HoodConstants.MOTOR_CONFIG,
                HoodConstants.MOMENT_OF_INERTIA,
                HoodConstants.GEAR_RATIO),
            HoodConstants.MOTOR_CONFIG);

    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  @Trace
  public void updateInputs(HoodIOInputs inputs) {

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltage = talonFXSim.getMotorVoltage();

    motorSim.setInputVoltage(motorVoltage);

    motorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotations = motorSim.getAngularPositionRotations() * motorSim.getGearing();
    double rotorVelocityRotationsPerSecond =
        motorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * motorSim.getGearing();
    talonFXSim.setRawRotorPosition(rotorPositionRotations);
    talonFXSim.setRotorVelocity(rotorVelocityRotationsPerSecond);

    super.updateInputs(inputs);
  }
}
