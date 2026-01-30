package frc.robot.subsystems.v1_Gamma.swank;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class V1_GammaSwankIOTalonFXSim extends V1_GammaSwankIOTalonFX { // is this even right?

  private final DCMotorSim swankMotorSim;
  private TalonFXSimState talonFXSim;
  private double motorVoltage;

  public V1_GammaSwankIOTalonFXSim() {
    talonFXSim = motor.getSimState();
    swankMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaSwankConstants.MOTOR_CONFIG,
                V1_GammaSwankConstants.MOMENT_OF_INERTIA,
                V1_GammaSwankConstants.GEAR_RATIO),
            V1_GammaSwankConstants.MOTOR_CONFIG);

    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  @Trace
  public void updateInputs(V1_GammaSwankIOInputs inputs) {

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltage = talonFXSim.getMotorVoltage();

    swankMotorSim.setInputVoltage(motorVoltage);

    swankMotorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotations =
        swankMotorSim.getAngularPositionRotations() * swankMotorSim.getGearing();
    double rotorVelocityRotationsPerSecond =
        swankMotorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * swankMotorSim.getGearing();
    talonFXSim.setRawRotorPosition(rotorPositionRotations);
    talonFXSim.setRotorVelocity(rotorVelocityRotationsPerSecond);

    super.updateInputs(inputs);
  }
}
