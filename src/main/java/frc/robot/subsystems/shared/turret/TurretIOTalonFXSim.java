package frc.robot.subsystems.shared.turret;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class TurretIOTalonFXSim extends TurretIOTalonFX { // is this even right?

  private final DCMotorSim motorSim;
  private TalonFXSimState talonFXSim;
  private double motorVoltage;
  private CANcoderSimState encoder1SimState;
  private CANcoderSimState encoder2SimState;

  public TurretIOTalonFXSim() {
    talonFXSim = talonFX.getSimState();
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TurretConstants.MOTOR_CONFIG,
                TurretConstants.MOMENT_OF_INERTIA,
                TurretConstants.GEAR_RATIO),
            TurretConstants.MOTOR_CONFIG);

    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  @Trace
  public void updateInputs(TurretIOInputs inputs) {

    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltage = talonFXSim.getMotorVoltage();

    motorSim.setInputVoltage(motorVoltage);

    motorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotations = motorSim.getAngularPositionRotations() * motorSim.getGearing();
    double rotorVelocityRotationsPerSecond =
        motorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * motorSim.getGearing();
    encoder1SimState.setRawPosition(
        motorSim.getAngularPositionRotations()
                * TurretConstants.TURRET_ANGLE_CALCULATION.GEAR_1_RATIO()
            + TurretConstants.E2_OFFSET_RADIANS);
    encoder2SimState.setRawPosition(
        motorSim.getAngularPositionRotations()
                * TurretConstants.TURRET_ANGLE_CALCULATION.GEAR_2_RATIO()
            + TurretConstants.E2_OFFSET_RADIANS);

    talonFXSim.setRawRotorPosition(rotorPositionRotations);
    talonFXSim.setRotorVelocity(rotorVelocityRotationsPerSecond);

    super.updateInputs(inputs);
  }
}
