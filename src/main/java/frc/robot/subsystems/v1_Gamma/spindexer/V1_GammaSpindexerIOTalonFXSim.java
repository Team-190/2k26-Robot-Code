package frc.robot.subsystems.v1_Gamma.spindexer;



import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class V1_GammaSpindexerIOTalonFXSim
    extends V1_GammaSpindexerIOTalonFX { // is this even right?

  private final DCMotorSim motorSim;
  private TalonFXSimState talonFXSim;
  private double motorVoltage;

  public V1_GammaSpindexerIOTalonFXSim() {
    talonFXSim = spindexerMotor.getSimState();
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaSpindexerConstants.MOTOR_CONFIG,
                V1_GammaSpindexerConstants.MOMENT_OF_INERTIA,
                V1_GammaSpindexerConstants.GEAR_RATIO),
            V1_GammaSpindexerConstants.MOTOR_CONFIG);

    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    talonFXSim.Orientation = V1_GammaSpindexerConstants.SPINDEXER_ORIENTATION;
  }

  @Override
  @Trace
  public void updateInputs(V1_GammaSpindexerIOInputs inputs) {
    super.updateInputs(inputs);
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltage = talonFXSim.getMotorVoltage();

    motorSim.setInputVoltage(motorVoltage);

    motorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotations = motorSim.getAngularPositionRad() / (2 * Math.PI);
    double rotorVelocityRotationsPerSecond = motorSim.getAngularVelocityRadPerSec() / (2 * Math.PI);
    talonFXSim.setRawRotorPosition(rotorPositionRotations);
    talonFXSim.setRotorVelocity(rotorVelocityRotationsPerSecond);

    inputs.appliedVolts = motorVoltage;
    
    }

  @Override
  public void setVoltage(double volts) {
    // no clue what goes here
    spindexerMotor.setVoltage(volts);
    spindexerMotor.getSimState().setMotorVoltage(volts);

  }
}
