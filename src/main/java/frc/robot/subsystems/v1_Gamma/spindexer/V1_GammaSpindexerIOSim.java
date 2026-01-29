package frc.robot.subsystems.v1_Gamma.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class V1_GammaSpindexerIOSim extends V1_GammaSpindexerIOTalonFXSim {
  private final DCMotorSim motorSim;

  public V1_GammaSpindexerIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaSpindexerConstants.MOTOR_CONFIG,
                V1_GammaSpindexerConstants.MOMENT_OF_INERTIA,
                V1_GammaSpindexerConstants.GEAR_RATIO),
            V1_GammaSpindexerConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(V1_GammaSpindexerIOInputs inputs) {
    double appliedVolts = (MathUtil.clamp(0, -12.0, 12.0)); // change 0 later

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(GompeiLib.getLoopPeriod());
  }
}
