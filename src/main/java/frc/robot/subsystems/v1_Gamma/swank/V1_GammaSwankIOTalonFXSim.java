package frc.robot.subsystems.v1_Gamma.swank;

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

public class V1_GammaSwankIOTalonFXSim extends V1_GammaSwankIOTalonFX {
  private final DCMotorSim swankSim;

  private TalonFXSimState swankController;

  public V1_GammaSwankIOTalonFXSim() {
    super();

    swankSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaSwankConstants.MOTOR_CONFIG,
                V1_GammaSwankConstants.MOMENT_OF_INERTIA,
                V1_GammaSwankConstants.GEAR_RATIO),
            V1_GammaSwankConstants.MOTOR_CONFIG);

    swankController = super.motor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(V1_GammaSwankIOInputs inputs) {
    swankController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double spindexerVoltage = swankController.getMotorVoltage();

    swankSim.setInputVoltage(spindexerVoltage);

    swankSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(swankSim.getAngularPositionRad() * swankSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            swankSim.getAngularVelocityRadPerSec() * swankSim.getGearing(), RadiansPerSecond);
    swankController.setRawRotorPosition(rotorPosition);
    swankController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
