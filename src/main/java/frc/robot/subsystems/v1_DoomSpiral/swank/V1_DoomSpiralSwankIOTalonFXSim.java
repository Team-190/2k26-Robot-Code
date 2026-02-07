package frc.robot.subsystems.v1_DoomSpiral.swank;

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

public class V1_DoomSpiralSwankIOTalonFXSim extends V1_DoomSpiralSwankIOTalonFX {
  private final DCMotorSim swankSim;

  private TalonFXSimState swankController;

  public V1_DoomSpiralSwankIOTalonFXSim() {
    super();

    swankSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_DoomSpiralSwankConstants.MOTOR_CONFIG,
                V1_DoomSpiralSwankConstants.MOMENT_OF_INERTIA,
                V1_DoomSpiralSwankConstants.GEAR_RATIO),
            V1_DoomSpiralSwankConstants.MOTOR_CONFIG);

    swankController = super.motor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(V1_DoomSpiralSwankIOInputs inputs) {
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
