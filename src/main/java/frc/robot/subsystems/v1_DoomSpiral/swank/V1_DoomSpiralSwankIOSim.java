package frc.robot.subsystems.v1_DoomSpiral.swank;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class V1_DoomSpiralSwankIOSim implements V1_DoomSpiralSwankIO {

  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  public V1_DoomSpiralSwankIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_DoomSpiralSwankConstants.MOTOR_CONFIG,
                V1_DoomSpiralSwankConstants.MOMENT_OF_INERTIA,
                V1_DoomSpiralSwankConstants.GEAR_RATIO),
            V1_DoomSpiralSwankConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(V1_DoomSpiralSwankIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = new Rotation2d(motorSim.getAngularPosition());
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.torqueCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    inputs.temperature = Celsius.of(-1);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
