package first.robot.v1_DoomSpiral.subsystems.swank;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class V1_DoomSpiralSwankIOSim implements V1_DoomSpiralSwankIO {

  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  public V1_DoomSpiralSwankIOSim() {
    motorSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                V1_DoomSpiralSwankConstants.MOTOR_CONFIG,
                V1_DoomSpiralSwankConstants.MOMENT_OF_INERTIA,
                V1_DoomSpiralSwankConstants.GEAR_RATIO),
            V1_DoomSpiralSwankConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(V1_DoomSpiralSwankIOInputs inputs) {
    motorSim.setInputVoltage(Math.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = new Rotation2d(motorSim.getAngularPosition());
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocity());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDraw());
    inputs.torqueCurrent = Amps.of(motorSim.getCurrentDraw());
    inputs.temperature = Celsius.of(-1);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
