package first.robot.v1_DoomSpiral.subsystems.spindexer;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

public class V1_DoomSpiralSpindexerIOSim implements V1_DoomSpiralSpindexerIO {
  private final DCMotorSim motorSim;

  private double appliedVolts;

  public V1_DoomSpiralSpindexerIOSim() {
    motorSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                V1_DoomSpiralSpindexerConstants.SPINDEXER_MOTOR_CONFIG,
                V1_DoomSpiralSpindexerConstants.SPINDEXER_MOMENT_OF_INERTIA,
                V1_DoomSpiralSpindexerConstants.SPINDEXER_GEAR_RATIO),
            V1_DoomSpiralSpindexerConstants.SPINDEXER_MOTOR_CONFIG);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(V1_DoomSpiralSpindexerIOInputs inputs) {
    appliedVolts = Math.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPosition());
    inputs.velocity = RadiansPerSecond.of(motorSim.getAngularVelocity());
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDraw());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
