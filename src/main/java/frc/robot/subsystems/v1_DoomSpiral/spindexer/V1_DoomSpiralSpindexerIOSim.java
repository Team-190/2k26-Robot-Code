package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class V1_DoomSpiralSpindexerIOSim implements V1_DoomSpiralSpindexerIO {
  private final DCMotorSim motorSim;

  private double appliedVolts;

  public V1_DoomSpiralSpindexerIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_DoomSpiralSpindexerConstants.MOTOR_CONFIG,
                V1_DoomSpiralSpindexerConstants.MOMENT_OF_INERTIA,
                V1_DoomSpiralSpindexerConstants.GEAR_RATIO),
            V1_DoomSpiralSpindexerConstants.MOTOR_CONFIG);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(V1_DoomSpiralSpindexerIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(motorSim.getAngularPositionRad());
    inputs.velocity = motorSim.getAngularVelocity();
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
