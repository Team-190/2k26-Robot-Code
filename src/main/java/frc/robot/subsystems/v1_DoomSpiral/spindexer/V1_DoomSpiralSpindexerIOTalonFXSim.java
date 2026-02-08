package frc.robot.subsystems.v1_DoomSpiral.spindexer;

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

public class V1_DoomSpiralSpindexerIOTalonFXSim extends V1_DoomSpiralSpindexerIOTalonFX {
  private final DCMotorSim spindexerSim;

  private TalonFXSimState spindexerController;

  public V1_DoomSpiralSpindexerIOTalonFXSim() {
    super();

    spindexerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_DoomSpiralSpindexerConstants.SPINDEXER_MOTOR_CONFIG,
                V1_DoomSpiralSpindexerConstants.SPINDEXER_MOMENT_OF_INERTIA,
                V1_DoomSpiralSpindexerConstants.SPINDEXER_GEAR_RATIO),
            V1_DoomSpiralSpindexerConstants.SPINDEXER_MOTOR_CONFIG);

    spindexerController = super.spindexerMotor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(V1_DoomSpiralSpindexerIOInputs inputs) {
    spindexerController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double spindexerVoltage = spindexerController.getMotorVoltage();

    spindexerSim.setInputVoltage(spindexerVoltage);

    spindexerSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            spindexerSim.getAngularPositionRad() * spindexerSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            spindexerSim.getAngularVelocityRadPerSec() * spindexerSim.getGearing(),
            RadiansPerSecond);
    spindexerController.setRawRotorPosition(rotorPosition);
    spindexerController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
