package first.robot.v1_DoomSpiral.subsystems.spindexer;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class V1_DoomSpiralSpindexerIOTalonFXSim extends V1_DoomSpiralSpindexerIOTalonFX {
  private final DCMotorSim spindexerSim;

  private TalonFXSimState spindexerController;

  public V1_DoomSpiralSpindexerIOTalonFXSim() {
    super();

    spindexerSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
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
        Angle.ofBaseUnits(spindexerSim.getAngularPosition() * spindexerSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            spindexerSim.getAngularVelocity() * spindexerSim.getGearing(), RadiansPerSecond);
    spindexerController.setRawRotorPosition(rotorPosition);
    spindexerController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
