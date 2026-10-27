package first.robot.v1_DoomSpiral.subsystems.swank;

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

public class V1_DoomSpiralSwankIOTalonFXSim extends V1_DoomSpiralSwankIOTalonFX {
  private final DCMotorSim swankSim;

  private TalonFXSimState swankController;

  public V1_DoomSpiralSwankIOTalonFXSim() {
    super();

    swankSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
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
        Angle.ofBaseUnits(swankSim.getAngularPosition() * swankSim.getGearing(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            swankSim.getAngularVelocity() * swankSim.getGearing(), RadiansPerSecond);
    swankController.setRawRotorPosition(rotorPosition);
    swankController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
