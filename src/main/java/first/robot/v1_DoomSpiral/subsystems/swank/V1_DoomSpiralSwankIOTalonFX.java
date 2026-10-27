package first.robot.v1_DoomSpiral.subsystems.swank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class V1_DoomSpiralSwankIOTalonFX implements V1_DoomSpiralSwankIO {

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;

  private final TalonFXConfiguration config;

  protected TalonFX motor;

  private final VoltageOut voltageControlRequest;

  public V1_DoomSpiralSwankIOTalonFX() {
    motor =
        new TalonFX(V1_DoomSpiralSwankConstants.MOTOR_CAN_ID, V1_DoomSpiralSwankConstants.CAN_LOOP);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = V1_DoomSpiralSwankConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V1_DoomSpiralSwankConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = V1_DoomSpiralSwankConstants.INVERSION;
    config.Feedback.SensorToMechanismRatio = V1_DoomSpiralSwankConstants.GEAR_RATIO;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    positionRotations = motor.getPosition();
    velocityRotationsPerSecond = motor.getVelocity();
    torqueCurrentAmps = motor.getTorqueCurrent();
    supplyCurrentAmps = motor.getSupplyCurrent();
    temperatureCelsius = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        positionRotations,
        velocityRotationsPerSecond,
        torqueCurrentAmps,
        supplyCurrentAmps,
        appliedVolts,
        temperatureCelsius);

    PhoenixUtil.registerSignals(
        V1_DoomSpiralSwankConstants.CAN_LOOP,
        positionRotations,
        velocityRotationsPerSecond,
        torqueCurrentAmps,
        supplyCurrentAmps,
        appliedVolts,
        temperatureCelsius);

    voltageControlRequest = new VoltageOut(0.0);
  }

  public void updateInputs(V1_DoomSpiralSwankIOInputs inputs) {

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocityRotationsPerSecond.getValue();
    inputs.supplyCurrent = supplyCurrentAmps.getValue();
    inputs.torqueCurrent = torqueCurrentAmps.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperatureCelsius.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControlRequest.withOutput(volts));
  }
}
