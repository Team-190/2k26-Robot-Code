package frc.robot.subsystems.v1_Gamma.swank;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class V1_GammaSwankIOTalonFX implements V1_GammaSwankIO {

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;

  private final TalonFXConfiguration config;

  protected TalonFX motor;

  private final VoltageOut voltageControlRequest;

  public V1_GammaSwankIOTalonFX() {
    motor = new TalonFX(V1_GammaSwankConstants.MOTOR_CAN_ID, V1_GammaSwankConstants.CAN_LOOP);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = V1_GammaSwankConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V1_GammaSwankConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = V1_GammaSwankConstants.INVERSION;
    config.Feedback.SensorToMechanismRatio = V1_GammaSwankConstants.GEAR_RATIO;

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
        V1_GammaSwankConstants.CAN_LOOP.isNetworkFD(),
        positionRotations,
        velocityRotationsPerSecond,
        torqueCurrentAmps,
        supplyCurrentAmps,
        appliedVolts,
        temperatureCelsius);

    voltageControlRequest = new VoltageOut(0.0);
  }

  public void updateInputs(V1_GammaSwankIOInputs inputs) {

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
