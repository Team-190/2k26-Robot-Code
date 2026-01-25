package frc.robot.subsystems.v1_Gamma.spindexer;

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

public class V1_GammaSpindexerIOTalonFX implements V1_GammaSpindexerIO {

  private final TalonFX spindexerMotor;

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Current> supplyCurrent;
  private StatusSignal<Current> torqueCurrent;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Temperature> temperature;

  private TalonFXConfiguration config;
  private VoltageOut voltageControlRequest;

  public V1_GammaSpindexerIOTalonFX() {
    spindexerMotor = new TalonFX(V1_GammaSpindexerConstants.MOTOR_CAN_ID);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = V1_GammaSpindexerConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V1_GammaSpindexerConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = V1_GammaSpindexerConstants.GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = V1_GammaSpindexerConstants.SPINDEXER_INVERTED_VALUE;

    positionRotations = spindexerMotor.getPosition();
    velocity = spindexerMotor.getVelocity();
    torqueCurrent = spindexerMotor.getTorqueCurrent();
    supplyCurrent = spindexerMotor.getSupplyCurrent();
    temperature = spindexerMotor.getDeviceTemp();
    appliedVolts = spindexerMotor.getMotorVoltage();

    voltageControlRequest = new VoltageOut(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature);
    spindexerMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        V1_GammaSpindexerConstants.IS_CAN_FD,
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature);

    PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(V1_GammaSpindexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        torqueCurrent, supplyCurrent, appliedVolts, temperature, positionRotations, velocity);

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocity.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperature.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    spindexerMotor.setControl(voltageControlRequest.withOutput(volts));
  }
}
