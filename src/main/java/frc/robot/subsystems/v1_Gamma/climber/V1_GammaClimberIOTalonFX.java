package frc.robot.subsystems.v1_Gamma.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class V1_GammaClimberIOTalonFX implements V1_GammaClimberIO {

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> torqueCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  private TalonFXConfiguration config;

  private TalonFX motor;
  private VoltageOut voltageControlRequest;

  public V1_GammaClimberIOTalonFX() {
    motor = new TalonFX(V1_GammaClimberConstants.MOTOR_CAN_ID);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = V1_GammaClimberConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V1_GammaClimberConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = V1_GammaClimberConstants.GEAR_RATIO;

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
        V1_GammaClimberConstants.IS_CAN_FD,
        positionRotations,
        velocityRotationsPerSecond,
        torqueCurrentAmps,
        supplyCurrentAmps,
        appliedVolts,
        temperatureCelsius);

    voltageControlRequest = new VoltageOut(0.0);
  }

  public void updateInputs(V1_GammaClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        torqueCurrentAmps,
        supplyCurrentAmps,
        appliedVolts,
        temperatureCelsius);

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

  @Override
  public boolean isClimbedL1() {
    double positionRotation2d = positionRotations.getValueAsDouble();
    return positionRotation2d
        >= Units.radiansToRotations(V1_GammaClimberConstants.CLIMBER_CLIMBED_L1_RADIANS);
  }

  @Override
  public boolean isClimbedL2() {
    double positionRotation2d = positionRotations.getValueAsDouble();
    return positionRotation2d
        <= Units.radiansToRotations(V1_GammaClimberConstants.CLIMBER_CLIMBED_L2_RADIANS);
  }
}
