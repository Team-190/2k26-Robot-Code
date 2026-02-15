package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;

  protected final HoodConstants constants;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private Rotation2d positionGoal;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;

  private final TalonFXConfiguration config;
  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  public HoodIOTalonFX(HoodConstants constants) {

    this.constants = constants;

    hoodMotor = new TalonFX(constants.motorCanId, constants.canBus);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = constants.currentLimits;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = constants.invertedValue;
    config.Feedback.SensorToMechanismRatio = constants.gearRatio;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.maxAngle.getRotations();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.minAngle.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.kP = constants.gains.kp().get();
    config.Slot0.kD = constants.gains.kd().get();
    config.Slot0.kS = constants.gains.ks().get();
    config.Slot0.kV = constants.gains.kv().get();
    config.Slot0.kA = constants.gains.ka().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        constants.constraints.maxVelocityRadiansPerSecond().get();
    config.MotionMagic.MotionMagicAcceleration =
        constants.constraints.maxAccelerationRadiansPerSecondSqaured().get();
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));

    positionRotations = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temperature = hoodMotor.getDeviceTemp();
    appliedVolts = hoodMotor.getMotorVoltage();
    positionSetpointRotations = hoodMotor.getClosedLoopReference();
    positionErrorRotations = hoodMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations);
    hoodMotor.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        constants.canBus.isNetworkFD(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
    positionGoal = Rotation2d.kZero;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocity.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperature.getValue();
    inputs.positionGoal = positionGoal;
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionGoal = position;
    hoodMotor.setControl(
        positionControlRequest.withPosition(positionGoal.getRotations()).withEnableFOC(true));
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    config.Slot0.kS = ks;
    config.Slot0.kV = kv;
    config.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    config.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public boolean atGoal() {
    return Math.abs(positionGoal.getRotations() - positionRotations.getValueAsDouble())
        <= constants.constraints.goalToleranceRadians().get();
  }
}
