package frc.robot.subsystems.shared.linkage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class FourBarLinkageIOTalonFX implements FourBarLinkageIO {
  private final TalonFX talonFX;

  private final CANcoder canCoder;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Double> positionGoalRotations;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;
  private final StatusSignal<Angle> absolutePositionRotations;

  private final TalonFXConfiguration config;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  private final FourBarLinkageConstants constants;

  public FourBarLinkageIOTalonFX(FourBarLinkageConstants constants) {
    this.constants = constants;
    if (constants.IS_CAN_FD) {
      talonFX = new TalonFX(constants.MOTOR_CAN_ID, constants.DRIVE_CONFIG.canBus());
    } else {
      talonFX = new TalonFX(constants.MOTOR_CAN_ID);
    }

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = constants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = constants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = constants.GEAR_RATIO;
    config.Slot0.kP = constants.GAINS.kp().get();
    config.Slot0.kD = constants.GAINS.kd().get();
    config.Slot0.kS = constants.GAINS.ks().get();
    config.Slot0.kV = constants.GAINS.kv().get();
    config.Slot0.kA = constants.GAINS.ka().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        constants.CONSTRAINTS.maxVelocityRadiansPerSecond().get();
    config.MotionMagic.MotionMagicAcceleration =
        constants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get();
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(constants.MAX_ANGLE.getRotations())
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(constants.MIN_ANGLE.getRotations())
        .withReverseSoftLimitEnable(true);

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    canCoder = new CANcoder(constants.CAN_CODER_CAN_ID);

    talonFX.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

    positionRotations = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    torqueCurrent = talonFX.getTorqueCurrent();
    supplyCurrent = talonFX.getSupplyCurrent();
    temperature = talonFX.getDeviceTemp();
    appliedVolts = talonFX.getMotorVoltage();
    positionGoalRotations = talonFX.getClosedLoopReference();
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();
    absolutePositionRotations = canCoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionGoalRotations,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePositionRotations);

    talonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        constants.IS_CAN_FD,
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePositionRotations);

    voltageControlRequest = new VoltageOut(0.0).withEnableFOC(constants.ENABLE_FOC);
    positionControlRequest = new MotionMagicVoltage(0.0).withEnableFOC(constants.ENABLE_FOC);
  }

  @Override
  public void updateInputs(FourBarLinkageIOInputs inputs) {

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocity.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperature.getValue();
    inputs.positionGoal = Rotation2d.fromRotations(positionGoalRotations.getValueAsDouble());
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts));
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    talonFX.setControl(positionControlRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    config.Slot0.kP = kp;
    config.Slot0.kI = ki;
    config.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setFeedforward(double ks, double kv, double ka) {
    config.Slot0.kS = ks;
    config.Slot0.kV = kv;
    config.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    config.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public boolean atGoal() {
    return Math.abs(Units.rotationsToRadians(positionErrorRotations.getValueAsDouble()))
        <= constants.CONSTRAINTS.goalToleranceRadians().get();
  }
}
