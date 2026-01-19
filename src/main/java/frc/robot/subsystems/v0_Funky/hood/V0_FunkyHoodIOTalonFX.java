package frc.robot.subsystems.v0_Funky.hood;

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
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;
import frc.robot.subsystems.v0_Funky.hood.V0_FunkyHoodConstants.HoodGoal;

public class V0_FunkyHoodIOTalonFX implements V0_FunkyHoodIO {
  private final TalonFX hoodMotor;

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

  public V0_FunkyHoodIOTalonFX() {
    if (V0_FunkyHoodConstants.IS_CAN_FD) {
      hoodMotor =
          new TalonFX(V0_FunkyHoodConstants.MOTOR_CAN_ID, V0_FunkyConstants.DRIVE_CONFIG.canBus());
    } else {
      hoodMotor = new TalonFX(V0_FunkyHoodConstants.MOTOR_CAN_ID);
    }

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyHoodConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = V0_FunkyHoodConstants.GEAR_RATIO;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = V0_FunkyHoodConstants.MIN_ANGLE;
    config.Slot0.kP = V0_FunkyHoodConstants.GAINS.kp().get();
    config.Slot0.kD = V0_FunkyHoodConstants.GAINS.kd().get();
    config.Slot0.kS = V0_FunkyHoodConstants.GAINS.ks().get();
    config.Slot0.kV = V0_FunkyHoodConstants.GAINS.kv().get();
    config.Slot0.kA = V0_FunkyHoodConstants.GAINS.ka().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyHoodConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get();
    config.MotionMagic.MotionMagicAcceleration =
        V0_FunkyHoodConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get();
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(V0_FunkyHoodConstants.MIN_ANGLE)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(V0_FunkyHoodConstants.MIN_ANGLE)
        .withReverseSoftLimitEnable(true);
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));

    hoodMotor.setPosition(HoodGoal.STOW.getAngle().getRotations());

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
        V0_FunkyHoodConstants.IS_CAN_FD,
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(HoodGoal.STOW.getAngle().getRotations());
    positionGoal = HoodGoal.STOW.getAngle();
  }

  @Override
  public void updateInputs(V0_FunkyHoodIOInputs inputs) {
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
    hoodMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionGoal = position;
    hoodMotor.setControl(
        positionControlRequest.withPosition(positionGoal.getRotations()).withUpdateFreqHz(1000.0));
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
        <= V0_FunkyHoodConstants.CONSTRAINTS.goalToleranceRadians().get();
  }
}
