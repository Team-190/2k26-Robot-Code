package frc.robot.subsystems.shared.linkage;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class LinkageIOTalonFX implements LinkageIO {
  private final TalonFX talonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Double> positionGoalRotations;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;

  private final TalonFXConfiguration config;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  public LinkageIOTalonFX() {
    if (LinkageConstants.IS_CAN_FD) {
      talonFX = new TalonFX(LinkageConstants.MOTOR_CAN_ID, LinkageConstants.DRIVE_CONFIG.canBus());
    } else {
      talonFX = new TalonFX(LinkageConstants.MOTOR_CAN_ID);
    }

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = LinkageConstants.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = LinkageConstants.GEAR_RATIO;
    config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = LinkageConstants.MIN_ANGLE;
    config.Slot0.kP = LinkageConstants.GAINS.kp().get();
    config.Slot0.kD = LinkageConstants.GAINS.kd().get();
    config.Slot0.kS = LinkageConstants.GAINS.ks().get();
    config.Slot0.kV = LinkageConstants.GAINS.kv().get();
    config.Slot0.kA = LinkageConstants.GAINS.ka().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        LinkageConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get();
    config.MotionMagic.MotionMagicAcceleration =
        LinkageConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get();
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(LinkageConstants.MAX_ANGLE)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(LinkageConstants.MIN_ANGLE)
        .withReverseSoftLimitEnable(true);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    positionRotations = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    torqueCurrent = talonFX.getTorqueCurrent();
    supplyCurrent = talonFX.getSupplyCurrent();
    temperature = talonFX.getDeviceTemp();
    appliedVolts = talonFX.getMotorVoltage();
    positionGoalRotations = talonFX.getClosedLoopReference();
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

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
        positionErrorRotations);

    talonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        LinkageConstants.IS_CAN_FD,
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
  }

  @Override
  public void updateInputs(LinkageIOInputs inputs) {

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
    talonFX.setControl(
        voltageControlRequest.withOutput(volts).withEnableFOC(true).withUpdateFreqHz(100.0));
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    talonFX.setControl(
        positionControlRequest
            .withPosition(position.getRotations())
            .withEnableFOC(true)
            .withUpdateFreqHz(100.0));
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
        <= LinkageConstants.CONSTRAINTS.goalToleranceRadians().get();
  }
}
