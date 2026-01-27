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
  private final TalonFX rightTalonFX;
  private final TalonFX leftTalonFX;

  private final StatusSignal<Angle> leftPositionRotations;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Temperature> leftTemperature;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Double> leftPositionGoal;
  private final StatusSignal<Double> leftPositionSetpointRotations;
  private final StatusSignal<Double> leftPositionErrorRotations;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Temperature> rightTemperature;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Double> rightPositionGoalRotations;
  private final StatusSignal<Double> rightPositionSetpointRotations;
  private final StatusSignal<Double> rightPositionErrorRotations;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  public LinkageIOTalonFX() {
    if (LinkageConstants.IS_CAN_FD) {
      leftTalonFX =
          new TalonFX(LinkageConstants.LEFT_MOTOR_CAN_ID, LinkageConstants.DRIVE_CONFIG.canBus());
      rightTalonFX =
          new TalonFX(LinkageConstants.RIGHT_MOTOR_CAN_ID, LinkageConstants.DRIVE_CONFIG.canBus());
    } else {
      leftTalonFX = new TalonFX(LinkageConstants.LEFT_MOTOR_CAN_ID);
      rightTalonFX = new TalonFX(LinkageConstants.RIGHT_MOTOR_CAN_ID);
    }

    leftConfig = new TalonFXConfiguration();

    leftConfig.CurrentLimits.SupplyCurrentLimit = LinkageConstants.CURRENT_LIMIT;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Feedback.SensorToMechanismRatio = LinkageConstants.GEAR_RATIO;
    leftConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = LinkageConstants.MIN_ANGLE;
    leftConfig.Slot0.kP = LinkageConstants.GAINS.kp().get();
    leftConfig.Slot0.kD = LinkageConstants.GAINS.kd().get();
    leftConfig.Slot0.kS = LinkageConstants.GAINS.ks().get();
    leftConfig.Slot0.kV = LinkageConstants.GAINS.kv().get();
    leftConfig.Slot0.kA = LinkageConstants.GAINS.ka().get();
    leftConfig.MotionMagic.MotionMagicCruiseVelocity =
        LinkageConstants.CONSTRAINTS.maxVelocityRadiansPerSecond().get();
    leftConfig.MotionMagic.MotionMagicAcceleration =
        LinkageConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSqaured().get();
    leftConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitThreshold(LinkageConstants.MAX_ANGLE)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(LinkageConstants.MIN_ANGLE)
        .withReverseSoftLimitEnable(true);
    PhoenixUtil.tryUntilOk(5, () -> leftTalonFX.getConfigurator().apply(leftConfig, 0.25));

    rightConfig = leftConfig.clone();

    PhoenixUtil.tryUntilOk(5, () -> rightTalonFX.getConfigurator().apply(rightConfig, 0.25));

    leftPositionRotations = leftTalonFX.getPosition();
    leftVelocity = leftTalonFX.getVelocity();
    leftTorqueCurrent = leftTalonFX.getTorqueCurrent();
    leftSupplyCurrent = leftTalonFX.getSupplyCurrent();
    leftTemperature = leftTalonFX.getDeviceTemp();
    leftAppliedVolts = leftTalonFX.getMotorVoltage();
    leftPositionGoal = leftTalonFX.getClosedLoopReference();
    leftPositionSetpointRotations = leftTalonFX.getClosedLoopReference();
    leftPositionErrorRotations = leftTalonFX.getClosedLoopError();

    rightPosition = rightTalonFX.getPosition();
    rightVelocity = rightTalonFX.getVelocity();
    rightTorqueCurrent = rightTalonFX.getTorqueCurrent();
    rightSupplyCurrent = rightTalonFX.getSupplyCurrent();
    rightTemperature = rightTalonFX.getDeviceTemp();
    rightAppliedVolts = rightTalonFX.getMotorVoltage();
    rightPositionGoalRotations = rightTalonFX.getClosedLoopReference();
    rightPositionSetpointRotations = rightTalonFX.getClosedLoopReference();
    rightPositionErrorRotations = rightTalonFX.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        leftPositionRotations,
        leftVelocity,
        leftTorqueCurrent,
        leftSupplyCurrent,
        leftAppliedVolts,
        leftTemperature,
        leftPositionGoal,
        leftPositionSetpointRotations,
        leftPositionErrorRotations,
        rightPosition,
        rightVelocity,
        rightTorqueCurrent,
        rightSupplyCurrent,
        rightAppliedVolts,
        rightTemperature,
        rightPositionGoalRotations,
        rightPositionSetpointRotations,
        rightPositionErrorRotations);

    leftTalonFX.optimizeBusUtilization();
    rightTalonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        LinkageConstants.IS_CAN_FD,
        leftPositionRotations,
        leftVelocity,
        leftTorqueCurrent,
        leftSupplyCurrent,
        leftAppliedVolts,
        leftTemperature,
        leftPositionSetpointRotations,
        leftPositionErrorRotations,
        rightPosition,
        rightVelocity,
        rightTorqueCurrent,
        rightSupplyCurrent,
        rightAppliedVolts,
        rightTemperature,
        rightPositionSetpointRotations,
        rightPositionErrorRotations);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(LinkageIOInputs inputs) {
    inputs.rightPosition = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
    inputs.rightVelocity = rightVelocity.getValue();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
    inputs.rightTorqueCurrent = rightTorqueCurrent.getValue();
    inputs.rightAppliedVolts = rightAppliedVolts.getValue();
    inputs.rightTemperature = rightTemperature.getValue();
    inputs.rightPositionGoal =
        Rotation2d.fromRotations(rightPositionGoalRotations.getValueAsDouble());
    inputs.rightPositionSetpoint =
        Rotation2d.fromRotations(rightPositionSetpointRotations.getValueAsDouble());
    inputs.rightPositionError =
        Rotation2d.fromRotations(rightPositionErrorRotations.getValueAsDouble());

    inputs.leftPosition = Rotation2d.fromRotations(leftPositionRotations.getValueAsDouble());
    inputs.leftVelocity = leftVelocity.getValue();
    inputs.leftSupplyCurrent = leftSupplyCurrent.getValue();
    inputs.leftTorqueCurrent = leftTorqueCurrent.getValue();
    inputs.leftAppliedVolts = leftAppliedVolts.getValue();
    inputs.leftTemperature = leftTemperature.getValue();
    inputs.leftPositionGoal = Rotation2d.fromRotations(leftPositionGoal.getValueAsDouble());
    inputs.leftPositionSetpoint =
        Rotation2d.fromRotations(leftPositionSetpointRotations.getValueAsDouble());
    inputs.leftPositionError =
        Rotation2d.fromRotations(leftPositionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltageLeft(double volts) {
    leftTalonFX.setControl(
        voltageControlRequest.withOutput(volts).withEnableFOC(true).withUpdateFreqHz(100.0));
  }

  @Override
  public void setVoltageRight(double volts) {
    rightTalonFX.setControl(
        voltageControlRequest.withOutput(volts).withEnableFOC(true).withUpdateFreqHz(100.0));
  }

  @Override
  public void setPositionGoalLeft(Rotation2d position) {
    leftTalonFX.setControl(
        positionControlRequest
            .withPosition(position.getRotations())
            .withEnableFOC(true)
            .withUpdateFreqHz(100.0));
  }

  @Override
  public void setPositionGoalRight(Rotation2d position) {
    rightTalonFX.setControl(
        positionControlRequest
            .withPosition(position.getRotations())
            .withEnableFOC(true)
            .withUpdateFreqHz(100.0));
  }

  @Override
  public void setPositionLeft(Rotation2d position) {
    leftTalonFX.setPosition(position.getRotations());
  }

  @Override
  public void setPositionRight(Rotation2d position) {
    rightTalonFX.setPosition(position.getRotations());
  }

  @Override
  public void setPIDLeft(double kp, double ki, double kd) {
    leftConfig.Slot0.kP = kp;
    leftConfig.Slot0.kI = ki;
    leftConfig.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> leftTalonFX.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setPIDRight(double kp, double ki, double kd) {
    rightConfig.Slot0.kP = kp;
    rightConfig.Slot0.kI = ki;
    rightConfig.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> rightTalonFX.getConfigurator().apply(rightConfig, 0.25));
  }

  @Override
  public void setFeedforwardLeft(double ks, double kv, double ka) {
    leftConfig.Slot0.kS = ks;
    leftConfig.Slot0.kV = kv;
    leftConfig.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> leftTalonFX.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setFeedforwardRight(double ks, double kv, double ka) {
    rightConfig.Slot0.kS = ks;
    rightConfig.Slot0.kV = kv;
    rightConfig.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> rightTalonFX.getConfigurator().apply(rightConfig, 0.25));
  }

  @Override
  public void setProfileLeft(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    leftConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> leftTalonFX.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setProfileRight(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    rightConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> rightTalonFX.getConfigurator().apply(rightConfig, 0.25));
  }

  @Override
  public boolean atGoalLeft() {
    return Math.abs(Units.rotationsToRadians(leftPositionErrorRotations.getValueAsDouble()))
        <= LinkageConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  @Override
  public boolean atGoalRight() {
    return Math.abs(Units.rotationsToRadians(rightPositionErrorRotations.getValueAsDouble()))
        <= LinkageConstants.CONSTRAINTS.goalToleranceRadians().get();
  }
}
