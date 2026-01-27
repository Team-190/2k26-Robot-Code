package frc.robot.subsystems.shared.linkage;

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

public class LinkageIOTalonFX implements LinkageIO {
  private final TalonFX rightLinkageMotor;
  private final TalonFX leftLinkageMotor;

  private final StatusSignal<Angle> leftPositionRotations;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Temperature> leftTemperature;
  private final StatusSignal<Double> leftPositionSetpointRotations;
  private final StatusSignal<Double> leftPositionErrorRotations;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Voltage> leftAppliedVolts;

  private final StatusSignal<Angle> rightPositionRotations;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Temperature> rightTemperature;
  private final StatusSignal<Double> rightPositionSetpointRotations;
  private final StatusSignal<Double> rightPositionErrorRotations;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Voltage> rightAppliedVolts;

  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;
  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  public LinkageIOTalonFX() {
    if (LinkageConstants.IS_CAN_FD) {
      leftLinkageMotor =
          new TalonFX(LinkageConstants.LEFT_MOTOR_CAN_ID, LinkageConstants.DRIVE_CONFIG.canBus());
      rightLinkageMotor =
          new TalonFX(LinkageConstants.RIGHT_MOTOR_CAN_ID, LinkageConstants.DRIVE_CONFIG.canBus());
    } else {
      leftLinkageMotor = new TalonFX(LinkageConstants.LEFT_MOTOR_CAN_ID);
      rightLinkageMotor = new TalonFX(LinkageConstants.RIGHT_MOTOR_CAN_ID);
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
        .withForwardSoftLimitThreshold(LinkageConstants.MIN_ANGLE)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(LinkageConstants.MIN_ANGLE)
        .withReverseSoftLimitEnable(true);
    PhoenixUtil.tryUntilOk(5, () -> leftLinkageMotor.getConfigurator().apply(leftConfig, 0.25));

    rightConfig = leftConfig.clone();

    PhoenixUtil.tryUntilOk(5, () -> rightLinkageMotor.getConfigurator().apply(rightConfig, 0.25));

    leftPositionRotations = leftLinkageMotor.getPosition();
    leftVelocity = leftLinkageMotor.getVelocity();
    leftTorqueCurrent = leftLinkageMotor.getTorqueCurrent();
    leftSupplyCurrent = leftLinkageMotor.getSupplyCurrent();
    leftTemperature = leftLinkageMotor.getDeviceTemp();
    leftAppliedVolts = leftLinkageMotor.getMotorVoltage();
    leftPositionSetpointRotations = leftLinkageMotor.getClosedLoopReference();
    leftPositionErrorRotations = leftLinkageMotor.getClosedLoopError();

    rightPositionRotations = rightLinkageMotor.getPosition();
    rightVelocity = rightLinkageMotor.getVelocity();
    rightTorqueCurrent = rightLinkageMotor.getTorqueCurrent();
    rightSupplyCurrent = rightLinkageMotor.getSupplyCurrent();
    rightTemperature = rightLinkageMotor.getDeviceTemp();
    rightAppliedVolts = rightLinkageMotor.getMotorVoltage();
    rightPositionSetpointRotations = rightLinkageMotor.getClosedLoopReference();
    rightPositionErrorRotations = rightLinkageMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        leftPositionRotations,
        leftVelocity,
        leftTorqueCurrent,
        leftSupplyCurrent,
        leftAppliedVolts,
        leftTemperature,
        leftPositionSetpointRotations,
        leftPositionErrorRotations,
        rightPositionRotations,
        rightVelocity,
        rightTorqueCurrent,
        rightSupplyCurrent,
        rightAppliedVolts,
        rightTemperature,
        rightPositionSetpointRotations,
        rightPositionErrorRotations);

    leftLinkageMotor.optimizeBusUtilization();
    rightLinkageMotor.optimizeBusUtilization();

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
        rightPositionRotations,
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
    inputs.rightPosition = Rotation2d.fromRotations(rightPositionRotations.getValueAsDouble());
    inputs.rightVelocity = rightVelocity.getValue();
    inputs.rightSupplyCurrent = rightSupplyCurrent.getValue();
    inputs.rightTorqueCurrent = rightTorqueCurrent.getValue();
    inputs.rightAppliedVolts = rightAppliedVolts.getValue();
    inputs.rightTemperature = rightTemperature.getValue();
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
    inputs.leftPositionSetpoint =
        Rotation2d.fromRotations(leftPositionSetpointRotations.getValueAsDouble());
    inputs.leftPositionError =
        Rotation2d.fromRotations(leftPositionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setVoltageLeft(double volts) {
    leftLinkageMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setVoltageRight(double volts) {
    rightLinkageMotor.setControl(voltageControlRequest.withOutput(volts).withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPositionLeft(Rotation2d position) {
    leftLinkageMotor.setControl(
        positionControlRequest
            .withPosition(position.getRotations())
            .withEnableFOC(true)
            .withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPositionRight(Rotation2d position) {
    rightLinkageMotor.setControl(
        positionControlRequest
            .withPosition(position.getRotations())
            .withEnableFOC(true)
            .withUpdateFreqHz(1000.0));
  }

  @Override
  public void setPIDLeft(double kp, double ki, double kd) {
    leftConfig.Slot0.kP = kp;
    leftConfig.Slot0.kI = ki;
    leftConfig.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> leftLinkageMotor.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setPIDRight(double kp, double ki, double kd) {
    rightConfig.Slot0.kP = kp;
    rightConfig.Slot0.kI = ki;
    rightConfig.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> rightLinkageMotor.getConfigurator().apply(rightConfig, 0.25));
  }

  @Override
  public void setFeedforwardLeft(double ks, double kv, double ka) {
    leftConfig.Slot0.kS = ks;
    leftConfig.Slot0.kV = kv;
    leftConfig.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> leftLinkageMotor.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setFeedforwardRight(double ks, double kv, double ka) {
    rightConfig.Slot0.kS = ks;
    rightConfig.Slot0.kV = kv;
    rightConfig.Slot0.kA = ka;
    PhoenixUtil.tryUntilOk(5, () -> rightLinkageMotor.getConfigurator().apply(rightConfig, 0.25));
  }

  @Override
  public void setProfileLeft(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    leftConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> leftLinkageMotor.getConfigurator().apply(leftConfig, 0.25));
  }

  @Override
  public void setProfileRight(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocityRadiansPerSecond;
    rightConfig.MotionMagic.MotionMagicAcceleration = maxAccelerationRadiansPerSecondSquared;
    PhoenixUtil.tryUntilOk(5, () -> rightLinkageMotor.getConfigurator().apply(rightConfig, 0.25));
  }

  // TODO: REPLACE WITH ACTUAL GEOMETRIC CALCULATION

  @Override
  public boolean atGoalLeft() {
    return false;
  }

  @Override
  public boolean atGoalRight() {
    return false;
  }
}
