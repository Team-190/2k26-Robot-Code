package frc.robot.subsystems.shared.fourbarlinkage;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;

public class FourBarLinkageIOTalonFX implements FourBarLinkageIO {
  private final TalonFX talonFX;

  private final CANcoder canCoder;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;
  private final StatusSignal<Angle> absolutePositionRotations;

  private final TalonFXConfiguration talonFXConfig;
  private final CANcoderConfiguration canCoderConfig;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  private final FourBarLinkageConstants constants;

  public FourBarLinkageIOTalonFX(FourBarLinkageConstants constants) {
    this.constants = constants;
    talonFX = new TalonFX(constants.motorCanId, constants.canBus);

    talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfig.Feedback.SensorToMechanismRatio = constants.gearRatio;
    talonFXConfig.Slot0.kP = constants.gains.kP().get();
    talonFXConfig.Slot0.kD = constants.gains.kD().get();
    talonFXConfig.Slot0.kS = constants.gains.kS().get();
    talonFXConfig.Slot0.kG = constants.gains.kG().get();
    talonFXConfig.Slot0.kV = constants.gains.kV().get();
    talonFXConfig.Slot0.kA = constants.gains.kA().get();
    talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        constants.constraints.maxVelocity().get().in(RotationsPerSecond);
    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        constants.constraints.maxAcceleration().get().in(RotationsPerSecondPerSecond);
    talonFXConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitThreshold(constants.maxAngle.getRotations())
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(constants.minAngle.getRotations())
        .withReverseSoftLimitEnable(false);

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));

    canCoder = new CANcoder(constants.canCoderCanId, constants.canBus);

    canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.SensorDirection = constants.cancoderSensorDirection;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    canCoderConfig.MagnetSensor.MagnetOffset = constants.canCoderOffset.getRotations();

    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(canCoderConfig, 0.25));

    // talonFX.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

    talonFX.setPosition(constants.minAngle.getRotations());
    positionRotations = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    torqueCurrent = talonFX.getTorqueCurrent();
    supplyCurrent = talonFX.getSupplyCurrent();
    temperature = talonFX.getDeviceTemp();
    appliedVolts = talonFX.getMotorVoltage();
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
        positionSetpointRotations,
        positionErrorRotations,
        absolutePositionRotations);

    talonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        constants.canBus.isNetworkFD(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePositionRotations);

    voltageControlRequest = new VoltageOut(0.0).withEnableFOC(constants.enableFoc);
    positionControlRequest = new MotionMagicVoltage(0.0).withEnableFOC(constants.enableFoc);
  }

  @Override
  public void updateInputs(FourBarLinkageIOInputs inputs) {

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocity.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperature.getValue();
    inputs.positionGoal = new Rotation2d(positionControlRequest.getPositionMeasure());
    inputs.positionSetpoint =
        Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());

    inputs.canCoderAbsolutePosition =
        Rotation2d.fromRotations(absolutePositionRotations.getValueAsDouble() / 2.0);
  }

  @Override
  public void setVoltageGoal(Voltage volts) {
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
  public void setGains(Gains gains) {
    talonFXConfig.Slot0.kP = gains.getKP();
    talonFXConfig.Slot0.kI = gains.getKI();
    talonFXConfig.Slot0.kD = gains.getKD();
    talonFXConfig.Slot0.kS = gains.getKS();
    talonFXConfig.Slot0.kV = gains.getKV();
    talonFXConfig.Slot0.kA = gains.getKA();
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));
  }

  @Override
  public void setProfile(AngularPositionConstraints constraints) {
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        constraints.maxVelocity().get(RotationsPerSecond);
    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        constraints.maxAcceleration().get(RotationsPerSecondPerSecond);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(
            positionRotations.getValue().minus(positionReference.getMeasure()).in(Rotations))
        <= constants.constraints.goalTolerance().get().in(Rotations);
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(appliedVolts.getValue(), Millivolts.of(500));
  }
}
