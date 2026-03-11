package frc.robot.subsystems.shared.linearextension;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import static edu.wpi.first.units.Units.Rotations;


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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;

public class LinearExtensionIOTalonFX implements LinearExtensionIO {
  private final TalonFX talonFX;

  private final CANcoder canCoder;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;
  private Rotation2d positionGoalRotations;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;
  private final StatusSignal<Angle> absolutePositionRotations;

  private final TalonFXConfiguration talonFXConfig;
  private final CANcoderConfiguration canCoderConfig;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  private final LinearExtensionConstants constants;

  public LinearExtensionIOTalonFX(LinearExtensionConstants constants) {
    this.constants = constants;
    talonFX = new TalonFX(constants.MOTOR_CAN_ID, constants.CAN_LOOP);

    talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.SupplyCurrentLimit = constants.SUPPLY_CURRENT_LIMIT;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = constants.STATOR_CURRENT_LIMIT;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfig.Feedback.SensorToMechanismRatio = constants.GEAR_RATIO;
    talonFXConfig.Slot0.kP = constants.GAINS.kP().get();
    talonFXConfig.Slot0.kD = constants.GAINS.kD().get();
    talonFXConfig.Slot0.kS = constants.GAINS.kS().get();
    talonFXConfig.Slot0.kG = constants.GAINS.kG().get();
    talonFXConfig.Slot0.kV = constants.GAINS.kV().get();
    talonFXConfig.Slot0.kA = constants.GAINS.kA().get();
    talonFXConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        constants.CONSTRAINTS.maxVelocity().get().in(RadiansPerSecond);
    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        constants.CONSTRAINTS.maxAcceleration().get().in(RadiansPerSecondPerSecond);

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));

    canCoder = new CANcoder(constants.CAN_CODER_CAN_ID, constants.CAN_LOOP);

    canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.SensorDirection = constants.CANCODER_SENSOR_DIRECTION;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    canCoderConfig.MagnetSensor.MagnetOffset = constants.CAN_CODER_OFFSET.getRotations();

    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(canCoderConfig, 0.25));

    // talonFX.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

    positionRotations = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    torqueCurrent = talonFX.getTorqueCurrent();
    supplyCurrent = talonFX.getSupplyCurrent();
    temperature = talonFX.getDeviceTemp();
    appliedVolts = talonFX.getMotorVoltage();
    positionGoalRotations = Rotation2d.kZero;
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
        constants.CAN_LOOP.isNetworkFD(),
        positionRotations,
        velocity,
        torqueCurrent,
        supplyCurrent,
        appliedVolts,
        temperature,
        positionSetpointRotations,
        positionErrorRotations,
        absolutePositionRotations);

    // talonFX.setPosition(
    //     (canCoder.getAbsolutePosition().getValueAsDouble() / 2.0)
    //         + constants.MIN_ANGLE.getRotations());

    voltageControlRequest = new VoltageOut(0.0).withEnableFOC(false);
    positionControlRequest = new MotionMagicVoltage(0.0).withEnableFOC(false);
  }

  @Override
  public void updateInputs(LinearExtensionIOInputs inputs) {

    inputs.position = positionRotations.getValueAsDouble();
    inputs.velocity = RadiansPerSecond.of(velocity.getValueAsDouble());
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.temperature = temperature.getValue();
    inputs.positionGoal = positionGoalRotations.getRotations();
    inputs.positionSetpoint =
        positionSetpointRotations.getValueAsDouble();
    inputs.positionError = positionErrorRotations.getValueAsDouble();

    inputs.canCoderAbsolutePosition =
        Rotation2d.fromRotations(absolutePositionRotations.getValueAsDouble() / 2.0);
  }


  public void setVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts));
  }

  public void setPositionGoal(Rotation2d position) {
    positionGoalRotations = position;
    talonFX.setControl(positionControlRequest.withPosition(position.getRotations()));
  }

  public void setPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  @Override
  public void setPID(double kp, double ki, double kd) {
    talonFXConfig.Slot0.kP = kp;
    talonFXConfig.Slot0.kI = ki;
    talonFXConfig.Slot0.kD = kd;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));
  }

  @Override
  public void setFeedforward(double ks, double kv, double kg, double ka) {
    talonFXConfig.Slot0.kS = ks;
    talonFXConfig.Slot0.kV = kv;
    talonFXConfig.Slot0.kA = ka;
    talonFXConfig.Slot0.kG = kg;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));
  }

  @Override
  public void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity =
        Units.radiansToRotations(maxVelocityRadiansPerSecond);
    talonFXConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfig, 0.25));
  }

  @Override
  public boolean atGoal() {
    return Math.abs(
            positionRotations.getValue().minus(positionGoalRotations.getMeasure()).in(Rotations))
        <= constants.CONSTRAINTS.goalTolerance().get().in(Rotations);
  }
}
