package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;

public class TurretIOTalonFX implements TurretIO {

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> positionSetpoint;
  private final StatusSignal<Double> positionError;
  private final StatusSignal<Double> positionGoal;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;

  private final StatusSignal<Angle> e1;
  private final StatusSignal<Angle> e2;

  private final TalonFX talonFX;
  private final TalonFXConfiguration config;

  private final CANcoder rightCANCoder;
  private final CANcoder leftCANCoder;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  /*
   * Gear Information:
   * Variables that store amount of gear teeth
   */

  /** Constructor for V0_FunkyTurretIOTalonFX */
  public TurretIOTalonFX() {

    if (TurretConstants.IS_CAN_FD) {
      talonFX = new TalonFX(TurretConstants.TURRET_CAN_ID, V0_FunkyConstants.DRIVE_CONFIG.canBus());
    } else {
      talonFX = new TalonFX(TurretConstants.TURRET_CAN_ID);
    }

    leftCANCoder = new CANcoder(TurretConstants.LEFT_ENCODER_ID, talonFX.getNetwork());
    rightCANCoder = new CANcoder(TurretConstants.RIGHT_ENCODER_ID, talonFX.getNetwork());

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

    config.Slot0.kP = TurretConstants.GAINS.kP().get();
    config.Slot0.kD = TurretConstants.GAINS.kD().get();
    config.Slot0.kV = TurretConstants.GAINS.kV().get();
    config.Slot0.kA = TurretConstants.GAINS.kA().get();
    config.Slot0.kS = TurretConstants.GAINS.kS().get();

    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.MAX_ANGLE / (2 * Math.PI);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.MIN_ANGLE / (2 * Math.PI);
    config.MotionMagic.MotionMagicAcceleration =
        TurretConstants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        TurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get();
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    var leftCANcoderConfig = new CANcoderConfiguration();
    leftCANcoderConfig
        .MagnetSensor
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(Radians.of(TurretConstants.E1_OFFSET_RADIANS));
    PhoenixUtil.tryUntilOk(5, () -> leftCANCoder.getConfigurator().apply(leftCANcoderConfig, 0.25));

    var rightCANcoderConfig =
        leftCANcoderConfig
            .clone()
            .withMagnetSensor(
                leftCANcoderConfig
                    .MagnetSensor
                    .clone()
                    .withMagnetOffset(Radians.of(TurretConstants.E2_OFFSET_RADIANS)));
    PhoenixUtil.tryUntilOk(
        5, () -> rightCANCoder.getConfigurator().apply(rightCANcoderConfig, 0.25));

    position = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    temperature = talonFX.getDeviceTemp();
    positionSetpoint = talonFX.getClosedLoopOutput();
    positionError = talonFX.getClosedLoopError();
    positionGoal = talonFX.getClosedLoopReference();
    supplyCurrent = talonFX.getSupplyCurrent();
    torqueCurrent = talonFX.getTorqueCurrent();
    appliedVolts = talonFX.getMotorVoltage();

    e1 = leftCANCoder.getPosition();
    e2 = rightCANCoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        position,
        velocity,
        temperature,
        positionSetpoint,
        positionError,
        positionGoal,
        supplyCurrent,
        torqueCurrent,
        appliedVolts,
        e1,
        e2);
    talonFX.optimizeBusUtilization();
    leftCANCoder.optimizeBusUtilization();
    rightCANCoder.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        TurretConstants.IS_CAN_FD,
        position,
        velocity,
        temperature,
        positionSetpoint,
        positionError,
        positionGoal,
        supplyCurrent,
        torqueCurrent,
        appliedVolts,
        e1,
        e2);

    positionControlRequest = new MotionMagicVoltage(0);
    voltageControlRequest = new VoltageOut(0.0);
  }

  @Override
  public void setPosition(Rotation2d radians) {
    talonFX.setPosition(radians.getRotations());
  }

  @Override
  public void setTurretVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts));
  }

  @Override
  public void setTurretGoal(Rotation2d goal) {
    talonFX.setControl(
        positionControlRequest
            .withPosition(goal.getRotations())
            .withUseTimesync(true)
            .withEnableFOC(true));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.turretAngle = new Rotation2d(position.getValueAsDouble());
    inputs.turretVelocityRadiansPerSecond = velocity.getValue().in(Units.RadiansPerSecond);
    inputs.turretAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.turretTemperatureCelsius = temperature.getValueAsDouble();
    inputs.turretPositionSetpoint = Rotation2d.fromRotations(positionSetpoint.getValueAsDouble());
    inputs.turretPositionError = new Rotation2d(positionError.getValueAsDouble());
    inputs.turretGoal = new Rotation2d(positionGoal.getValueAsDouble());

    inputs.encoder1Position = new Rotation2d(e1.getValue());
    inputs.encoder2Position = new Rotation2d(e2.getValue());
  }

  @Override
  public boolean atTurretPositionGoal() {
    double positionRotations = position.getValueAsDouble();
    return Math.abs(positionGoal.getValue() - positionRotations) * 2 * Math.PI
        <= TurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public Angle getEncoder1Position() {
    return leftCANCoder.getAbsolutePosition().getValue();
  }

  @Override
  public Angle getEncoder2Position() {
    return rightCANCoder.getAbsolutePosition().getValue();
  }
}
