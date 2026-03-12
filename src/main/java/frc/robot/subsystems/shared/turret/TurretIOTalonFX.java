package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {

  protected final TurretConstants constants;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> positionSetpoint;
  private final StatusSignal<Double> positionError;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> appliedVolts;

  private final StatusSignal<Angle> e1;
  private final StatusSignal<Angle> e2;

  protected final TalonFX talonFX;
  private final TalonFXConfiguration config;

  protected final CANcoder encoder2;
  protected final CANcoder encoder1;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  /*
   * Gear Information:
   * Variables that store amount of gear teeth
   */

  /** Constructor for V0_FunkyTurretIOTalonFX */
  public TurretIOTalonFX(TurretConstants constants) {
    this.constants = constants;

    talonFX = new TalonFX(constants.turretCANID, constants.canBus);

    encoder1 = new CANcoder(constants.encoder1ID, talonFX.getNetwork());
    encoder2 = new CANcoder(constants.encoder2ID, talonFX.getNetwork());

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = constants.gearRatio;

    config.Slot0.kP = constants.gains.kP().get();
    config.Slot0.kD = constants.gains.kD().get();
    config.Slot0.kV = constants.gains.kV().get();
    config.Slot0.kA = constants.gains.kA().get();
    config.Slot0.kS = constants.gains.kS().get();

    config.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.maxAngle.getRotations();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.minAngle.getRotations();
    config.MotionMagic.MotionMagicAcceleration =
        constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond);
    config.MotionMagic.MotionMagicCruiseVelocity =
        constants.constraints.maxVelocity().get().in(RadiansPerSecond);

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    var e1CANcoderConfig = new CANcoderConfiguration();
    e1CANcoderConfig
        .MagnetSensor
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(constants.encoderInversion)
        .withMagnetOffset(Radians.of(constants.e1Offset.getRadians()));
    PhoenixUtil.tryUntilOk(5, () -> encoder1.getConfigurator().apply(e1CANcoderConfig, 0.25));

    var e2CANcoderConfig =
        e1CANcoderConfig
            .clone()
            .withMagnetSensor(
                e1CANcoderConfig
                    .MagnetSensor
                    .clone()
                    .withMagnetOffset(Radians.of(constants.e2Offset.getRadians())));
    PhoenixUtil.tryUntilOk(5, () -> encoder2.getConfigurator().apply(e2CANcoderConfig, 0.25));

    position = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    temperature = talonFX.getDeviceTemp();
    positionSetpoint = talonFX.getClosedLoopReference();
    positionError = talonFX.getClosedLoopError();
    supplyCurrent = talonFX.getSupplyCurrent();
    torqueCurrent = talonFX.getTorqueCurrent();
    appliedVolts = talonFX.getMotorVoltage();

    e1 = encoder1.getAbsolutePosition();
    e2 = encoder2.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        position,
        velocity,
        temperature,
        positionSetpoint,
        positionError,
        supplyCurrent,
        torqueCurrent,
        appliedVolts,
        e1,
        e2);
    talonFX.optimizeBusUtilization();
    encoder1.optimizeBusUtilization();
    encoder2.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        constants.canBus.isNetworkFD(),
        position,
        velocity,
        temperature,
        positionSetpoint,
        positionError,
        supplyCurrent,
        torqueCurrent,
        appliedVolts,
        e1,
        e2);

    positionControlRequest = new MotionMagicVoltage(0).withUseTimesync(true).withEnableFOC(true);
    voltageControlRequest = new VoltageOut(0.0).withUseTimesync(true).withEnableFOC(true);
  }

  @Override
  public void setPosition(Rotation2d position) {
    talonFX.setPosition(position.getRotations());
  }

  @Override
  public void setVoltage(Voltage volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts));
  }

  @Override
  public void setPositionGoal(Rotation2d goal) {
    talonFX.setControl(positionControlRequest.withPosition(goal.getRotations()));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    inputs.angle = new Rotation2d(position.getValue());
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
    inputs.positionSetpoint = Rotation2d.fromRotations(positionSetpoint.getValueAsDouble());
    inputs.positionError = Rotation2d.fromRotations(positionError.getValueAsDouble());
    inputs.positionGoal = Rotation2d.fromRotations(positionControlRequest.Position);

    inputs.encoder1Position = new Rotation2d(e1.getValue());
    inputs.encoder2Position = new Rotation2d(e2.getValue());
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRotations() - position.getValueAsDouble())
        <= constants.constraints.goalTolerance().get().in(Rotations);
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(appliedVolts.getValue(), Millivolts.of(500));
  }

  @Override
  public void updateGains(Gains gains) {
    config.Slot0.kP = gains.getKP();
    config.Slot0.kI = gains.getKI();
    config.Slot0.kD = gains.getKD();
    config.Slot0.kS = gains.getKS();
    config.Slot0.kV = gains.getKV();
    config.Slot0.kA = gains.getKA();
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateConstraints(AngularPositionConstraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity =
        constraints.maxVelocity().get(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        constraints.maxAcceleration().get(RotationsPerSecondPerSecond);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  @Override
  public Angle getEncoder1Position() {
    return e1.getValue();
  }

  @Override
  public Angle getEncoder2Position() {
    return e2.getValue();
  }
}
