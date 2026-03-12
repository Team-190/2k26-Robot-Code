package frc.robot.subsystems.shared.hood;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;

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

    config.Slot0.kP = constants.gains.kP().get();
    config.Slot0.kD = constants.gains.kD().get();
    config.Slot0.kS = constants.gains.kS().get();
    config.Slot0.kV = constants.gains.kV().get();
    config.Slot0.kA = constants.gains.kA().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        constants.constraints.maxVelocity().get().in(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        constants.constraints.maxAcceleration().get().in(RotationsPerSecondPerSecond);
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

    hoodMotor.setPosition(0);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
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
  }

  @Override
  public void setVoltage(Voltage volts) {
    hoodMotor.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPositionGoal(Rotation2d position) {
    hoodMotor.setControl(
        positionControlRequest.withPosition(positionGoal.getRotations()).withEnableFOC(true));
  }

  public void setPosition(Rotation2d position) {
    hoodMotor.setPosition(position.getRotations());
  }

  @Override
  public void setGains(Gains gains) {
    config.Slot0.kP = gains.getKP();
    config.Slot0.kI = gains.getKI();
    config.Slot0.kD = gains.getKD();
    config.Slot0.kS = gains.getKS();
    config.Slot0.kV = gains.getKV();
    config.Slot0.kA = gains.getKA();
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setProfile(AngularPositionConstraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity =
        constraints.maxVelocity().get(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        constraints.maxAcceleration().get(RotationsPerSecondPerSecond);
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(positionReference.getRotations() - positionRotations.getValueAsDouble())
        <= constants.constraints.goalTolerance().get().in(Rotations);
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageReference.isNear(appliedVolts.getValue(), Millivolts.of(500));
  }
}
