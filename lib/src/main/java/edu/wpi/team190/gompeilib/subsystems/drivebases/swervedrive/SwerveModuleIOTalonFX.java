package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  protected final TalonFX driveTalonFX;
  protected final TalonFX turnTalonFX;
  protected final CANcoder cancoder;

  private final TalonFXConfiguration driveConfig;
  private final TalonFXConfiguration turnConfig;
  private final CANcoderConfiguration cancoderConfig;

  private final StatusSignal<Angle> drivePositionRotations;
  private final StatusSignal<AngularVelocity> driveVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;
  private final StatusSignal<Temperature> driveTemperatureCelcius;
  private final StatusSignal<Double> driveVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> driveVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> turnAbsolutePositionRotations;
  private final StatusSignal<Angle> turnPositionRotations;
  private final StatusSignal<AngularVelocity> turnVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;
  private final StatusSignal<Temperature> turnTemperatureCelcius;
  private Rotation2d turnPositionGoal;
  private final StatusSignal<Double> turnPositionSetpointRotations;
  private final StatusSignal<Double> turnPositionErrorRotations;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final TorqueCurrentFOC torqueCurrentRequest;
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;
  private final VelocityVoltage velocityVoltageRequest;
  private final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest;
  private final MotionMagicVoltage positionVoltageRequest;

  private final SwerveModuleConstants.ClosedLoopOutputType driveClosedLoopOutputType;
  private final SwerveModuleConstants.ClosedLoopOutputType turnClosedLoopOutputType;

  public SwerveModuleIOTalonFX(
      SwerveDriveConstants driveConstants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    driveTalonFX = new TalonFX(constants.DriveMotorId, driveConstants.driveConfig.canBus());
    turnTalonFX = new TalonFX(constants.SteerMotorId, driveConstants.driveConfig.canBus());
    cancoder = new CANcoder(constants.EncoderId, driveConstants.driveConfig.canBus());

    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = GompeiLib.getLoopPeriod();
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalonFX.setPosition(0.0, 0.25));

    turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new IllegalArgumentException("Unexpected value: " + constants.FeedbackSource);
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    turnConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = GompeiLib.getLoopPeriod();
    turnConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));

    cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    drivePositionRotations = driveTalonFX.getPosition();
    driveVelocityRotationsPerSecond = driveTalonFX.getVelocity();
    driveAppliedVolts = driveTalonFX.getMotorVoltage();
    driveSupplyCurrentAmps = driveTalonFX.getSupplyCurrent();
    driveTorqueCurrentAmps = driveTalonFX.getTorqueCurrent();
    driveTemperatureCelcius = driveTalonFX.getDeviceTemp();
    driveVelocitySetpointRotationsPerSecond = driveTalonFX.getClosedLoopReference();
    driveVelocityErrorRotationsPerSecond = driveTalonFX.getClosedLoopError();

    turnAbsolutePositionRotations = cancoder.getAbsolutePosition();
    turnPositionRotations = turnTalonFX.getPosition();
    turnVelocityRotationsPerSecond = turnTalonFX.getVelocity();
    turnAppliedVolts = turnTalonFX.getMotorVoltage();
    turnSupplyCurrentAmps = turnTalonFX.getSupplyCurrent();
    turnTorqueCurrentAmps = turnTalonFX.getTorqueCurrent();
    turnTemperatureCelcius = turnTalonFX.getDeviceTemp();
    turnPositionGoal = new Rotation2d();
    turnPositionSetpointRotations = turnTalonFX.getClosedLoopReference();
    turnPositionErrorRotations = turnTalonFX.getClosedLoopError();

    timestampQueue = PhoenixOdometryThread.getInstance(driveConstants).makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance(driveConstants)
            .registerSignal(driveTalonFX.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance(driveConstants).registerSignal(turnTalonFX.getPosition());

    torqueCurrentRequest = new TorqueCurrentFOC(0.0);
    velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);
    velocityVoltageRequest = new VelocityVoltage(0.0);
    positionTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0.0);
    positionVoltageRequest = new MotionMagicVoltage(0.0);

    driveClosedLoopOutputType = constants.DriveMotorClosedLoopOutput;
    turnClosedLoopOutputType = constants.SteerMotorClosedLoopOutput;

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        driveConstants.odometryFrequency,
        drivePositionRotations,
        turnPositionRotations,
        turnAbsolutePositionRotations);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);

    driveTalonFX.optimizeBusUtilization();
    turnTalonFX.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        true,
        drivePositionRotations,
        turnPositionRotations,
        turnAbsolutePositionRotations,
        driveVelocityRotationsPerSecond,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTemperatureCelcius,
        driveVelocitySetpointRotationsPerSecond,
        driveVelocityErrorRotationsPerSecond,
        turnVelocityRotationsPerSecond,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTemperatureCelcius,
        turnPositionSetpointRotations,
        turnPositionErrorRotations);
  }

  @Override
  @Trace
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRadians =
        Units.rotationsToRadians(drivePositionRotations.getValueAsDouble());
    inputs.driveVelocityRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityRotationsPerSecond.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrentAmps.getValueAsDouble();
    inputs.driveTemperatureCelcius = driveTemperatureCelcius.getValueAsDouble();
    inputs.driveVelocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(driveVelocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.driveVelocityErrorRadiansPerSecond =
        Units.rotationsToRadians(driveVelocityErrorRotationsPerSecond.getValueAsDouble());

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePositionRotations.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPositionRotations.getValueAsDouble());
    inputs.turnVelocityRadiansPerSecond =
        Units.rotationsToRadians(turnVelocityRotationsPerSecond.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrentAmps.getValueAsDouble();
    inputs.turnTemperatureCelcius = turnTemperatureCelcius.getValueAsDouble();
    inputs.turnPositionGoal = turnPositionGoal;
    inputs.turnPositionSetpoint =
        Rotation2d.fromRotations(turnPositionSetpointRotations.getValueAsDouble());
    inputs.turnPositionError =
        Rotation2d.fromRotations(turnPositionErrorRotations.getValueAsDouble());

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRadians =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  @Trace
  public void setDriveAmps(double currentAmps) {
    driveTalonFX.setControl(torqueCurrentRequest.withOutput(currentAmps));
  }

  @Override
  @Trace
  public void setTurnAmps(double currentAmps) {
    turnTalonFX.setControl(torqueCurrentRequest.withOutput(currentAmps));
  }

  @Override
  @Trace
  public void setDriveVelocity(double velocityRadiansPerSecond, double currentFeedforward) {
    if (driveClosedLoopOutputType.equals(SwerveModuleConstants.ClosedLoopOutputType.Voltage)) {
      driveTalonFX.setControl(
          velocityVoltageRequest
              .withVelocity(Units.radiansToRotations(velocityRadiansPerSecond))
              .withFeedForward(currentFeedforward));
    } else if (driveClosedLoopOutputType.equals(
        SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC)) {
      driveTalonFX.setControl(
          velocityTorqueCurrentRequest
              .withVelocity(Units.radiansToRotations(velocityRadiansPerSecond))
              .withFeedForward(currentFeedforward));
    }
  }

  @Override
  @Trace
  public void setTurnPosition(Rotation2d rotation) {
    turnPositionGoal = rotation;
    if (turnClosedLoopOutputType.equals(SwerveModuleConstants.ClosedLoopOutputType.Voltage))
      turnTalonFX.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    if (turnClosedLoopOutputType.equals(
        SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC))
      turnTalonFX.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }

  @Override
  @Trace
  public void setPID(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {
    driveConfig.Slot0.kP = drive_Kp;
    driveConfig.Slot0.kD = drive_Kd;
    turnConfig.Slot0.kP = turn_Kp;
    turnConfig.Slot0.kD = turn_Kd;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  @Trace
  public void setFeedforward(double drive_Ks, double drive_Kv) {
    driveConfig.Slot0.kS = drive_Ks;
    driveConfig.Slot0.kV = drive_Kv;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  @Trace
  public void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {
    driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimit;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimit;
    tryUntilOk(5, () -> driveTalonFX.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnTalonFX.getConfigurator().apply(turnConfig, 0.25));
  }
}
