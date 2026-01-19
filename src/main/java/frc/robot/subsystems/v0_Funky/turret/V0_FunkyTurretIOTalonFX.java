package frc.robot.subsystems.v0_Funky.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurretIO.V0_FunkyTurretIOInputs;
import frc.robot.util.InternalLoggedTracer;

public class V0_FunkyTurretIOTalonFX implements V0_FunkyTurretIO {

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

  private static CANcoder rightCANCoder;
  private static CANcoder leftCANCoder;

  private final double maxAngle = V0_FunkyTurretConstants.MAX_ANGLE;
  private final double minAngle = V0_FunkyTurretConstants.MIN_ANGLE;

  private final VoltageOut voltageControlRequest;
  private final MotionMagicVoltage positionControlRequest;

  /*
   * Gear Information:
   * Variables that store amount of gear teeth
   */

  /** Constructor for V0_FunkyTurretIOTalonFX */
  public V0_FunkyTurretIOTalonFX() {

    if (V0_FunkyTurretConstants.IS_CAN_FD) {
      talonFX =
          new TalonFX(
              V0_FunkyTurretConstants.TURRET_CAN_ID, V0_FunkyConstants.DRIVE_CONFIG.canBus());
    } else {
      talonFX = new TalonFX(V0_FunkyTurretConstants.TURRET_CAN_ID);
    }

    leftCANCoder = new CANcoder(V0_FunkyTurretConstants.LEFT_ENCODER_ID, talonFX.getNetwork());
    rightCANCoder = new CANcoder(V0_FunkyTurretConstants.RIGHT_ENCODER_ID, talonFX.getNetwork());

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = V0_FunkyTurretConstants.GEAR_RATIO;

    config.Slot0.kP = V0_FunkyTurretConstants.GAINS.kP().get();
    config.Slot0.kD = V0_FunkyTurretConstants.GAINS.kD().get();
    config.Slot0.kV = V0_FunkyTurretConstants.GAINS.kV().get();
    config.Slot0.kA = V0_FunkyTurretConstants.GAINS.kA().get();
    config.Slot0.kS = V0_FunkyTurretConstants.GAINS.kS().get();

    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyTurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V0_FunkyTurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V0_FunkyTurretConstants.MAX_ANGLE / (2 * Math.PI);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V0_FunkyTurretConstants.MIN_ANGLE / (2 * Math.PI);
    config.MotionMagic.MotionMagicAcceleration =
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get();
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    var leftCANcoderConfig = new CANcoderConfiguration();
    leftCANcoderConfig
        .MagnetSensor
        .withMagnetOffset(V0_FunkyTurretConstants.E1_OFFSET_RADIANS / (2 * Math.PI))
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    PhoenixUtil.tryUntilOk(5, () -> leftCANCoder.getConfigurator().apply(leftCANcoderConfig, 0.25));

    var rightCANcoderConfig =
        leftCANcoderConfig
            .clone()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withMagnetOffset(V0_FunkyTurretConstants.E2_OFFSET_RADIANS / (2 * Math.PI)));
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

    positionControlRequest =
        new MotionMagicVoltage(
            V0_FunkyTurret.calculateTurretAngle(e1.getValue(), e2.getValue()).getRotations());
    voltageControlRequest = new VoltageOut(0.0);
  }

  public void setPosition(Rotation2d radians) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(radians.getRotations());
    InternalLoggedTracer.record("Set Position", "Turret/TalonFX");
  }

  @Override
  public void setTurretVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageControlRequest.withOutput(volts));
    InternalLoggedTracer.record("Set Voltage", "Turret/TalonFX");
  }

  @Override
  public void setTurretGoal(Rotation2d goal) {
    // Wrap the goal into the valid range
    double targetGoal = goal.getRadians();
    while (targetGoal < minAngle) {
      targetGoal += 2 * Math.PI;
    }
    while (targetGoal > maxAngle) {
      targetGoal -= 2 * Math.PI;
    }

    // Clamp to valid range as safety
    targetGoal = Math.max(minAngle, Math.min(maxAngle, targetGoal));

    InternalLoggedTracer.reset();
    // Send absolute position to MotionMagic in rotations
    talonFX.setControl(
        positionControlRequest
            .withPosition(targetGoal / (2 * Math.PI)) // Convert radians to rotations
            .withUseTimesync(true)
            .withUpdateFreqHz(200)
            .withEnableFOC(true));
    InternalLoggedTracer.record("Set Goal", "Turret/TalonFX");
  }

  public void updateInputs(V0_FunkyTurretIOInputs inputs) {

    inputs.turretAngle = new Rotation2d(position.getValue());
    inputs.turretVelocityRadiansPerSecond = velocity.getValue().in(Units.RadiansPerSecond);
    inputs.turretAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.turretSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.turretTorqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.turretTemperatureCelsius = temperature.getValueAsDouble();
    inputs.turretPositionSetpoint = Rotation2d.fromRotations(positionSetpoint.getValueAsDouble());
    inputs.turretPositionError = new Rotation2d(positionError.getValueAsDouble());
    inputs.turretGoal = new Rotation2d(positionGoal.getValueAsDouble());
  }

  public boolean atTurretPositionGoal() {
    double positionRotations = position.getValue().in(Units.Rotations);
    return Math.abs(positionGoal.getValue() - positionRotations) * 2 * Math.PI
        <= V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get();
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }
}
