package frc.robot.subsystems.v0_Funky.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurretIO.V0_FunkyTurretIOInputs;
import frc.robot.util.InternalLoggedTracer;

public class V0_FunkyTurretIOTalonFX {

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

  private double directionalGoal;

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
      leftCANCoder =
          new CANcoder(
              V0_FunkyTurretConstants.LEFT_ENCODER_ID, V0_FunkyConstants.DRIVE_CONFIG.canBus());
      rightCANCoder =
          new CANcoder(
              V0_FunkyTurretConstants.RIGHT_ENCODER_ID, V0_FunkyConstants.DRIVE_CONFIG.canBus());
    } else {
      talonFX = new TalonFX(V0_FunkyTurretConstants.TURRET_CAN_ID);
      leftCANCoder = new CANcoder(V0_FunkyTurretConstants.LEFT_ENCODER_ID);
      rightCANCoder = new CANcoder(V0_FunkyTurretConstants.RIGHT_ENCODER_ID);
    }

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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = V0_FunkyTurretConstants.MAX_ANGLE;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = V0_FunkyTurretConstants.MIN_ANGLE;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration =
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get();
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    var turretCANcoderConfig = new CANcoderConfiguration();

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

  /**
   * Method that sets position of turret
   *
   * @param radians
   */
  public void setPosition(Rotation2d radians) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(radians.getRotations());
    InternalLoggedTracer.record("Set Position", "Turret/TalonFX");
  }

  /**
   * Sets voltage of turret
   *
   * @param volts
   */
  public void setTurretVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageControlRequest.withOutput(volts));
    InternalLoggedTracer.record("Set Voltage", "Turret/TalonFX");
  }

  /**
   * Method that sets goal position of turret and applies voltage to go to that poisition
   *
   * @param goalRadians
   */
  public void setTurretGoal(double goalRadians) {
    double directionalGoalRadians = 0;
    double positiveDiff =
        goalRadians
            - V0_FunkyTurret.calculateTurretAngle(e1.getValue(), e2.getValue()).getRadians();
    double negativeDiff = positiveDiff - 2 * Math.PI;
    if (Math.abs(positiveDiff) < Math.abs(negativeDiff)
        && goalRadians <= maxAngle
        && goalRadians >= minAngle) {
      directionalGoalRadians = positiveDiff;
    } else if ((goalRadians - 2 * Math.PI) <= maxAngle && (goalRadians - 2 * Math.PI) >= minAngle) {
      directionalGoalRadians = negativeDiff;
    }
    directionalGoal = directionalGoalRadians;
    InternalLoggedTracer.reset();
    talonFX.setControl(checkDirectionalMotion());
    InternalLoggedTracer.record("Set Goal", "Turret/TalonFX");
  }

  /**
   * Method that updates inputs of turret
   *
   * @param inputs
   */
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

  /**
   * Method that checks if turret is at goal position
   *
   * @return boolean value of whether turret is at goal position
   */
  public boolean atTurretPositionGoal() {
    double positionRadians = position.getValue().in(Units.Radians);
    return (Math.abs(positionGoal.getValue() - positionRadians)
        <= V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get());
  }

  /**
   * Method that updates gains of turret
   *
   * @param kP
   * @param kD
   * @param kV
   * @param kA
   */
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  /**
   * Method that calculates turret angle based on encoder values
   *
   * @return
   */

  /**
   * Method that updates constraints of turret
   *
   * @param maxAcceleration
   * @param maxVelocity
   * @param goalTolerance
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));
  }

  public MotionMagicVoltage checkDirectionalMotion() {
    if (positionGoal.getValue() > maxAngle) {
      return positionControlRequest
          .withPosition(-directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI))
          .withUseTimesync(true)
          .withUpdateFreqHz(200)
          .withEnableFOC(true);
    }
    return positionControlRequest
        .withPosition(directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI))
        .withUseTimesync(true)
        .withUpdateFreqHz(200)
        .withEnableFOC(true);
  }
}
