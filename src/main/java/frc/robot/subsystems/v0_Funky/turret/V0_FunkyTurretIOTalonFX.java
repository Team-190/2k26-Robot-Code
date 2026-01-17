package frc.robot.subsystems.v0_Funky.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;
import frc.robot.util.InternalLoggedTracer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.*;

public class V0_FunkyTurretIOTalonFX {

  private StatusSignal<Angle> positionRotations;
  private TalonFX talonFX;
  private TalonFXConfiguration config;
  private double positionGoalRadians;
  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;
  private CANcoder rightCANcoder;
  private CANcoder leftCanCoder;

  private double e1 = rightCANcoder.getPosition().getValue().in(Units.Radians);
  private double e2 = leftCanCoder.getPosition().getValue().in(Units.Radians);

  private double maxAngle = V0_FunkyTurretConstants.MAX_ANGLE;
  private double minAngle = V0_FunkyTurretConstants.MIN_ANGLE;

  private static final double GEAR_0_TOOTH_COUNT = 70.0; // fake values, update later
  private static final double GEAR_1_TOOTH_COUNT = 30.0;
  private static final double GEAR_2_TOOTH_COUNT = 20.0;
  private static final double  SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
      / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

  public V0_FunkyTurretIOTalonFX() {

    talonFX = new TalonFX(V0_FunkyTurretConstants.CAN_ID);

    config = new TalonFXConfiguration();
    config.Slot0.kP = V0_FunkyTurretConstants.GAINS.kP().get();
    config.Slot0.kD = V0_FunkyTurretConstants.GAINS.kD().get();
    config.Slot0.kV = V0_FunkyTurretConstants.GAINS.kV().get();
    config.Slot0.kA = V0_FunkyTurretConstants.GAINS.kA().get();

    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyTurretConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V0_FunkyTurretConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V0_FunkyTurretConstants.MAX_ANGLE * V0_FunkyTurretConstants.GEAR_RATIO;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V0_FunkyTurretConstants.MIN_ANGLE * V0_FunkyTurretConstants.GEAR_RATIO;

    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration =
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;

    positionRotations = talonFX.getPosition();
    positionVoltageRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);
  }

  public void setPosition(double radians) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(radians * V0_FunkyTurretConstants.GEAR_RATIO);
    InternalLoggedTracer.record("Set Position", "Turret/TalonFX");
  }

  public void setTurretVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageRequest.withOutput(volts));
    InternalLoggedTracer.record("Set Voltage", "Turret/TalonFX");
  }

  public void setTurretGoal(double goalRadians) {
    double directionalGoal = 0;
    double positiveDiff = goalRadians - positionRotations.getValue().in(Units.Radians);
    double negativeDiff = positiveDiff - 2*Math.PI;
    if ( Math.abs(positiveDiff) < Math.abs(negativeDiff) && goalRadians <= maxAngle && goalRadians >= minAngle){
      directionalGoal = positiveDiff;
    }
    else if ((goalRadians - 2*Math.PI) <= maxAngle && (goalRadians - 2*Math.PI) >= minAngle){
      directionalGoal = negativeDiff;
    }
    InternalLoggedTracer.reset();
    talonFX.setControl(
        positionVoltageRequest
            .withPosition(directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO/(2*Math.PI))
            .withSlot(0));
    InternalLoggedTracer.record("Set Goal", "Turret/TalonFX");
    
  }

  public void updateInputs(V0_FunkyTurretIOInputsAutoLogged inputs) {
    inputs.turretAngle =
        new Rotation2d(
            positionRotations.getValue().in(Units.Rotations));
    inputs.turretVelocityRadiansPerSecond =
        talonFX.getVelocity().getValue().in(Units.RadiansPerSecond) / V0_FunkyTurretConstants.GEAR_RATIO;
    inputs.turretAppliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
    inputs.turretSupplyCurrentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
    inputs.turretTorqueCurrentAmps = talonFX.getStatorCurrent().getValueAsDouble();
    inputs.turretTemperatureCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.turretPositionSetpoint = talonFX.getPosition().getValueAsDouble();
    inputs.turretPositionError = talonFX.getClosedLoopError().getValueAsDouble();
    inputs.turretTemperatureCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.turretGoal = positionGoalRadians;
  }

  public void stopTurret() {
    setTurretVoltage(0);
  }

  public boolean atTurretPositionGoal() {
    double positionRadians = positionRotations.getValue().in(Units.Radians);
    return (Math.abs(positionGoalRadians - positionRadians)
        <= V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get());
  }

  public void updateGains(double kP, double kD, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    talonFX.getConfigurator().apply(config);
  }

  public static double calculateTurretAngle(double e1, double e2) {
    double diff = e2 - e1;
    if (diff > V0_FunkyTurretConstants.MAX_ANGLE) {
      diff -= Math.toRadians(360);
    } else if (diff < V0_FunkyTurretConstants.MIN_ANGLE) {
      diff += Math.toRadians(360);
    }
    diff *= SLOPE;

    double e1Rotations = (diff * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle = (e1RotationsFloored * 360 + e1) * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
    if (turretAngle - diff < -100) {
      turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360;
    } else if (turretAngle - diff > 100) {
      turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360;
    }
    return turretAngle;
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration * V0_FunkyTurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity * V0_FunkyTurretConstants.GEAR_RATIO;
    talonFX.getConfigurator().apply(config);
  }

  public void resetTurret() {
    setPosition(0);
  }

  public void goToZero() {
    setTurretGoal(0.0);
  }

  public void setTurretGoal(Pose3d goal) {
    Pose2d robotPosition = V0_FunkyRobotState.getGlobalPose();
    Pose2d aimGoal = goal.toPose2d();
    double diffY = aimGoal.getY() - robotPosition.getY();
    double diffX = aimGoal.getX() - robotPosition.getX();

    double angle = Math.atan2(diffY, diffX);
    setTurretGoal(angle);
  }

  public void goToFieldRelativeAngle(double angleRadians) {
    double turretAngle = angleRadians - positionRotations.getValue().in(Units.Radians);
    setTurretGoal(turretAngle);
  }

  }
