package frc.robot.subsystems.v0_Funky.turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;
import frc.robot.util.InternalLoggedTracer;

public class V0_FunkyTurretIOTalonFX {

  private StatusSignal<Angle> positionRotations;
  private TalonFX talonFX;
  private TalonFXConfiguration config;
  private double positionGoalRadians;
  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;
  private static CANcoder rightCANcoder;
  private static CANcoder leftCanCoder;

  private double e1;
  private double e2;

  private double maxAngle = V0_FunkyTurretConstants.MAX_ANGLE;
  private double minAngle = V0_FunkyTurretConstants.MIN_ANGLE;

  private double directionalGoal;

  /*
   * Gear Information:
   * Variables that store amount of gear teeth
   */

  private static final double GEAR_0_TOOTH_COUNT = 70.0; // fake values, update later
  private static final double GEAR_1_TOOTH_COUNT = 30.0;
  private static final double GEAR_2_TOOTH_COUNT = 20.0;
  private static final double SLOPE =
      (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
          / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

  /** Constructor for V0_FunkyTurretIOTalonFX */
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
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get()
            * V0_FunkyTurretConstants.GEAR_RATIO;

    positionRotations = talonFX.getPosition();
    positionVoltageRequest = new MotionMagicVoltage(0);
    voltageRequest = new VoltageOut(0);
  }

  /**
   * Method that gets encoder 1 value
   *
   * @return rightCANcoder value
   */
  public static double getEncoder1Value() {
    return rightCANcoder.getPosition().getValue().in(Units.Radians)
        - V0_FunkyTurretConstants.E1_OFFSET_RADIANS;
  }

  /**
   * Method that gets encoder 2 value
   *
   * @return leftCANcoder value
   */
  public static double getEncoder2Value() {
    return leftCanCoder.getPosition().getValue().in(Units.Radians)
        - V0_FunkyTurretConstants.E2_OFFSET_RADIANS;
  }

  /**
   * Method that sets position of turret
   *
   * @param radians
   */
  public void setPosition(double radians) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(radians * V0_FunkyTurretConstants.GEAR_RATIO);
    InternalLoggedTracer.record("Set Position", "Turret/TalonFX");
  }

  /**
   * Sets voltage of turret
   *
   * @param volts
   */
  public void setTurretVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageRequest.withOutput(volts));
    InternalLoggedTracer.record("Set Voltage", "Turret/TalonFX");
  }

  /**
   * Method that sets goal position of turret and applies voltage to go to that poisition
   *
   * @param goalRadians
   */
  public void setTurretGoal(double goalRadians) {
    double directionalGoalRadians = 0;
    double positiveDiff = goalRadians - calculateTurretAngle(e1, e2);
    double negativeDiff = positiveDiff - 2 * Math.PI;
    if (Math.abs(positiveDiff) < Math.abs(negativeDiff)
        && goalRadians <= maxAngle
        && goalRadians >= minAngle) {
      directionalGoalRadians = positiveDiff;
    } else if ((goalRadians - 2 * Math.PI) <= maxAngle && (goalRadians - 2 * Math.PI) >= minAngle) {
      directionalGoalRadians = negativeDiff;
    }
    InternalLoggedTracer.reset();
    talonFX.setControl(checkDirectionalMotion());
    InternalLoggedTracer.record("Set Goal", "Turret/TalonFX");
  }

  /**
   * Method that updates inputs of turret
   *
   * @param inputs
   */
  public void updateInputs(V0_FunkyTurretIOInputsAutoLogged inputs) {
    inputs.turretAngle = new Rotation2d(calculateTurretAngle(e1, e2));
    inputs.turretVelocityRadiansPerSecond =
        talonFX.getVelocity().getValue().in(Units.RadiansPerSecond)
            / V0_FunkyTurretConstants.GEAR_RATIO;
    inputs.turretAppliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
    inputs.turretSupplyCurrentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
    inputs.turretTorqueCurrentAmps = talonFX.getStatorCurrent().getValueAsDouble();
    inputs.turretTemperatureCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.turretPositionSetpoint = talonFX.getPosition().getValueAsDouble();
    inputs.turretPositionError = talonFX.getClosedLoopError().getValueAsDouble();
    inputs.turretTemperatureCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.turretGoal = positionGoalRadians;
  }

  /** Method that stops turret */
  public void stopTurret() {
    setTurretVoltage(0);
  }

  /**
   * Method that checks if turret is at goal position
   *
   * @return boolean value of whether turret is at goal position
   */
  public boolean atTurretPositionGoal() {
    double positionRadians = positionRotations.getValue().in(Units.Radians);
    return (Math.abs(positionGoalRadians - positionRadians)
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
    talonFX.getConfigurator().apply(config);
  }

  /**
   * Method that calculates turret angle based on encoder values
   *
   * @return
   */
  public static double calculateTurretAngle(double e1, double e2) {
    e1 = getEncoder1Value();
    e2 = getEncoder2Value();
    double diff = Math.abs(e2 - e1);
    if (diff > V0_FunkyTurretConstants.MAX_ANGLE) {
      diff -= Math.toRadians(360);
    } else if (diff < V0_FunkyTurretConstants.MIN_ANGLE) {
      diff += Math.toRadians(360);
    }
    diff *= SLOPE;

    double e1Rotations = (diff * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360 + e1) * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
    if (turretAngle - diff < -100) {
      turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360;
    } else if (turretAngle - diff > 100) {
      turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360;
    }
    return turretAngle;
  }

  /**
   * Method that updates constraints of turret
   *
   * @param maxAcceleration
   * @param maxVelocity
   * @param goalTolerance
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    config.MotionMagic.MotionMagicAcceleration =
        maxAcceleration * V0_FunkyTurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity * V0_FunkyTurretConstants.GEAR_RATIO;
    talonFX.getConfigurator().apply(config);
  }

  /** Method that resets turret position to zero */
  public void resetTurret() {
    setPosition(0);
  }

  /** Method that sets turret position to zero */
  public void goToZero() {
    setTurretGoal(0.0);
  }

  /**
   * Method that sets turret goal based on Pose3d goal
   *
   * @param goal
   */
  public void setTurretGoal(Pose3d goal) {
    Pose2d robotPosition = V0_FunkyRobotState.getGlobalPose();
    Pose2d aimGoal = goal.toPose2d();
    double diffY = aimGoal.getY() - robotPosition.getY();
    double diffX = aimGoal.getX() - robotPosition.getX();

    double angle = Math.atan2(diffY, diffX);
    setTurretGoal(angle);
  }

  /**
   * Method that sets turret goal based on field relative angle
   *
   * @param angleRadians: field relative angle in radians
   */
  public void goToFieldRelativeAngle(double angleRadians) {
    double turretAngle =
        V0_FunkyRobotState.getHeading().getRadians()
            + angleRadians
            - positionRotations.getValue().in(Units.Radians);
    setTurretGoal(turretAngle);
  }

  public MotionMagicVoltage checkDirectionalMotion() {
    if (positionGoalRadians < minAngle) {
      return positionVoltageRequest
          .withPosition(directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI))
          .withSlot(0);
    } else if (positionGoalRadians > maxAngle) {
      return positionVoltageRequest
          .withPosition(-1 * directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI))
          .withSlot(0);
    }

    return positionVoltageRequest
        .withPosition(directionalGoal * V0_FunkyTurretConstants.GEAR_RATIO / (2 * Math.PI))
        .withSlot(0);
  }
}
