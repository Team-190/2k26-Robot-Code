package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.subsystems.shared.hood.HoodConstants;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretAngleCalculation;

public class V2_DeltaShooterConstants {

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(31)
          .withCanBus(CANBus.roboRIO())
          .withEnableFOC(true)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(60.0))
                  .withStatorCurrentLimit(Amps.of(80.0))
                  .build())
          .withMomentOfInertia(0.05)
          .withGearRatio(28.0 / 24.0)
          .withMotorConfig(DCMotor.getKrakenX60Foc(2))
          .withVoltageGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/VoltageKp", .5))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/VoltageKd", 0.0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/VoltageKs", 0.21467))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/VoltageKv", 0.14015))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/VoltageKa", 0.0045447))
                  .build())
          .withTorqueGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/TorqueKp", 10))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/TorqueKd", 0.1))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/TorqueKs", 2.25))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/TorqueKv", 0.067114))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/TorqueKa", 0.11882))
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(1000)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration", RadiansPerSecondPerSecond.of(1000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/GoalTolerance", RadiansPerSecond.of(5)))
                  .build())
          .withOpposedFollowerCANID(30)
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
          .withVoltageOffsetStep(Volts.of(1))
          .build();

  public static final HoodConstants HOOD_CONSTANTS =
      HoodConstants.builder()
          .withMotorCanId(32)
          .withCurrentLimits(40.0)
          .withGearRatio((36.0 / 12.0) * (24.0 / 18.0) * (296.0 / 14.0))
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withCanBus(CANBus.roboRIO())
          .withLengthMeters(0.101596)
          .withMinAngle(Rotation2d.fromDegrees(0.1))
          .withMaxAngle(Rotation2d.fromDegrees(20))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(40.0))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withOffsetStep(Degrees.of(0.5))
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxVelocity", RadiansPerSecond.of(200)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxAcceleration", RadiansPerSecondPerSecond.of(1000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Hood/GoalTolerance", Degrees.of(1.0)))
                  .build())
          .withGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Hood/Kp", 600))
                  .withKD(new LoggedTunableNumber("Shooter/Hood/Kd", 2))
                  .withKS(new LoggedTunableNumber("Shooter/Hood/Ks", 0.32492))
                  .withKV(new LoggedTunableNumber("Shooter/Hood/Kv", 1.406))
                  .withKA(new LoggedTunableNumber("Shooter/Hood/Ka", 0))
                  .build())
          .withVoltageStep(Volts.of(0.5))
          .build();

  public static final TurretConstants TURRET_CONSTANTS =
      TurretConstants.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMomentOfInertia(0.004)
          .withTurretCANID(2)
          .withMotorInversion(InvertedValue.CounterClockwise_Positive)
          .withEncoderInversion(SensorDirectionValue.Clockwise_Positive)
          .withCanBus(CANBus.roboRIO())
          .withEncoder1ID(16)
          .withEncoder2ID(15)
          .withMaxAngle(Rotation2d.fromRadians(2 * Math.PI))
          .withMinAngle(Rotation2d.fromRadians(-2 * Math.PI))
          .withGearRatio(120.0 / 20)
          .withSupplyCurrentLimit(30.0)
          .withStatorCurrentLimit(30.0)
          .withE1Offset(
              Rotation2d.fromRotations(-0.521973)
                  .minus(Rotation2d.fromDegrees(309.726563 + 301.201172)))
          .withE2Offset(
              Rotation2d.fromRotations(-0.44458).minus(Rotation2d.fromDegrees(325.371094)))
          .withGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Turret/Kp", 2.8624920))
                  .withKD(new LoggedTunableNumber("Turret/Kd", 0.0))
                  .withKS(new LoggedTunableNumber("Turret/Ks", 0.158040))
                  .withKV(new LoggedTunableNumber("Turret/Kv", 0.11377))
                  .withKA(new LoggedTunableNumber("Turret/Ka", 0.0031713))
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Turret/MaxAcceleration",
                          RadiansPerSecondPerSecond.of(35.566371)))
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Turret/MaxVelocity", RadiansPerSecond.of(89.566371)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Turret/GoalTolerance", Degrees.of(3)))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(120)
                  .withGear1ToothCount(16)
                  .withGear2ToothCount(17)
                  .build())
          .withRobotToTurretTransform(
              new Transform3d(-0.017463, -0.163513, -0.371475, new Rotation3d()))
          .withVoltageStep(Volts.of(0.5))
          .withAngleStep(Rotation2d.fromDegrees(1.0))
          .build();

  public enum ShooterGoal {
    SCORE,
    FEED,
    STOW,
    OVERRIDE_TURRET,
    OVERRIDE_HOOD,
    OVERRIDE_FLYWHEEL,
    SYSID
  }
}