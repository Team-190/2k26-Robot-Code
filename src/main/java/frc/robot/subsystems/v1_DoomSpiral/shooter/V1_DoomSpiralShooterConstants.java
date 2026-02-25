package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.subsystems.shared.hood.HoodConstants;

public class V1_DoomSpiralShooterConstants {

  public static final double FLYWHEEL_VELOCITY_INCREMENT_RPS = 5;
  public static final Rotation2d HOOD_ANGLE_INCREMENT_MAGNITUDE = Rotation2d.fromDegrees(0.5);

  public static final AngularVelocity TRENCH_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(400);
  public static final Rotation2d TRENCH_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(19.0);

  public static final AngularVelocity HUB_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(300);
  public static final Rotation2d HUB_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(0.0);

  public static final AngularVelocity TOWER_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(425.00);
  public static final Rotation2d TOWER_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(17.0);

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
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/VoltageKd", 0))
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
          .withConstraints( // Currently using positional angular constraints, should switch to
              // velocity angular constraints later.
              AngularConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(1000)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration", RadiansPerSecondPerSecond.of(1000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Flywheel/GoalTolerance", Radians.of(5)))
                  .build())
          .withOpposedFollowerCANID(30)
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
          .withConstraints(
              AngularConstraints.builder()
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
          .build();
}
