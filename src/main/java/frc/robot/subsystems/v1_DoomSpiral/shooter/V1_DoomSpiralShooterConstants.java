package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.subsystems.shared.hood.HoodConstants;

public class V1_DoomSpiralShooterConstants {

  public static final double FLYWHEEL_VELOCITY_INCREMENT_RPS = 1;
  public static final Rotation2d HOOD_ANGLE_INCREMENT_ROTATIONS = Rotation2d.fromDegrees(1);

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withCanBus(CANBus.roboRIO())
          .withEnableFOC(true)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(new GenericFlywheelConstants.CurrentLimits(60.0, 40.0))
          .withMomentOfInertia(0.05)
          .withGearRatio(24.0 / 18.0)
          .withMotorConfig(DCMotor.getKrakenX60Foc(2))
          .withGains(
              new GenericFlywheelConstants.Gains(
                  new LoggedTunableNumber("Shooter/Flywheel/Ks", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kv", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Ka", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kp", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kd", 0)))
          .withConstraints(
              new GenericFlywheelConstants.Constraints(
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/MaxAccelerationRadiansPerSecondSquared", 6),
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/CruisingVelocityRadiansPerSecondSquared", 4),
                  new LoggedTunableNumber("Shooter/Flywheel/GoalToleranceRadians", 0.05)))
          .withOpposedFollowerCANID(31)
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
              HoodConstants.Constraints.builder()
                  .withMaxVelocityRadiansPerSecond(
                      new LoggedTunableNumber("Shooter/Hood/MaxVelocityRadiansPerSecond", 200))
                  .withMaxAccelerationRadiansPerSecondSqaured(
                      new LoggedTunableNumber(
                          "Shooter/Hood/MaxAccelerationRadiansPerSecondSquared", 1000))
                  .withGoalToleranceRadians(
                      new LoggedTunableNumber(
                          "Shooter/Hood/GoalToleranceRadians", Units.degreesToRadians(0.01)))
                  .build())
          .withGains(
              HoodConstants.Gains.builder()
                  .withKp(new LoggedTunableNumber("Shooter/Hood/Kp", 600))
                  .withKd(new LoggedTunableNumber("Shooter/Hood/Kd", 2))
                  .withKs(new LoggedTunableNumber("Shooter/Hood/Ks", 0.32492))
                  .withKa(new LoggedTunableNumber("Shooter/Hood/Ka", 0))
                  .withKv(new LoggedTunableNumber("Shooter/Hood/Kv", 1.406))
                  .build())
          .build();
}
