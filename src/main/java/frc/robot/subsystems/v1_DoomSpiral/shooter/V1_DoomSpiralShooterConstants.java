package frc.robot.subsystems.v1_DoomSpiral.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliamps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.subsystems.shared.hood.HoodConstants;

public class V1_DoomSpiralShooterConstants {

  public static final double FLYWHEEL_VELOCITY_INCREMENT_RPS = 1;
  public static final Rotation2d HOOD_ANGLE_INCREMENT_ROTATIONS = Rotation2d.fromDegrees(1);

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(new GenericFlywheelConstants.CurrentLimits(60.0, 40.0))
          .withMomentOfInertia(0.05)
          .withGearRatio(1.0)
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
                      "Shooter/Flywheel/MaxAccelerationRotationsPerSecondSquared", 6),
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/CruisingVelocityRotationsPerSecondSquared", 4),
                  new LoggedTunableNumber("Shooter/Flywheel/GoalToleranceRadians", 0.05)))
          .withOpposedFollowerCANID(31)
          .build();

  public static final HoodConstants HOOD_CONSTANTS = // TODO: Fix these constants
      HoodConstants.builder()
          .withMotorCanId(-1)
          .withCurrentLimits(40.0)
          .withGearRatio(1.0)
          .withMomentOfInertia(0.0001)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withCanBus(CANBus.roboRIO())
          .withLengthMeters(.1)
          .withMinAngle(new Rotation2d())
          .withMaxAngle(new Rotation2d(Math.PI / 2))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(40.0))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withConstraints(HoodConstants.Constraints.builder().build())
          .withGains(HoodConstants.Gains.builder().build())
          .build();
}
