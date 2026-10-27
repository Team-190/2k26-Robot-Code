package first.robot.v1_DoomSpiral.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliamps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import first.robot.shared.subsystems.hood.HoodConstants;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;

public class V1_DoomSpiralShooterConstants {

  public static final AngularVelocity TRENCH_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(420.0);
  public static final Rotation2d TRENCH_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(20.0);

  public static final AngularVelocity HUB_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(350);
  public static final Rotation2d HUB_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(5.0);

  public static final AngularVelocity TOWER_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(402.00);
  public static final Rotation2d TOWER_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(18.5);

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(31)
          .withCanBus(new CANBus())
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
                  .withPrefix("Shooter/Flywheel/Voltage")
                  .withKP(0.5)
                  .withKD(0.0)
                  .withKS(0.21467)
                  .withKV(0.14015)
                  .withKA(0.0045447)
                  .build())
          .withTorqueGains(
              Gains.builder()
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(10)
                  .withKD(0.1)
                  .withKS(2.25)
                  .withKV(0.067114)
                  .withKA(0.11882)
                  .build())
          .withConstraints( // Currently using positional angular constraints, should switch to
              // velocity angular constraints later.
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(1000))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(RadiansPerSecond.of(5))
                  .build())
          .withOpposedFollowerCANID(30)
          .withVelocityOffsetStep(RadiansPerSecond.of(10))
          .withVoltageOffsetStep(Volts.of(1))
          .build();

  public static final HoodConstants HOOD_CONSTANTS =
      HoodConstants.builder()
          .withMotorCanId(32)
          .withCurrentLimits(new CurrentLimits(40, 30))
          .withGearRatio((36.0 / 12.0) * (24.0 / 18.0) * (296.0 / 14.0))
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withCanBus(new CANBus())
          .withLengthMeters(0.101596)
          .withMinAngle(Rotation2d.fromDegrees(2))
          .withMaxAngle(Rotation2d.fromDegrees(20))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(40.0))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withOffsetStep(Degrees.of(0.5))
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Hood")
                  .withMaxVelocity(RadiansPerSecond.of(200))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(Degrees.of(1.0))
                  .build())
          .withGains(
              Gains.builder()
                  .withPrefix("Shooter/Hood")
                  .withKP(600)
                  .withKD(2)
                  .withKS(0.32492)
                  .withKV(1.406)
                  .withKA(0)
                  .build())
          .withVoltageStep(Volts.of(0.5))
          .build();

  public static Transform2d SHOOTER_POSE =
      new Transform2d(-0.015, 0.255299, Rotation2d.fromDegrees(-90.0));

  public enum HoodGoal {
    SCORE,
    FEED,
    STOW,
    OVERRIDE
  }
}
