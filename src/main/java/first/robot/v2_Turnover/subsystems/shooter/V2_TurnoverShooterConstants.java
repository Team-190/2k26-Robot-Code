package first.robot.v2_Turnover.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliamps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import first.robot.FieldConstants;
import first.robot.shared.subsystems.hood.HoodConstants;
import first.robot.shared.subsystems.turret.TurretConstants;
import first.robot.shared.subsystems.turret.TurretConstants.TurretAngleCalculation;
import first.robot.v2_Turnover.V2_TurnoverConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;

public class V2_TurnoverShooterConstants {

  public static final AngularVelocity LEFT_TRENCH_SHOT_FLYWHEEL_SPEED =
      RotationsPerSecond.of(26.679);
  public static final Rotation2d LEFT_TRENCH_SHOT_HOOD_ANGLE = Rotation2d.fromRotations(0.273);
  public static final Translation2d LEFT_TRENCH_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.LeftTrench.openingTopLeft.getX(),
          FieldConstants.LeftTrench.openingTopLeft.getY() - FieldConstants.LeftTrench.width / 2);

  public static final AngularVelocity RIGHT_TRENCH_SHOT_FLYWHEEL_SPEED =
      RotationsPerSecond.of(27.980);
  public static final Rotation2d RIGHT_TRENCH_SHOT_HOOD_ANGLE = Rotation2d.fromRotations(0.314);
  public static final Translation2d RIGHT_TRENCH_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.RightTrench.openingTopLeft.getX(),
          FieldConstants.RightTrench.openingTopLeft.getY() - FieldConstants.RightTrench.width / 2);

  public static final AngularVelocity HUB_SHOT_FLYWHEEL_SPEED = RotationsPerSecond.of(23);
  public static final Rotation2d HUB_SHOT_HOOD_ANGLE = Rotation2d.fromRotations(0);
  public static final Translation2d HUB_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.LinesVertical.starting - Units.inchesToMeters(29.000 / 2),
          FieldConstants.LinesHorizontal.center); // Robot width: 29.000

  public static final AngularVelocity RIGHT_CORNER_SHOT_FLYWHEEL_SPEED = RotationsPerSecond.of(35);
  public static final Rotation2d RIGHT_CORNER_SHOT_HOOD_ANGLE = Rotation2d.fromRotations(0.44);
  public static final Translation2d RIGHT_CORNER_SHOT_LOCATION =
      new Translation2d(Units.inchesToMeters(29.000 / 2), Units.inchesToMeters(29.000 / 2));

  public static final AngularVelocity LEFT_CORNER_SHOT_FLYWHEEL_SPEED = RotationsPerSecond.of(35);
  public static final Rotation2d LEFT_CORNER_SHOT_HOOD_ANGLE = Rotation2d.fromRotations(0.440);
  public static final Translation2d LEFT_CORNER_SHOT_LOCATION =
      new Translation2d(
          Units.inchesToMeters(29.000 / 2),
          FieldConstants.fieldWidth - Units.inchesToMeters(29.000 / 2));

  public static final AngularVelocity TOWER_SHOT_FLYWHEEL_SPEED = RotationsPerSecond.of(26.49);
  public static final Rotation2d TOWER_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(0.267);
  public static final Translation2d TOWER_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.Tower.frontFaceX + Units.inchesToMeters(29.000 / 2),
          FieldConstants.LinesHorizontal.center);

  public static final double HOOD_TUCK_TIME = 0.5; // seconds

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(21)
          .withCanBus(new CANBus())
          .withEnableFOC(true)
          .withLeaderInversion(InvertedValue.Clockwise_Positive)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(60.0))
                  .withStatorCurrentLimit(Amps.of(80.0))
                  .build())
          .withMomentOfInertia(0.05)
          .withGearRatio((28.0 / 26.0))
          .withMotorConfig(DCMotor.getKrakenX60Foc(4))
          .withVoltageGains(
              Gains.builder()
                  .withPrefix("Shooter/Flywheel/Voltage")
                  .withKP(0.5)
                  .withKD(0.0)
                  .withKS(.23467)
                  .withKV(.12959)
                  .withKA(0.0084495)
                  .build())
          .withTorqueGains(
              Gains.builder()
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(15)
                  .withKD(0)
                  .withKS(3.6)
                  .withKV(0.003)
                  .withKA(0)
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(0))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(0))
                  .withGoalTolerance(RadiansPerSecond.of(15))
                  .build())
          .withAlignedFollowerCANID(23)
          .withOpposedFollowerCANID(22)
          .withOpposedFollowerCANID(24)
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
          .withVoltageOffsetStep(Volts.of(0.25))
          .build();

  public static final HoodConstants HOOD_CONSTANTS =
      HoodConstants.builder()
          .withMotorCanId(20)
          .withCurrentLimits(new CurrentLimits(40, 20))
          .withGearRatio((18.0 / 12.0) * (28.0 / 8.0) * (324.0 / 14.0))
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withCanBus(new CANBus())
          .withLengthMeters(0.101596)
          .withMinAngle(Rotation2d.fromDegrees(0.3))
          .withMaxAngle(Rotation2d.fromDegrees(27.5))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(15))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withOffsetStep(Degrees.of(0.5))
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Hood")
                  .withMaxVelocity(RadiansPerSecond.of(400))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(2000))
                  .withGoalTolerance(Degrees.of(1.0))
                  .build())
          .withGains(
              Gains.builder()
                  .withPrefix("Shooter/Hood")
                  .withKP(1400)
                  .withKD(30)
                  .withKS(.42216)
                  .withKV(11.141)
                  .withKA(0.20086)
                  .build())
          .withVoltageStep(Volts.of(0.5))
          .build();

  public static final TurretConstants TURRET_CONSTANTS =
      TurretConstants.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMomentOfInertia(0.004)
          .withTurretCANID(25)
          .withMotorInversion(InvertedValue.CounterClockwise_Positive)
          .withEncoderInversion(SensorDirectionValue.CounterClockwise_Positive)
          .withCanBus(new CANBus())
          .withEncoder1ID(26)
          .withEncoder2ID(27)
          .withMaxAngle(Rotation2d.fromDegrees(615))
          .withMinAngle(Rotation2d.fromDegrees(0))
          .withGearRatio((54.0 / 8.0) * (86.0 / 10.0))
          .withSupplyCurrentLimit(30.0)
          .withStatorCurrentLimit(30.0)
          .withE1Offset(Rotation2d.fromRotations(-.153076171875))
          .withE2Offset(Rotation2d.fromRotations(-.3720703125))
          .withGains(
              Gains.builder()
                  .withPrefix("Turret")
                  .withKP(250)
                  .withKD(2.5)
                  .withKS(0.29684)
                  .withKV(6.7445)
                  .withKA(0.10718)
                  .withKG(0.067749)
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Turret")
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(250))
                  .withMaxVelocity(RadiansPerSecond.of(50))
                  .withGoalTolerance(Degrees.of(5))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(86)
                  .withGear1ToothCount(13)
                  .withGear2ToothCount(14)
                  .build())
          .withRobotToTurretTransform(V2_TurnoverConstants.ROBOT_TO_SHOOTER_TRANSFORM)
          .withVoltageStep(Volts.of(0.5))
          .withAngleStep(Rotation2d.fromDegrees(1.0))
          .withAimingFeedforwardGains(
              Gains.builder()
                  .withPrefix("Turret/Aiming")
                  .withKS(0.5)
                  .withKV(0.1)
                  .withKA(0.025)
                  .build())
          .build();

  public enum ShooterGoal {
    SCORE,
    FEED,
    STOW,
    FIXED_SHOTS,
    OVERRIDE_TURRET,
    OVERRIDE_HOOD,
    OVERRIDE_FLYWHEEL,
    SYSID,
    ZERO,
    IDLE
  }

  @RequiredArgsConstructor
  @Getter
  public enum ShooterState {
    MOVING,
    STATIONARY
  }
}
