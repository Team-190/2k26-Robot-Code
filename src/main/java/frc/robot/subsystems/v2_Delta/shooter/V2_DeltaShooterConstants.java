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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shared.hood.HoodConstants;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretAngleCalculation;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class V2_DeltaShooterConstants {

  public static final AngularVelocity LEFT_TRENCH_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(420.0); // fake
  public static final Rotation2d LEFT_TRENCH_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(20.0); // fake
  public static final Translation2d LEFT_TRENCH_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.LeftTrench.openingTopLeft.getX(),
          FieldConstants.LeftTrench.openingTopLeft.getY() - FieldConstants.LeftTrench.width / 2);

  public static final AngularVelocity RIGHT_TRENCH_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(420.0); // fake
  public static final Rotation2d RIGHT_TRENCH_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(20.0); // fake
  public static final Translation2d RIGHT_TRENCH_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.RightTrench.openingTopLeft.getX(),
          FieldConstants.RightTrench.openingTopLeft.getY() - FieldConstants.RightTrench.width / 2);

  public static final AngularVelocity HUB_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(205);
  public static final Rotation2d HUB_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(0.175781); // fake
  public static final Translation2d HUB_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.LinesVertical.starting - Units.inchesToMeters(29.000 / 2),
          FieldConstants.LinesHorizontal.center); // Robot width: 29.000

  public static final AngularVelocity RIGHT_CORNER_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(308.181632);
  public static final Rotation2d RIGHT_CORNER_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(21.269531); // fake
  public static final Translation2d RIGHT_CORNER_SHOT_LOCATION =
      new Translation2d(Units.inchesToMeters(29.000 / 2), Units.inchesToMeters(29.000 / 2));

  public static final AngularVelocity LEFT_CORNER_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(308.181632);
  public static final Rotation2d LEFT_CORNER_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(21.269531); // fake
  public static final Translation2d LEFT_CORNER_SHOT_LOCATION =
      new Translation2d(
          Units.inchesToMeters(29.000 / 2),
          FieldConstants.fieldWidth - Units.inchesToMeters(29.000 / 2));

  public static final AngularVelocity TOWER_SHOT_FLYWHEEL_SPEED = RadiansPerSecond.of(238.049275);
  public static final Rotation2d TOWER_SHOT_HOOD_ANGLE = Rotation2d.fromDegrees(14.589844);
  public static final Translation2d TOWER_SHOT_LOCATION =
      new Translation2d(
          FieldConstants.Tower.frontFaceX + Units.inchesToMeters(29.000 / 2),
          FieldConstants.LinesHorizontal.center);

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(21)
          .withCanBus(CANBus.roboRIO())
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
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/VoltageKp", .65))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/VoltageKd", 0.0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/VoltageKs", 0.2651))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/VoltageKv", 0.13047))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/VoltageKa", 0.0046038))
                  .build())
          .withTorqueGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/TorqueKp", 6.1402131239))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/TorqueKd", 0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/TorqueKs", 3.3))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/TorqueKv", .01))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/TorqueKa", 0))
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(0)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration", RadiansPerSecondPerSecond.of(0)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/GoalTolerance", RadiansPerSecond.of(10)))
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
          .withCurrentLimits(40.0)
          .withGearRatio((18.0 / 12.0) * (28.0 / 8.0) * (324.0 / 14.0))
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withCanBus(CANBus.roboRIO())
          .withLengthMeters(0.101596)
          .withMinAngle(Rotation2d.fromDegrees(0.3))
          .withMaxAngle(Rotation2d.fromDegrees(30.549609))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(40.0))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withOffsetStep(Degrees.of(0.5))
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxVelocity", RadiansPerSecond.of(400)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxAcceleration", RadiansPerSecondPerSecond.of(2000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Hood/GoalTolerance", Degrees.of(1.0)))
                  .build())
          .withGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Hood/Kp", 1000))
                  .withKD(new LoggedTunableNumber("Shooter/Hood/Kd", 5))
                  .withKS(new LoggedTunableNumber("Shooter/Hood/Ks", 0.13629))
                  .withKV(new LoggedTunableNumber("Shooter/Hood/Kv", 15.786))
                  .withKA(new LoggedTunableNumber("Shooter/Hood/Ka", 0.17393))
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
          .withCanBus(CANBus.roboRIO())
          .withEncoder1ID(26)
          .withEncoder2ID(27)
          .withMaxAngle(Rotation2d.fromDegrees(-48))
          .withMinAngle(Rotation2d.fromDegrees(-650))
          .withGearRatio((54.0 / 8.0) * (86.0 / 10.0))
          .withSupplyCurrentLimit(30.0)
          .withStatorCurrentLimit(30.0)
          .withE1Offset(Rotation2d.fromRotations(-.153076171875))
          .withE2Offset(Rotation2d.fromRotations(-.3720703125))
          .withGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Turret/Kp", 250))
                  .withKD(new LoggedTunableNumber("Turret/Kd", 2.5))
                  .withKS(new LoggedTunableNumber("Turret/Ks", 0.29684))
                  .withKV(new LoggedTunableNumber("Turret/Kv", 6.7445))
                  .withKA(new LoggedTunableNumber("Turret/Ka", 0.10718))
                  .withKG(new LoggedTunableNumber("Turret/Kg", 0.067749))
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Turret/MaxAcceleration", RadiansPerSecondPerSecond.of(250)))
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Turret/MaxVelocity", RadiansPerSecond.of(50)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Turret/GoalTolerance", Degrees.of(3)))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(86)
                  .withGear1ToothCount(13)
                  .withGear2ToothCount(14)
                  .build())
          .withRobotToTurretTransform(V2_DeltaConstants.ROBOT_TO_SHOOTER_TRANSFORM)
          .withVoltageStep(Volts.of(0.5))
          .withAngleStep(Rotation2d.fromDegrees(1.0))
          .withAimingFeedforwardGains(
              Gains.fromDoubles()
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
    IDLE
  }

  @RequiredArgsConstructor
  @Getter
  public enum ShooterState {
    MOVING,
    STATIONARY
  }
}
