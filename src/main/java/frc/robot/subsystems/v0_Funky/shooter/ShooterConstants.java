package frc.robot.subsystems.v0_Funky.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretAngleCalculation;

public class ShooterConstants {
  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withCanBus(CANBus.roboRIO())
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(40.0))
                  .withStatorCurrentLimit(Amps.of(80))
                  .build())
          .withMomentOfInertia(0.05)
          .withGearRatio(1.0)
          .withMotorConfig(DCMotor.getKrakenX60Foc(2))
          .withVoltageGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/Voltage/Kp", 0))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/Voltage/Kd", 0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/Voltage/Ks", 0))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/Voltage/Kv", 0))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/Voltage/Ka", 0))
                  .build())
          .withTorqueGains(
              Gains.fromDoubles()
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.2)
                  .withKV(0.12)
                  .withKA(0.0)
                  .withKG(0.0)
                  .build())
          .withConstraints(
              AngularConstraints.builder()
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration", RadiansPerSecondPerSecond.of(6)))
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(4)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Flywheel/GoalTolerance", Radians.of(5)))
                  .build())
          .withOpposedFollowerCANID(31)
          .withEnableFOC(false)
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
                  .withKP(new LoggedTunableNumber("Turret/Kp", 28.624920))
                  .withKD(new LoggedTunableNumber("Turret/Kd", 0.5))
                  .withKS(new LoggedTunableNumber("Turret/Ks", 0.158040))
                  .withKV(new LoggedTunableNumber("Turret/Kv", 0.11377))
                  .withKA(new LoggedTunableNumber("Turret/Ka", 0.0031713))
                  .build())
          .withConstraints(
              AngularConstraints.builder()
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration",
                          RadiansPerSecondPerSecond.of(35.566371)))
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(89.566371)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Flywheel/GoalTolerance", Radians.of(3)))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(120)
                  .withGear1ToothCount(16)
                  .withGear2ToothCount(17)
                  .build())
          .build();
}
