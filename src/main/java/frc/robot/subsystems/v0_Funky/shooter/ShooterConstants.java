package frc.robot.subsystems.v0_Funky.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretAngleCalculation;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretConstraints;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretGains;

public class ShooterConstants {
  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCurrentLimit(new CurrentLimits(60.0, 40.0))
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

  public static final TurretConstants TURRET_CONSTANTS =
      TurretConstants.builder()
          .withMotorConfig(DCMotor.getKrakenX60Foc(1))
          .withMomentOfInertia(0.004)
          .withTurretCANID(2)
          .withMotorInversion(InvertedValue.CounterClockwise_Positive)
          .withEncoderInversion(SensorDirectionValue.Clockwise_Positive)
          .withCanBus(new CANBus())
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
              TurretGains.builder()
                  .withKP(new LoggedTunableNumber("Turret/Kp", 28.624920))
                  .withKD(new LoggedTunableNumber("Turret/Kd", 0.5))
                  .withKS(new LoggedTunableNumber("Turret/Ks", 0.158040))
                  .withKV(new LoggedTunableNumber("Turret/Kv", 0.11377))
                  .withKA(new LoggedTunableNumber("Turret/Ka", 0.0031713))
                  .build())
          .withConstraints(
              TurretConstraints.builder()
                  .withCruisingVelocityRadiansPerSecond(
                      new LoggedTunableNumber("Turret/CruisingVelocityRadiansPerSecond", 35.566371))
                  .withMaxAccelerationRadiansPerSecondSquared(
                      new LoggedTunableNumber(
                          "Turret/MaxAccelerationRadiansPerSecondSquared", 89.566371))
                  .withGoalToleranceRadians(
                      new LoggedTunableNumber(
                          "Turret/GoalToleranceRadians", Units.degreesToRadians(3)))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(120)
                  .withGear1ToothCount(16)
                  .withGear2ToothCount(17)
                  .build())
          .build();
}
