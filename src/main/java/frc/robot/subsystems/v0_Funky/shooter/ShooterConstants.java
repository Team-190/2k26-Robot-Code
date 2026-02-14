package frc.robot.subsystems.v0_Funky.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import frc.robot.subsystems.shared.turret.TurretConstants;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretAngleCalculation;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretConstraints;
import frc.robot.subsystems.shared.turret.TurretConstants.TurretGains;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;

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
          .withTurretCANID(1)
          .withCanBus(V0_FunkyConstants.DRIVE_CONFIG.canBus())
          .withLeftEncoderID(2)
          .withRightEncoderID(3)
          .withMaxAngle(2 * Math.PI)
          .withMinAngle(-2 * Math.PI)
          .withGearRatio(5.0)
          .withSupplyCurrentLimit(30.0)
          .withStatorCurrentLimit(30.0)
          .withE1Offset(Rotation2d.kZero)
          .withE2Offset(Rotation2d.kZero)
          .withGains(
              TurretGains.builder()
                  .withKP(new LoggedTunableNumber("Turret/Kp", 0))
                  .withKD(new LoggedTunableNumber("Turret/Kd", 0))
                  .withKS(new LoggedTunableNumber("Turret/Ks", 0))
                  .withKV(new LoggedTunableNumber("Turret/Kv", 0))
                  .withKA(new LoggedTunableNumber("Turret/Ka", 0))
                  .build())
          .withConstraints(
              TurretConstraints.builder()
                  .withCruisingVelocityRadiansPerSecond(
                      new LoggedTunableNumber("Turret/CruisingVelocityRadiansPerSecond", 0))
                  .withMaxAccelerationRadiansPerSecondSquared(
                      new LoggedTunableNumber("Turret/MaxAccelerationRadiansPerSecondSquared", 0))
                  .withGoalToleranceRadians(
                      new LoggedTunableNumber("Turret/GoalToleranceRadians", 0))
                  .build())
          .withTurretAngleCalculation(
              TurretAngleCalculation.builder()
                  .withGear0ToothCount(70.0)
                  .withGear1ToothCount(38.0)
                  .withGear2ToothCount(36.0)
                  .build())
          .build();
}
