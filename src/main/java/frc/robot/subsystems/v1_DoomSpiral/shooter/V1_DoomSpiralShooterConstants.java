package frc.robot.subsystems.v1_DoomSpiral.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Constraints;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.Gains;
import frc.robot.subsystems.shared.hood.HoodConstants;

public class V1_DoomSpiralShooterConstants {
  public static final GenericFlywheelConstants FLYWHEEL_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(30)
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCanBus(CANBus.roboRIO())
          .withEnableFOC(true)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(40.0)
                  .withStatorCurrentLimit(60.0)
                  .build())
          .withMomentOfInertia(0.001655755892)
          .withGearRatio(1.3)
          .withMotorConfig(DCMotor.getKrakenX60Foc(2))
          .withGains(
              Gains.builder()
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/Gains/Kp", 0.0))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/Gains/Kd", 0.0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/Gains/Ks", 0.0))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/Kv", 0.0))
                  .build())
          .withConstraints(
              Constraints.builder()
                  .withCruisingVelocityRadiansPerSecond(
                      new LoggedTunableNumber("Shooter/Flywheel/Constraints/Max Velocity", 0.0))
                  .withMaxAccelerationRadiansPerSecondSquared(
                      new LoggedTunableNumber("Shooter/Flywheel/Constraints/Max Acceleration", 0.0))
                  .withGoalToleranceRadiansPerSecond(
                      new LoggedTunableNumber("Shooter/Flywheel/Constraints/Goal Tolerance", 0.0))
                  .build())
          .withOpposedFollowerCANID(31)
          .build();

  public static final HoodConstants HOOD_CONSTANTS =
      HoodConstants.builder()
          .withCanID(32)
          .withCanBus(CANBus.roboRIO())
          .withGearRatio((36.0 / 12.0) * (24.0 / 18.0) * (296.0 / 14.0))
          .withSupplyCurrentLimit(40.0)
          .withMomentOfInertia(0.00534705076)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withLengthMeters(Units.inchesToMeters(6.675537))
          .withMinAngle(0.0)
          .withMaxAngle(Units.degreesToRadians(20.0))
          .withGains(
              HoodConstants.Gains.builder()
                  .withKp(new LoggedTunableNumber("Shooter/Hood/Gains/Kp"))
                  .withKd(new LoggedTunableNumber("Shooter/Hood/Gains/Kd"))
                  .withKs(new LoggedTunableNumber("Shooter/Hood/Gains/Ks"))
                  .withKv(new LoggedTunableNumber("Shooter/Hood/Gains/Kv"))
                  .build())
          .withConstraints(
              HoodConstants.Constraints.builder()
                  .withMaxVelocityRadiansPerSecond(
                      new LoggedTunableNumber("Shooter/Hood/Constraints/Max Velocity"))
                  .withMaxAccelerationRadiansPerSecondSqaured(
                      new LoggedTunableNumber("Shooter/Hood/Constraints/Max Acceleration"))
                  .withGoalToleranceRadians(
                      new LoggedTunableNumber("Shooter/Hood/Constraints/Goal Tolerance"))
                  .build())
          .build();
}
