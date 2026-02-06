package frc.robot.subsystems.v1_Gamma.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;

public class V1_GammaShooterConstants {
  GenericFlywheelConstants SHOOTER_FLYWHEEL_CONSTANTS =
      new GenericFlywheelConstants(
          30,
          1,
          false,
          false,
          40.0,
          0.001642124,
          new GenericFlywheelConstants.Gains(
              new LoggedTunableNumber("Shooter/Flywheel/Ks", 0),
              new LoggedTunableNumber("Shooter/Flywheel/Kv", 0),
              new LoggedTunableNumber("Shooter/Flywheel/Ka", 0),
              new LoggedTunableNumber("Shooter/Flywheel/Kp", 0),
              new LoggedTunableNumber("Shooter/Flywheel/Kd", 0)),
          DCMotor.getKrakenX60Foc(1),
          new int[] {},
          new int[] {31},
          new GenericFlywheelConstants.Constraints(
              new LoggedTunableNumber(
                  "Shooter/Flywheel/MaxAccelerationRotationsPerSecondSquared", 6),
              new LoggedTunableNumber(
                  "Shooter/Flywheel/CruisingVelocityRotationsPerSecondSquared", 4),
              new LoggedTunableNumber("Shooter/Flywheel/GoalToleranceRadians", 0.05)),
          3 / 4,
          InvertedValue.Clockwise_Positive);
}
