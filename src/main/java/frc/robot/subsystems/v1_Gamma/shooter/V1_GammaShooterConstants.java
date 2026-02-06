package frc.robot.subsystems.v1_Gamma.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;

public class V1_GammaShooterConstants {
  GenericFlywheelConstants SHOOTER_FLYWHEEL_CONSTANTS =
      GenericFlywheelConstants.builder()
          .LEADER_CAN_ID(30)
          .LEADER_INVERSION(InvertedValue.Clockwise_Positive)
          .ENABLE_FOC(true)
          .CURRENT_LIMIT(new CurrentLimits(40.0, 40.0))
          .MOMENT_OF_INERTIA(0.001642124)
          .GEAR_RATIO(0.75)
          .MOTOR_CONFIG(DCMotor.getKrakenX60Foc(2))
          .GAINS(
              new GenericFlywheelConstants.Gains(
                  new LoggedTunableNumber("Shooter/Flywheel/Ks", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kv", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Ka", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kp", 0),
                  new LoggedTunableNumber("Shooter/Flywheel/Kd", 0)))
          .CONSTRAINTS(
              new GenericFlywheelConstants.Constraints(
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/MaxAccelerationRotationsPerSecondSquared", 6),
                  new LoggedTunableNumber(
                      "Shooter/Flywheel/CruisingVelocityRotationsPerSecondSquared", 4),
                  new LoggedTunableNumber("Shooter/Flywheel/GoalToleranceRadians", 0.05)))
          .OPPOSED_FOLLOWER_CAN_ID(31)
          .build();
}
