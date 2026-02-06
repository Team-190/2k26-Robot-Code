package frc.robot.subsystems.v1_Gamma.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants.CurrentLimits;
import frc.robot.subsystems.shared.hood.HoodConstants;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;

public class V1_GammaShooterConstants {
  public static final GenericFlywheelConstants SHOOTER_FLYWHEEL_CONSTANTS =
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

  public static final HoodConstants SHOOTER_HOOD_CONSTANTS =
      HoodConstants.builder()
          .MOTOR_CAN_ID(32)
          .CAN_LOOP(V1_GammaConstants.DRIVE_CONFIG.canBus())
          .GEAR_RATIO(85.0)
          .CURRENT_LIMIT(40)
          .MOMENT_OF_INERTIA(0.004)
          .MOTOR_CONFIG(DCMotor.getKrakenX60Foc(1))
          .LENGTH_METERS(0.381)
          .MIN_ANGLE(Units.degreesToRadians(0.0))
          .MAX_ANGLE(Units.degreesToRadians(90.0))
          .GAINS(
              new HoodConstants.Gains(
                  new LoggedTunableNumber("Hood/KP", 0.0),
                  new LoggedTunableNumber("Hood/KD", 0.0),
                  new LoggedTunableNumber("Hood/KS", 0.0),
                  new LoggedTunableNumber("Hood/KV", 0.0),
                  new LoggedTunableNumber("Hood/KA", 0.0)))
          .CONSTRAINTS(
              new HoodConstants.Constraints(
                  new LoggedTunableNumber("Hood/Max Velocity", 120.0),
                  new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
                  new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0))))
          .build();
}
