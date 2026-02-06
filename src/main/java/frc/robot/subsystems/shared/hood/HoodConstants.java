package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import frc.robot.subsystems.v1_Gamma.V1_GammaConstants;
import lombok.Builder;

@Builder
public class HoodConstants {
  public final int MOTOR_CAN_ID;
  @Builder.Default public final CANBus CAN_LOOP = new CANBus();

  public final double GEAR_RATIO;
  public final double CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final DCMotor MOTOR_CONFIG;
  public final double LENGTH_METERS;
  public final double MIN_ANGLE;
  public final double MAX_ANGLE;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  public static final HoodConstants V1_GAMMA_HOOD_CONSTANTS;

  static {
    V1_GAMMA_HOOD_CONSTANTS =
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
                new Gains(
                    new LoggedTunableNumber("Hood/KP", 0.0),
                    new LoggedTunableNumber("Hood/KD", 0.0),
                    new LoggedTunableNumber("Hood/KS", 0.0),
                    new LoggedTunableNumber("Hood/KV", 0.0),
                    new LoggedTunableNumber("Hood/KA", 0.0)))
            .CONSTRAINTS(
                new Constraints(
                    new LoggedTunableNumber("Hood/Max Velocity", 120.0),
                    new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
                    new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0))))
            .build();
  }

  public enum HoodGoal {
    SCORE,
    FEED,
    STOW
  }

  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber kd,
      LoggedTunableNumber ks,
      LoggedTunableNumber kv,
      LoggedTunableNumber ka) {}

  public record Constraints(
      LoggedTunableNumber maxVelocityRadiansPerSecond,
      LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      LoggedTunableNumber goalToleranceRadians) {}
}
