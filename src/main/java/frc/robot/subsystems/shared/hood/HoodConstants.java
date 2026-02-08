package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
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

  public enum HoodGoal {
    SCORE,
    FEED,
    STOW
  }

  @Builder(setterPrefix = "with")
  public record Gains(
      LoggedTunableNumber kp,
      LoggedTunableNumber kd,
      LoggedTunableNumber ks,
      LoggedTunableNumber kv,
      LoggedTunableNumber ka) {}

  @Builder(setterPrefix = "with")
  public record Constraints(
      LoggedTunableNumber maxVelocityRadiansPerSecond,
      LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      LoggedTunableNumber goalToleranceRadians) {}
}
