package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;

@Builder(setterPrefix = "with")
public class HoodConstants {
  public final int canID;
  @Builder.Default public final CANBus canBus = new CANBus();

  public final double gearRatio;
  public final double supplyCurrentLimit;
  public final double momentOfInertia;
  public final DCMotor motorConfig;
  public final double lengthMeters;
  public final double minAngle;
  public final double maxAngle;

  public final Gains gains;
  public final Constraints constraints;

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
