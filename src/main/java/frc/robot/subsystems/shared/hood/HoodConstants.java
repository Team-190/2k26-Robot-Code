package frc.robot.subsystems.shared.hood;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class HoodConstants {
  @NonNull public final Integer motorCanId;
  @NonNull public final CANBus canBus;

  @NonNull public final Double gearRatio;
  @NonNull public final Double currentLimits;
  @NonNull public final Double momentOfInertia;
  @NonNull public final DCMotor motorConfig;
  @NonNull public final Double lengthMeters;
  @NonNull public final Rotation2d minAngle;
  @NonNull public final Rotation2d maxAngle;

  @NonNull public final Voltage zeroVoltage;
  @NonNull public final Current zeroCurrentThreshold;
  @NonNull public final Current zeroCurrentEpsilon;

  @NonNull public final Gains gains;
  @NonNull public final Constraints constraints;

  public enum HoodGoal {
    SCORE,
    FEED,
    STOW,
    OVERRIDE
  }

  @Builder(setterPrefix = "with")
  public record Gains(
      @NonNull LoggedTunableNumber kp,
      @NonNull LoggedTunableNumber kd,
      @NonNull LoggedTunableNumber ks,
      @NonNull LoggedTunableNumber kv,
      @NonNull LoggedTunableNumber ka) {}

  @Builder(setterPrefix = "with")
  public record Constraints(
      @NonNull LoggedTunableNumber maxVelocityRadiansPerSecond,
      @NonNull LoggedTunableNumber maxAccelerationRadiansPerSecondSqaured,
      @NonNull LoggedTunableNumber goalToleranceRadians) {}
}
