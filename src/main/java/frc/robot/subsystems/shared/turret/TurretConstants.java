package frc.robot.subsystems.shared.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class TurretConstants {

  @NonNull public final Integer turretCANID;
  @NonNull public final Integer leftEncoderID;
  @NonNull public final Integer rightEncoderID;
  @NonNull public final Double maxAngle;
  @NonNull public final Double minAngle;
  @NonNull public final Double gearRatio;
  @NonNull public final TurretGains gains;
  @NonNull public final TurretConstraints constraints;

  @NonNull public final Rotation2d e1Offset;
  @NonNull public final Rotation2d e2Offset;

  @NonNull public final Double supplyCurrentLimit;
  @NonNull public final Double statorCurrentLimit;
  @NonNull public final DCMotor motorConfig;
  @NonNull public final Double momentOfInertia;
  @NonNull public final TurretAngleCalculation turretAngleCalculation;

  @NonNull public final CANBus canBus;

  @Builder(setterPrefix = "with")
  public record TurretGains(
      @NonNull LoggedTunableNumber kP,
      @NonNull LoggedTunableNumber kD,
      @NonNull LoggedTunableNumber kV,
      @NonNull LoggedTunableNumber kA,
      @NonNull LoggedTunableNumber kS) {}

  @Builder(setterPrefix = "with")
  public record TurretConstraints(
      @NonNull LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      @NonNull LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      @NonNull LoggedTunableNumber goalToleranceRadians) {}

  @Builder(setterPrefix = "with")
  public record TurretAngleCalculation(
      int gear0ToothCount, int gear1ToothCount, int gear2ToothCount) {

    public double GEAR_1_RATIO() {
      return gear0ToothCount / gear1ToothCount;
    }

    public double GEAR_2_RATIO() {
      return gear0ToothCount / gear2ToothCount;
    }

    /**
     * Calculates the difference between the gear 1 ratio and gear 2 ratio.
     *
     * @return the difference between the gear 1 ratio and gear 2 ratio.
     */
    public double GEAR_RATIO_DIFFERENCE() {
      return GEAR_1_RATIO() - GEAR_2_RATIO();
    }
  }
}
