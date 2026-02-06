package frc.robot.subsystems.shared.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import frc.robot.subsystems.v0_Funky.V0_FunkyConstants;
import lombok.Builder;

@Builder
public class TurretConstants {

  public final int TURRET_CAN_ID;
  public final int LEFT_ENCODER_ID;
  public final int RIGHT_ENCODER_ID;
  public final double MAX_ANGLE;
  public final double MIN_ANGLE;
  public final double GEAR_RATIO;
  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  public final double E1_OFFSET_RADIANS;
  public final double E2_OFFSET_RADIANS;

  public final double SUPPLY_CURRENT_LIMIT;
  public final double STATOR_CURRENT_LIMIT;
  public final DCMotor MOTOR_CONFIG;
  public final double MOMENT_OF_INERTIA;
  public final TurretAngleCalculation TURRET_ANGLE_CALCULATION;

  @Builder.Default public final CANBus CAN_LOOP = new CANBus();

  public static final TurretConstants V0_FUNKY_TURRET_CONSTANTS;

  static {
    V0_FUNKY_TURRET_CONSTANTS =
        new TurretConstantsBuilder()
            .MOTOR_CONFIG(DCMotor.getKrakenX60Foc(1))
            .MOMENT_OF_INERTIA(0.004)
            .TURRET_CAN_ID(1)
            .CAN_LOOP(V0_FunkyConstants.DRIVE_CONFIG.canBus())
            .LEFT_ENCODER_ID(2)
            .RIGHT_ENCODER_ID(3)
            .MAX_ANGLE(2 * Math.PI)
            .MIN_ANGLE(-2 * Math.PI)
            .GEAR_RATIO(5)
            .SUPPLY_CURRENT_LIMIT(30)
            .STATOR_CURRENT_LIMIT(30)
            .E1_OFFSET_RADIANS(0)
            .E2_OFFSET_RADIANS(0)
            .GAINS(
                new Gains(
                    new edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber("Turret/kP", 0),
                    new LoggedTunableNumber("Turret/kD", 0),
                    new LoggedTunableNumber("Turret/kV", 0),
                    new LoggedTunableNumber("Turret/kA", 0),
                    new LoggedTunableNumber("Turret/kS", 0)))
            .CONSTRAINTS(
                new Constraints(
                    new LoggedTunableNumber("Turret/Max Acceleration", 0),
                    new LoggedTunableNumber("Turret/Cruising Velocity", 0),
                    new LoggedTunableNumber("Turret/Goal Tolerance", 0)))
            .TURRET_ANGLE_CALCULATION(new TurretAngleCalculation(70, 38, 36))
            .build();
  }

  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA,
      LoggedTunableNumber kS) {}

  public record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_RADIANS_PER_SECOND,
      LoggedTunableNumber GOAL_TOLERANCE_RADIANS) {}

  public record TurretAngleCalculation(
      double GEAR_0_TOOTH_COUNT, double GEAR_1_TOOTH_COUNT, double GEAR_2_TOOTH_COUNT) {

    public double GEAR_1_RATIO() {
      return GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT;
    }

    public double GEAR_2_RATIO() {
      return GEAR_0_TOOTH_COUNT / GEAR_2_TOOTH_COUNT;
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
