package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;

public class V0_FunkyTurretConstants {

  public static final int CAN_ID;
  public static final double MAX_ANGLE;
  public static final double MIN_ANGLE;
  public static final double GEAR_RATIO;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  public static final double E1_OFFSET_RADIANS;
  public static final double E2_OFFSET_RADIANS;

  @Getter @Setter public static double CURRENT_ANGLE;
  @Getter @Setter public static double VOLTAGE;

  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double STATOR_CURRENT_LIMIT;
  public static final DCMotor MOTOR_CONFIG;
  public static final double MOMENT_OF_INERTIA;

  static {
    CAN_ID = 1;
    MAX_ANGLE = Math.PI; // Output angle in degress
    MIN_ANGLE = -Math.PI;
    GEAR_RATIO = 5;
    SUPPLY_CURRENT_LIMIT = 30;
    STATOR_CURRENT_LIMIT = 30;
    CURRENT_ANGLE = 0;
    VOLTAGE = 0;
    E1_OFFSET_RADIANS = 0;
    E2_OFFSET_RADIANS = 0;
    MOTOR_CONFIG = null;
    MOMENT_OF_INERTIA = 0;

    GAINS =
        new Gains(
            new edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber("Turret/kP", 0),
            new LoggedTunableNumber("Turret/kD", 0),
            new LoggedTunableNumber("Turret/kV", 0),
            new LoggedTunableNumber("Turret/kA", 0),
            new LoggedTunableNumber("Turret/kS", 0));

    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Turret/Max Acceleration", 0),
            new LoggedTunableNumber("Turret/Cruising Velocity", 0),
            new LoggedTunableNumber("Turrent/Goal Tolerance", 0));
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA,
      LoggedTunableNumber kS) {}

  public static record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_RADIANS_PER_SECOND,
      LoggedTunableNumber GOAL_TOLERANCE_RADIANS) {}
}
