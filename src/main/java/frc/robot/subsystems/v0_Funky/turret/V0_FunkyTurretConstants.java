package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class V0_FunkyTurretConstants {

  public static final int CAN_ID;
  public static final double MAX_ANGLE;
  public static final double MIN_ANGLE;
  public static final double GEAR_RATIO;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double STATOR_CURRENT_LIMIT;

  static {
    CAN_ID = 1;
    MAX_ANGLE = Math.PI; // Output angle in degress
    MIN_ANGLE = -Math.PI;
    GEAR_RATIO = 5;
    SUPPLY_CURRENT_LIMIT = 30;
    STATOR_CURRENT_LIMIT = 30;

    GAINS =
        new Gains(
            new edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber("Turret/kP", 0),
            new LoggedTunableNumber("Turret/kD", 0),
            new LoggedTunableNumber("Turret/kV", 0),
            new LoggedTunableNumber("Turret/kA", 0));

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
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_ROTATIONS_PER_SECOND,
      LoggedTunableNumber GOAL_TOLERANCE_RADIANS) {}

  private final double voltage = 0;

  public double getVoltage() {
    return voltage;
  }

  private final Rotation2d angle = new Rotation2d();

  public Rotation2d getAngle() {
    return angle;
  }
}
