package frc.robot.subsystems.v0_Funky.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotState;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;

public class V0_FunkyHoodConstants {
  public static final int MOTOR_CAN_ID;
  public static final boolean IS_CAN_FD;

  public static final double GEAR_RATIO;
  public static final double CURRENT_LIMIT;
  public static final double MOMENT_OF_INERTIA;
  public static final DCMotor MOTOR_CONFIG;
  public static final double LENGTH_METERS;
  public static final double MIN_ANGLE;
  public static final double MAX_ANGLE;

  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    MOTOR_CAN_ID = 7; //TODO: IDEK what the correct ids are
    IS_CAN_FD = true;

    GEAR_RATIO = 85.0;
    CURRENT_LIMIT = 40.0;
    MOMENT_OF_INERTIA = 0.004;
    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    LENGTH_METERS = 0.381;
    MIN_ANGLE = Units.degreesToRadians(0.0);
    MAX_ANGLE = Units.degreesToRadians(90.0);

    GAINS =
        new Gains(
            new LoggedTunableNumber("Hood/KP", 0.0),
            new LoggedTunableNumber("Hood/KD", 0.0),
            new LoggedTunableNumber("Hood/KS", 0.0),
            new LoggedTunableNumber("Hood/KV", 0.0),
            new LoggedTunableNumber("Hood/KA", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Hood/Max Velocity", 120.0),
            new LoggedTunableNumber("Hood/Max Acceleration", 120.0),
            new LoggedTunableNumber("Hood/Goal Tolerance", Units.degreesToRadians(1.0)));
  }

  @RequiredArgsConstructor
  public enum HoodGoal {
    STOW(() -> new Rotation2d()),
    SCORE(V0_FunkyRobotState::getScoreAngle),
    FEED(V0_FunkyRobotState::getFeedAngle);

    private final Supplier<Rotation2d> angle;

    public Rotation2d getAngle() {
      return angle.get();
    }
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
