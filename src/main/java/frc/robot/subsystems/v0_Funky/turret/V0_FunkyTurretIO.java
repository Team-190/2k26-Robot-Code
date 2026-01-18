package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for V0 (Funky)'s turret subsystem */
public interface V0_FunkyTurretIO {

  /** Inputs for Funky's turret subsystem */
  @AutoLog
  public static class V0_FunkyTurretIOInputs {
    public Rotation2d turretAngle = new Rotation2d();
    public double turretVelocityRadiansPerSecond = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretTemperatureCelsius = 0.0;
    public Rotation2d turretGoal = new Rotation2d();
    public Rotation2d turretPositionSetpoint = new Rotation2d();
    public Rotation2d turretPositionError = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(V0_FunkyTurretIOInputs inputs) {}

  /** Sets the turret motor voltage. */
  public default void setTurretVoltage(double volts) {}

  /**
   * Sets the turret goal.
   *
   * @param goal The turret goal as a Pose3d type.
   */
  public default void setTurretGoal(Rotation2d goal) {}

  /** Checks whether the turret is within the tolerance of its goal and returns a boolean. */
  public default boolean atTurretPositionGoal() {
    return false;
  }

  public default void updateGains(double kP, double kD, double kS, double kV, double kA) {}

  public default void updateConstraints(
      double maxAcceleration, double maxVelocity, double goalTolerance) {}

  /**
   * Sets the turret's current position to a value.
   *
   * @param angle The angle is type double and is in radians.
   */
  public default void setPosition(Rotation2d position) {}

  /** Performs end-of-travel unwrapping depending on which way the turret is moving. */
  public default void checkDirectionalMotion() {}
}
