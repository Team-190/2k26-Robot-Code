package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for V0 (Funky)'s turret subsystem */
public interface V0_FunkyTurretIO {

/** 
 * Inputs for Funky's turret subsystem
 */
  @AutoLog
  public static class V0_FunkyTurretIOInputs {
    public Rotation2d turretAngle = new Rotation2d(0.0);
    public double turretVelocityRadiansPerSecond = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretGoal = 0.0;
    public double turretTemperatureCelsius;
    public double turretPositionSetpoint = 0.0;
    public double turretPositionError = 0.0;
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(V0_FunkyTurretIOInputs inputs) {}

  /** Sets the turret motor voltage. */
  public default void setTurretVoltage(double volts) {}

  /** 
   * Sets the turret goal. 
   * @param goal The turret goal as a Pose3d type. */
  public default void setTurretGoal(Pose3d goal) {}

  /** Sets the turret goal based on a goal position. 
   * @param goal The goal is a double type in radians.
  */
  public default void setTurretGoal(double goal) {}
  
  /** Stops the turret; i.e. sets the turret's voltage to 0. */
  public default void stopTurret() {}
  
  /** Checks whether the turret is within the tolerance of its goal and returns a boolean. */
  public default boolean atTurretPositionGoal() {
    return false;
  }

  public default void updateGains(double kP, double kD, double kV, double kA) {}

  public default void updateConstraints(
      double maxAcceleration, double maxVelocity, double goalTolerance) {}

  /** Sets the turret's current position to a value.
   * 
   * @param angle The angle is type double and is in radians.
   */
  public default void setPosition(double angle) {}

  /** Sets the turret's current position to 0. */
  public default void resetTurret() {}

  /**Sets the turret's goal to 0. */
  public default void goToZero() {}
}
