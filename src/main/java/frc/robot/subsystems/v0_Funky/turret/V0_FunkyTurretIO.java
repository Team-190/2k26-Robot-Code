package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for V0 (Funky)'s turret subsystem */
public interface V0_FunkyTurretIO {

  /** Inputs for Funky's turret subsystem */
  @AutoLog
  class V0_FunkyTurretIOInputs {
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
  default void updateInputs(V0_FunkyTurretIOInputs inputs) {}

  /** Sets the turret motor voltage. */
  default void setTurretVoltage(double volts) {}

  /**
   * Sets the turret goal.
   *
   * @param goal The turret goal as a Pose3d type.
   */
  default void setTurretGoal(Rotation2d goal) {}

  /** Checks whether the turret is within the tolerance of its goal and returns a boolean. */
  default boolean atTurretPositionGoal() {
    return false;
  }

  default void updateGains(double kP, double kD, double kS, double kV, double kA) {}

  default void updateConstraints(
      double maxAcceleration, double maxVelocity, double goalTolerance) {}

  /**
   * Sets the turret's current position to a value.
   *
   * @param position the position is a rotation2d object of the turret's angle on the 2d plane.
   */
  default void setPosition(Rotation2d position) {}

  default double clampAngle(double targetAngle) {

    while (targetAngle < V0_FunkyTurretConstants.MIN_ANGLE) {
      targetAngle += 2 * Math.PI;
    }
    while (targetAngle > V0_FunkyTurretConstants.MAX_ANGLE) {
      targetAngle -= 2 * Math.PI;
    }

    targetAngle =
        MathUtil.clamp(
            targetAngle, V0_FunkyTurretConstants.MIN_ANGLE, V0_FunkyTurretConstants.MAX_ANGLE);
    return targetAngle;
  }
}
