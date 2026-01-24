package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
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

    public Rotation2d encoder1Position = new Rotation2d();
    public Rotation2d encoder2Position = new Rotation2d();
  }

  /**
   * Updates AdvantageKit inputs.
   *
   * @param inputs The V0_FunkyTurretIOInputs object to be populated.
   */
  default void updateInputs(V0_FunkyTurretIOInputs inputs) {}

  /**
   * Sets the turret motor voltage.
   *
   * @param volts The voltage to set the turret motor to.
   */
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

  /**
   * Gets the position of encoder 1.
   *
   * @return position of encoder 1.
   */
  default Angle getEncoder1Position() {
    return Angle.ofBaseUnits(0, Radians);
  }

  /**
   * Gets the position of encoder 2.
   *
   * @return position of encoder 2.
   */
  default Angle getEncoder2Position() {
    return Angle.ofBaseUnits(0, Radians);
  }
}
