package frc.robot.subsystems.v0_Funky.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Funky's hood subsystem. */
public interface V0_FunkyHoodIO {

  /**
   * Inputs for Funky's hood subsystem. Positions and velocities are of the output shaft, not the
   * motor shaft.
   */
  @AutoLog
  public static class V0_FunkyHoodIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(V0_FunkyHoodIOInputs inputs) {}

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {}

  /** Sets motor closed loop position setpoint. */
  public default void setPosition(Rotation2d position) {}

  public default void setPID(double kp, double ki, double kd) {}

  public default void setFeedforward(double ks, double kv, double ka) {}

  public default void setProfile(
      double maxVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {}
  /** Checks if the hood is within tolerance */
  public default boolean atGoal() {
    return false;
  }
}
