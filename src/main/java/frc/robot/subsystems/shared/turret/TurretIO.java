package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.littletonrobotics.junction.AutoLog;

/** Interface for V0 (Funky)'s turret subsystem */
public interface TurretIO {

  /** Inputs for Funky's turret subsystem */
  @AutoLog
  class TurretIOInputs {
    public Rotation2d angle = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();

    public Rotation2d encoder1Position = new Rotation2d();
    public Rotation2d encoder2Position = new Rotation2d();
  }

  /**
   * Updates AdvantageKit inputs.
   *
   * @param inputs The TurretIOInputs object to be populated.
   */
  default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Sets the turret motor voltage.
   *
   * @param volts The voltage to set the turret motor to.
   */
  default void setVoltageGoal(Voltage volts) {}

  /**
   * Sets the turret goal.
   *
   * @param goal The turret goal as a Pose3d type.
   */
  default void setPositionGoal(Rotation2d goal) {}

  /** Checks whether the turret is within the tolerance of its goal and returns a boolean. */
  default boolean atPositionGoal(Rotation2d positionReference) {
    return false;
  }

  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }

  default void updateGains(Gains gains) {}

  default void updateConstraints(AngularPositionConstraints constraints) {}

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
