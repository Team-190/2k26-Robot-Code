package frc.robot.subsystems.shared.fourbarlinkage;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.littletonrobotics.junction.AutoLog;

public interface FourBarLinkageIO {
  @AutoLog
  class FourBarLinkageIOInputs {

    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();

    public Rotation2d canCoderAbsolutePosition = new Rotation2d();
  }

  /** Updates AdvantageKit inputs. */
  default void updateInputs(FourBarLinkageIOInputs inputs) {}

  /** Sets motor voltage. */
  default void setVoltageGoal(Voltage volts) {}

  /** Sets motor closed loop position. */
  default void setPosition(Rotation2d position) {}

  /** Sets motor closed loop position setpoint. */
  default void setPositionGoal(Rotation2d position) {}

  /** Sets motor PID gains. */
  default void setGains(Gains gains) {}

  /** Sets motor profile. */
  default void setProfile(AngularPositionConstraints constraints) {}

  /** Checks if the linkage is at the goal position. */
  default boolean atPositionGoal(Rotation2d positionReference) {
    return false;
  }

  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }
}
