package first.robot.v1_DoomSpiral.subsystems.spindexer;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/** Interface for DoomSpiral's Spindexer subsystem. */
public interface V1_DoomSpiralSpindexerIO {

  /** Inputs for DoomSpiral's Spindexer subsystem. */
  @AutoLog
  public static class V1_DoomSpiralSpindexerIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
  }

  /** Updates AdvantageKit inputs. */
  public default void updateInputs(V1_DoomSpiralSpindexerIOInputs inputs) {}

  /** Sets motor voltage. */
  public default void setVoltage(double volts) {}
}
