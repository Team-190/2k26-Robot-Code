package frc.robot.subsystems.v1_DoomSpiral.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

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
