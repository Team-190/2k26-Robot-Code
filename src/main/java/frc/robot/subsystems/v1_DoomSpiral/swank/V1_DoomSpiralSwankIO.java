package frc.robot.subsystems.v1_DoomSpiral.swank;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface V1_DoomSpiralSwankIO {

  @AutoLog
  public static class V1_DoomSpiralSwankIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
  }

  /**
   * Updates inputs.
   *
   * <p>This method should be called at the beginning of the periodic iteration. It is responsible
   * for updating the values of the inputs object with the current state of the subsystem.
   *
   * @param inputs The inputs object to be updated.
   */
  public default void updateInputs(V1_DoomSpiralSwankIOInputs inputs) {}

  /**
   * Sets the voltage being passed into the subsystem.
   *
   * @param volts The voltage to be passed into the subsystem.
   */
  public default void setVoltage(double volts) {}
}
