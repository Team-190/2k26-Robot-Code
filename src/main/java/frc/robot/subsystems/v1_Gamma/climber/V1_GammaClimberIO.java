package frc.robot.subsystems.v1_Gamma.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface V1_GammaClimberIO {

  @AutoLog
  public static class V1_GammaClimberIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
  }

  /**
   * Updates the inputs for the V1_GammaClimberIO interface.
   * 
   * This method is intended to be called periodically by the robot code to update the
   * inputs for the V1_GammaClimberIO interface.
   * 
   * @param inputs the inputs to update
   */
  public default void updateInputs(V1_GammaClimberIOInputs inputs) {
    
  }


  /**
   * Sets the voltage to the climber motor
   * @param volts the voltage to set, in volts
   */
  public default void setVoltage(double volts) {
    
  }


/**
 * Returns whether the climber is currently at the first
 * climb position
 *
 * @return whether the climber is currently at the first
 *         climb position
 */
  public default boolean isClimbedL1() {
    return false;
  }


/**
 * Returns whether the climber is currently at the second
 * climb position
 *
 * @return whether the climber is currently at the second
 *         climb position
 */
  public default boolean isClimbedL2() {
    return false;
  }
}
