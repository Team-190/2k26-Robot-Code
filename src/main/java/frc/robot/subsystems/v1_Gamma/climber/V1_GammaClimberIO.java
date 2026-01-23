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

  public default void updateInputs(V1_GammaClimberIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default boolean isClimbedL1() {
    return false;
  }

  public default boolean isClimbedL2() {
    return false;
  }
}
