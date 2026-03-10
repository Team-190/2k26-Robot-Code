package frc.robot.subsystems.v2_Delta;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V2_DeltaIntakeIO {

  @AutoLog
  public static class V2_DeltaIntakeIOInputs {
    public double extensionPositionMeters = 0.0;
    public double extensionVelocityMetersPerSecond = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionSupplyCurrentAmps = 0.0;
    public double extensionTorqueCurrentAmps = 0.0;
    public double extensionTemperatureCelsius = 0.0;
    public double extensionGoal = 0.0;
    public double extensionPositionSetpoint = 0.0;
    public double extensionPositionError = 0.0;

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;
  }

  public default void updateInputs(V2_DeltaIntakeIOInputs inputs) {}

  public default void setExtensionVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setExtensionGoal(double position) {}

  public default void stopRoller() {}

  public default boolean atExtensionPositionGoal() {
    return false;
  }

  public default void updateGains(double kP, double kD, double kS, double kV, double kA) {}

  public default void updateConstraints(double maxAcceleration, double maxVelocity) {}

  public default void resetExtension() {}

  public default void maxExt() {}
}
