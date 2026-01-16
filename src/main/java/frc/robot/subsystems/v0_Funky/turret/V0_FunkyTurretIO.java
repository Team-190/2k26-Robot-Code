package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface V0_FunkyTurretIO {

  @AutoLog
  public static class V0_FunkyTurretIOInputs {
    public double turretAngle = 0.0;
    public double turretVelocityRadiansPerSecond = 0.0;
    public double turretAppliedVolts = 0.0;
    public double turretSupplyCurrentAmps = 0.0;
    public double turretTorqueCurrentAmps = 0.0;
    public double turretGoal = 0.0;
    public double turretTemperatureCelsius;
    public double turretPositionSetpoint = 0.0;
    public double turretPositionError = 0.0;
  }

  public default void updateInputs(V0_FunkyTurretIOInputs inputs) {}

  public default void setTurretVoltage(double volts) {}

  public default void setTurretGoal(Pose3d goal) {}

  public default void stopTurret() {}

  public default boolean atTurretPositionGoal() {
    return false;
  }

  public default void updateGains(double kP, double kD, double kV, double kA) {}

  public default void updateConstraints(
      double maxAcceleration, double maxVelocity, double goalTolerance) {}

  public default void resetTurret() {}
}
