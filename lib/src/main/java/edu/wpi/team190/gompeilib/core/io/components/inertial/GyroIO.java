package edu.wpi.team190.gompeilib.core.io.components.inertial;

import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import java.util.Queue;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVelocityRadPerSec = 0.0;

    public Rotation2d pitchPosition = new Rotation2d();
    public AngularVelocity pitchVelocity = RadiansPerSecond.zero();

    public Rotation2d rollPosition = new Rotation2d();
    public AngularVelocity rollVelocity = RadiansPerSecond.zero();
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void updateInputs(
      GyroIOInputs inputs, Queue<Double> yawTimestampQueue, Queue<Double> yawPositionQueue) {}

  public default StatusSignal<Angle> getYaw() {
    return null;
  }

  public default StatusSignal<Angle> getRoll() {
    return null;
  }

  public default StatusSignal<Angle> getPitch() {
    return null;
  }
}
