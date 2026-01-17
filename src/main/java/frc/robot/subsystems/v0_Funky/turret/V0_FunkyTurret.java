package frc.robot.subsystems.v0_Funky.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurretIO.V0_FunkyTurretIOInputs;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyTurret {
  private final V0_FunkyTurretIO io;
  private final V0_FunkyTurretIOInputsAutoLogged inputs;

  private Rotation2d previousPosition;
  private Rotation2d desiredRotations;

  private boolean atSetPoint;

  public V0_FunkyTurret(V0_FunkyTurretIO io) {
    this.io = io;
    inputs = new V0_FunkyTurretIOInputsAutoLogged();

    previousPosition = inputs.turretAngle;
    desiredRotations = new Rotation2d();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret At Setpoint", atSetPoint);
    Logger.recordOutput(
        "Turret At Setpoint", previousPosition.getRadians() - desiredRotations.getRadians());
  }

  public boolean withinRange(Rotation2d angle) {
    return (previousPosition.getDegrees() + angle.getDegrees() <= V0_FunkyTurretConstants.MAX_ANGLE
        && previousPosition.getDegrees() + angle.getDegrees() >= V0_FunkyTurretConstants.MIN_ANGLE);
  }

  public void updateInputs(V0_FunkyTurretIO io, V0_FunkyTurretIOInputs inputs) {
    io.updateInputs(inputs);
  }

  public void setTurretVoltage(V0_FunkyTurretIO io, double volts) {
    io.setTurretVoltage(volts);
  }

  public void setTurretGoal(V0_FunkyTurretIO io, Pose3d goal) {
    io.setTurretGoal(goal);
  }

  public void stopTurret(V0_FunkyTurretIO io) {
    io.stopTurret();
  }

  public boolean atTurretPositionGoal(V0_FunkyTurretIO io) {
    io.atTurretPositionGoal();
    return false;
  }

  public void incrementTurret(double increment) {
    Rotation2d incrementRotation = new Rotation2d(increment);
    Rotation2d currentPosition = inputs.turretAngle;
    io.setTurretGoal(currentPosition.getDegrees() + incrementRotation.getDegrees());
  }

  public void updateGains(double kP, double kD, double kV, double kA) {
    io.updateGains(kP, kD, kV, kA);
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    io.updateConstraints(maxAcceleration, maxVelocity, goalTolerance);
  }

  public void resetTurret() {
    io.resetTurret();
  }
}
