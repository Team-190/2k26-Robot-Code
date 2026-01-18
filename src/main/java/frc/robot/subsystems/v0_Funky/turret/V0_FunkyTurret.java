package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v0_Funky.turret.V0_FunkyTurretIO.V0_FunkyTurretIOInputs;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyTurret {
  private final V0_FunkyTurretIO io;
  private final String aKitTopic;
  private final V0_FunkyTurretIOInputsAutoLogged inputs;

  private Rotation2d previousPosition;
  private Rotation2d desiredRotations;

  private SysIdRoutine characterizationRoutine;

  private boolean atSetPoint;
  private boolean isClosedLoop;

  private Pose3d goal;

  public V0_FunkyTurret(V0_FunkyTurretIO io, Subsystem subsystem, int index) {
    this.io = io;
    inputs = new V0_FunkyTurretIOInputsAutoLogged();

    previousPosition = inputs.turretAngle;
    desiredRotations = new Rotation2d();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Turret/sysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setTurretVoltage(volts.in(Volts)), null, subsystem));

    aKitTopic = subsystem.getName() + "/Hood" + index;
    isClosedLoop = false;
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (isClosedLoop) {
      io.setPosition(goal.toPose2d().getRotation().getRadians());
    }

    Logger.processInputs(aKitTopic, inputs);
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
    isClosedLoop = false;
  }

  public void setTurretGoal(V0_FunkyTurretIO io, double goal) {
    io.setTurretGoal(goal);
  }

  public void setTurretGoal(V0_FunkyTurretIO io, Pose3d goal) {
    io.setTurretGoal(goal.toPose2d().getRotation().getRadians());
    isClosedLoop = true;
  }

  public void stopTurret(V0_FunkyTurretIO io) {
    io.stopTurret();
  }

  public boolean atTurretPositionGoal(V0_FunkyTurretIO io) {
    return io.atTurretPositionGoal();
  }

  public void incrementTurret(double incrementRadians) {
    Rotation2d incrementRotation = new Rotation2d(incrementRadians);
    Rotation2d currentPosition = inputs.turretAngle;
    io.setTurretGoal(currentPosition.getRadians() + incrementRotation.getRadians());
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    io.updateConstraints(maxAcceleration, maxVelocity, goalTolerance);
  }

  public void resetTurret() {
    io.resetTurret();
  }

  public void checkDirectionalMotion() {
    io.checkDirectionalMotion();
  }

  public boolean isInRange(Rotation2d angle) {
    return (angle.getDegrees() <= V0_FunkyTurretConstants.MAX_ANGLE
        && angle.getDegrees() >= V0_FunkyTurretConstants.MIN_ANGLE);
  }

  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> !isInRange(Rotation2d.fromRadians(V0_FunkyTurretConstants.CURRENT_ANGLE))),
        Commands.waitSeconds(3),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> !isInRange(Rotation2d.fromRadians(V0_FunkyTurretConstants.CURRENT_ANGLE))),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
