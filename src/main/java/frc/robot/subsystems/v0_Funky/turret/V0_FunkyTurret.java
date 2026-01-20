package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyTurret {
  private final V0_FunkyTurretIO io;
  private final String aKitTopic;
  private final V0_FunkyTurretIOInputsAutoLogged inputs;

  private final Rotation2d previousPosition;

  private final SysIdRoutine characterizationRoutine;

  private V0_FunkyTurretState state;

  public V0_FunkyTurret(V0_FunkyTurretIO io, Subsystem subsystem, int index) {
    this.io = io;
    inputs = new V0_FunkyTurretIOInputsAutoLogged();
    previousPosition = inputs.turretAngle;
    aKitTopic = subsystem.getName() + "/Turret" + index;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setTurretVoltage(volts.in(Volts)), null, subsystem));

    state = V0_FunkyTurretState.IDLE;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/At Goal", atTurretPositionGoal());

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL -> io.setTurretGoal(state.getRotation());
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setTurretVoltage(state.getVoltage());
      default -> {}
    }
  }

  public boolean outOfRange(Rotation2d angle) {
    return (!(previousPosition.getDegrees() + angle.getDegrees()
            <= V0_FunkyTurretConstants.MAX_ANGLE)
        || !(previousPosition.getDegrees() + angle.getDegrees()
            >= V0_FunkyTurretConstants.MIN_ANGLE));
  }

  public Command setTurretVoltage(double volts) {
    return Commands.run(
        () -> {
          state = V0_FunkyTurretState.OPEN_LOOP_VOLTAGE_CONTROL;
          state.setVoltage(volts);
        });
  }

  public Command setTurretGoal(Rotation2d goal) {
    return Commands.run(
        () -> {
          state = V0_FunkyTurretState.CLOSED_LOOP_POSITION_CONTROL;
          state.setRotation(clampShortest(goal, inputs.turretAngle));
        });
  }

  public Command stopTurret() {
    return setTurretVoltage(0);
  }

  public boolean atTurretPositionGoal() {
    return io.atTurretPositionGoal();
  }

  public Command waitUntilTurretAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atTurretPositionGoal));
  }

  public Command incrementTurret(Rotation2d increment) {
    return setTurretGoal(inputs.turretAngle.plus(increment));
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  public void updateConstraints(double maxAcceleration, double maxVelocity, double goalTolerance) {
    io.updateConstraints(maxAcceleration, maxVelocity, goalTolerance);
  }

  public Command resetTurret() {
    return setTurretGoal(new Rotation2d())
        .andThen(stopTurret())
        .andThen(Commands.runOnce(() -> io.setPosition(new Rotation2d())));
  }

  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> state = V0_FunkyTurretState.IDLE),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> outOfRange(previousPosition)),
        Commands.waitSeconds(3),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> outOfRange(previousPosition)),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  private Rotation2d clampShortest(Rotation2d target, Rotation2d current) {
    double targetRad = target.getRadians();
    double currentRad = current.getRadians();
    double minRad = V0_FunkyTurretConstants.MIN_ANGLE;
    double maxRad = V0_FunkyTurretConstants.MAX_ANGLE;

    // Try both the direct target and the wrapped alternatives
    double[] candidates = {targetRad, targetRad + 2 * Math.PI, targetRad - 2 * Math.PI};

    double bestValid = Double.NaN;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (double candidate : candidates) {
      // Check if this candidate is within limits
      if (candidate >= minRad && candidate <= maxRad) {
        double distance = Math.abs(candidate - currentRad);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestValid = candidate;
        }
      }
    }

    // If we found a valid angle, use it
    if (!Double.isNaN(bestValid)) {
      return Rotation2d.fromRadians(bestValid);
    }

    return Rotation2d.fromRadians(
        currentRad < minRad
            ? minRad
            : currentRad > maxRad
                ? maxRad
                : (Math.abs(currentRad - minRad) < Math.abs(currentRad - maxRad)
                    ? minRad
                    : maxRad));
  }
}
