package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private final TurretIO io;
  private final String aKitTopic;
  private final TurretIOInputsAutoLogged inputs;

  private final Rotation2d previousPosition;

  private final SysIdRoutine characterizationRoutine;

  private TurretState state;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final TurretConstants constants;

  public Turret(
      TurretIO io,
      Subsystem subsystem,
      int index,
      Supplier<Pose2d> robotPoseSupplier,
      TurretConstants constants) {
    this.io = io;
    inputs = new TurretIOInputsAutoLogged();
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

    this.robotPoseSupplier = robotPoseSupplier;

    state = TurretState.IDLE;

    this.constants = constants;

    io.setPosition(calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));
  }

  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.updateGains(
                constants.gains.kP().get(),
                constants.gains.kD().get(),
                constants.gains.kS().get(),
                constants.gains.kV().get(),
                constants.gains.kA().get()),
        constants.gains.kP(),
        constants.gains.kD(),
        constants.gains.kS(),
        constants.gains.kV(),
        constants.gains.kA());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.updateConstraints(
                constants.constraints.maxAccelerationRadiansPerSecondSquared().get(),
                constants.constraints.cruisingVelocityRadiansPerSecond().get(),
                constants.constraints.goalToleranceRadians().get()),
        constants.constraints.maxAccelerationRadiansPerSecondSquared(),
        constants.constraints.cruisingVelocityRadiansPerSecond(),
        constants.constraints.goalToleranceRadians());

    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    Logger.recordOutput(
        aKitTopic + "CRT Angle",
        calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));

    Logger.recordOutput(aKitTopic + "/At Goal", atTurretPositionGoal());
    Logger.recordOutput(aKitTopic + "/State", state.name());

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setTurretGoal(clampShortest(state.getRotation(), inputs.turretAngle));
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setTurretVoltage(state.getVoltage());
      case CLOSED_LOOP_AUTO_AIM_CONTROL ->
          io.setTurretGoal(
              clampShortest(
                  state
                      .getTranslation()
                      .minus(robotPoseSupplier.get().getTranslation())
                      .getAngle()
                      .minus(robotPoseSupplier.get().getRotation()),
                  inputs.turretAngle));
      default -> {}
    }
  }

  public boolean outOfRange(Rotation2d angle) {
    return (!(previousPosition.getDegrees() + angle.getDegrees() <= constants.maxAngle)
        || !(previousPosition.getDegrees() + angle.getDegrees() >= constants.minAngle));
  }

  public Command setTurretVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
          state.setVoltage(volts);
        });
  }

  public Command setTurretGoal(Rotation2d goal) {
    return Commands.runOnce(
        () -> {
          state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
          state.setRotation(goal);
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

  public Command resetTurret() {
    return setTurretGoal(new Rotation2d())
        .andThen(stopTurret())
        .finallyDo(() -> io.setPosition(new Rotation2d()));
  }

  public Command setFieldRelativeGoal(Translation2d goal) {
    return Commands.runOnce(
        () -> {
          state = TurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
          state.setTranslation(goal);
        });
  }

  public Command runSysId() {
    return Commands.sequence(
        Commands.runOnce(() -> state = TurretState.IDLE),
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
    double minRad = constants.minAngle;
    double maxRad = constants.maxAngle;

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

  /**
   * Method that calculates turret angle based on encoder values. Uses the Chinese Remainder
   * Theorem.
   */
  public static Rotation2d calculateTurretAngle(Angle e1, Angle e2) {
    final int g1ToothCount = 16;
    final int g2ToothCount = 17;
    final int g0ToothCount = 120;

    double e1Rotations = e1.in(Rotations) % 1;
    if (e1Rotations < 0) {
      e1Rotations += g1ToothCount;
    }
    double e2Rotations = e2.in(Rotations) % 1;
    if (e2Rotations < 0) {
      e2Rotations += g2ToothCount;
    }

    int e1ToothRotations = (int) Math.floor(e1Rotations * g1ToothCount + 1e-9);
    int e2ToothRotations = (int) Math.floor(e2Rotations * g2ToothCount + 1e-9);

    int k = ((e2ToothRotations - e1ToothRotations) * g1ToothCount) % g2ToothCount;
    if (k < 0) {
      k += g2ToothCount;
    }

    int turretToothRotations =
        e1ToothRotations + (k * g1ToothCount) % (g1ToothCount * g2ToothCount);

    return Rotation2d.fromRotations((double) turretToothRotations / g0ToothCount);
  }
}
