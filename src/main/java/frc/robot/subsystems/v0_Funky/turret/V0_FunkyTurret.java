package frc.robot.subsystems.v0_Funky.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
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

public class V0_FunkyTurret {
  private final V0_FunkyTurretIO io;
  private final String aKitTopic;
  private final V0_FunkyTurretIOInputsAutoLogged inputs;

  private final Rotation2d previousPosition;

  private final SysIdRoutine characterizationRoutine;

  private V0_FunkyTurretState state;

  private final Supplier<Pose2d> robotPoseSupplier;

  public V0_FunkyTurret(
      V0_FunkyTurretIO io, Subsystem subsystem, int index, Supplier<Pose2d> robotPoseSupplier) {
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

    this.robotPoseSupplier = robotPoseSupplier;

    state = V0_FunkyTurretState.IDLE;

    io.setPosition(calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));
  }

  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.updateGains(
                V0_FunkyTurretConstants.GAINS.kP().get(),
                V0_FunkyTurretConstants.GAINS.kD().get(),
                V0_FunkyTurretConstants.GAINS.kS().get(),
                V0_FunkyTurretConstants.GAINS.kV().get(),
                V0_FunkyTurretConstants.GAINS.kA().get()),
        V0_FunkyTurretConstants.GAINS.kP(),
        V0_FunkyTurretConstants.GAINS.kD(),
        V0_FunkyTurretConstants.GAINS.kS(),
        V0_FunkyTurretConstants.GAINS.kV(),
        V0_FunkyTurretConstants.GAINS.kA());

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            io.updateConstraints(
                V0_FunkyTurretConstants.CONSTRAINTS
                    .MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED()
                    .get(),
                V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get(),
                V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS().get()),
        V0_FunkyTurretConstants.CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED(),
        V0_FunkyTurretConstants.CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND(),
        V0_FunkyTurretConstants.CONSTRAINTS.GOAL_TOLERANCE_RADIANS());

    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/At Goal", atTurretPositionGoal());
    Logger.recordOutput(aKitTopic + "/State", state.name());

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL -> io.setTurretGoal(
          clampShortest(state.getRotation(), inputs.turretAngle));
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setTurretVoltage(state.getVoltage());
      case CLOSED_LOOP_AUTO_AIM_CONTROL -> io.setTurretGoal(
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
    return (!(previousPosition.getDegrees() + angle.getDegrees()
            <= V0_FunkyTurretConstants.MAX_ANGLE)
        || !(previousPosition.getDegrees() + angle.getDegrees()
            >= V0_FunkyTurretConstants.MIN_ANGLE));
  }

  public Command setTurretVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          state = V0_FunkyTurretState.OPEN_LOOP_VOLTAGE_CONTROL;
          state.setVoltage(volts);
        });
  }

  public Command setTurretGoal(Rotation2d goal) {
    return Commands.runOnce(
        () -> {
          state = V0_FunkyTurretState.CLOSED_LOOP_POSITION_CONTROL;
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
          state = V0_FunkyTurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
          state.setTranslation(goal);
        });
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

  /** Method that calculates turret angle based on encoder values. Can be non coprime or coprime! */
  private Rotation2d calculateTurretAngle(Angle e1, Angle e2) {
    // 1. Get raw radians in [0, 2pi)
    // We use 0 to 2pi to clarify the subtraction logic
    double a1 = MathUtil.inputModulus(e1.in(Units.Radians), 0, 2 * Math.PI);
    double a2 = MathUtil.inputModulus(e2.in(Units.Radians), 0, 2 * Math.PI);

    // 4. Calculate the Phase Difference
    // We wrap this difference to [-pi, pi) to handle the 0/360 crossover point gracefully.
    double d_x12 = MathUtil.angleModulus(a1 - a2);

    // 5. Calculate Coarse Angle (The "Vernier" Estimate)
    // dividing by 'V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.BEAT()' is mathematically
    // identical to multiplying by SLOPE
    double coarseAngle =
        d_x12 / V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.GEAR_RATIO_DIFFERENCE();

    // 6. Refine using Encoder 1 (High Precision)
    // We use the coarse angle to find which rotation "k" Encoder 1 is on.
    // Expected = Coarse * n1
    double expectedEnc1Total =
        coarseAngle * V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.GEAR_1_RATIO();

    // Find integer k to unwrap a1
    // k = round( (Expected - Actual) / 2pi )
    double k = Math.round((expectedEnc1Total - a1) / (2.0 * Math.PI));

    // 7. Calculate Final Angle
    double finalEnc1Total = a1 + (k * 2.0 * Math.PI);
    double turretAngle =
        finalEnc1Total / V0_FunkyTurretConstants.TURRET_ANGLE_CALCULATION.GEAR_1_RATIO();

    return Rotation2d.fromRadians(MathUtil.angleModulus(turretAngle));
  }
}
