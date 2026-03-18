package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private final TurretIO io;
  private final String aKitTopic;
  private final TurretIOInputsAutoLogged inputs;

  private Rotation2d positionGoal;
  private Rotation2d previousPosition;

  private Voltage voltageGoal;

  private Translation2d translationGoal;

  private final SysIdRoutine characterizationRoutine;

  private TurretState state;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final TurretConstants constants;

  public Turret(
      TurretIO io,
      Subsystem subsystem,
      String name,
      Supplier<Pose2d> robotPoseSupplier,
      TurretConstants constants) {
    this.io = io;
    inputs = new TurretIOInputsAutoLogged();
    previousPosition = inputs.angle;
    aKitTopic = subsystem.getName() + "/Turret" + name;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds),
                Volts.of(2),
                Seconds.of(5),
                (state) -> Logger.recordOutput(aKitTopic + "/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    this.robotPoseSupplier = robotPoseSupplier;

    translationGoal = new Translation2d();

    state = TurretState.IDLE;
    positionGoal = Rotation2d.kZero;

    this.constants = constants;

    voltageGoal = Volts.of(0.0);
    positionGoal = Rotation2d.kZero;

    io.setPosition(calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    Logger.recordOutput(
        aKitTopic + "/CRT Angle",
        calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));

    Logger.recordOutput(aKitTopic + "/At Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/State", state.name());

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal(wrapRotationWithinBounds((positionGoal), inputs.angle));
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal(voltageGoal);
      case CLOSED_LOOP_AUTO_AIM_CONTROL -> {
        positionGoal =
            translationGoal
                .minus(robotPoseSupplier.get().getTranslation())
                .getAngle()
                .minus(robotPoseSupplier.get().getRotation());
        io.setPositionGoal(wrapRotationWithinBounds(positionGoal, inputs.angle));
      }
      default -> {}
    }
  }

  public boolean outOfRange(Rotation2d angle) {
    return (!(previousPosition.getDegrees() + angle.getDegrees() <= constants.maxAngle.getDegrees())
        || !(previousPosition.getDegrees() + angle.getDegrees()
            >= constants.minAngle.getDegrees()));
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public Command stopTurret() {
    return Commands.runOnce(() -> setVoltageGoal(Volts.zero()));
  }

  public boolean atPositionGoal() {
    return io.atPositionGoal(positionGoal);
  }

  public boolean atPositionGoal(Rotation2d positionReference) {
    return io.atPositionGoal(positionReference);
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal(voltageGoal);
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atPositionGoal));
  }

  public void incrementTurret(Rotation2d increment) {
    setPositionGoal(inputs.angle.plus(increment));
  }

  public Command reset() {
    return Commands.runOnce(() -> setPositionGoal(new Rotation2d()))
        .andThen(stopTurret())
        .finallyDo(() -> io.setPosition(new Rotation2d()));
  }

  public void setFieldRelativeGoal(Translation2d goal) {
    state = TurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
    translationGoal = goal;
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

  private Rotation2d wrapRotationWithinBounds(Rotation2d target, Rotation2d current) {
    double currentTotalRad = current.getRadians();
    double minRad = constants.minAngle.getRadians();
    double maxRad = constants.maxAngle.getRadians();

    double diff = target.getRadians() - currentTotalRad;
    double unwound = currentTotalRad + Math.IEEEremainder(diff, 2 * Math.PI);

    if (unwound >= minRad && unwound <= maxRad) {
      return Rotation2d.fromRadians(unwound);
    }

    if (unwound < minRad) {
      double n = Math.ceil((minRad - unwound) / (2 * Math.PI));
      double candidate = unwound + n * 2 * Math.PI;

      if (candidate <= maxRad) {
        return Rotation2d.fromRadians(candidate);
      }
    } else if (unwound > maxRad) {
      double n = Math.ceil((unwound - maxRad) / (2 * Math.PI));
      double candidate = unwound - n * 2 * Math.PI;

      if (candidate >= minRad) {
        return Rotation2d.fromRadians(candidate);
      }
    }

    // Not possible... return target angle (io handles this by going to the closest bound)
    return target;
  }

  /**
   * Method that calculates turret angle based on encoder values. Uses the Chinese Remainder
   * Theorem.
   */
  public Rotation2d calculateTurretAngle(Angle e1, Angle e2) {

    // Get encoder positions in rotations (0 to 1), using full floating point precision
    double e1Rotations = e1.in(Rotations) % 1.0;
    if (e1Rotations < 0) {
      e1Rotations += 1.0;
    }

    double e2Rotations = e2.in(Rotations) % 1.0;
    if (e2Rotations < 0) {
      e2Rotations += 1.0;
    }

    // Calculate difference (preserving full precision)
    double diff = (e1Rotations - e2Rotations);

    // Normalize difference to [0, 1)
    diff = diff % 1.0;
    if (diff < 0) {
      diff += 1.0;
    }

    // Direct calculation: θ = Δ * (g1*g2/g0)
    double turretRotations =
        diff
            * constants.turretAngleCalculation.gear1ToothCount()
            * constants.turretAngleCalculation.gear2ToothCount()
            / (double) constants.turretAngleCalculation.gear0ToothCount();

    return Rotation2d.fromRotations(turretRotations);
  }
}
