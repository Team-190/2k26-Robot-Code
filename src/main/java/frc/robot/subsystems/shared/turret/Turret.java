package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private final TurretIO io;
  private final String aKitTopic;
  private final TurretIOInputsAutoLogged inputs;

  private final Rotation2d previousPosition;

  private final Setpoint<VoltageUnit> voltageGoal;
  private final Setpoint<AngleUnit> positionGoal;
  private final Setpoint<AngularVelocityUnit> angularVelocityGoal;

  private Translation2d translationGoal;

  private final SysIdRoutine characterizationRoutine;

  private TurretState state;

  private final Supplier<Pose2d> robotPoseSupplier;
  Supplier<ChassisSpeeds> fieldVelocitySupplier;

  private final TurretConstants constants;

  private final SimpleMotorFeedforward feedforwardController;

  public Turret(
      TurretIO io,
      Subsystem subsystem,
      String name,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> fieldVelocitySupplier,
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
    this.fieldVelocitySupplier = fieldVelocitySupplier;

    translationGoal = new Translation2d();

    state = TurretState.IDLE;

    this.constants = constants;

    voltageGoal = new Setpoint<>(Volts.of(0), constants.voltageStep, Volts.of(-12), Volts.of(12));
    positionGoal =
        new Setpoint<>(
            calculateTurretAngle(
                    io.getEncoder1Position(),
                    io.getEncoder2Position(),
                    Turret.this.constants.turretAngleCalculation)
                .getMeasure(),
            constants.angleStep.getMeasure(),
            constants.minAngle.getMeasure(),
            constants.maxAngle.getMeasure());
    angularVelocityGoal = new Setpoint<>(RadiansPerSecond.zero(), RadiansPerSecond.of(0.25));

    io.setPosition(new Rotation2d());

    feedforwardController =
        new SimpleMotorFeedforward(
            constants.aimingFeedforwardGains.getKS(),
            constants.aimingFeedforwardGains.getKV(),
            constants.aimingFeedforwardGains.getKA());
  }

  @Trace
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    Logger.recordOutput(
        aKitTopic + "/CRT Angle",
        calculateTurretAngle(
            io.getEncoder1Position(), io.getEncoder2Position(), constants.turretAngleCalculation));

    Logger.recordOutput(aKitTopic + "/At Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/State", state.name());

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal(
              wrapRotationWithinBounds(
                  new Rotation2d((Angle) positionGoal.getNewSetpoint()),
                  inputs.angle,
                  constants.minAngle.getRadians(),
                  constants.maxAngle.getRadians()),
              (AngularVelocity) angularVelocityGoal.getNewSetpoint(),
              0.0);
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_AUTO_AIM_CONTROL -> {
        positionGoal.setSetpoint(
            angleToGoal(translationGoal, robotPoseSupplier.get(), constants.turretOffset)
                .getMeasure());
        io.setPositionGoal(
            wrapRotationWithinBounds(
                new Rotation2d((Angle) positionGoal.getNewSetpoint()),
                inputs.angle,
                constants.minAngle.getRadians(),
                constants.maxAngle.getRadians()),
            (AngularVelocity) angularVelocityGoal.getNewSetpoint(),
            calculateFeedforwardVoltage(
                feedforwardController,
                translationGoal,
                robotPoseSupplier.get(),
                fieldVelocitySupplier.get()));
      }
      default -> {}
    }
  }

  private static Rotation2d angleToGoal(
      Translation2d translationGoal, Pose2d robotPose, Translation2d turretOffset) {

    Translation2d turretPosition =
        robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

    return translationGoal
        .minus(turretPosition)
        .getAngle()
        .minus(robotPose.getRotation())
        .unaryMinus();
  }

  private static double calculateFeedforwardVoltage(
      SimpleMotorFeedforward feedforwardController,
      Translation2d translationGoal,
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity) {

    double rx = translationGoal.getX() - robotPose.getX();
    double ry = translationGoal.getY() - robotPose.getY();

    double distanceSq = (rx * rx) + (ry * ry);

    if (distanceSq < 0.01) {
      return 0.0;
    }

    double targetOmega =
        (ry * fieldVelocity.vxMetersPerSecond - rx * fieldVelocity.vyMetersPerSecond) / distanceSq;

    return -feedforwardController.calculate(fieldVelocity.omegaRadiansPerSecond - targetOmega); //still needs testing
  }

  public boolean outOfRange(Rotation2d angle) {
    return (!(previousPosition.getDegrees() + angle.getDegrees() <= constants.maxAngle.getDegrees())
        || !(previousPosition.getDegrees() + angle.getDegrees()
            >= constants.minAngle.getDegrees()));
  }

  public void setVoltageGoal(Voltage volts) {
    state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
    voltageGoal.setSetpoint(volts);
  }

  public void setPositionGoal(Rotation2d goal) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal.setSetpoint(goal.getMeasure());
  }

  public void setPositionGoal(Rotation2d goal, AngularVelocity velocity) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal.setSetpoint(goal.getMeasure());
    angularVelocityGoal.setSetpoint(velocity);
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public Command stopTurret() {
    return Commands.runOnce(() -> setVoltageGoal(Volts.zero()));
  }

  public boolean atPositionGoal() {
    return io.atPositionGoal(new Rotation2d((Angle) positionGoal.getNewSetpoint()));
  }

  public boolean atPositionGoal(Rotation2d positionReference) {
    return io.atPositionGoal(positionReference);
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atPositionGoal));
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

  public void setFieldRelativeGoal(Translation2d goal, AngularVelocity velocity) {
    state = TurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
    angularVelocityGoal.setSetpoint(velocity);
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

  private static Rotation2d wrapRotationWithinBounds(
      Rotation2d target, Rotation2d current, double minRad, double maxRad) {
    double currentTotalRad = current.getRadians();

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

    // Not possible... return unwound angle (io handles this by clamping to the closest bound)
    return Rotation2d.fromRadians(unwound);
  }

  /**
   * Method that calculates turret angle based on encoder values. Uses the Chinese Remainder
   * Theorem.
   */
  public static Rotation2d calculateTurretAngle(
      Angle e1, Angle e2, TurretConstants.TurretAngleCalculation gearRatios) {

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
            * gearRatios.gear1ToothCount()
            * gearRatios.gear2ToothCount()
            / (double) gearRatios.gear0ToothCount();

    return Rotation2d.fromRotations(turretRotations);
  }

  public void updateGains(Gains gains) {
    io.updateGains(gains);
  }

  public void updateConstraints(AngularPositionConstraints constraints) {
    io.updateConstraints(constraints);
  }

  public void updateAutoAimGains(Gains gains) {
    feedforwardController.setKs(gains.getKS());
    feedforwardController.setKv(gains.getKV());
    feedforwardController.setKa(gains.getKA());
  }
}
