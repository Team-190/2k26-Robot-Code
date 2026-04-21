package frc.robot.subsystems.shared.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private final TurretIO io;
  private final String aKitTopic;
  private final TurretIOInputsAutoLogged inputs;

  private final Setpoint<VoltageUnit> voltageGoal;
  private final Setpoint<AngleUnit> positionGoal;
  private final Setpoint<AngularVelocityUnit> angularVelocityGoal;

  private Translation2d translationGoal;

  private final SysIdRoutine characterizationRoutine;

  private TurretState state;
  private TurretState previousState;
  @Getter private boolean isWrapping;

  private final Supplier<Pose2d> robotPoseSupplier;
  Supplier<ChassisSpeeds> robotVelocitySuppiler;

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
    aKitTopic = subsystem.getName() + "/Turret" + name;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Seconds),
                Volts.of(4),
                Seconds.of(5),
                (state) -> Logger.recordOutput(aKitTopic + "/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    this.robotPoseSupplier = robotPoseSupplier;
    this.robotVelocitySuppiler = fieldVelocitySupplier;

    translationGoal = new Translation2d();

    state = TurretState.IDLE;
    previousState = TurretState.IDLE;
    isWrapping = false;

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

    Rotation2d startingAngle =
        calculateTurretAngle(
            io.getEncoder1Position(), io.getEncoder2Position(), constants.turretAngleCalculation);
    if (startingAngle.getRadians() > constants.maxAngle.getRadians()) {
      startingAngle =
          startingAngle.minus(
              Rotation2d.fromRotations(
                  (double) constants.turretAngleCalculation.gear1ToothCount()
                      * (double) constants.turretAngleCalculation.gear2ToothCount()
                      / (double) constants.turretAngleCalculation.gear0ToothCount()));
    }

    // io.setPosition(startingAngle);
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
    Logger.recordOutput(aKitTopic + "/Out Of Range", outOfRange());

    if (outOfRange()
        && state != TurretState.UNWRAPPING
        && state != TurretState.IDLE
        && state != TurretState.OPEN_LOOP_VOLTAGE_CONTROL) {
      previousState = state;
      state = TurretState.UNWRAPPING;
    }

    isWrapping = state == TurretState.UNWRAPPING;
    switch (state) {
      case UNWRAPPING -> {
        double midPointAbsoluteRad =
            (constants.maxAngle.getRadians() + constants.minAngle.getRadians()) / 2.0;

        Rotation2d safeRelativeTarget =
            Rotation2d.fromRadians(
                midPointAbsoluteRad
                    + Math.IEEEremainder(
                        positionGoal.getNewSetpoint().in(Radians) - midPointAbsoluteRad,
                        2.0 * Math.PI));

        io.setPositionGoal(safeRelativeTarget, 0.0);

        if (Math.abs(inputs.angle.getDegrees() - safeRelativeTarget.getDegrees()) < 5) {
          state = (previousState == TurretState.UNWRAPPING) ? TurretState.IDLE : previousState;
          previousState = TurretState.UNWRAPPING;
        }
      }
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal(
              findClosest(new Rotation2d((Angle) positionGoal.getNewSetpoint()), inputs.angle),
              (AngularVelocity) angularVelocityGoal.getNewSetpoint(),
              0.0);
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_AUTO_AIM_CONTROL -> {
        positionGoal.setSetpoint(
            angleToGoal(translationGoal, robotPoseSupplier.get()).getMeasure());
        io.setPositionGoal(
            findClosest(new Rotation2d((Angle) positionGoal.getNewSetpoint()), inputs.angle),
            (AngularVelocity) angularVelocityGoal.getNewSetpoint(),
            calculateFeedforwardVoltage(translationGoal));
      }
      default -> {}
    }
  }

  public Rotation2d angleToGoal(Translation2d translationGoal, Pose2d robotPose) {

    Transform2d robotToTurretTransform =
        new Transform2d(
            constants.robotToTurretTransform.getX(),
            constants.robotToTurretTransform.getY(),
            constants.robotToTurretTransform.getRotation().toRotation2d());
    Pose2d turretPose = robotPose.transformBy(robotToTurretTransform);
    Translation2d turretToTarget = translationGoal.minus(turretPose.getTranslation());
    return turretToTarget.getAngle().minus(turretPose.getRotation());
  }

  private double calculateFeedforwardVoltage(Translation2d translationGoal) {

    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotVelocitySuppiler.get(), robotPoseSupplier.get().getRotation());

    double rx = translationGoal.getX() - robotPoseSupplier.get().getX();
    double ry = translationGoal.getY() - robotPoseSupplier.get().getY();

    double distanceSq = (rx * rx) + (ry * ry);

    double targetOmega;
    if (distanceSq < 0.01) {
      targetOmega = 0.0;
    } else {
      targetOmega =
          (ry * fieldVelocity.vxMetersPerSecond - rx * fieldVelocity.vyMetersPerSecond)
              / distanceSq;
    }

    return feedforwardController.calculate(targetOmega - fieldVelocity.omegaRadiansPerSecond);
  }

  public boolean outOfRange() {
    return inputs
            .angle
            .getMeasure()
            .lte(
                constants
                    .minAngle
                    .getMeasure()
                    .plus(inputs.velocity.times(TurretConstants.TURRET_RANGE_LOOKAHEAD)))
        || inputs
            .angle
            .getMeasure()
            .gte(
                constants
                    .maxAngle
                    .getMeasure()
                    .minus(inputs.velocity.times(TurretConstants.TURRET_RANGE_LOOKAHEAD)));
  }

  public void setVoltageGoal(Voltage volts) {
    if (state != TurretState.UNWRAPPING) {
      state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
      voltageGoal.setSetpoint(volts);
    }
  }

  public void setPositionGoal(Rotation2d goal) {
    if (state != TurretState.UNWRAPPING) {
      state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
      positionGoal.setSetpoint(goal.getMeasure());
    }
  }

  public void setPositionGoal(Rotation2d goal, AngularVelocity velocity) {
    if (state != TurretState.UNWRAPPING) {
      state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
      positionGoal.setSetpoint(goal.getMeasure());
      angularVelocityGoal.setSetpoint(velocity);
    }
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public Command stopTurret() {
    return Commands.runOnce(() -> setVoltageGoal(Volts.zero()));
  }

  public boolean atPositionGoal() {
    return io.atPositionGoal(inputs.positionGoal);
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
    if (state != TurretState.UNWRAPPING) state = TurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
    translationGoal = goal;
  }

  public void setFieldRelativeGoal(Translation2d goal, AngularVelocity velocity) {
    setFieldRelativeGoal(goal);
    angularVelocityGoal.setSetpoint(velocity);
  }

  public Command runSysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> state = TurretState.IDLE),
        characterizationRoutine.quasistatic(Direction.kForward).until(() -> outOfRange()),
        Commands.waitSeconds(3),
        characterizationRoutine.quasistatic(Direction.kReverse).until(() -> outOfRange()),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  private static Rotation2d findClosest(Rotation2d target, Rotation2d current) {
    double currentRad = current.getRadians();
    double diff = target.getRadians() - currentRad;

    double closest = currentRad + Math.IEEEremainder(diff, 2 * Math.PI);

    return Rotation2d.fromRadians(closest);
  }

  /**
   * Method that calculates turret angle based on encoder values. Uses the Chinese Remainder
   * Theorem.
   */
  public static Rotation2d calculateTurretAngle(
      Angle e1, Angle e2, TurretConstants.TurretAngleCalculation gearRatios) {

    // Get encoder positions in rotations (0 to 1), using full floating point
    // precision
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

  public Rotation2d getPosition() {
    return inputs.angle;
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }
}
