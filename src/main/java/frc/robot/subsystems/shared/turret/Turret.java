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
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod(GeometryUtil.class)
public class Turret {
  private final TurretIO io;
  private final String aKitTopic;
  private final TurretIOInputsAutoLogged inputs;

  private Setpoint<VoltageUnit> voltageGoal;
  private Setpoint<AngleUnit> positionGoal;
  private final Setpoint<AngularVelocityUnit> angularVelocityGoal;

  private Translation2d translationGoal;

  private final SysIdRoutine characterizationRoutine;

  @Getter private boolean isWrapping;

  private TurretState state;
  private TurretState previousState;

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
    previousState = TurretState.IDLE;

    isWrapping = false;

    this.constants = constants;

    voltageGoal = new Setpoint<>(Volts.of(0), constants.voltageStep, Volts.of(-12), Volts.of(12));

    positionGoal =
        new Setpoint<>(
            calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()).getMeasure(),
            constants.angleStep.getMeasure(),
            constants.minAngle.getMeasure(),
            constants.maxAngle.getMeasure());

    angularVelocityGoal = new Setpoint<>(RadiansPerSecond.zero(), RadiansPerSecond.of(0.05));

    io.setPosition(new Rotation2d());

    feedforwardController =
        new SimpleMotorFeedforward(
            constants.aimingFeedforwardGains.getKS(),
            constants.aimingFeedforwardGains.getKV(),
            constants.aimingFeedforwardGains.getKA());
  }

  private Rotation2d nearestEquivalent(Rotation2d goal, Rotation2d current) {
    double goalRad = goal.getRadians();
    double currentRad = current.getRadians();

    double k = Math.round((currentRad - goalRad) / (2.0 * Math.PI));
    return new Rotation2d(goalRad + k * 2.0 * Math.PI);
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Rotation2d currentAngle = inputs.angle;
    isWrapping = TurretState.UNWRAPPING == state;

    if (outOfRange(currentAngle)
        && (state != TurretState.IDLE
            && state != TurretState.OPEN_LOOP_VOLTAGE_CONTROL
            && state != TurretState.UNWRAPPING)) {
      previousState = state;
      state = TurretState.UNWRAPPING;
    }

    switch (state) {
      case CLOSED_LOOP_POSITION_CONTROL -> {
        Rotation2d rawGoal = (Rotation2d) positionGoal.getNewSetpoint();
        Rotation2d adjustedGoal = nearestEquivalent(rawGoal, currentAngle);

        io.setPositionGoal(adjustedGoal, 0.0);
      }

      case CLOSED_LOOP_AUTO_AIM_CONTROL -> {
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds fieldVelocity = fieldVelocitySupplier.get();

        Rotation2d rawGoal = fieldToTurret(robotPose);

        Rotation2d adjustedGoal = nearestEquivalent(rawGoal, currentAngle);

        double ffVolts =
            calculateFeedforwardVoltage(
                feedforwardController, translationGoal, robotPose, fieldVelocity);

        io.setPositionGoal(
            adjustedGoal, (AngularVelocity) angularVelocityGoal.getNewSetpoint(), ffVolts);
      }

      case UNWRAPPING -> {
        Rotation2d unwrapGoal = unwrapTarget(currentAngle);

        io.setPositionGoal(unwrapGoal, 0.0);

        double error = Math.abs(currentAngle.getRadians() - unwrapGoal.getRadians());

        if (error < 0.3) {
          state = previousState;
        }
      }

      case OPEN_LOOP_VOLTAGE_CONTROL -> {
        io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      }

      default -> {}
    }

    Logger.recordOutput(aKitTopic + "/At Goal", atPositionGoal());
    Logger.recordOutput(aKitTopic + "/State", state.name());

    Logger.recordOutput(
        aKitTopic + "/CRT Angle",
        calculateTurretAngle(io.getEncoder1Position(), io.getEncoder2Position()));

    Logger.recordOutput(
        aKitTopic + "/Pose",
        new Pose2d(
            robotPoseSupplier
                .get()
                .transformBy(
                    new Transform2d(
                        constants.robotToTurretTransform.getX(),
                        constants.robotToTurretTransform.getY(),
                        Rotation2d.kZero))
                .getTranslation(),
            inputs.angle.plus(robotPoseSupplier.get().getRotation())));
  }

  private Rotation2d unwrapTarget(Rotation2d current) {
    double min = constants.minAngle.getRadians();
    double max = constants.maxAngle.getRadians();
    double currentRad = current.getRadians();

    if (Math.abs(currentRad - max) < Math.abs(currentRad - min)) {
      return new Rotation2d(min);
    } else {
      return new Rotation2d(max);
    }
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

    return feedforwardController.calculate(
        fieldVelocity.omegaRadiansPerSecond + targetOmega); // still needs testing
  }

  public Rotation2d fieldToTurret(Pose2d robotPose) {
    Transform2d robotToTurretTransform =
        new Transform2d(
            constants.robotToTurretTransform.getX(),
            constants.robotToTurretTransform.getY(),
            Rotation2d.kZero);

    Pose2d turretPose = robotPose.transformBy(robotToTurretTransform);
    Translation2d turretToTarget = translationGoal.minus(turretPose.getTranslation());

    return turretToTarget.getAngle().minus(turretPose.getRotation());
  }

  public void setFieldRelativeGoal(Translation2d goal) {
    state = TurretState.CLOSED_LOOP_AUTO_AIM_CONTROL;
    translationGoal = goal;
  }

  public boolean outOfRange(Rotation2d angle) {
    return (!(angle.getDegrees() <= constants.maxAngle.getDegrees())
        || !(angle.getDegrees() >= constants.minAngle.getDegrees()));
  }

  public void setVoltageGoal(Voltage volts) {
    state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
    voltageGoal.setSetpoint(volts);
  }

  public void setPositionGoal(Rotation2d goal) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal.setSetpoint(goal.getMeasure());
  }

  public void setPositionGoal(Setpoint<AngleUnit> goal) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal = goal;
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> goal) {
    state = TurretState.OPEN_LOOP_VOLTAGE_CONTROL;
    voltageGoal = goal;
  }

  public void setPositionGoal(Rotation2d goal, AngularVelocity velocity) {
    state = TurretState.CLOSED_LOOP_POSITION_CONTROL;
    positionGoal.setSetpoint(goal.getMeasure());
    angularVelocityGoal.setSetpoint(velocity);
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public boolean atPositionGoal() {
    return io.atPositionGoal(
        nearestEquivalent((Rotation2d) positionGoal.getNewSetpoint(), inputs.angle));
  }

  public boolean atVoltageGoal() {
    return io.atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atPositionGoal));
  }

  public Command reset() {
    return Commands.runOnce(() -> setPositionGoal(new Rotation2d()))
        .andThen(Commands.runOnce(() -> setVoltageGoal(Volts.zero())))
        .finallyDo(() -> io.setPosition(new Rotation2d()));
  }

  public Rotation2d getPosition() {
    return inputs.angle;
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

  private Rotation2d calculateTurretAngle(Angle e1, Angle e2) {
    TurretConstants.TurretAngleCalculation gearRatios = constants.turretAngleCalculation;

    double e1Rot = e1.in(Rotations) % 1.0;
    if (e1Rot < 0) e1Rot += 1.0;

    double e2Rot = e2.in(Rotations) % 1.0;
    if (e2Rot < 0) e2Rot += 1.0;

    double diff = (e1Rot - e2Rot) % 1.0;
    if (diff < 0) diff += 1.0;

    double turretRot =
        diff
            * gearRatios.gear1ToothCount()
            * gearRatios.gear2ToothCount()
            / (double) gearRatios.gear0ToothCount();

    return Rotation2d.fromRotations(turretRot);
  }

  public Command runSysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> state = TurretState.IDLE),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> outOfRange(inputs.angle)),
        Commands.waitSeconds(3),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> outOfRange(inputs.angle)),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(3),
        characterizationRoutine.dynamic(Direction.kReverse));
  }
}
