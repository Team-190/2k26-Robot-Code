package frc.robot.subsystems.v2_Turnover.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shared.hood.Hood;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v2_Turnover.V2_TurnoverRobotState;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooterConstants.ShooterGoal;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class V2_TurnoverShooter extends SubsystemBase {

  private final Turret turret;

  private final Hood hood;

  private final GenericFlywheel flywheel;
  @Getter private ShooterGoal shooterGoal;
  private V2_TurnoverRobotState.FixedShotParameters fixedShotParameters;
  private Voltage overrideTurretVoltage;
  private Voltage overrideHoodVoltage;
  private Voltage overrideFlywheelVoltage;
  private final BooleanSupplier staticShooterSupplier;

  private final Setpoint<AngleUnit> hoodSetpoint;
  private final Setpoint<AngleUnit> hoodStowSetpoint;

  private final Trigger flywheelShootingTrigger;
  private final Trigger flywheelFeedingTrigger;
  private final Trigger hoodTuckTrigger;
  private final Trigger hoodFeedingTrigger;
  private final Trigger turretFeedingTrigger;

  private double flywheelVelocityThresholdRadPerSec;

  public V2_TurnoverShooter(
      TurretIO turretIO,
      HoodIO hoodIO,
      GenericFlywheelIO flywheelIO,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      BooleanSupplier staticShooterSupplier) {
    setName("Shooter");

    hoodSetpoint =
        new Setpoint<>(
            Radians.zero(),
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.offsetStep,
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.minAngle.getMeasure(),
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.maxAngle.getMeasure());
    hoodStowSetpoint =
        new Setpoint<>(
            Radians.zero(),
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.offsetStep,
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.minAngle.getMeasure(),
            V2_TurnoverShooterConstants.HOOD_CONSTANTS.maxAngle.getMeasure());

    flywheel =
        new GenericFlywheel(flywheelIO, this, V2_TurnoverShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V2_TurnoverShooterConstants.HOOD_CONSTANTS, this, "", hoodStowSetpoint);

    turret =
        new Turret(
            turretIO,
            this,
            "",
            V2_TurnoverRobotState::getLookaheadPose,
            chassisSpeedsSupplier,
            V2_TurnoverShooterConstants.TURRET_CONSTANTS);

    shooterGoal = ShooterGoal.STOW;
    overrideTurretVoltage = Volts.of(0.0);
    overrideHoodVoltage = Volts.of(0.0);
    overrideFlywheelVoltage = Volts.of(0.0);

    fixedShotParameters = V2_TurnoverRobotState.FixedShots.HUB.getParameters();

    flywheelVelocityThresholdRadPerSec =
        V2_TurnoverShooterConstants.SHOOT_CONSTANTS
            .constraints
            .goalTolerance()
            .get(RadiansPerSecond);

    flywheelShootingTrigger =
        new Trigger(
                () ->
                    Math.abs(
                            flywheel
                                .getFlywheelVelocity()
                                .minus(flywheel.getVelocityGoal().getNewSetpoint())
                                .in(RadiansPerSecond))
                        <= flywheelVelocityThresholdRadPerSec)
            .debounce(.75, Debouncer.DebounceType.kFalling);
    flywheelFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getFlywheelVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= (flywheelVelocityThresholdRadPerSec + 35));
    hoodTuckTrigger =
        new Trigger(V2_TurnoverRobotState::isShouldHoodTuck)
            .debounce(.5, Debouncer.DebounceType.kFalling);

    hoodFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        hood.getAngle()
                            .getMeasure()
                            .minus(hood.getPositionGoal().getNewSetpoint())
                            .in(Radians))
                    <= (V2_TurnoverShooterConstants.HOOD_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get(Radians)
                        + Units.degreesToRadians(5)));

    turretFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        (turret.getPosition().getMeasure().in(Radians) % (2 * Math.PI))
                            - (turret.getPositionGoal().getNewSetpoint().in(Radians)
                                % (2 * Math.PI)))
                    <= (V2_TurnoverShooterConstants.TURRET_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get(Radians)
                        + Units.degreesToRadians(10)));

    this.staticShooterSupplier = staticShooterSupplier;
  }

  @Trace
  public void periodic() {

    if (V2_TurnoverRobotState.isIntakeAtStow()) {
      hood.setPositionGoal(hoodStowSetpoint);
      turret.setVoltageGoal(Volts.zero());
      flywheel.stop();
    } else if (hoodTuckTrigger.getAsBoolean()) {
      hood.setPositionGoal(hoodStowSetpoint);
    } else {
      switch (shooterGoal) {
        case STOW:
          hood.setPositionGoal(hoodStowSetpoint);
          hood.setPositionGoal(Rotation2d.kZero);
          flywheel.stop();
          if (DriverStation.isAutonomous()) {
            turret.setFieldRelativeGoal(
                AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
                V2_TurnoverRobotState.getTurretVelocity());
          }
          break;
        case SCORE:
          hood.setPositionGoal(hoodSetpoint);
          if (!staticShooterSupplier.getAsBoolean())
            turret.setFieldRelativeGoal(
                AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
                V2_TurnoverRobotState.getTurretVelocity());
          else turret.setVoltageGoal(Volts.zero());
          hood.setPositionGoal(V2_TurnoverRobotState.getHoodAngle());
          flywheel.setVelocityGoal(V2_TurnoverRobotState.getFlywheelVelocity());
          break;
        case FEED:
          hood.setPositionGoal(hoodSetpoint);
          if (!staticShooterSupplier.getAsBoolean())
            turret.setFieldRelativeGoal(
                V2_TurnoverRobotState.getFeedTranslation(),
                V2_TurnoverRobotState.getTurretVelocity());
          else turret.setVoltageGoal(Volts.zero());
          hood.setPositionGoal(V2_TurnoverRobotState.getHoodAngle());
          flywheel.setVelocityGoal(V2_TurnoverRobotState.getFlywheelVelocity());
          break;
        case OVERRIDE_TURRET:
          turret.setVoltageGoal(overrideTurretVoltage);
          break;
        case OVERRIDE_HOOD:
          hood.setVoltageGoal(overrideHoodVoltage);
          break;
        case OVERRIDE_FLYWHEEL:
          flywheel.setVoltageGoal(overrideFlywheelVoltage);
          break;
        case FIXED_SHOTS:
          hood.setPositionGoal(hoodSetpoint);
          turret.setPositionGoal(
              turret.angleToGoal(
                  AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d()),
                  new Pose2d(
                      AllianceFlipUtil.apply(fixedShotParameters.robotPosition()),
                      V2_TurnoverRobotState.getHubZonePose().getRotation())));
          hood.setPositionGoal(fixedShotParameters.hoodAngle());
          flywheel.setVelocityGoal(fixedShotParameters.flywheelSpeed());
          break;
        case ZERO:
          turret.zero();
          hood.setPositionGoal(Rotation2d.kZero);
          flywheel.stop();
          break;
        case SYSID:
        default:
          break;
      }
    }

    hood.periodic();
    flywheel.periodic();
    turret.periodic();

    Logger.recordOutput(
        "Shooter/Fixed Shot/RobotPose",
        new Pose2d(
            fixedShotParameters.robotPosition(),
            V2_TurnoverRobotState.getGlobalPose().getRotation()));

    Logger.recordOutput("Shooter/Goal", shooterGoal);
    Logger.recordOutput(
        "Shooter/Pose",
        new Pose2d(
            V2_TurnoverRobotState.getHubZonePose()
                .transformBy(
                    new Transform2d(
                        V2_TurnoverShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getX(),
                        V2_TurnoverShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getY(),
                        Rotation2d.kZero))
                .getTranslation(),
            turret
                .getPosition()
                .plus(
                    V2_TurnoverShooterConstants.TURRET_CONSTANTS
                        .robotToTurretTransform
                        .getRotation()
                        .toRotation2d())
                .plus(V2_TurnoverRobotState.getGlobalPose().getRotation())));

    Logger.recordOutput(
        "Shooter/Flywheel Ready",
        V2_TurnoverRobotState.isInAllianceZone()
            ? flywheelShootingTrigger.getAsBoolean()
            : flywheelFeedingTrigger.getAsBoolean());
    Logger.recordOutput("Shooter/Should Hood Tuck", hoodTuckTrigger.getAsBoolean());

    Logger.recordOutput("Shooter/Static Turret", staticShooterSupplier.getAsBoolean());

    Logger.recordOutput(
        "Shooter/Turret At Goal",
        (V2_TurnoverRobotState.isInAllianceZone())
            ? turret.atPositionGoal()
            : turretFeedingTrigger.getAsBoolean());
    Logger.recordOutput(
        "Shooter/Hood At Goal",
        (V2_TurnoverRobotState.isInAllianceZone())
            ? hood.atPositionGoal()
            : hoodFeedingTrigger.getAsBoolean());

    Logger.recordOutput(
        "Elastic/Shooter/Flywheel/Velocity Magnitude",
        String.format("%.0f", Math.abs(flywheel.getFlywheelVelocity().in(RadiansPerSecond))));
    Logger.recordOutput(
        "Elastic/Shooter/Flywheel/Velocity Threshold",
        String.format("%.0f", flywheelVelocityThresholdRadPerSec));
    Logger.recordOutput(
        "Elastic/Shooter/Flywheel/Velocity Offset",
        String.format("%.0f", flywheel.getVelocityGoal().getOffset().in(RadiansPerSecond)));

    Logger.recordOutput(
        "Elastic/Shooter/Hood/Angle", String.format("%.1f", hood.getAngle().getDegrees()));
    Logger.recordOutput(
        "Elastic/Shooter/Hood/Angle Offset",
        String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
  }

  public Command setGoal(ShooterGoal shooterGoal) {
    return this.runOnce(() -> this.shooterGoal = shooterGoal);
  }

  /** NEVER use this anywhere but auto. It will clash with other autos! */
  public Command setNonRequiringGoal(ShooterGoal shooterGoal) {
    return Commands.runOnce(() -> this.shooterGoal = shooterGoal);
  }

  public Command setGoal(Supplier<ShooterGoal> goalSupplier) {
    return this.run(() -> this.shooterGoal = goalSupplier.get());
  }

  public Command runFixedShot(V2_TurnoverRobotState.FixedShots shot) {
    return setGoal(ShooterGoal.FIXED_SHOTS)
        .alongWith(Commands.runOnce(() -> fixedShotParameters = shot.getParameters()));
  }

  public boolean atGoal() {
    return (V2_TurnoverRobotState.isInAllianceZone()
        ? (flywheelShootingTrigger.getAsBoolean()
            && hood.atPositionGoal()
            && (turret.atPositionGoal() || staticShooterSupplier.getAsBoolean()))
        : (flywheelFeedingTrigger.getAsBoolean()
            && hoodFeedingTrigger.getAsBoolean()
            && (turret.atPositionGoal() || staticShooterSupplier.getAsBoolean())));
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  public Command waitUntilHoodAtGoal() {
    return hood.waitUntilAtGoal();
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command preSysId() {
    return Commands.runOnce(
        () -> {
          shooterGoal = ShooterGoal.SYSID;
          overrideTurretVoltage = Volts.of(0.0);
          overrideHoodVoltage = Volts.of(0.0);
          overrideFlywheelVoltage = Volts.of(0.0);
        });
  }

  public Command turretSysId() {
    return Commands.sequence(preSysId(), turret.runSysIdRoutine())
        .alongWith(
            Commands.run(
                () -> {
                  Logger.recordOutput(
                      "Shooter/Turret/Sysid/position", turret.getPosition().getRotations());

                  Logger.recordOutput(
                      "Shooter/Turret/Sysid/velocity", turret.getVelocity().in(RotationsPerSecond));
                }));
  }

  public Command hoodSysId() {
    return Commands.sequence(preSysId(), hood.runSysIdRoutine())
        .alongWith(
            Commands.run(
                () -> {
                  Logger.recordOutput(
                      "Shooter/Hood/Sysid/position", hood.getAngle().getRotations());
                  Logger.recordOutput(
                      "Shooter/Hood/Sysid/Velocity", hood.getVelocity().in(RotationsPerSecond));
                }));
  }

  public Command flywheelSysId() {
    return Commands.sequence(preSysId(), flywheel.sysIdRoutineVoltage())
        .alongWith(
            Commands.run(
                () -> {
                  Logger.recordOutput(
                      "Shooter/Flywheel/Sysid/position",
                      flywheel.getFlywheelPosition().getRotations());

                  Logger.recordOutput(
                      "Shooter/Flywheel/Sysid/velocity",
                      flywheel.getFlywheelVelocity().in(RotationsPerSecond));
                }));
  }

  public Command zero() {
    return setGoal(ShooterGoal.ZERO);
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementFlywheelVelocityThreshold() {
    return Commands.runOnce(() -> flywheelVelocityThresholdRadPerSec += 5);
  }

  public Command decrementFlywheelVelocityThreshold() {
    return Commands.runOnce(() -> flywheelVelocityThresholdRadPerSec -= 5);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hoodSetpoint::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hoodSetpoint::decrement);
  }

  public Rotation2d getTurretRotation() {
    return turret.getPosition();
  }

  public Rotation2d getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheel.getFlywheelVelocity();
  }

  public boolean isTurretWrapping() {
    return turret.isWrapping();
  }

  public Command resetTurretZero() {
    return Commands.runOnce(() -> turret.setPosition(new Rotation2d(0)));
  }

  public Command incrementTurretZero() {
    return Commands.runOnce(turret.getPositionGoal()::increment);
  }

  public Command decrementTurretZero() {
    return Commands.runOnce(turret.getPositionGoal()::decrement);
  }

  public Command resetHoodZero() {
    return hood.resetHoodZero();
  }

  public Command counterClockwiseSlow() {
    return Commands.runEnd(
        () -> {
          shooterGoal = ShooterGoal.OVERRIDE_TURRET;
          overrideTurretVoltage = Volts.of(2);
        },
        () -> overrideTurretVoltage = Volts.of(0));
  }

  public Command stopTurret() {
    return Commands.runOnce(
        () -> {
          overrideTurretVoltage = Volts.of(0);
          shooterGoal = ShooterGoal.OVERRIDE_TURRET;
        });
  }

  public Command clockwiseSlow() {
    return Commands.runEnd(
        () -> {
          shooterGoal = ShooterGoal.OVERRIDE_TURRET;
          overrideTurretVoltage = Volts.of(-2);
        },
        () -> overrideTurretVoltage = Volts.of(0));
  }

  public Command setHoodAngle(Rotation2d angle) {
    return setGoal(ShooterGoal.IDLE).andThen(Commands.runOnce(() -> hood.setPositionGoal(angle)));
  }

  public void setHoodGains(Gains hoodGains) {
    hood.setGains(hoodGains);
  }

  public void setHoodConstraints(AngularPositionConstraints constraints) {
    hood.setProfile(constraints);
  }

  public void setFlywheelGains(Gains flywheelGains) {
    flywheel.updateGains(flywheelGains, GainSlot.ZERO);
  }

  public void setFlywheelConstraints(AngularVelocityConstraints constraints) {
    flywheel.updateConstraints(constraints);
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return setGoal(ShooterGoal.IDLE)
        .andThen(Commands.runOnce(() -> flywheel.setVelocityGoal(velocity)));
  }

  public void setTurretGains(Gains turretGains) {
    turret.updateGains(turretGains);
  }

  public void setTurretConstraints(AngularPositionConstraints constraints) {
    turret.updateConstraints(constraints);
  }

  public Command setTurretGoal(Rotation2d goal) {
    return setGoal(ShooterGoal.IDLE)
        .andThen(Commands.runOnce(() -> turret.setPositionGoal(goal, RadiansPerSecond.zero())));
  }

  public Command waitUntilTurretAtGoal() {
    return turret.waitUntilAtGoal();
  }

  public Command stopAll() {
    return Commands.parallel(
        stopTurret(),
        Commands.runOnce(() -> hood.setVoltageGoal(Volts.zero())),
        Commands.runOnce(() -> flywheel.stop()));
  }
}
