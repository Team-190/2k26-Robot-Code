package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
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
import frc.robot.subsystems.v2_Delta.V2_DeltaRobotState;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants.ShooterGoal;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class V2_DeltaShooter extends SubsystemBase {

  private final Turret turret;

  private final Hood hood;

  private final GenericFlywheel flywheel;

  private ShooterGoal shooterGoal;
  private V2_DeltaRobotState.FixedShotParameters fixedShotParameters;
  private Voltage overrideTurretVoltage;
  private Voltage overrideHoodVoltage;
  private Voltage overrideFlywheelVoltage;

  public V2_DeltaShooter(
      TurretIO turretIO,
      HoodIO hoodIO,
      GenericFlywheelIO flywheelIO,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    setName("Shooter");

    flywheel = new GenericFlywheel(flywheelIO, this, V2_DeltaShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V2_DeltaShooterConstants.HOOD_CONSTANTS, this, "");

    turret =
        new Turret(
            turretIO,
            this,
            "",
            V2_DeltaRobotState::getLookaheadPose,
            chassisSpeedsSupplier,
            V2_DeltaShooterConstants.TURRET_CONSTANTS);

    shooterGoal = ShooterGoal.STOW;
    overrideTurretVoltage = Volts.of(0.0);
    overrideHoodVoltage = Volts.of(0.0);
    overrideFlywheelVoltage = Volts.of(0.0);

    fixedShotParameters =
        new V2_DeltaRobotState.FixedShotParameters(
            Rotation2d.kZero, Rotation2d.kZero, RadiansPerSecond.of(0));
  }

  @Trace
  public void periodic() {
    if (V2_DeltaRobotState.isShouldHoodTuck()) {
      hood.setPositionGoal(Rotation2d.kZero);
    } else {
      switch (shooterGoal) {
        case STOW:
          hood.setPositionGoal(Rotation2d.kZero);
          flywheel.stop();
          break;
        case SCORE:
          turret.setFieldRelativeGoal(
              AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
              V2_DeltaRobotState.getTurretVelocity());
          hood.setPositionGoal(V2_DeltaRobotState.getScoreAngle());
          flywheel.setVelocityGoal(V2_DeltaRobotState.getScoreVelocity());
          break;
        case FEED:
          turret.setFieldRelativeGoal(V2_DeltaRobotState.getFeedTranslation());
          hood.setPositionGoal(V2_DeltaRobotState.getFeedAngle());
          flywheel.setVelocityGoal(V2_DeltaRobotState.getFeedVelocity());
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
          turret.setPositionGoal(fixedShotParameters.turretAngle());
          hood.setPositionGoal(fixedShotParameters.hoodAngle());
          flywheel.setVelocityGoal(fixedShotParameters.flywheelSpeed());
          break;
        case SYSID:
        default:
          break;
      }
    }

    turret.periodic();
    hood.periodic();
    flywheel.periodic();

    Logger.recordOutput("Shooter/Goal", shooterGoal);
    Logger.recordOutput(
        "Shooter/Pose2d",
        new Pose2d(
            V2_DeltaRobotState.getGlobalPose()
                .transformBy(
                    new Transform2d(
                        V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getX(),
                        V2_DeltaShooterConstants.TURRET_CONSTANTS.robotToTurretTransform.getY(),
                        Rotation2d.kZero))
                .getTranslation(),
            turret.getPosition().plus(V2_DeltaRobotState.getGlobalPose().getRotation())));
  }

  public Command setGoal(ShooterGoal shooterGoal) {
    return Commands.runOnce(() -> this.shooterGoal = shooterGoal);
  }

  public Command setGoal(Supplier<ShooterGoal> goalSupplier) {
    return Commands.run(() -> this.shooterGoal = goalSupplier.get());
  }

  public Command runFixedShot(V2_DeltaRobotState.FixedShots shot) {
    return setGoal(ShooterGoal.FIXED_SHOTS)
        .andThen(Commands.runOnce(() -> fixedShotParameters = shot.getParameters()));
  }

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atVelocityGoal() && turret.atPositionGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal()
        .alongWith(flywheel.waitUntilAtGoal())
        .alongWith(turret.waitUntilAtGoal());
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
                      "Shooter/Hood/Sysid/position", hood.getPosition().getRotations());
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

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::decrement);
  }

  public Rotation2d getTurretRotation() {
    return turret.getPosition();
  }

  public boolean isTurretWrapping() {
    return turret.isWrapping();
  }

  public Command resetTurretZero() {
    return Commands.runOnce(() -> turret.setPosition(new Rotation2d(0)));
  }

  public Command incrementTurretZero() {
    return Commands.runOnce(
        () ->
            turret.setPosition(
                turret.getPosition().plus(new Rotation2d(Units.degreesToRadians(1)))));
  }

  public Command decrementTurretZero() {
    return Commands.runOnce(
        () ->
            turret.setPosition(
                turret.getPosition().minus(new Rotation2d(Units.degreesToRadians(1)))));
  }

  public Command resetHoodZero() {
    return hood.resetHoodZero();
  }

  public Command counterClockwiseSlow() {
    return Commands.runEnd(
        () -> {
          shooterGoal = ShooterGoal.OVERRIDE_TURRET;
          overrideTurretVoltage = Volts.of(3);
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
          overrideTurretVoltage = Volts.of(-3);
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
    return setGoal(ShooterGoal.IDLE).andThen(Commands.runOnce(() -> turret.setPositionGoal(goal)));
  }

  public Command waitUntilTurretAtGoal() {
    return turret.waitUntilAtGoal();
  }
}
