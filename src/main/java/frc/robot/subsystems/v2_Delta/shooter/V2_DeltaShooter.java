package frc.robot.subsystems.v2_Delta.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
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
import org.littletonrobotics.junction.Logger;

public class V2_DeltaShooter extends SubsystemBase {

  private final Turret turret;

  private final Hood hood;

  private final GenericFlywheel flywheel;

  private ShooterGoal shooterGoal;
  private Voltage overrideTurretVoltage;
  private Voltage overrideHoodVoltage;
  private Voltage overrideFlywheelVoltage;

  public V2_DeltaShooter(TurretIO turretIO, HoodIO hoodIO, GenericFlywheelIO flywheelIO) {
    setName("Shooter");

    flywheel = new GenericFlywheel(flywheelIO, this, V2_DeltaShooterConstants.SHOOT_CONSTANTS, "");
    hood = new Hood(hoodIO, V2_DeltaShooterConstants.HOOD_CONSTANTS, this, "");

    turret =
        new Turret(
            turretIO,
            this,
            "",
            V2_DeltaRobotState::getHubZonePose,
            ChassisSpeeds::new,
            V2_DeltaShooterConstants.TURRET_CONSTANTS);

    shooterGoal = ShooterGoal.STOW;
    overrideTurretVoltage = Volts.of(0.0);
    overrideHoodVoltage = Volts.of(0.0);
    overrideFlywheelVoltage = Volts.of(0.0);
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
              AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()));
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
          hood.setVoltageGoal(Volts.of(0.0));
          flywheel.setVoltageGoal(Volts.of(0.0));
          break;
        case OVERRIDE_HOOD:
          turret.setVoltageGoal(Volts.of(0.0));
          hood.setVoltageGoal(overrideHoodVoltage);
          flywheel.setVoltageGoal(Volts.of(0.0));
          break;
        case OVERRIDE_FLYWHEEL:
          turret.setVoltageGoal(Volts.of(0.0));
          hood.setVoltageGoal(Volts.of(0.0));
          flywheel.setVoltageGoal(overrideFlywheelVoltage);
          break;
        case SYSID:
        default:
          turret.setVoltageGoal(Volts.of(0.0));
          hood.setVoltageGoal(Volts.of(0.0));
          flywheel.setVoltageGoal(Volts.of(0.0));
          break;
      }

      turret.periodic();
      hood.periodic();
      flywheel.periodic();
    }

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

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atVelocityGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
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
    return Commands.sequence(preSysId(), turret.runSysIdRoutine());
  }

  public Command hoodSysId() {
    return Commands.sequence(preSysId(), hood.runSysIdRoutine());
  }

  public Command flywheelSysId() {
    return Commands.sequence(preSysId(), flywheel.sysIdRoutineVoltage());
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

  public Rotation2d getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheel.getFlywheelVelocity();
  }

  public boolean isTurretWrapping() {
    return turret.isWrapping();
  }
}
