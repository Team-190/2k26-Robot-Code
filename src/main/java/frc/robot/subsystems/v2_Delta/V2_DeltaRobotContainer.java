package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOSim;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.AdjustPathCommand;
import frc.robot.commands.v2_Delta.autonomous.V2_TurretTestAuto;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.hood.HoodIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.shared.turret.TurretIOSim;
import frc.robot.subsystems.shared.turret.TurretIOTalonFX;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;
import frc.robot.subsystems.v2_Delta.leds.V2_DeltaCANdle;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.util.BetterAutoChooser;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;

public class V2_DeltaRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private Climber climber;
  private Intake intake;
  private V2_DeltaClopper clopper;
  private Vision vision;
  private V2_DeltaCANdle leds;
  private V2_DeltaShooter shooter;

  private final BetterAutoChooser autoChooser;

  public V2_DeltaRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_DELTA:
          gyroIO =
              new GyroIOPigeon2(
                  V2_DeltaConstants.DRIVE_CONSTANTS, V2_DeltaRobotState::setHeadingUpdateTimestamp);
          drive =
              new SwerveDrive(
                  V2_DeltaConstants.DRIVE_CONSTANTS,
                  gyroIO,
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_DeltaRobotState::getGlobalPose,
                  V2_DeltaRobotState::resetPose);
          climber =
              new Climber(
                  new ArmIOTalonFX(ClimberConstants.CLIMBER_CONSTANTS),
                  gyroIO.getRoll().asSupplier());
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS));
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          shooter =
              new V2_DeltaShooter(
                  new TurretIOTalonFX(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFX(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFX(V2_DeltaShooterConstants.SHOOT_CONSTANTS));
          leds = new V2_DeltaCANdle();

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          break;
        case V2_DELTA_SIM:
          drive =
              new SwerveDrive(
                  V2_DeltaConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_DeltaRobotState::getGlobalPose,
                  V2_DeltaRobotState::resetPose);
          climber = new Climber(new ArmIOSim(ClimberConstants.CLIMBER_CONSTANTS), Radians::zero);
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS));
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          leds = new V2_DeltaCANdle();
          shooter =
              new V2_DeltaShooter(
                  new TurretIOSim(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFXSim(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFXSim(V2_DeltaShooterConstants.SHOOT_CONSTANTS));
          break;
        default:
      }
    }
    if (gyroIO == null) {
      gyroIO = new GyroIO() {};
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              V2_DeltaConstants.DRIVE_CONSTANTS,
              gyroIO,
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V2_DeltaRobotState::getGlobalPose,
              V2_DeltaRobotState::resetPose);
    }
    if (climber == null) {
      climber = new Climber(new ArmIO() {}, Radians::zero);
    }
    if (intake == null) {
      intake = new Intake(new GenericRollerIO() {}, new FourBarLinkageIO() {});
    }
    if (clopper == null) {
      clopper = new V2_DeltaClopper(new GenericRollerIO() {}, new GenericRollerIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }
    if (leds == null) {
      leds = new V2_DeltaCANdle();
    }
    if (shooter == null) {
      shooter = new V2_DeltaShooter(new TurretIO() {}, new HoodIO() {}, new GenericFlywheelIO() {});
    }

    autoChooser = new BetterAutoChooser(V2_DeltaRobotState::resetPose);
    configureAutos();
  }

  private void configureButtonBindings() {
    //
  }

  private void configureAutos() {
    autoChooser.addRoutineConfig("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive, shooter));
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    V2_DeltaRobotState.periodic(
        drive.getRawGyroRotation(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        shooter.getTurretRotation(),
        shooter.isTurretWrapping());
  }

  @Override
  public Command getAutonomousCommand() {
    AdjustPathCommand adjustPathCommand = null;
    try {
      adjustPathCommand =
          new AdjustPathCommand(
              () -> new Pose2d(new Translation2d(3.45, 4.23), Rotation2d.fromDegrees(40)),
              1.0,
              2.0,
              drive);
    } catch (IOException e) {
      e.printStackTrace();
    } catch (ParseException e) {
      e.printStackTrace();
    }

    AdjustPathCommand finalAdjustPathCommand = adjustPathCommand;

    return Commands.sequence(
        Commands.runOnce(
            () ->
                V2_DeltaRobotState.resetPose(
                    new Pose2d(new Translation2d(4.5, 6.7), Rotation2d.fromDegrees(90)))),
        Commands.print("Finished Reset Pose"),
        finalAdjustPathCommand.withTimeout(3),
        Commands.print("Finished Adjust Path Command"));
    // return autoChooser.selectedCommand();
  }
}
