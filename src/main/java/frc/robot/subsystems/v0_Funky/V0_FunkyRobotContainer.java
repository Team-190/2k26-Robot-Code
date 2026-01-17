package frc.robot.subsystems.v0_Funky;

import choreo.auto.AutoChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.v0_Funky.feed.V0_Feed;
import frc.robot.subsystems.v0_Funky.shoot.V0_Shoot;
import java.util.List;

public class V0_FunkyRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private Vision vision;
  private V0_Feed feed;
  private V0_Shoot shoot;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  public V0_FunkyRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V0_FUNKY:
          drive =
              new SwerveDrive(
                  V0_FunkyConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(V0_FunkyConstants.DRIVE_CONSTANTS),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  V0_FunkyRobotState::getGlobalPose,
                  V0_FunkyRobotState::resetPose);
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraLimelight(
                      new CameraIOLimelight(V0_FunkyConstants.LIMELIGHT_CONFIG),
                      V0_FunkyConstants.LIMELIGHT_CONFIG,
                      V0_FunkyRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(V0_FunkyRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()));
          feed = new V0_Feed(new GenericRollerIOTalonFX(V0_FunkyConstants.FEED_CONSTANTS));
          shoot = new V0_Shoot(new GenericFlywheelIOTalonFX(V0_FunkyConstants.SHOOT_CONSTANTS));
          break;

        case V0_FUNKY_SIM:
          drive =
              new SwerveDrive(
                  V0_FunkyConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  () -> Pose2d.kZero,
                  V0_FunkyRobotState::resetPose);
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
          feed = new V0_Feed(new GenericRollerIOSim(V0_FunkyConstants.FEED_CONSTANTS));
          shoot = new V0_Shoot(new GenericFlywheelIOSim(V0_FunkyConstants.SHOOT_CONSTANTS));
          break;

        default:
          break;
      }
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              V0_FunkyConstants.DRIVE_CONSTANTS,
              new GyroIOPigeon2(V0_FunkyConstants.DRIVE_CONSTANTS),
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V0_FunkyRobotState::getGlobalPose,
              V0_FunkyRobotState::resetPose);
    }

    if (vision == null) {
      vision =
          new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
    }

    if (feed == null) {
      feed = new V0_Feed(new GenericRollerIO() {});
    }
    if (shoot == null) {
      shoot = new V0_Shoot(new GenericFlywheelIO() {});
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            V0_FunkyConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> driver.getRightX(),
            V0_FunkyRobotState::getHeading));

    feed.setDefaultCommand(feed.setVoltageCommand(() -> driver.getLeftTriggerAxis() * 7.0));
    shoot.setDefaultCommand(shoot.setVoltageCommand(() -> driver.getRightTriggerAxis() * 12.0));

    driver
        .povDown()
        .onTrue(
            SharedCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));
  }

  private void configureAutos() {
    // Autos here
  }

  @Override
  public void robotPeriodic() {

    V0_FunkyRobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
