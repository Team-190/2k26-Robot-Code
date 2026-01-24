package frc.robot.subsystems.v0_Funky;

import choreo.auto.AutoChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.v0_Funky.feeder.Feeder;
import frc.robot.subsystems.v0_Funky.feeder.FeederConstants;
import frc.robot.subsystems.v0_Funky.shooter.Shooter;
import frc.robot.subsystems.v0_Funky.shooter.ShooterConstants;
import java.util.List;

public class V0_FunkyRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Vision vision;

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
          shooter =
              new Shooter(
                  new GenericFlywheelIOTalonFX(ShooterConstants.SHOOTER_FLYWHEEL_CONSTANTS));
          feeder = new Feeder(new GenericRollerIOTalonFX(FeederConstants.FEEDER_CONSTANTS));
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraLimelight(
                      new CameraIOLimelight(V0_FunkyConstants.LIMELIGHT_CONFIG),
                      V0_FunkyConstants.LIMELIGHT_CONFIG,
                      V0_FunkyRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(),
                      List.of()));
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
          shooter =
              new Shooter(new GenericFlywheelIOSim(ShooterConstants.SHOOTER_FLYWHEEL_CONSTANTS));
          feeder = new Feeder(new GenericRollerIOSim(FeederConstants.FEEDER_CONSTANTS));
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

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

    if (shooter == null) {
      shooter = new Shooter(new GenericFlywheelIO() {});
    }

    if (feeder == null) {
      feeder = new Feeder(new GenericRollerIO() {});
    }

    if (vision == null) {
      new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            V0_FunkyConstants.DRIVE_CONSTANTS,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> driver.getRightX(),
            drive::getRawGyroRotation));

    driver
        .povDown()
        .onTrue(
            SharedCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));

    driver
        .rightTrigger(0.05)
        .whileTrue(
            Commands.run(
                () -> {
                  System.out.println("Speed: " + driver.getRightTriggerAxis());
                  shooter.setVoltage(12 * driver.getRightTriggerAxis());
                }))
        .whileFalse(Commands.runOnce(() -> shooter.setVoltage(0)));

    driver
        .leftBumper()
        .whileTrue(Commands.run(() -> feeder.setVoltage(-12.0)))
        .whileFalse(Commands.runOnce(() -> feeder.setVoltage(0)));
  }

  private void configureAutos() {
    // Autos here
  }

  @Override
  public void robotPeriodic() {
    V0_FunkyRobotState.periodic(drive.getRawGyroRotation(), drive.getModulePositions());
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
