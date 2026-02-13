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
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.v0_Funky.feeder.Feeder;
import frc.robot.subsystems.v0_Funky.feeder.FeederConstants;
import frc.robot.subsystems.v0_Funky.shooter.Shooter;
import frc.robot.subsystems.v0_Funky.shooter.ShooterConstants;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.util.XKeysInput;
import java.util.List;

public class V0_FunkyRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private Shooter shooter;
  private Feeder feeder;
  private Vision vision;
  private V1_DoomSpiralIntake intake;
  private GenericRollerIO topIO;
  private GenericRollerIO bottomIO;
  private FourBarLinkageIO linkageIO;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  private final XKeysInput xkeys = new XKeysInput(1);

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
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V0_FunkyRobotState::getGlobalPose,
                  V0_FunkyRobotState::resetPose);
          // shooter =
          //     new Shooter(
          //         new GenericFlywheelIOTalonFX(ShooterConstants.SHOOTER_FLYWHEEL_CONSTANTS));
          // feeder = new Feeder(new GenericRollerIOTalonFX(V0_FunkyConstants.FEED_CONSTANTS));
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
           intake = 
                new V1_DoomSpiralIntake(topIO, bottomIO, linkageIO);

          break;

        case V0_FUNKY_SIM:
          drive =
              new SwerveDrive(
                  V0_FunkyConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  () -> Pose2d.kZero,
                  V0_FunkyRobotState::resetPose);
          shooter = new Shooter(new GenericFlywheelIOSim(ShooterConstants.SHOOT_CONSTANTS));
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
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            V0_FunkyRobotState::getHeading));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));

    driver
        .rightTrigger(V0_FunkyConstants.TRIGGER_DEADBAND)
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

    xkeys
        .a1()
        .or(xkeys.a2())
        .or(xkeys.a3())
        .or(xkeys.a4())
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));

    xkeys
        .a5()
        .or(xkeys.a6())
        .or(xkeys.a7())
        .whileTrue(Commands.run(() -> shooter.setVoltage(V0_FunkyConstants.SHOOTER_VOLTAGE)))
        .whileFalse(Commands.run(() -> shooter.setVoltage(0)));

    xkeys
        .c8()
        .or(xkeys.c9())
        .or(xkeys.c10())
        .whileTrue(Commands.run(() -> feeder.setVoltage(V0_FunkyConstants.FEEDER_VOLTAGE)))
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
