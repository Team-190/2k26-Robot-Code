package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class V0_FunkyRobotContainer implements RobotContainer {

  // Subsystems
  private SwerveDrive drive;
  private Roller intake;
  private Flywheel shooter;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Modes");

  public V0_FunkyRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V0_FUNKY:
          drive =
              new SwerveDrive(
                  new GyroIOPigeon2(),
                  new SwerveModuleIOTalonFX(0, SwerveModuleConstants.FRONT_LEFT),
                  new SwerveModuleIOTalonFX(1, SwerveModuleConstants.FRONT_RIGHT),
                  new SwerveModuleIOTalonFX(2, SwerveModuleConstants.BACK_LEFT),
                  new SwerveModuleIOTalonFX(3, SwerveModuleConstants.BACK_RIGHT));
          // TODO: add sup
          intake = new V0_FunkyRoller(new V0_FunkyRollerIOTalonFX());
          shooter = new V0_FunkyFlyweel(new V0_FunkyFlyweelIOTalonFX());
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  VisionConstants.RobotCameras.V0_FUNKY_CAMS);
          break;
        case V0_FUNKY_SIM:
          drive =
              new SwerveDrive(
                  new GyroIO() {},
                  new SwerveModuleIOSim(DriveConstants.FRONT_LEFT),
                  new SwerveModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new SwerveModuleIOSim(DriveConstants.BACK_LEFT),
                  new SwerveModuleIOSim(DriveConstants.BACK_RIGHT));
          roller = new Roller(new RollerIOTalonFX() {});
          vision =
              new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
          break;
        default:
          break;
      }
    }

    LTNUpdater.registerDrive(drive);
  }

  public void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> driver.b().getAsBoolean()));
    driver.y().onTrue(SharedCommands.resetHeading(drive));

    roller.setDefaultCommand(
        roller.runRoller(() -> driver.getLeftTriggerAxis(), () -> driver.getRightTriggerAxis()));

    driver.a().whileTrue(DriveCommands.autoAlignReefCoral(drive, vision.getCameras()));

    driver.povLeft().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    driver.povRight().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));
  }

  public void configureAutos() {
    autoChooser.addOption(
        "Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
  }

  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        vision.getCameras());

    LoggedTunableNumber.updateAll();
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
