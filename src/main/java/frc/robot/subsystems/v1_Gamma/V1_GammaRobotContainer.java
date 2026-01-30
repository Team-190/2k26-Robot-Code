package frc.robot.subsystems.v1_Gamma;

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
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_Gamma.autonomous.V1_GammaAutonomousTest;
import frc.robot.subsystems.shared.turret.Turret;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.v1_Gamma.spindexer.V1_GammaSpindexer;
import frc.robot.subsystems.v1_Gamma.spindexer.V1_GammaSpindexerIOTalonFXSim;
import frc.robot.subsystems.v1_Gamma.swank.V1_GammaSwank;
import frc.robot.subsystems.v1_Gamma.swank.V1_GammaSwankIOTalonFX;
import frc.robot.subsystems.v1_Gamma.swank.V1_GammaSwankIOTalonFXSim;
import java.util.List;

public class V1_GammaRobotContainer implements RobotContainer {

  private SwerveDrive drive;
  private Vision vision;

  private V1_GammaSpindexer spindexer;
  private Turret turret;
  private TurretIO turretIO;
  private V1_GammaSwank swank;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  public V1_GammaRobotContainer() {

    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V1_GAMMA:
          drive =
              new SwerveDrive(
                  V1_GammaConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(V1_GammaConstants.DRIVE_CONSTANTS),
                  new SwerveModuleIOTalonFX(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOTalonFX(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOTalonFX(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOTalonFX(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  V1_GammaRobotState::getGlobalPose,
                  V1_GammaRobotState::resetPose);
          swank = new V1_GammaSwank(new V1_GammaSwankIOTalonFX());
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraLimelight(
                      new CameraIOLimelight(V1_GammaConstants.LIMELIGHT_CONFIG),
                      V1_GammaConstants.LIMELIGHT_CONFIG,
                      V1_GammaRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(V1_GammaRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()));
          break;

        case V1_GAMMA_SIM:
          drive =
              new SwerveDrive(
                  V1_GammaConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOSim(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOSim(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOSim(
                      V1_GammaConstants.DRIVE_CONSTANTS,
                      V1_GammaConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  () -> Pose2d.kZero,
                  V1_GammaRobotState::resetPose);

          spindexer = new V1_GammaSpindexer(new V1_GammaSpindexerIOTalonFXSim());
          swank = new V1_GammaSwank(new V1_GammaSwankIOTalonFXSim());
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
              V1_GammaConstants.DRIVE_CONSTANTS,
              new GyroIOPigeon2(V1_GammaConstants.DRIVE_CONSTANTS),
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V1_GammaRobotState::getGlobalPose,
              V1_GammaRobotState::resetPose);
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
            V1_GammaConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> driver.getRightX(),
            V1_GammaRobotState::getHeading));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V1_GammaRobotState::resetPose,
                () -> V1_GammaRobotState.getGlobalPose().getTranslation()));
  }

  private void configureAutos() {
    // add autos here
  }

  @Override
  public void robotPeriodic() {

    V1_GammaRobotState.periodic(
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
