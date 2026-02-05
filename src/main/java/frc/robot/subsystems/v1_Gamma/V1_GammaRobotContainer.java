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
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_Gamma.autonomous.V1_GammaIntakeTest;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntake;
import frc.robot.subsystems.v1_Gamma.intake.V1_GammaIntakeConstants;
import java.util.List;

public class V1_GammaRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private Vision vision;

  private V1_GammaIntake intake;

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
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

          intake =
              new V1_GammaIntake(
                  new GenericRollerIOSim(V1_GammaIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new GenericRollerIOSim(V1_GammaIntakeConstants.INTAKE_ROLLER_CONSTANTS_BOTTOM),
                  new FourBarLinkageIOSim(V1_GammaIntakeConstants.LINKAGE_CONSTANTS));

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
    // Autos here
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
