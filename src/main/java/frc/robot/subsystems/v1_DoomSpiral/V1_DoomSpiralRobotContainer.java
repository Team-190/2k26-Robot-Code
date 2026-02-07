package frc.robot.subsystems.v1_DoomSpiral;

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
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOSim;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.autonomous.V1_DoomSpiralIntakeTest;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOTalonFX;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIO;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFX;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFXSim;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V1_DoomSpiralClimber climber;
  private V1_DoomSpiralIntake intake;
  private V1_DoomSpiralSpindexer spindexer;
  private Vision vision;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  public V1_DoomSpiralRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V1_DOOMSPIRAL:
          drive =
              new SwerveDrive(
                  V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(V1_DoomSpiralConstants.DRIVE_CONSTANTS),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  V1_DoomSpiralRobotState::getGlobalPose,
                  V1_DoomSpiralRobotState::resetPose);
          climber =
              new V1_DoomSpiralClimber(
                  new ArmIOTalonFX(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS));
          intake =
              new V1_DoomSpiralIntake(
                  new GenericRollerIOTalonFX(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new GenericRollerIOTalonFX(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_BOTTOM),
                  new FourBarLinkageIOTalonFX(V1_DoomSpiralIntakeConstants.LINKAGE_CONSTANTS));
          spindexer =
              new V1_DoomSpiralSpindexer(
                  new V1_DoomSpiralSpindexerIOTalonFX(),
                  new GenericRollerIOTalonFX(
                      V1_DoomSpiralSpindexerConstants.KICKER_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFX(
                      V1_DoomSpiralSpindexerConstants.FEEDER_ROLLER_CONSTANTS),
                  "Kicker",
                  "Feeder");
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraLimelight(
                      new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_CONFIG),
                      V1_DoomSpiralConstants.LIMELIGHT_CONFIG,
                      V1_DoomSpiralRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(V1_DoomSpiralRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()));
          break;

        case V1_DOOMSPIRAL_SIM:
          drive =
              new SwerveDrive(
                  V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  () -> Pose2d.kZero,
                  V1_DoomSpiralRobotState::resetPose);
          climber =
              new V1_DoomSpiralClimber(
                  new ArmIOSim(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS));
          intake =
              new V1_DoomSpiralIntake(
                  new GenericRollerIOSim(V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new GenericRollerIOSim(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_BOTTOM),
                  new FourBarLinkageIOSim(V1_DoomSpiralIntakeConstants.LINKAGE_CONSTANTS));
          spindexer =
              new V1_DoomSpiralSpindexer(
                  new V1_DoomSpiralSpindexerIOTalonFXSim(),
                  new GenericRollerIOSim(V1_DoomSpiralSpindexerConstants.KICKER_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(V1_DoomSpiralSpindexerConstants.FEEDER_ROLLER_CONSTANTS),
                  "Kicker",
                  "Feeder");
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
              V1_DoomSpiralConstants.DRIVE_CONSTANTS,
              new GyroIOPigeon2(V1_DoomSpiralConstants.DRIVE_CONSTANTS),
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V1_DoomSpiralRobotState::getGlobalPose,
              V1_DoomSpiralRobotState::resetPose);
    }

    if (climber == null) {
      climber = new V1_DoomSpiralClimber(new ArmIO() {});
    }

    if (intake == null) {
      intake =
          new V1_DoomSpiralIntake(
              new GenericRollerIO() {}, new GenericRollerIO() {}, new FourBarLinkageIO() {});
    }

    if (spindexer == null) {
      spindexer =
          new V1_DoomSpiralSpindexer(
              new V1_DoomSpiralSpindexerIO() {},
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              "Kicker",
              "Feeder");
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
            V1_DoomSpiralConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> driver.getRightX(),
            V1_DoomSpiralRobotState::getHeading));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V1_DoomSpiralRobotState::resetPose,
                () -> V1_DoomSpiralRobotState.getGlobalPose().getTranslation()));
  }

  private void configureAutos() {
    // Autos here
  }

  @Override
  public void robotPeriodic() {

    V1_DoomSpiralRobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions());

    Logger.recordOutput(
        "Mechanism 3d",
        V1_DoomSpiralMechanism3d.getPoses(
            spindexer.getSpindexerPosition(), climber.getArmPosition(), intake));
  }

  @Override
  public Command getAutonomousCommand() {
    return V1_DoomSpiralIntakeTest.testIntake(intake);
  }
}
