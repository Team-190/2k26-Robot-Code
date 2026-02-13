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
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.*;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.autonomous.V1_DoomSpiralIntakeTest;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.hood.HoodIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimber;
import frc.robot.subsystems.v1_DoomSpiral.climber.V1_DoomSpiralClimberConstants;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntake;
import frc.robot.subsystems.v1_DoomSpiral.intake.V1_DoomSpiralIntakeConstants;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIO;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFX;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFXSim;
import frc.robot.subsystems.v1_DoomSpiral.swank.*;
import frc.robot.util.XKeysInput;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V1_DoomSpiralSwank swank;
  private V1_DoomSpiralClimber climber;
  private V1_DoomSpiralIntake intake;
  private V1_DoomSpiralSpindexer spindexer;
  private Vision vision;

  private V1_DoomSpiralShooter shooter;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  private final XKeysInput xkeys = new XKeysInput(1);

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
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V1_DoomSpiralRobotState::getGlobalPose,
                  V1_DoomSpiralRobotState::resetPose);
          swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIOTalonFX());
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

          shooter =
              new V1_DoomSpiralShooter(
                  new GenericFlywheelIOTalonFX(V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS),
                  new HoodIOTalonFX(V1_DoomSpiralShooterConstants.HOOD_CONSTANTS));

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
                  new SwerveModuleIOTalonFXSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFXSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFXSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFXSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  () -> Pose2d.kZero,
                  V1_DoomSpiralRobotState::resetPose);
          swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIOTalonFXSim());
          climber =
              new V1_DoomSpiralClimber(
                  new ArmIOTalonFXSim(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS));
          intake =
              new V1_DoomSpiralIntake(
                  new GenericRollerIOTalonFXSim(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new GenericRollerIOTalonFXSim(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_BOTTOM),
                  new FourBarLinkageIOSim(V1_DoomSpiralIntakeConstants.LINKAGE_CONSTANTS));
          spindexer =
              new V1_DoomSpiralSpindexer(
                  new V1_DoomSpiralSpindexerIOTalonFXSim(),
                  new GenericRollerIOTalonFXSim(
                      V1_DoomSpiralSpindexerConstants.KICKER_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFXSim(
                      V1_DoomSpiralSpindexerConstants.FEEDER_ROLLER_CONSTANTS),
                  "Kicker",
                  "Feeder");
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

          shooter =
              new V1_DoomSpiralShooter(
                  new GenericFlywheelIOTalonFXSim(V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS),
                  new HoodIOTalonFXSim(V1_DoomSpiralShooterConstants.HOOD_CONSTANTS));
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
    if (swank == null) {
      swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIO() {});
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

    if (shooter == null) {
      shooter = new V1_DoomSpiralShooter(new GenericFlywheelIO() {}, new HoodIO() {});
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

    driver.leftBumper().onTrue(intake.toggleIntake());

    driver.b().whileTrue(V1_DoomSpiralCompositeCommands.feedCommand(shooter, spindexer));

    // Climber button board commands
    xkeys.d8().onTrue(climber.setPositionDefault());

    xkeys.d9().onTrue(climber.setPositionL1());

    xkeys.d10().onTrue(climber.climbSequenceL3());

    xkeys.e8().whileTrue(climber.clockwiseSlow());

    xkeys.e9().whileTrue(climber.counterClockwiseSlow());

    xkeys.e10().onTrue(climber.runZeroSequence());

    // Spindexer button board commands
    xkeys
        .b5()
        .whileTrue(spindexer.setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE));
    xkeys
        .b6()
        .whileTrue(spindexer.setVoltage(-V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE));
    xkeys.c5().onTrue(spindexer.increaseSpindexerVoltage());
    xkeys.c6().onTrue(spindexer.decreaseSpindexerVoltage());
    xkeys.d5().onTrue(spindexer.increaseFeederVoltage());
    xkeys.d5().onTrue(spindexer.decreaseFeederVoltage());

    // Chassis button board commands
    xkeys
        .b8()
        .whileTrue(
            swank.setVoltage(
                -V1_DoomSpiralSwankConstants.SWANK_VOLTAGE)); // Left -> CW -> neg volts
    xkeys.b9().whileTrue(swank.setVoltage(V1_DoomSpiralSwankConstants.SWANK_VOLTAGE));
    xkeys
        .b10()
        .whileTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V1_DoomSpiralRobotState::resetPose,
                V1_DoomSpiralRobotState.getGlobalPose()::getTranslation));
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
