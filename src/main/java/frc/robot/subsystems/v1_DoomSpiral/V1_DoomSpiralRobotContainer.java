package frc.robot.subsystems.v1_DoomSpiral;

import static edu.wpi.first.units.Units.Radians;

import choreo.auto.AutoChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.*;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.autonomous.*;
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
import frc.robot.subsystems.v1_DoomSpiral.leds.V1_DoomSpiralCANdle;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIO;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFX;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFXSim;
import frc.robot.subsystems.v1_DoomSpiral.swank.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.input.XKeysInput;
import frc.robot.util.input.XboxElite2Input;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class V1_DoomSpiralRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private GyroIO gyroIO;
  private V1_DoomSpiralSwank swank;
  private V1_DoomSpiralClimber climber;
  private V1_DoomSpiralIntake intake;
  private V1_DoomSpiralSpindexer spindexer;
  private Vision vision;
  private V1_DoomSpiralCANdle leds;
  private V1_DoomSpiralShooter shooter;

  private final XboxElite2Input driver = new XboxElite2Input(0);
  private final XKeysInput xkeys = new XKeysInput(1);

  private final AutoChooser autoChooser = new AutoChooser();

  public V1_DoomSpiralRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V1_DOOMSPIRAL:
          gyroIO = new GyroIOPigeon2(V1_DoomSpiralConstants.DRIVE_CONSTANTS);
          drive =
              new SwerveDrive(
                  V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                  gyroIO,
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
          //          swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIOTalonFX());
          climber =
              new V1_DoomSpiralClimber(
                  new ArmIOTalonFX(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS),
                  gyroIO.getRoll().asSupplier());
          intake =
              new V1_DoomSpiralIntake(
                  new GenericRollerIOTalonFX(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
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
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                  new CameraLimelight(
                      new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V1_DoomSpiralConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V1_DoomSpiralRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(V1_DoomSpiralRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()),
                  new CameraLimelight(
                      new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_CLIMBER_CONFIG),
                      V1_DoomSpiralConstants.LIMELIGHT_CLIMBER_CONFIG,
                      V1_DoomSpiralRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(V1_DoomSpiralRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()));
          //   new CameraLimelight(
          //       new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_RIGHT_CONFIG),
          //       V1_DoomSpiralConstants.LIMELIGHT_RIGHT_CONFIG,
          //       V1_DoomSpiralRobotState::getHeading,
          //       NetworkTablesJNI::now,
          //       List.of(V1_DoomSpiralRobotState::addFieldLocalizerVisionMeasurement),
          //       List.of()));
          break;

        case V1_DOOMSPIRAL_SIM:
          drive =
              new SwerveDrive(
                  V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                      V1_DoomSpiralConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V1_DoomSpiralRobotState::getGlobalPose,
                  V1_DoomSpiralRobotState::resetPose);
          //          swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIOTalonFXSim());
          climber =
              new V1_DoomSpiralClimber(
                  new ArmIOTalonFXSim(V1_DoomSpiralClimberConstants.CLIMBER_CONSTANTS),
                  Radians::zero);
          intake =
              new V1_DoomSpiralIntake(
                  new GenericRollerIOTalonFXSim(
                      V1_DoomSpiralIntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
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
          shooter =
              new V1_DoomSpiralShooter(
                  new GenericFlywheelIOTalonFXSim(V1_DoomSpiralShooterConstants.SHOOT_CONSTANTS),
                  new HoodIOTalonFXSim(V1_DoomSpiralShooterConstants.HOOD_CONSTANTS));
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
          leds = new V1_DoomSpiralCANdle();

          break;

        default:
          break;
      }
    }
    if (gyroIO == null) {
      gyroIO = new GyroIO() {};
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              V1_DoomSpiralConstants.DRIVE_CONSTANTS,
              gyroIO,
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
      climber = new V1_DoomSpiralClimber(new ArmIO() {}, Radians::zero);
    }

    if (intake == null) {
      intake = new V1_DoomSpiralIntake(new GenericRollerIO() {}, new FourBarLinkageIO() {});
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
      vision =
          new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
    }

    if (leds == null) {
      leds = new V1_DoomSpiralCANdle();
    }

    configureButtonBindings();
    configureAutos();
  }

  @Trace
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDriveRotationLock(
                drive,
                V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                V1_DoomSpiralRobotState::getHeading,
                driver.rightTrigger(),
                () ->
                    (AllianceFlipUtil.shouldFlip()
                            ? FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()
                            : FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
                        .getAngle()
                        .minus(Rotation2d.kCW_Pi_2)
                        .getRadians(),
                driver.leftTrigger(),
                () ->
                    Math.round(V1_DoomSpiralRobotState.getHeading().getRadians() / (Math.PI / 2.0))
                        * (Math.PI / 2.0),
                driver.x(),
                () ->
                    Math.round(V1_DoomSpiralRobotState.getHeading().getRadians() / (Math.PI / 2.0))
                        * (Math.PI / 2.0))
            .withName("driver-default-drive"));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V1_DoomSpiralRobotState::resetPose,
                    () -> V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
                .withName("driver-povDown-true"));

    driver.leftBumper().onTrue(intake.collect().withName("driver-leftBumper-true"));

    driver.a().onTrue(intake.stopCollect().withName("driver-a-true"));

    driver.rightBumper().whileTrue(intake.agitate().withName("driver-rightBumper-while"));

    driver
        .b()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.feedCommand(shooter, spindexer)
                .withName("driver-b-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-b-false"));

    driver
        .x()
        .onTrue(
            V1_DoomSpiralCompositeCommands.deployClimber(intake, climber)
                .withName("driver-x-true"));

    driver
        .y()
        .whileTrue(climber.climbSequenceL3().withName("driver-y-while"))
        .onFalse(climber.stop().withName("driver-y-false"));

    driver
        .rightTrigger()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.scoreCommand(shooter, intake, spindexer)
                .withName("driver-rightTrigger-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-rightTrigger-false"));

    driver
        .topLeftPaddle()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    spindexer,
                    V1_DoomSpiralRobotState.FixedShots.LEFT_TRENCH.getParameters())
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-topLeftPaddle-false"));

    driver
        .topRightPaddle()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    spindexer,
                    V1_DoomSpiralRobotState.FixedShots.RIGHT_TRENCH.getParameters())
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-topRightPaddle-false"));

    driver
        .bottomLeftPaddle()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    spindexer,
                    V1_DoomSpiralRobotState.FixedShots.HUB.getParameters())
                .withName("driver-bottomLeftPaddle-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-bottomLeftPaddle-false"));

    driver
        .bottomRightPaddle()
        .whileTrue(
            V1_DoomSpiralCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    spindexer,
                    V1_DoomSpiralRobotState.FixedShots.TOWER.getParameters())
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-bottomRightPaddle-false"));

    xkeys.f5().onTrue(shooter.incrementFlywheelVelocity().withName("xkeys-f5-true"));
    xkeys.f6().onTrue(shooter.decrementFlywheelVelocity().withName("xkeys-f6-true"));
    xkeys.g5().onTrue(shooter.incrementHoodAngle().withName("xkeys-g5-true"));
    xkeys.g6().onTrue(shooter.decrementHoodAngle().withName("xkeys-g6-true"));

    xkeys.d8().onTrue(climber.setPositionDefault().withName("xkeys-d8-true"));
    xkeys.d9().onTrue(climber.setPositionL1().withName("xkeys-d9-true"));
    xkeys.d10().onTrue(climber.climbSequenceL3().withName("xkeys-d10-true"));

    xkeys
        .e9()
        .whileTrue(climber.clockwiseSlow().withName("xkeys-e9-while"))
        .onFalse(climber.setVoltage(0).withName("xkeys-e9-false"));

    xkeys
        .e8()
        .whileTrue(climber.counterClockwiseSlow().withName("xkeys-e8-while"))
        .onFalse(climber.setVoltage(0).withName("xkeys-e8-false"));

    xkeys
        .e10()
        .onTrue(
            V1_DoomSpiralCompositeCommands.unClimbPostAuto(intake, climber)
                .withName("xkeys-e10-true"));

    xkeys
        .b5()
        .whileTrue(
            spindexer
                .setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE)
                .withName("xkeys-b5-while"))
        .onFalse(spindexer.setVoltage(0).withName("xkeys-b5-false"));

    xkeys
        .b6()
        .whileTrue(
            spindexer
                .setVoltage(-V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE)
                .withName("xkeys-b6-while"))
        .onFalse(spindexer.setVoltage(0).withName("xkeys-b6-false"));

    xkeys.c5().onTrue(spindexer.increaseSpindexerVoltage().withName("xkeys-c5-true"));
    xkeys.c6().onTrue(spindexer.decreaseSpindexerVoltage().withName("xkeys-c6-true"));
    xkeys.d5().onTrue(spindexer.increaseFeederVoltage().withName("xkeys-d5-true"));
    xkeys.d5().onTrue(spindexer.decreaseFeederVoltage().withName("xkeys-d5b-true"));

    xkeys.h8().onTrue(climber.resetClimberZero().withName("xkeys-h8-true"));
    xkeys.h9().onTrue(intake.resetIntakeZero().withName("xkeys-h9-true"));
    xkeys.h10().whileTrue(shooter.zeroHood().withName("xkeys-h10-while"));

    xkeys
        .b8()
        .whileTrue(
            swank
                .setVoltage(-V1_DoomSpiralSwankConstants.SWANK_VOLTAGE)
                .withName("xkeys-b8-while"));

    xkeys
        .b9()
        .whileTrue(
            swank.setVoltage(V1_DoomSpiralSwankConstants.SWANK_VOLTAGE).withName("xkeys-b9-while"));

    xkeys
        .b10()
        .whileTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V1_DoomSpiralRobotState::resetPose,
                    V1_DoomSpiralRobotState.getGlobalPose()::getTranslation)
                .withName("xkeys-b10-while"));

    xkeys.b1().onTrue(intake.stopRoller().alongWith(intake.stow()).withName("xkeys-b1-true"));
    xkeys.b3().onTrue(intake.decrementStowOffset().withName("xkeys-b3-true"));
    xkeys.b2().onTrue(intake.incrementStowOffset().withName("xkeys-b2-true"));

    xkeys.c1().onTrue(intake.bump().alongWith(intake.stopRoller()).withName("xkeys-c1-true"));
    xkeys.c3().onTrue(intake.decrementBumpOffset().withName("xkeys-c3-true"));
    xkeys.c2().onTrue(intake.incrementBumpOffset().withName("xkeys-c2-true"));

    xkeys.d1().onTrue(intake.deploy().alongWith(intake.stopRoller()).withName("xkeys-d1-true"));
    xkeys.d3().onTrue(intake.decrementCollectOffset().withName("xkeys-d3-true"));
    xkeys.d2().onTrue(intake.incrementCollectOffset().withName("xkeys-d2-true"));

    xkeys
        .e1()
        .whileTrue(
            intake
                .setRollerVoltage(V1_DoomSpiralIntakeConstants.INTAKE_VOLTAGE)
                .withName("xkeys-e1-while"))
        .onFalse(intake.stopRoller().withName("xkeys-e1-false"));

    xkeys
        .e2()
        .whileTrue(
            intake
                .setRollerVoltage(V1_DoomSpiralIntakeConstants.EXTAKE_VOLTAGE)
                .withName("xkeys-e2-while"))
        .onFalse(intake.stopRoller().withName("xkeys-e2-false"));

    xkeys.f1().onTrue(intake.increaseSpeedOffset().withName("xkeys-f1-true"));
    xkeys.f2().onTrue(intake.decreaseSpeedOffset().withName("xkeys-f2-true"));

    xkeys
        .g1()
        .whileTrue(
            intake
                .setLinkageVoltage(-V1_DoomSpiralIntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("xkeys-g1-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("xkeys-g1-false"));

    xkeys
        .g2()
        .whileTrue(
            intake
                .setLinkageVoltage(V1_DoomSpiralIntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("xkeys-g2-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("xkeys-g2-false"));

    xkeys
        .h1()
        .or(xkeys.h2().or(xkeys.h3()))
        .whileTrue(intake.agitate().withName("xkeys-h1-h2-h3-while"));
  }

  private void configureAutos() {
    new V1_DoomSpiralAutoTrajectoryCache();

    autoChooser.addRoutine(
        "Left Trench Simple",
        () ->
            V1_DoomSpiralAutoLeftTrenchSimple.getAutoRoutine(
                drive, intake, shooter, spindexer, climber));
    autoChooser.addRoutine(
        "Left Trench Anti Bucks",
        () ->
            V1_DoomSpiralAutoLeftTrenchAntiBucks.getAutoRoutine(
                drive, intake, shooter, spindexer, climber));
    autoChooser.addRoutine(
        "Right Trench Simple",
        () ->
            V1_DoomSpiralAutoRightTrenchSimple.getAutoRoutine(
                drive, intake, shooter, spindexer, climber));
    autoChooser.addRoutine(
        "Depot And Back Hub",
        () ->
            V1_DoomSpiralAutoDepotAndBackHub.getAutoRoutine(
                drive, intake, shooter, spindexer, climber));
    autoChooser.addRoutine(
        "Climb",
        () -> V1_DoomSpiralAutoClimb.getAutoRoutine(drive, intake, shooter, spindexer, climber));

    SmartDashboard.putData("Autonomous Modes", autoChooser);
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
    return autoChooser.selectedCommand();
  }
}
