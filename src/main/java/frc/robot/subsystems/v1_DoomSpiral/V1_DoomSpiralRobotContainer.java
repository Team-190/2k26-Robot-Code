package frc.robot.subsystems.v1_DoomSpiral;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
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
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
// import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.AdjustPathCommand.PathAdjustmentMode;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.autonomous.*;
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
import frc.robot.subsystems.v1_DoomSpiral.leds.V1_DoomSpiralCANdle;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooter;
import frc.robot.subsystems.v1_DoomSpiral.shooter.V1_DoomSpiralShooterConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexer;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerConstants;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIO;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFX;
import frc.robot.subsystems.v1_DoomSpiral.spindexer.V1_DoomSpiralSpindexerIOTalonFXSim;
import frc.robot.subsystems.v1_DoomSpiral.swank.*;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.input.XKeysInput;
import frc.robot.util.input.XboxElite2Input;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class V1_DoomSpiralRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private GyroIO gyroIO;
  private V1_DoomSpiralSwank swank;
  private Climber climber;
  private Intake intake;
  private V1_DoomSpiralSpindexer spindexer;
  private Vision vision;
  private V1_DoomSpiralCANdle leds;
  private V1_DoomSpiralShooter shooter;

  private final XboxElite2Input driver = new XboxElite2Input(0);
  private final XKeysInput xkeys = new XKeysInput(1);
  private final CommandXboxController operator = new CommandXboxController(2);

  private final BetterAutoChooser autoChooser;
  private final LoggedNetworkBoolean returnToMidChooser;

  private final NetworkTable pathAdjustmentTable =
      NetworkTableInstance.getDefault().getTable("PathAdjustmentModes");

  private final BooleanEntry leftBumpEntry =
      pathAdjustmentTable.getBooleanTopic("Left Bump").getEntry(false);
  private final BooleanEntry rightBumpEntry =
      pathAdjustmentTable.getBooleanTopic("Right Bump").getEntry(false);
  private final BooleanEntry leftTrenchEntry =
      pathAdjustmentTable.getBooleanTopic("Left Trench").getEntry(false);
  private final BooleanEntry rightTrenchEntry =
      pathAdjustmentTable.getBooleanTopic("Right Trench").getEntry(false);

  public V1_DoomSpiralRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V1_DOOMSPIRAL:
          gyroIO =
              new GyroIOPigeon2(
                  V1_DoomSpiralConstants.DRIVE_CONSTANTS,
                  V1_DoomSpiralRobotState::setHeadingUpdateTimestamp);
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
          climber =
              new Climber(
                  new ArmIOTalonFX(ClimberConstants.CLIMBER_CONSTANTS),
                  gyroIO.getRoll().asSupplier());
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V1_DoomSpiralRobotState.getLedStates()::setIntakeIn,
                      V1_DoomSpiralRobotState.getLedStates()::setIntakeCollecting,
                      V1_DoomSpiralRobotState.getLedStates()::setSpitting,
                      s -> {}));
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
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V1_DoomSpiralConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V1_DoomSpiralRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V1_DoomSpiralRobotState::getHeadingUpdateTimestamp,
                      List.of(V1_DoomSpiralRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_CLIMBER_CONFIG),
                      V1_DoomSpiralConstants.LIMELIGHT_CLIMBER_CONFIG,
                      V1_DoomSpiralRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V1_DoomSpiralRobotState::getHeadingUpdateTimestamp,
                      List.of(V1_DoomSpiralRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          // new CameraLimelight(
          // new CameraIOLimelight(V1_DoomSpiralConstants.LIMELIGHT_RIGHT_CONFIG),
          // V1_DoomSpiralConstants.LIMELIGHT_RIGHT_CONFIG,
          // V1_DoomSpiralRobotState::getHeading,
          // NetworkTablesJNI::now,
          // List.of(V1_DoomSpiralRobotState::addFieldLocalizerVisionMeasurement),
          // List.of()));
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
          // swank = new V1_DoomSpiralSwank(new V1_DoomSpiralSwankIOTalonFXSim());
          climber =
              new Climber(new ArmIOTalonFXSim(ClimberConstants.CLIMBER_CONSTANTS), Radians::zero);
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V1_DoomSpiralRobotState.getLedStates()::setIntakeIn,
                      V1_DoomSpiralRobotState.getLedStates()::setIntakeCollecting,
                      V1_DoomSpiralRobotState.getLedStates()::setSpitting,
                      s -> {}));
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
              new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark));
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
      climber = new Climber(new ArmIO() {}, Radians::zero);
    }

    if (intake == null) {
      intake =
          new Intake(
              new GenericRollerIO() {},
              new FourBarLinkageIO() {},
              driver.leftBumper(),
              new Intake.IntakeStateSetter());
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

    autoChooser = new BetterAutoChooser(V1_DoomSpiralRobotState::resetPose);
    returnToMidChooser = new LoggedNetworkBoolean("ReturnToNeutralZone", true);

    leftBumpEntry.set(false);
    rightBumpEntry.set(false);
    leftTrenchEntry.set(false);
    rightTrenchEntry.set(false);

    configureButtonBindings();
    configureAutos();
  }

  public Supplier<PathAdjustmentMode[]> getAdjustmentModeSupplier() {
    return () -> {
      List<PathAdjustmentMode> modes = new ArrayList<>();
      if (leftBumpEntry.get()) modes.add(PathAdjustmentMode.LEFT_BUMP);
      if (rightBumpEntry.get()) modes.add(PathAdjustmentMode.RIGHT_BUMP);
      if (leftTrenchEntry.get()) modes.add(PathAdjustmentMode.LEFT_TRENCH);
      if (rightTrenchEntry.get()) modes.add(PathAdjustmentMode.RIGHT_TRENCH);
      if (modes.isEmpty()) modes.add(PathAdjustmentMode.USE_ANY_AVAILABLE);
      return modes.toArray(new PathAdjustmentMode[0]);
    };
  }

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
                () -> V1_DoomSpiralRobotState.getRobotToHubAngle().getRadians(),
                () -> 0.0,
                driver.leftTrigger().or(driver.x()),
                driver.x())
            .withName("joystickDriveRotationLock"));

    driver
        .x()
        .or(driver.leftTrigger())
        .onTrue(
            Commands.runOnce(
                    () ->
                        DriveCommands.setLastCardinalDirection(
                            Math.round(
                                    V1_DoomSpiralRobotState.getHeading().getRadians()
                                        / (Math.PI / 2.0))
                                * (Math.PI / 2.0)))
                .withName("cardinal-direction-set"));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V1_DoomSpiralRobotState::resetPose,
                    () -> V1_DoomSpiralRobotState.getGlobalPose().getTranslation())
                .withName("driver-povDown-true"));

    driver.povUp().onTrue(drive.runOnce(() -> drive.stopWithX()).withName("driver-povUp"));

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
                    intake,
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
                    intake,
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
                    intake,
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
                    intake,
                    V1_DoomSpiralRobotState.FixedShots.TOWER.getParameters())
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V1_DoomSpiralCompositeCommands.stopShooterCommand(shooter, spindexer)
                .withName("driver-bottomRightPaddle-false"));

    xkeys.f4().onTrue(shooter.incrementFlywheelVelocity().withName("xkeys-f4-true"));
    xkeys.f5().onTrue(shooter.decrementFlywheelVelocity().withName("xkeys-f5-true"));
    xkeys.f6().onTrue(shooter.incrementHoodAngle().withName("xkeys-f6-true"));
    xkeys.f7().onTrue(shooter.decrementHoodAngle().withName("xkeys-f7-true"));

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
        .b4()
        .whileTrue(
            spindexer
                .setVoltage(V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE)
                .withName("xkeys-b4-while"))
        .onFalse(spindexer.setVoltage(0).withName("xkeys-b4-false"));

    xkeys
        .b5()
        .whileTrue(
            spindexer
                .setVoltage(-V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE)
                .withName("xkeys-b5-while"))
        .onFalse(spindexer.setVoltage(0).withName("xkeys-b5-false"));

    xkeys.b6().onTrue(spindexer.increaseSpindexerVoltage().withName("xkeys-b6-true"));
    xkeys.b7().onTrue(spindexer.decreaseSpindexerVoltage().withName("xkeys-b7-true"));
    xkeys.c6().onTrue(spindexer.increaseFeederVoltage().withName("xkeys-c6-true"));
    xkeys.c7().onTrue(spindexer.decreaseFeederVoltage().withName("xkeys-c7-true"));

    xkeys.h8().onTrue(climber.resetClimberZero().withName("xkeys-h8-true"));
    xkeys.h9().onTrue(intake.resetIntakeZero().withName("xkeys-h9-true"));
    xkeys.h10().whileTrue(shooter.zeroHood().withName("xkeys-h10-while"));

    xkeys.g8().whileTrue(drive.run(() -> drive.stopWithX()).withName("xkeys-g8"));

    xkeys
        .b10()
        .whileTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V1_DoomSpiralRobotState::resetPose,
                    V1_DoomSpiralRobotState.getGlobalPose()::getTranslation)
                .withName("xkeys-b10-while"));

    xkeys
        .b1()
        .onTrue(intake.stopRollerOverride().alongWith(intake.stow()).withName("xkeys-b1-true"));
    xkeys.b3().onTrue(intake.decrementStowOffset().withName("xkeys-b3-true"));
    xkeys.b2().onTrue(intake.incrementStowOffset().withName("xkeys-b2-true"));

    xkeys
        .c1()
        .onTrue(intake.deploy().alongWith(intake.stopRollerOverride()).withName("xkeys-c1-true"));
    xkeys.c3().onTrue(intake.decrementCollectOffset().withName("xkeys-c3-true"));
    xkeys.c2().onTrue(intake.incrementCollectOffset().withName("xkeys-c2-true"));

    xkeys
        .d1()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)
                .withName("xkeys-d1-while"))
        .onFalse(intake.stopRollerOverride().withName("xkeys-d1-false"));

    xkeys
        .d2()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(IntakeConstants.EXTAKE_VOLTAGE)
                .withName("xkeys-d2-while"))
        .onFalse(intake.stopRollerOverride().withName("xkeys-d2-false"));

    xkeys.e1().onTrue(intake.increaseSpeedOffset().withName("xkeys-e1-true"));
    xkeys.e2().onTrue(intake.decreaseSpeedOffset().withName("xkeys-e2-true"));

    xkeys
        .f1()
        .whileTrue(
            intake
                .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("xkeys-f1-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("xkeys-f1-false"));

    xkeys
        .f2()
        .whileTrue(
            intake
                .setLinkageVoltage(IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("xkeys-f2-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("xkeys-f2-false"));

    xkeys
        .g1()
        .or(xkeys.g2().or(xkeys.g3()))
        .whileTrue(intake.agitate().withName("xkeys-g1-g2-g3-while"));

    operator
        .leftTrigger()
        .onTrue(intake.stopRollerOverride().alongWith(intake.stow()).withName("op-lt"));
    operator
        .rightTrigger()
        .onTrue(intake.deploy().alongWith(intake.stopRollerOverride()).withName("op-rt"));
    operator
        .leftBumper()
        .whileTrue(
            intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE).withName("op-lb-while"))
        .onFalse(intake.stopRollerOverride().withName("op-lb-false"));
    operator
        .rightBumper()
        .whileTrue(
            intake.setOverrideRollerVoltage(IntakeConstants.EXTAKE_VOLTAGE).withName("op-rb-while"))
        .onFalse(intake.stopRollerOverride().withName("op-rb-false"));

    operator.povUp().onTrue(climber.setPositionDefault().withName("op-povUp"));
    operator.povDown().onTrue(climber.resetClimberZero().withName("op-povDown"));

    operator
        .povRight()
        .whileTrue(climber.clockwiseSlow().withName("op-povRight-while"))
        .onFalse(climber.setVoltage(0).withName("op-povRight-false"));

    operator
        .povLeft()
        .whileTrue(climber.counterClockwiseSlow().withName("op-povLeft-while"))
        .onFalse(climber.setVoltage(0).withName("op-povLeft-false"));

    operator.y().onTrue(shooter.incrementFlywheelVelocity().withName("op-y-true"));
    operator.x().onTrue(shooter.decrementFlywheelVelocity().withName("op-x-true"));
    operator.b().onTrue(shooter.incrementHoodAngle().withName("op-b-true"));
    operator.a().onTrue(shooter.decrementHoodAngle().withName("op-a-true"));

    operator
        .leftStick()
        .whileTrue(
            intake.setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE).withName("op-ls-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("op-ls-false"));

    operator
        .rightStick()
        .whileTrue(
            intake.setLinkageVoltage(IntakeConstants.LINKAGE_SLOW_VOLTAGE).withName("op-rs-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("op-rs-false"));

    operator.start().onTrue(intake.resetIntakeZero().withName("op-start"));

    operator
        .back()
        .whileTrue(
            spindexer
                .setVoltage(-V1_DoomSpiralSpindexerConstants.SPINDEXER_SLOW_VOLTAGE)
                .withName("op-back-while"))
        .onFalse(spindexer.setVoltage(0).withName("op-back-false"));
  }

  private void configureAutos() {
    new V1_DoomSpiralAutoTrajectoryCache();

    autoChooser.addRoutineConfig(
        "Left Trench Simple",
        V1_DoomSpiralAutoLeftTrenchSimple.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Left Trench Anti Bucks",
        V1_DoomSpiralAutoLeftTrenchAntiBucks.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Right Trench Simple",
        V1_DoomSpiralAutoRightTrenchSimple.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Right Trench Anti Bucks",
        V1_DoomSpiralAutoRightTrenchAntiBucks.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Depot And Back Hub",
        V1_DoomSpiralAutoDepotAndBackHub.getAutoRoutine(drive, intake, shooter, spindexer));
    autoChooser.addRoutineConfig(
        "Climb", V1_DoomSpiralAutoClimb.getAutoRoutine(drive, intake, shooter, spindexer, climber));
    autoChooser.addRoutineConfig(
        "Left Trench Simple Crosses",
        V1_DoomSpiralAutoLeftTrenchSimpleCrosses.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Right Trench Simple Crosses",
        V1_DoomSpiralAutoRightTrenchSimpleCrosses.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Left Trench Anti Bucks Crosses",
        V1_DoomSpiralAutoLeftTrenchAntiBucksCrosses.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));
    autoChooser.addRoutineConfig(
        "Right Trench Anti Bucks Crosses",
        V1_DoomSpiralAutoRightTrenchAntiBucksCrosses.getAutoRoutine(
            drive, intake, shooter, spindexer, returnToMidChooser, getAdjustmentModeSupplier()));

    SmartDashboard.putData("Autonomous Modes", autoChooser);

    RobotModeTriggers.autonomous()
        .negate()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      autoChooser.setResetPose(false);
                      V1_DoomSpiralRobotState.setAutoTrajectory();
                    })
                .ignoringDisable(true));
  }

  @Override
  public void robotPeriodic() {

    V1_DoomSpiralRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);

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
