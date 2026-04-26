package frc.robot.subsystems.v2_TurnOver;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.AdjustPathCommand.PathAdjustmentMode;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v2_TurnOver.V2_TurnOverCompositeCommands;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoDepot;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoFollowDepot;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoFollowFeedFullLeft;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoFollowFeedFullRight;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoFollowFeedMiddleLeft;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoFollowFeedMiddleRight;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoLeftHalf;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoLeftOP;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoLeftOPBucks;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoRightHalf;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoRightOP;
import frc.robot.commands.v2_TurnOver.autonomous.V2_TurnOverAutoRightOPBucks;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.hood.HoodIOTalonFX;
import frc.robot.subsystems.shared.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.shared.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.shared.turret.TurretIOSim;
import frc.robot.subsystems.shared.turret.TurretIOTalonFX;
import frc.robot.subsystems.v2_TurnOver.clopper.V2_TurnOverClopper;
import frc.robot.subsystems.v2_TurnOver.clopper.V2_TurnOverClopperConstants;
import frc.robot.subsystems.v2_TurnOver.leds.V2_TurnOverCANdle;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooter;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverShooterConstants;
import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverSimFuelCount;
// import frc.robot.subsystems.v2_TurnOver.shooter.V2_TurnOverSimFuelCount;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FuelSimulator;
import frc.robot.util.LTNUpdater;
import frc.robot.util.command.ContinuousConditionalCommand;
import frc.robot.util.input.XKeysInput;
import frc.robot.util.input.XboxElite2Input;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class V2_TurnOverRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private V2_TurnOverClopper clopper;
  private Vision vision;
  private V2_TurnOverShooter shooter;
  private Intake intake;
  private FuelSimulator fuelSimulator;
  private V2_TurnOverSimFuelCount simFuelCount;

  private boolean staticShooter = false;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);
  private final XKeysInput xkeys = new XKeysInput(1);

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

  public V2_TurnOverRobotContainer() {

    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_TURNOVER:
          gyroIO =
              new GyroIOPigeon2(
                  V2_TurnOverConstants.DRIVE_CONSTANTS, V2_TurnOverRobotState::setHeadingUpdateTimestamp);
          drive =
              new SwerveDrive(
                  V2_TurnOverConstants.DRIVE_CONSTANTS,
                  gyroIO,
                  new SwerveModuleIOTalonFX(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_TurnOverRobotState::getGlobalPose,
                  V2_TurnOverRobotState::resetPose);
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V2_TurnOverRobotState.getLedStates()::setIntakeIn,
                      V2_TurnOverRobotState.getLedStates()::setIntakeCollecting,
                      s -> {},
                      V2_TurnOverRobotState.getLedStates()::setIntakeSlowRolling));
          clopper =
              new V2_TurnOverClopper(
                  new GenericRollerIOTalonFX(V2_TurnOverClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_TurnOverClopperConstants.BALL_TUNNEL_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_TurnOverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS));
          shooter =
              new V2_TurnOverShooter(
                  new TurretIOTalonFX(V2_TurnOverShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFX(V2_TurnOverShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFX(V2_TurnOverShooterConstants.SHOOT_CONSTANTS),
                  drive::getMeasuredChassisSpeeds,
                  () -> staticShooter);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnOverConstants.LIMELIGHT_INTAKE_CONFIG),
                      V2_TurnOverConstants.LIMELIGHT_INTAKE_CONFIG,
                      V2_TurnOverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnOverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnOverRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnOverConstants.LIMELIGHT_LEFT_CONFIG),
                      V2_TurnOverConstants.LIMELIGHT_LEFT_CONFIG,
                      V2_TurnOverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnOverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnOverRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnOverConstants.LIMELIGHT_RIGHT_CONFIG),
                      V2_TurnOverConstants.LIMELIGHT_RIGHT_CONFIG,
                      V2_TurnOverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnOverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnOverRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          new V2_TurnOverCANdle();
          break;
        case V2_TURNOVER_SIM:
          drive =
              new SwerveDrive(
                  V2_TurnOverConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V2_TurnOverConstants.DRIVE_CONSTANTS,
                      V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_TurnOverRobotState::getGlobalPose,
                  V2_TurnOverRobotState::resetPose);
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V2_TurnOverRobotState.getLedStates()::setIntakeIn,
                      V2_TurnOverRobotState.getLedStates()::setIntakeCollecting,
                      s -> {},
                      V2_TurnOverRobotState.getLedStates()::setIntakeSlowRolling));
          clopper =
              new V2_TurnOverClopper(
                  new GenericRollerIOTalonFXSim(V2_TurnOverClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_TurnOverClopperConstants.BALL_TUNNEL_CONSTANTS),
                  new GenericRollerIOTalonFXSim(
                      V2_TurnOverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS));

          shooter =
              new V2_TurnOverShooter(
                  new TurretIOSim(V2_TurnOverShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFXSim(V2_TurnOverShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFXSim(V2_TurnOverShooterConstants.SHOOT_CONSTANTS),
                  drive::getMeasuredChassisSpeeds,
                  () -> staticShooter);
          vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
          break;
        default:
      }
    }
    if (gyroIO == null) {
      gyroIO = new GyroIO() {};
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              V2_TurnOverConstants.DRIVE_CONSTANTS,
              gyroIO,
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V2_TurnOverRobotState::getGlobalPose,
              V2_TurnOverRobotState::resetPose);
    }
    if (intake == null) {
      intake =
          new Intake(
              new GenericRollerIO() {},
              new FourBarLinkageIO() {},
              () -> false,
              new Intake.IntakeStateSetter());
    }
    if (clopper == null) {
      clopper =
          new V2_TurnOverClopper(
              new GenericRollerIO() {}, new GenericRollerIO() {}, new GenericRollerIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }
    if (shooter == null) {
      shooter =
          new V2_TurnOverShooter(
              new TurretIO() {},
              new HoodIO() {},
              new GenericFlywheelIO() {},
              drive::getMeasuredChassisSpeeds,
              () -> false);
    }

    leftBumpEntry.set(false);
    rightBumpEntry.set(false);
    leftTrenchEntry.set(false);
    rightTrenchEntry.set(false);

    autoChooser = new LoggedDashboardChooser<>("Autonomous Modes");
    configureButtonBindings();
    configureAutos();
    configureFuelSim();
    if (Constants.TUNING_MODE) LTNUpdater.registerV2(drive, intake, shooter);
  }

  private void configureFuelSim() {

    fuelSimulator = new FuelSimulator("FuelSim");
    if (RobotBase.isSimulation()) {
      simFuelCount = new V2_TurnOverSimFuelCount(8);

      fuelSimulator.registerRobot(
          V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth(),
          V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.bumperLength(),
          Units.inchesToMeters(6.0),
          V2_TurnOverRobotState::getGlobalPose,
          () ->
              new ChassisSpeeds(
                  drive.getFieldRelativeVelocity().getX(),
                  drive.getFieldRelativeVelocity().getY(),
                  drive.getMeasuredChassisSpeeds().omegaRadiansPerSecond));

      fuelSimulator.registerIntake(
          IntakeConstants.LINKAGE_OFFSET.getX() - Units.inchesToMeters(4),
          IntakeConstants.LINKAGE_OFFSET.getX(),
          -V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
          V2_TurnOverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
          () ->
              intake.getIntakeState().equals(IntakeState.INTAKE)
                  && intake.atGoal()
                  && simFuelCount.getFuelStored() < V2_TurnOverSimFuelCount.getCapacity(),
          () ->
              simFuelCount.setFuelStored(
                  Math.min(simFuelCount.getFuelStored() + 1, V2_TurnOverSimFuelCount.getCapacity())));

      fuelSimulator.registerShooter(
          () -> simFuelCount.getFuelStored() > 0 && !V2_TurnOverRobotState.isProhibitShot(),
          () -> simFuelCount.setFuelStored(simFuelCount.getFuelStored() - 1),
          shooter.getHoodAngle()::getMeasure,
          shooter.getTurretRotation()::getMeasure,
          shooter::getFlywheelVelocity,
          V2_TurnOverConstants.ROBOT_TO_SHOOTER_TRANSFORM.getMeasureZ());

      fuelSimulator.setSubticks(1);
      fuelSimulator.start();
      fuelSimulator.spawnStartingFuel();
      fuelSimulator.enableAirResistance();

      fuelSimulator.start();
      RobotModeTriggers.autonomous()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    fuelSimulator.clearFuel();
                    fuelSimulator.spawnStartingFuel();
                    simFuelCount.setFuelStored(8);
                  }));
    } else {
      fuelSimulator.stop();
    }
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

  private void configureAutos() {
    // Named commands that are used during the paths
    NamedCommands.registerCommand(
        "SCORE_OR_FEED", V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper));

    NamedCommands.registerCommand(
        "SCORE_NO_ROLLER",
        V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
            .alongWith(intake.setOverrideRollerVoltage(0)));

    NamedCommands.registerCommand("STOP_OVERRIDE_ROLLER", intake.stopRollerOverride());

    NamedCommands.registerCommand(
        "SCORE_AGITATE_OP_1",
        shooter
            .setGoal(V2_TurnOverShooterConstants.ShooterGoal.SCORE)
            .alongWith(
                clopper.intake(),
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    intake
                        .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE / 2)
                        .alongWith(intake.stopRollerOverride()),
                    Commands.waitSeconds(5),
                    intake.deploy())));

    NamedCommands.registerCommand(
        "SCORE_AGITATE_OP_2",
        V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
            .alongWith(
                Commands.sequence(
                    intake.stopRollerOverride(),
                    Commands.waitSeconds(.25),
                    intake
                        .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                        .alongWith(intake.stopRollerOverride()),
                    Commands.waitSeconds(1.5),
                    intake.deploy())));

    NamedCommands.registerCommand("HOLD", V2_TurnOverCompositeCommands.hold(clopper, shooter));

    NamedCommands.registerCommand(
        "HOLD_WHILE_INTAKE",
        V2_TurnOverCompositeCommands.hold(clopper, shooter)
            .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)));

    NamedCommands.registerCommand(
        "DEPLOY",
        intake.deploy().alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)));

    NamedCommands.registerCommand("STOP_ROLLER", intake.setOverrideRollerVoltage(0));

    CommandScheduler.getInstance()
        .schedule(FollowPathCommand.warmupCommand(), intake.deploy().ignoringDisable(true));
    CommandScheduler.getInstance()
        .schedule(PathfindingCommand.warmupCommand(), intake.deploy().ignoringDisable(true));
    final boolean BRING_UP = false;

    if (Constants.TUNING_MODE) {

      // autoChooser.addOption("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive,
      // shooter));
      autoChooser.addOption(
          "Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(drive, V2_TurnOverConstants.DRIVE_CONSTANTS));
      autoChooser.addOption("Intake Linkage SysID", intake.linkageSysId());
      autoChooser.addOption(
          "Intake Agitate",
          intake
              .deploy()
              .andThen(
                  intake.waitUntilIntakeAtGoal(), intake.stow(), intake.waitUntilIntakeAtGoal())
              .repeatedly());
      autoChooser.addOption("Shooter Hood SysID", shooter.hoodSysId());
      autoChooser.addOption(
          "Shooter Hood Agitate",
          Commands.sequence(
                  shooter.setHoodAngle(V2_TurnOverShooterConstants.HOOD_CONSTANTS.minAngle),
                  shooter.waitUntilHoodAtGoal(),
                  shooter.setHoodAngle(V2_TurnOverShooterConstants.HOOD_CONSTANTS.maxAngle),
                  shooter.waitUntilHoodAtGoal())
              .repeatedly());

      autoChooser.addOption("Shooter Flywheel SysID", shooter.flywheelSysId());
      autoChooser.addOption(
          "Shooter Flywheel Agitate",
          shooter
              .setFlywheelVelocity(RadiansPerSecond.of(500))
              .andThen(
                  shooter.waitUntilFlywheelAtGoal(),
                  Commands.waitSeconds(5),
                  shooter.setFlywheelVelocity(RadiansPerSecond.of(0)),
                  shooter.waitUntilFlywheelAtGoal(),
                  Commands.waitSeconds(5))
              .repeatedly());
      autoChooser.addOption("Shooter Turret SysID", shooter.turretSysId());

      autoChooser.addOption(
          "Shooter Turret Agitate",
          shooter
              .setTurretGoal(V2_TurnOverShooterConstants.TURRET_CONSTANTS.maxAngle)
              .andThen(
                  shooter.waitUntilTurretAtGoal(),
                  shooter.setTurretGoal(V2_TurnOverShooterConstants.TURRET_CONSTANTS.minAngle),
                  shooter.waitUntilTurretAtGoal())
              .repeatedly());
    }

    autoChooser.addOption(
        "Left OP",
        V2_TurnOverAutoLeftOP.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right OP",
        V2_TurnOverAutoRightOP.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right OP Bucks",
        V2_TurnOverAutoRightOPBucks.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Left OP Bucks",
        V2_TurnOverAutoLeftOPBucks.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Left Half",
        V2_TurnOverAutoLeftHalf.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right Half",
        V2_TurnOverAutoRightHalf.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Depot", V2_TurnOverAutoDepot.getAutoRoutine(drive, intake, clopper, shooter));
    autoChooser.addOption(
        "Feed Full Right",
        V2_TurnOverAutoFollowFeedFullRight.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Full Left",
        V2_TurnOverAutoFollowFeedFullLeft.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Middle Right",
        V2_TurnOverAutoFollowFeedMiddleRight.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Middle Left",
        V2_TurnOverAutoFollowFeedMiddleLeft.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Follow Depot",
        V2_TurnOverAutoFollowDepot.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
  }

  @Trace
  private void configureButtonBindings() {
    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V2_TurnOverRobotState::resetPose,
                () -> V2_TurnOverRobotState.getGlobalPose().getTranslation()));

    drive.setDefaultCommand(
        new ContinuousConditionalCommand(
                DriveCommands.joystickDriveRotationLock(
                    drive,
                    V2_TurnOverConstants.DRIVE_CONSTANTS,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    V2_TurnOverRobotState::getHeading,
                    driver.rightBumper().negate(),
                    () -> {
                      Pose2d robotPose = V2_TurnOverRobotState.getLookaheadPose();
                      return (V2_TurnOverRobotState.isInAllianceZone()
                              ? AllianceFlipUtil.apply(
                                  FieldConstants.Hub.topCenterPoint.toTranslation2d())
                              : V2_TurnOverRobotState.getFeedTranslation())
                          .minus(
                              robotPose
                                  .transformBy(
                                      new Transform2d(
                                          V2_TurnOverShooterConstants.TURRET_CONSTANTS
                                              .robotToTurretTransform.getX(),
                                          V2_TurnOverShooterConstants.TURRET_CONSTANTS
                                              .robotToTurretTransform.getY(),
                                          Rotation2d.kZero))
                                  .getTranslation())
                          .getAngle()
                          .minus(
                              shooter
                                  .getTurretRotation()
                                  .plus(
                                      V2_TurnOverShooterConstants.TURRET_CONSTANTS
                                          .robotToTurretTransform
                                          .getRotation()
                                          .toRotation2d()))
                          .getRadians();
                    },
                    () -> V2_TurnOverRobotState.getTurretVelocity().in(RadiansPerSecond),
                    driver.leftTrigger(),
                    driver.rightTrigger(),
                    DriveCommands::getSlowFactor),
                DriveCommands.joystickDriveWithCardinalDirection(
                    drive,
                    V2_TurnOverConstants.DRIVE_CONSTANTS,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    V2_TurnOverRobotState::getHeading,
                    driver.leftTrigger(),
                    driver.rightTrigger()),
                () -> staticShooter)
            .withName("joystick-drive"));

    driver
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        DriveCommands.setLastCardinalDirection(
                            Math.round(
                                    V2_TurnOverRobotState.getHeading().getRadians() / (Math.PI / 2.0))
                                * (Math.PI / 2.0)))
                .withName("cardinal-direction-set"));
    xkeys.b9().onTrue(DriveCommands.incrementSlowFactor().withName("xkeys-b9-true"));
    xkeys.b10().onTrue(DriveCommands.decrementSlowFactor().withName("xkeys-b10-true"));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V2_TurnOverRobotState::resetPose,
                    () -> V2_TurnOverRobotState.getGlobalPose().getTranslation())
                .withName("driver-povDown-true"));

    driver.leftBumper().onTrue(intake.collect().withName("driver-leftBumper-true"));

    driver
        .rightBumper()
        .whileTrue(
            V2_TurnOverCompositeCommands.hold(clopper, shooter).withName("driver-rightBumper-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-rightBumper-false"));
    xkeys
        .b8()
        .onTrue(
            V2_TurnOverCompositeCommands.hold(clopper, shooter).withName("driver-rightBumper-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-rightBumper-false"));

    driver
        .topLeftPaddle()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.LEFT_TRENCH)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .topRightPaddle()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.RIGHT_TRENCH)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .bottomLeftPaddle()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.LEFT_CORNER)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .bottomRightPaddle()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.RIGHT_CORNER)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .a()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.HUB)
                .withName("driver-bottomLeftPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-bottomLeftPaddle-false"));

    driver
        .b()
        .whileTrue(
            V2_TurnOverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnOverRobotState.FixedShots.TOWER)
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-bottomRightPaddle-false"));

    driver
        .x()
        .whileTrue(
            intake
                .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("xkeys-f1-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("xkeys-f1-false"));

    xkeys
        .b1()
        .onTrue(intake.stopRollerOverride().alongWith(intake.stow()).withName("xkeys-b1-true"));
    xkeys.b2().onTrue(intake.decrementStowOffset().withName("xkeys-b2-true"));
    xkeys.b3().onTrue(intake.incrementStowOffset().withName("xkeys-b3-true"));

    xkeys
        .b4()
        .whileTrue(
            clopper
                .setOverrideRollerFloorVoltage(
                    V2_TurnOverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE_SLOW)
                .withName("xkeys-b5-while"));

    xkeys
        .b5()
        .whileTrue(
            clopper
                .setOverrideRollerFloorVoltage(
                    V2_TurnOverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE_SLOW.unaryMinus())
                .withName("xkeys-b6-true"));

    xkeys
        .c4()
        .whileTrue(
            clopper
                .setOverrideBallTunnelVoltage(V2_TurnOverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE)
                .withName("xkeys-c4-true"));
    xkeys
        .c5()
        .whileTrue(
            clopper
                .setOverrideBallTunnelVoltage(
                    V2_TurnOverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE.unaryMinus())
                .withName("xkeys-c5-true"));

    xkeys
        .d4()
        .whileTrue(
            clopper
                .setOverrideBallsToWallVoltage(
                    V2_TurnOverClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE)
                .withName("xkeys-d4-true"));

    xkeys
        .d5()
        .whileTrue(
            clopper
                .setOverrideBallsToWallVoltage(
                    V2_TurnOverClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE.unaryMinus())
                .withName("xkeys-d5-true"));

    xkeys
        .b10()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V2_TurnOverRobotState::resetPose,
                    V2_TurnOverRobotState.getGlobalPose()::getTranslation)
                .withName("xkeys-b10-true"));

    xkeys
        .c1()
        .onTrue(intake.deploy().alongWith(intake.stopRollerOverride()).withName("xkeys-c1-true"));
    xkeys.c2().onTrue(intake.decrementCollectOffset().withName("xkeys-c2-true"));
    xkeys.c3().onTrue(intake.incrementCollectOffset().withName("xkeys-c3-true"));

    xkeys.b6().onTrue(clopper.incrementRollerFloorVelocity().withName("xkeys-b6-true"));
    xkeys.b7().onTrue(clopper.decrementRollerFloorVelocity().withName("xkeys-b7-true"));

    xkeys.c6().onTrue(clopper.incrementBallTunnelVelocity().withName("xkeys-c6-true"));
    xkeys.c7().onTrue(clopper.decrementBallTunnelVelocity().withName("xkeys-c7-true"));

    xkeys.d6().onTrue(clopper.incrementBallsToWallVelocity().withName("xkeys-d6-true"));
    xkeys.d7().onTrue(clopper.decrementBallsToWallVelocity().withName("xkeys-d7-true"));

    xkeys
        .d1()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)
                .withName("xkeys-d1-while"))
        .onFalse(intake.stopRollerOverride().withName("xkeys-d1-false"));

    xkeys
        .d2()
        .or(driver.y())
        .whileTrue(
            intake
                .setOverrideRollerVoltage(-IntakeConstants.INTAKE_VOLTAGE)
                .withName("xkeys-d2-while"))
        .onFalse(intake.stopRollerOverride().withName("xkeys-d2-false"));
    driver
        .y()
        .whileTrue(
            intake
                .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                .withName("driver-y-while"))
        .onFalse(intake.setLinkageVoltage(0).withName("driver-y-false"));
    xkeys.h5().onTrue(shooter.decrementFlywheelVelocityThreshold().withName("xkeys-h5-true"));
    xkeys.h4().onTrue(shooter.incrementFlywheelVelocityThreshold().withName("xkeys-h4-true"));

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

    xkeys.f4().onTrue(shooter.incrementFlywheelVelocity().withName("xkeys-f4-true"));
    xkeys.f5().onTrue(shooter.decrementFlywheelVelocity().withName("xkeys-f5-true"));
    xkeys.f6().onTrue(shooter.incrementHoodAngle().withName("xkeys-f6-true"));
    xkeys.f7().onTrue(shooter.decrementHoodAngle().withName("xkeys-f7-true"));

    xkeys
        .g1()
        .or(xkeys.g2().or(xkeys.g3()))
        .whileTrue(intake.agitate().withName("xkeys-g1-g2-g3-while"));

    xkeys
        .g4()
        .whileTrue(
            shooter
                .setGoal(() -> V2_TurnOverShooterConstants.ShooterGoal.STOW)
                .withName("xkeys-g4-true"))
        .onFalse(
            shooter
                .setGoal(() -> V2_TurnOverShooterConstants.ShooterGoal.IDLE)
                .withName("xkeys-g4-false"));

    xkeys
        .g5()
        .onTrue(Commands.runOnce(() -> staticShooter = !staticShooter).withName("xkeys-g5-true"));
    xkeys.g6().onTrue(shooter.incrementTurretZero().withName("xkeys-g6-true"));
    xkeys.g7().onTrue(shooter.decrementTurretZero().withName("xkeys-g7-true"));

    xkeys.g9().onTrue(shooter.zero().withName("xkeys-g9"));
    xkeys
        .f3()
        .whileTrue(
            clopper
                .feedShooterBallTunnel()
                .alongWith(clopper.feedShooterRollerFloor())
                .withName("xkeys-f3-while"))
        .onFalse(
            clopper
                .stopBallTunnel()
                .alongWith(clopper.stopRollerFloor())
                .withName("xkeys-f3-false"));

    xkeys.g10().onTrue(shooter.resetTurretZero().withName("xkeys-g10-true"));

    xkeys
        .h1()
        .or(xkeys.h2().or(xkeys.h3()))
        .whileTrue(
            shooter
                .setGoal(V2_TurnOverShooterConstants.ShooterGoal.SCORE)
                .alongWith(clopper.intake())
                .withName("xkeys-h1-h2-h3-while"))
        .onFalse(
            V2_TurnOverCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("xkeys-h1-h2-h3-false"));

    // xkeys.h4().onTrue(); SLOW WRAP MODE
    // xkeys.h5().onTrue(); FAST WRAP MODE

    xkeys
        .h6()
        .whileTrue(shooter.clockwiseSlow().withName("xkeys-h6-while"))
        .onFalse(shooter.stopTurret().withName("xkeys-h6-false"));
    xkeys
        .h7()
        .whileTrue(shooter.counterClockwiseSlow().withName("xkeys-h7-while"))
        .onFalse(shooter.stopTurret().withName("xkeys-h7-false"));

    xkeys.h9().onTrue(intake.resetIntakeZero().withName("xkeys-h9-true"));

    xkeys
        .h10()
        .whileTrue(shooter.resetHoodZero().withName("xkeys-h10-while"))
        .onFalse(
            shooter
                .setGoal(() -> V2_TurnOverShooterConstants.ShooterGoal.IDLE)
                .withName("xkeys-h10-false"));
  }

  @Override
  public void robotPeriodic() {
    V2_TurnOverRobotState.periodic(
        drive.getRawGyroRotation(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        shooter.getTurretRotation(),
        shooter.isTurretWrapping(),
        drive.getMeasuredChassisSpeeds(),
        intake.isIntakeAtStow(),
        driver.rightBumper().getAsBoolean());
    fuelSimulator.updateSim();
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
