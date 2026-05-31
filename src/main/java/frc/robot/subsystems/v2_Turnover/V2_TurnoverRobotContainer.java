package frc.robot.subsystems.v2_Turnover;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
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
import frc.robot.commands.v2_Turnover.V2_TurnoverCompositeCommands;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoDepot;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowDepot;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowDepotBC;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowFeedFullLeft;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowFeedFullRight;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowFeedMiddleLeft;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoFollowFeedMiddleRight;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoLeftHalf;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoLeftOP;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoLeftOPBC;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoLeftOPBucks;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoRightHalf;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoRightOP;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoRightOPBC;
import frc.robot.commands.v2_Turnover.autonomous.V2_TurnoverAutoRightOPBucks;
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
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopper;
import frc.robot.subsystems.v2_Turnover.clopper.V2_TurnoverClopperConstants;
import frc.robot.subsystems.v2_Turnover.leds.V2_TurnoverCANdle;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooter;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverShooterConstants;
import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverSimFuelCount;
// import frc.robot.subsystems.v2_Turnover.shooter.V2_TurnoverSimFuelCount;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FuelSimulator;
import frc.robot.util.LTNUpdater;
import frc.robot.util.command.ContinuousConditionalCommand;
import frc.robot.util.input.XKeysInput;
import frc.robot.util.input.XboxElite2Input;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class V2_TurnoverRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private V2_TurnoverClopper clopper;
  private Vision vision;
  private V2_TurnoverShooter shooter;
  private Intake intake;
  private FuelSimulator fuelSimulator;
  private V2_TurnoverSimFuelCount simFuelCount;

  private boolean staticShooter = false;

  private final LoggedDashboardChooser<Command> autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);
  private final XKeysInput xkeys = new XKeysInput(1);

  private final Map<LoggedNetworkBoolean, PathAdjustmentMode> pathAdjustmentModeMap;

  public V2_TurnoverRobotContainer() {

    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_TURNOVER:
          gyroIO =
              new GyroIOPigeon2(
                  V2_TurnoverConstants.DRIVE_CONSTANTS,
                  V2_TurnoverRobotState::setHeadingUpdateTimestamp);
          drive =
              new SwerveDrive(
                  V2_TurnoverConstants.DRIVE_CONSTANTS,
                  gyroIO,
                  new SwerveModuleIOTalonFX(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_TurnoverRobotState::getGlobalPose,
                  V2_TurnoverRobotState::resetPose);
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V2_TurnoverRobotState.getLedStates()::setIntakeIn,
                      V2_TurnoverRobotState.getLedStates()::setIntakeCollecting,
                      s -> {},
                      V2_TurnoverRobotState.getLedStates()::setIntakeSlowRolling));
          clopper =
              new V2_TurnoverClopper(
                  new GenericRollerIOTalonFX(V2_TurnoverClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_TurnoverClopperConstants.BALL_TUNNEL_TOP_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_TurnoverClopperConstants.BALL_TUNNEL_BOTTOM_CONSTANTS), 
                  new GenericRollerIOTalonFX(
                      V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS));
          shooter =
              new V2_TurnoverShooter(
                  new TurretIOTalonFX(V2_TurnoverShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFX(V2_TurnoverShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFX(V2_TurnoverShooterConstants.SHOOT_CONSTANTS),
                  drive::getMeasuredChassisSpeeds,
                  () -> staticShooter);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnoverConstants.LIMELIGHT_INTAKE_CONFIG),
                      V2_TurnoverConstants.LIMELIGHT_INTAKE_CONFIG,
                      V2_TurnoverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnoverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnoverRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnoverConstants.LIMELIGHT_LEFT_CONFIG),
                      V2_TurnoverConstants.LIMELIGHT_LEFT_CONFIG,
                      V2_TurnoverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnoverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnoverRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_TurnoverConstants.LIMELIGHT_RIGHT_CONFIG),
                      V2_TurnoverConstants.LIMELIGHT_RIGHT_CONFIG,
                      V2_TurnoverRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_TurnoverRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_TurnoverRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          new V2_TurnoverCANdle();
          break;
        case V2_TURNOVER_SIM:
          drive =
              new SwerveDrive(
                  V2_TurnoverConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V2_TurnoverConstants.DRIVE_CONSTANTS,
                      V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_TurnoverRobotState::getGlobalPose,
                  V2_TurnoverRobotState::resetPose);
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper(),
                  new Intake.IntakeStateSetter(
                      V2_TurnoverRobotState.getLedStates()::setIntakeIn,
                      V2_TurnoverRobotState.getLedStates()::setIntakeCollecting,
                      s -> {},
                      V2_TurnoverRobotState.getLedStates()::setIntakeSlowRolling));
          clopper =
              new V2_TurnoverClopper(
                  new GenericRollerIOTalonFXSim(V2_TurnoverClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_TurnoverClopperConstants.BALL_TUNNEL_TOP_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_TurnoverClopperConstants.BALL_TUNNEL_BOTTOM_CONSTANTS),
                  new GenericRollerIOTalonFXSim(
                      V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_CONSTANTS));

          shooter =
              new V2_TurnoverShooter(
                  new TurretIOSim(V2_TurnoverShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFXSim(V2_TurnoverShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFXSim(V2_TurnoverShooterConstants.SHOOT_CONSTANTS),
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
              V2_TurnoverConstants.DRIVE_CONSTANTS,
              gyroIO,
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V2_TurnoverRobotState::getGlobalPose,
              V2_TurnoverRobotState::resetPose);
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
          new V2_TurnoverClopper(
              new GenericRollerIO() {}, new GenericRollerIO() {}, new GenericRollerIO() {}, new GenericRollerIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }
    if (shooter == null) {
      shooter =
          new V2_TurnoverShooter(
              new TurretIO() {},
              new HoodIO() {},
              new GenericFlywheelIO() {},
              drive::getMeasuredChassisSpeeds,
              () -> false);
    }

    pathAdjustmentModeMap =
        new HashMap<>() {
          {
            put(
                new LoggedNetworkBoolean("PathAdjustmentMode/Left Bump", false),
                PathAdjustmentMode.LEFT_BUMP);
            put(
                new LoggedNetworkBoolean("PathAdjustmentMode/Right Bump", false),
                PathAdjustmentMode.RIGHT_BUMP);
            put(
                new LoggedNetworkBoolean("PathAdjustmentMode/Left Trench", false),
                PathAdjustmentMode.LEFT_TRENCH);
            put(
                new LoggedNetworkBoolean("PathAdjustmentMode/Right Trench", false),
                PathAdjustmentMode.RIGHT_TRENCH);
          }
        };

    autoChooser = new LoggedDashboardChooser<>("Autonomous Modes");
    configureButtonBindings();
    configureAutos();
    configureFuelSim();
    if (Constants.TUNING_MODE) LTNUpdater.registerV2(drive, intake, shooter);
  }

  private void configureFuelSim() {

    fuelSimulator = new FuelSimulator("FuelSim");
    if (RobotBase.isSimulation()) {
      simFuelCount = new V2_TurnoverSimFuelCount(8);

      fuelSimulator.registerRobot(
          V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth(),
          V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.bumperLength(),
          Units.inchesToMeters(6.0),
          V2_TurnoverRobotState::getGlobalPose,
          () ->
              new ChassisSpeeds(
                  drive.getFieldRelativeVelocity().getX(),
                  drive.getFieldRelativeVelocity().getY(),
                  drive.getMeasuredChassisSpeeds().omegaRadiansPerSecond));

      fuelSimulator.registerIntake(
          IntakeConstants.LINKAGE_OFFSET.getX() - Units.inchesToMeters(4),
          IntakeConstants.LINKAGE_OFFSET.getX(),
          -V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
          V2_TurnoverConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
          () ->
              intake.getIntakeState().equals(IntakeState.INTAKE)
                  && intake.atGoal()
                  && simFuelCount.getFuelStored() < V2_TurnoverSimFuelCount.getCapacity(),
          () ->
              simFuelCount.setFuelStored(
                  Math.min(
                      simFuelCount.getFuelStored() + 1, V2_TurnoverSimFuelCount.getCapacity())));

      fuelSimulator.registerShooter(
          () -> simFuelCount.getFuelStored() > 0 && !V2_TurnoverRobotState.isProhibitShot(),
          () -> simFuelCount.setFuelStored(simFuelCount.getFuelStored() - 1),
          shooter.getHoodAngle()::getMeasure,
          shooter.getTurretRotation()::getMeasure,
          shooter::getFlywheelVelocity,
          V2_TurnoverConstants.ROBOT_TO_SHOOTER_TRANSFORM.getMeasureZ());

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
    return () ->
        pathAdjustmentModeMap.entrySet().stream()
            .filter(entry -> entry.getKey().get())
            .map(Map.Entry::getValue)
            .collect(
                Collectors.collectingAndThen(
                    Collectors.<PathAdjustmentMode>toList(),
                    (List<PathAdjustmentMode> list) -> {
                      if (list.isEmpty()) {
                        list.add(PathAdjustmentMode.USE_ANY_AVAILABLE);
                      }
                      return list;
                    }))
            .toArray(PathAdjustmentMode[]::new);
  }

  private void configureAutos() {
    // Named commands that are used during the paths
    NamedCommands.registerCommand(
        "SCORE_OR_FEED",
        V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, () -> false));

    NamedCommands.registerCommand(
        "SCORE_NO_ROLLER",
        V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, () -> false)
            .alongWith(intake.setOverrideRollerVoltage(0)));

    NamedCommands.registerCommand("STOP_OVERRIDE_ROLLER", intake.stopRollerOverride());

    NamedCommands.registerCommand(
        "SCORE_AGITATE_OP_1",
        shooter
            .setGoal(V2_TurnoverShooterConstants.ShooterGoal.SCORE)
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
        V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, () -> false)
            .alongWith(
                Commands.sequence(
                    intake.stopRollerOverride(),
                    Commands.waitSeconds(.25),
                    intake
                        .setLinkageVoltage(-IntakeConstants.LINKAGE_SLOW_VOLTAGE)
                        .alongWith(intake.stopRollerOverride()),
                    Commands.waitSeconds(1.5),
                    intake.deploy())));

    NamedCommands.registerCommand("HOLD", V2_TurnoverCompositeCommands.hold(clopper, shooter));

    NamedCommands.registerCommand(
        "HOLD_WHILE_INTAKE",
        V2_TurnoverCompositeCommands.hold(clopper, shooter)
            .alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE))
            .withName("HOLD_WHILE_INTAKE"));

    NamedCommands.registerCommand(
        "DEPLOY",
        intake.deploy().alongWith(intake.setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)));

    NamedCommands.registerCommand("STOP_ROLLER", intake.setOverrideRollerVoltage(0));

    CommandScheduler.getInstance()
        .schedule(
            FollowPathCommand.warmupCommand(),
            PathfindingCommand.warmupCommand(),
            intake.deploy().ignoringDisable(true));

    if (Constants.TUNING_MODE) {

      // autoChooser.addOption("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive,
      // shooter));
      autoChooser.addOption(
          "Drive Feedforward Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(drive, V2_TurnoverConstants.DRIVE_CONSTANTS));
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
                  shooter.setHoodAngle(V2_TurnoverShooterConstants.HOOD_CONSTANTS.minAngle),
                  shooter.waitUntilHoodAtGoal(),
                  shooter.setHoodAngle(V2_TurnoverShooterConstants.HOOD_CONSTANTS.maxAngle),
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
              .setTurretGoal(V2_TurnoverShooterConstants.TURRET_CONSTANTS.maxAngle)
              .andThen(
                  shooter.waitUntilTurretAtGoal(),
                  shooter.setTurretGoal(V2_TurnoverShooterConstants.TURRET_CONSTANTS.minAngle),
                  shooter.waitUntilTurretAtGoal())
              .repeatedly());
    }

    autoChooser.addOption(
        "Left OP",
        V2_TurnoverAutoLeftOP.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right OP",
        V2_TurnoverAutoRightOP.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right OP Bucks",
        V2_TurnoverAutoRightOPBucks.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Left OP Bucks",
        V2_TurnoverAutoLeftOPBucks.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Left Half",
        V2_TurnoverAutoLeftHalf.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right Half",
        V2_TurnoverAutoRightHalf.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Depot", V2_TurnoverAutoDepot.getAutoRoutine(drive, intake, clopper, shooter));
    autoChooser.addOption(
        "Feed Full Right",
        V2_TurnoverAutoFollowFeedFullRight.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Full Left",
        V2_TurnoverAutoFollowFeedFullLeft.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Middle Right",
        V2_TurnoverAutoFollowFeedMiddleRight.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Feed Middle Left",
        V2_TurnoverAutoFollowFeedMiddleLeft.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Follow Depot",
        V2_TurnoverAutoFollowDepot.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Follow Depot BC",
        V2_TurnoverAutoFollowDepotBC.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Right OP BC",
        V2_TurnoverAutoRightOPBC.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));
    autoChooser.addOption(
        "Left OP BC",
        V2_TurnoverAutoLeftOPBC.getAutoRoutine(
            drive, intake, clopper, shooter, getAdjustmentModeSupplier()));

    PathPlannerLogging.setLogActivePathCallback(
        l -> Logger.recordOutput("Auto/PathPlanner/Path", l.toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogTargetPoseCallback(
        p -> Logger.recordOutput("Auto/PathPlanner/Target", p));
  }

  private void configureButtonBindings() {
    Trigger invertScoreLocation = driver.povRight();
    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V2_TurnoverRobotState::resetPose,
                () -> V2_TurnoverRobotState.getGlobalPose().getTranslation()));

    driver.povUp().onTrue(drive.runOnce(() -> drive.stopWithX()).withName("driver-povUp"));

    xkeys.g8().whileTrue(drive.run(() -> drive.stopWithX()).withName("xkeys-g8"));

    drive.setDefaultCommand(
        new ContinuousConditionalCommand(
                DriveCommands.joystickDriveRotationLock(
                    drive,
                    V2_TurnoverConstants.DRIVE_CONSTANTS,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    V2_TurnoverRobotState::getHeading,
                    driver.rightBumper().negate(),
                    () -> {
                      Pose2d robotPose = V2_TurnoverRobotState.getLookaheadPose();
                      return (V2_TurnoverRobotState.isInAllianceZone()
                              ? AllianceFlipUtil.apply(
                                  FieldConstants.Hub.topCenterPoint.toTranslation2d())
                              : V2_TurnoverRobotState.getFeedTranslation())
                          .minus(
                              robotPose
                                  .transformBy(
                                      new Transform2d(
                                          V2_TurnoverShooterConstants.TURRET_CONSTANTS
                                              .robotToTurretTransform.getX(),
                                          V2_TurnoverShooterConstants.TURRET_CONSTANTS
                                              .robotToTurretTransform.getY(),
                                          Rotation2d.kZero))
                                  .getTranslation())
                          .getAngle()
                          .minus(
                              shooter
                                  .getTurretRotation()
                                  .plus(
                                      V2_TurnoverShooterConstants.TURRET_CONSTANTS
                                          .robotToTurretTransform
                                          .getRotation()
                                          .toRotation2d()))
                          .getRadians();
                    },
                    () -> V2_TurnoverRobotState.getTurretVelocity().in(RadiansPerSecond),
                    driver.leftTrigger(),
                    driver.rightTrigger(),
                    DriveCommands::getSlowFactor),
                DriveCommands.joystickDriveWithCardinalDirection(
                    drive,
                    V2_TurnoverConstants.DRIVE_CONSTANTS,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    V2_TurnoverRobotState::getHeading,
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
                                    V2_TurnoverRobotState.getHeading().getRadians()
                                        / (Math.PI / 2.0))
                                * (Math.PI / 2.0)))
                .withName("cardinal-direction-set"));

    xkeys.b9().onTrue(DriveCommands.decrementSlowFactor().withName("xkeys-b10-true"));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V2_TurnoverRobotState::resetPose,
                    () -> V2_TurnoverRobotState.getGlobalPose().getTranslation())
                .withName("driver-povDown-true"));

    driver.leftBumper().onTrue(intake.collect().withName("driver-leftBumper-true"));

    driver
        .rightBumper()
        .whileTrue(
            V2_TurnoverCompositeCommands.hold(clopper, shooter)
                .withName("driver-rightBumper-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-rightBumper-false"));
    xkeys
        .b8()
        .onTrue(
            V2_TurnoverCompositeCommands.hold(clopper, shooter)
                .withName("driver-rightBumper-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-rightBumper-false"));

    xkeys
        .b10()
        .onTrue(
            V2_TurnoverCompositeCommands.holdAndIntake(clopper, shooter, intake)
                .withName("xkeys-b10-true"));

    driver
        .topLeftPaddle()
        .whileTrue(
            V2_TurnoverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnoverRobotState.FixedShots.LEFT_TRENCH)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-topLeftPaddle-false"));

    driver
        .topRightPaddle()
        .whileTrue(
            V2_TurnoverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnoverRobotState.FixedShots.RIGHT_TRENCH)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-topRightPaddle-false"));

    driver
        .bottomLeftPaddle()
        .whileTrue(
            V2_TurnoverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnoverRobotState.FixedShots.LEFT_CORNER)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-topLeftPaddle-false"));

    driver
        .bottomRightPaddle()
        .whileTrue(
            V2_TurnoverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnoverRobotState.FixedShots.RIGHT_CORNER)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
                .withName("driver-topRightPaddle-false"));

    driver.a()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(-IntakeConstants.INTAKE_VOLTAGE)
                .withName("driver-a-while"))
        .onFalse(intake.stopRollerOverride().withName("driver-a-false"));

    driver
        .b()
        .whileTrue(
            V2_TurnoverCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_TurnoverRobotState.FixedShots.TOWER)
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
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
                    V2_TurnoverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE_SLOW)
                .withName("xkeys-b5-while"));

    xkeys
        .b5()
        .whileTrue(
            clopper
                .setOverrideRollerFloorVoltage(
                    V2_TurnoverClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE_SLOW.unaryMinus())
                .withName("xkeys-b6-true"));

    xkeys
        .c4()
        .whileTrue(
            clopper
                .setOverrideBallTunnelVoltage(V2_TurnoverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE)
                .withName("xkeys-c4-true"));
    xkeys
        .c5()
        .whileTrue(
            clopper
                .setOverrideBallTunnelVoltage(
                    V2_TurnoverClopperConstants.BALL_TUNNEL_FEED_VOLTAGE.unaryMinus())
                .withName("xkeys-c5-true"));

    xkeys
        .d3()
        .whileTrue(
            clopper
                .marcusCommand())
        .onFalse(clopper.stopBallTunnel());


    xkeys
        .d4()
        .whileTrue(
            clopper
                .setOverrideBallsToWallVoltage(
                    V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE)
                .withName("xkeys-d4-true"));

    xkeys
        .d5()
        .whileTrue(
            clopper
                .setOverrideBallsToWallVoltage(
                    V2_TurnoverClopperConstants.BALLS_TO_THE_WALL_FORWARD_VOLTAGE.unaryMinus())
                .withName("xkeys-d5-true"));

    // xkeys
    //     .b10()
    //     .onTrue(
    //         SharedCompositeCommands.resetHeading(
    //                 drive,
    //                 V2_TurnoverRobotState::resetPose,
    //                 V2_TurnoverRobotState.getGlobalPose()::getTranslation)
    //             .withName("xkeys-b10-true"));

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
        .onTrue(
            shooter
                .setGoal(V2_TurnoverShooterConstants.ShooterGoal.STOW)
                .withName("xkeys-g4-true"));

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
                .setGoal(V2_TurnoverShooterConstants.ShooterGoal.SCORE)
                .alongWith(clopper.intake())
                .withName("xkeys-h1-h2-h3-while"))
        .onFalse(
            V2_TurnoverCompositeCommands.scoreOrFeedCommand(shooter, clopper, invertScoreLocation)
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
                .setGoal(() -> V2_TurnoverShooterConstants.ShooterGoal.IDLE)
                .withName("xkeys-h10-false"));
  }

  @Override
  public void robotPeriodic() {
    V2_TurnoverRobotState.periodic(
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
