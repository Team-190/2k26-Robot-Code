package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.TunableUpdaterRegistry;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOSim;
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
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;
import frc.robot.commands.v2_Delta.autonomous.V2_DeltaAutoLeftOP;
import frc.robot.commands.v2_Delta.autonomous.V2_DeltaAutoRightOP;
import frc.robot.commands.v2_Delta.autonomous.V2_TurretTestAuto;
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
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.shared.turret.TurretIOSim;
import frc.robot.subsystems.shared.turret.TurretIOTalonFX;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.util.BetterAutoChooser;
import frc.robot.util.input.XKeysInput;
import frc.robot.util.input.XboxElite2Input;
import java.util.List;

public class V2_DeltaRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private Climber climber;
  private Intake intake;
  private V2_DeltaClopper clopper;
  private Vision vision;
  private V2_DeltaShooter shooter;

  private final BetterAutoChooser autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);
  private final XKeysInput xkeys = new XKeysInput(1);

  public V2_DeltaRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V2_DELTA:
          gyroIO =
              new GyroIOPigeon2(
                  V2_DeltaConstants.DRIVE_CONSTANTS, V2_DeltaRobotState::setHeadingUpdateTimestamp);
          drive =
              new SwerveDrive(
                  V2_DeltaConstants.DRIVE_CONSTANTS,
                  gyroIO,
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_DeltaRobotState::getGlobalPose,
                  V2_DeltaRobotState::resetPose);
          // climber =
          // new Climber(
          // new ArmIOTalonFX(ClimberConstants.CLIMBER_CONSTANTS),
          // gyroIO.getRoll().asSupplier());
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper());
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          shooter =
              new V2_DeltaShooter(
                  new TurretIOTalonFX(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFX(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFX(V2_DeltaShooterConstants.SHOOT_CONSTANTS),
                  drive::getMeasuredChassisSpeeds);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_INTAKE_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_INTAKE_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_LEFT_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_LEFT_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_RIGHT_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_RIGHT_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          break;
        case V2_DELTA_SIM:
          drive =
              new SwerveDrive(
                  V2_DeltaConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V2_DeltaConstants.DRIVE_CONSTANTS,
                      V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V2_DeltaRobotState::getGlobalPose,
                  V2_DeltaRobotState::resetPose);
          climber = new Climber(new ArmIOSim(ClimberConstants.CLIMBER_CONSTANTS), Radians::zero);
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS),
                  driver.leftBumper());
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));

          shooter =
              new V2_DeltaShooter(
                  new TurretIOSim(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFXSim(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFXSim(V2_DeltaShooterConstants.SHOOT_CONSTANTS),
                  drive::getMeasuredChassisSpeeds);
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
              V2_DeltaConstants.DRIVE_CONSTANTS,
              gyroIO,
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V2_DeltaRobotState::getGlobalPose,
              V2_DeltaRobotState::resetPose);
    }
    if (climber == null) {
      climber = new Climber(new ArmIO() {}, Radians::zero);
    }
    if (intake == null) {
      intake = new Intake(new GenericRollerIO() {}, new FourBarLinkageIO() {}, () -> false);
    }
    if (clopper == null) {
      clopper = new V2_DeltaClopper(new GenericRollerIO() {}, new GenericRollerIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }
    if (shooter == null) {
      shooter =
          new V2_DeltaShooter(
              new TurretIO() {},
              new HoodIO() {},
              new GenericFlywheelIO() {},
              drive::getMeasuredChassisSpeeds);
    }

    autoChooser = new BetterAutoChooser(V2_DeltaRobotState::resetPose);
    configureButtonBindings();
    configureAutos();

    TunableUpdaterRegistry.registerGains(
        V2_DeltaConstants.DRIVE_GAINS,
        g -> {
          drive.setPIDGains(
              g.getKP(),
              g.getKD(),
              V2_DeltaConstants.TURN_GAINS.getKP(),
              V2_DeltaConstants.TURN_GAINS.getKD());
          drive.setFFGains(g.getKS(), g.getKV());
        });
    TunableUpdaterRegistry.registerGains(
        V2_DeltaConstants.TURN_GAINS,
        g ->
            drive.setPIDGains(
                V2_DeltaConstants.DRIVE_GAINS.getKP(),
                V2_DeltaConstants.DRIVE_GAINS.getKD(),
                g.getKP(),
                g.getKD()));

    TunableUpdaterRegistry.registerGains(
        IntakeConstants.LINKAGE_CONSTANTS.gains, intake.getLinkage()::setGains);

    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.HOOD_CONSTANTS.gains, shooter::setHoodGains);
    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.HOOD_CONSTANTS.constraints,
        c -> shooter.setHoodConstraints((AngularPositionConstraints) c));

    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.SHOOT_CONSTANTS.constraints,
        c -> shooter.setFlywheelConstraints((AngularVelocityConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.SHOOT_CONSTANTS.voltageGains, shooter::setFlywheelGains);

    TunableUpdaterRegistry.registerConstraints(
        V2_DeltaShooterConstants.TURRET_CONSTANTS.constraints,
        c -> shooter.setTurretConstraints((AngularPositionConstraints) c));
    TunableUpdaterRegistry.registerGains(
        V2_DeltaShooterConstants.TURRET_CONSTANTS.gains, shooter::setTurretGains);
  }

  private void configureAutos() {
    final boolean BRING_UP = false;

    if (BRING_UP) {

      autoChooser.addRoutineConfig("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive, shooter));
      autoChooser.addCmd(
          "Drive Feedforward Characterization",
          () -> DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addCmd(
          "Wheel Radius Characterization",
          () ->
              DriveCommands.wheelRadiusCharacterization(drive, V2_DeltaConstants.DRIVE_CONSTANTS));
      autoChooser.addCmd("Intake Linkage SysID", () -> intake.linkageSysId());
      autoChooser.addCmd(
          "Intake Agitate",
          intake
                  .deploy()
                  .andThen(
                      intake.waitUntilIntakeAtGoal(), intake.stow(), intake.waitUntilIntakeAtGoal())
              ::repeatedly);
      autoChooser.addCmd("Shooter Hood SysID", shooter::hoodSysId);
      autoChooser.addCmd(
          "Shooter Hood Agitate",
          Commands.sequence(
                  shooter.setHoodAngle(V2_DeltaShooterConstants.HOOD_CONSTANTS.minAngle),
                  shooter.waitUntilHoodAtGoal(),
                  shooter.setHoodAngle(V2_DeltaShooterConstants.HOOD_CONSTANTS.maxAngle),
                  shooter.waitUntilHoodAtGoal())
              ::repeatedly);

      autoChooser.addCmd("Shooter Flywheel SysID", shooter::flywheelSysId);
      autoChooser.addCmd(
          "Shooter Flywheel Agitate",
          shooter
                  .setFlywheelVelocity(RadiansPerSecond.of(500))
                  .andThen(
                      shooter.waitUntilFlywheelAtGoal(),
                      Commands.waitSeconds(1),
                      shooter.setFlywheelVelocity(RadiansPerSecond.of(0)),
                      shooter.waitUntilFlywheelAtGoal(),
                      Commands.waitSeconds(1))
              ::repeatedly);
      autoChooser.addCmd("Shooter Turret SysID", shooter::turretSysId);

      autoChooser.addCmd(
          "Shooter Turret Agitate",
          shooter
                  .setTurretGoal(V2_DeltaShooterConstants.TURRET_CONSTANTS.maxAngle)
                  .andThen(
                      shooter.waitUntilTurretAtGoal(),
                      shooter.setTurretGoal(V2_DeltaShooterConstants.TURRET_CONSTANTS.minAngle),
                      shooter.waitUntilTurretAtGoal())
              ::repeatedly);
    }

    autoChooser.addCmdRoutine(
        "Left OP",
        () -> V2_DeltaAutoLeftOP.getAutoRoutine(drive, intake, clopper, shooter),
        FollowPathCommand::warmupCommand);
    autoChooser.addCmdRoutine(
        "Right OP",
        () -> V2_DeltaAutoRightOP.getAutoRoutine(drive, intake, clopper, shooter),
        FollowPathCommand::warmupCommand);

    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  @Trace
  private void configureButtonBindings() {
    // drive.setDefaultCommand(
    // DriveCommands.joystickDrive(
    // drive,
    // V2_DeltaConstants.DRIVE_CONSTANTS,
    // () -> -driver.getLeftY(),
    // () -> -driver.getLeftX(),
    // () -> -driver.getRightX(),
    // V2_DeltaRobotState::getHeading));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V2_DeltaRobotState::resetPose,
                () -> V2_DeltaRobotState.getGlobalPose().getTranslation()));

    drive.setDefaultCommand(
        DriveCommands.joystickDriveWithCardinalDirection(
                drive,
                V2_DeltaConstants.DRIVE_CONSTANTS,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                V2_DeltaRobotState::getHeading,
                driver.leftTrigger(),
                driver.rightTrigger(), // TODO: Figure out how to do 50% on RT and 10% on X
                0.5)
            .withName("joystick-drive"));

    driver
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        DriveCommands.setLastCardinalDirection(
                            Math.round(
                                    V2_DeltaRobotState.getHeading().getRadians() / (Math.PI / 2.0))
                                * (Math.PI / 2.0)))
                .withName("cardinal-direction-set"));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V2_DeltaRobotState::resetPose,
                    () -> V2_DeltaRobotState.getGlobalPose().getTranslation())
                .withName("driver-povDown-true"));

    driver.leftBumper().onTrue(intake.collect().withName("driver-leftBumper-true"));

    driver
        .rightBumper()
        .whileTrue(
            V2_DeltaCompositeCommands.hold(clopper, shooter).withName("driver-rightBumper-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-rightBumper-false"));
    xkeys
        .b8()
        .onTrue(
            V2_DeltaCompositeCommands.hold(clopper, shooter).withName("driver-rightBumper-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-rightBumper-false"));

    driver
        .x()
        .onTrue(V2_DeltaCompositeCommands.deployClimber(intake, climber).withName("driver-x-true"));

    driver
        .y()
        .whileTrue(climber.climbSequenceL3().withName("driver-y-while"))
        .onFalse(climber.stop().withName("driver-y-false"));

    driver
        .topLeftPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.LEFT_TRENCH)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .topRightPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.RIGHT_TRENCH)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .bottomLeftPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.LEFT_CORNER)
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .bottomRightPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.RIGHT_CORNER)
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .a()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.HUB)
                .withName("driver-bottomLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-bottomLeftPaddle-false"));

    driver
        .b()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    shooter, clopper, V2_DeltaRobotState.FixedShots.TOWER)
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("driver-bottomRightPaddle-false"));

    xkeys.b1().onTrue(intake.stopRoller().alongWith(intake.stow()).withName("xkeys-b1-true"));
    xkeys.b2().onTrue(intake.decrementStowOffset().withName("xkeys-b2-true"));
    xkeys.b3().onTrue(intake.incrementStowOffset().withName("xkeys-b3-true"));

    xkeys
        .b4()
        .whileTrue(
            clopper
                .setRollerFloorVoltage(V2_DeltaClopperConstants.ROLLER_FLOOR_FEED_VOLTAGE_SLOW)
                .withName("xkeys-b5-while"))
        .onFalse(clopper.setRollerFloorVoltage(Volts.of(0)).withName("xkeys-b5-false"));

    xkeys.b5().whileTrue(clopper.setRollerFloorVoltage(Volts.of(0)).withName("xkeys-b6-true"));

    xkeys
        .b6()
        .whileTrue(
            clopper
                .setBallTunnelVoltage(V2_DeltaClopperConstants.BALL_TUNNEL_FEED_VOLTAGE)
                .withName("xkeys-b6-true"))
        .onFalse(clopper.setBallTunnelVoltage(Volts.zero()).withName("xkeys-b6-false"));
    xkeys
        .b7()
        .whileTrue(
            clopper
                .setBallTunnelVoltage(
                    V2_DeltaClopperConstants.BALL_TUNNEL_FEED_VOLTAGE.unaryMinus())
                .withName("xkeys-b7-true"))
        .onFalse(clopper.setBallTunnelVoltage(Volts.zero()).withName("xkeys-b7-false"));

    xkeys
        .b10()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                    drive,
                    V2_DeltaRobotState::resetPose,
                    V2_DeltaRobotState.getGlobalPose()::getTranslation)
                .withName("xkeys-b10-while"));

    xkeys.c1().onTrue(intake.deploy().alongWith(intake.stopRoller()).withName("xkeys-c1-true"));
    xkeys.c2().onTrue(intake.decrementCollectOffset().withName("xkeys-c2-true"));
    xkeys.c3().onTrue(intake.incrementCollectOffset().withName("xkeys-c3-true"));

    xkeys.c4().onTrue(clopper.incrementRollerFloorVelocity().withName("xkeys-c4-true"));
    xkeys.c5().onTrue(clopper.decrementRollerFloorVelocity().withName("xkeys-c5-true"));

    xkeys.c6().onTrue(clopper.incrementBallTunnelVelocity().withName("xkeys-c6-true"));
    xkeys.c7().onTrue(clopper.decrementBallTunnelVelocity().withName("xkeys-c7-true"));

    xkeys
        .d1()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(IntakeConstants.INTAKE_VOLTAGE)
                .withName("xkeys-d1-while"))
        .onFalse(intake.stopRoller().withName("xkeys-d1-false"));

    xkeys
        .d2()
        .whileTrue(
            intake
                .setOverrideRollerVoltage(IntakeConstants.EXTAKE_VOLTAGE)
                .withName("xkeys-d2-while"))
        .onFalse(intake.stopRoller().withName("xkeys-d2-false"));

    xkeys.d8().onTrue(climber.setPositionDefault().withName("xkeys-d8-true"));
    xkeys.d9().onTrue(climber.setPositionL1().withName("xkeys-d9-true"));
    xkeys
        .d10()
        .whileTrue(climber.climbSequenceL3().withName("xkeys-d10-true"))
        .onFalse(climber.setVoltage(0).withName("xkeys-d10-false"));

    xkeys.e1().onTrue(intake.increaseSpeedOffset().withName("xkeys-e1-true"));
    xkeys.e2().onTrue(intake.decreaseSpeedOffset().withName("xkeys-e2-true"));

    xkeys
        .e8()
        .whileTrue(climber.clockwiseSlow().withName("xkeys-e8-while"))
        .onFalse(climber.setVoltage(0).withName("xkeys-e8-false"));

    xkeys
        .e9()
        .whileTrue(climber.counterClockwiseSlow().withName("xkeys-e9-while"))
        .onFalse(climber.setVoltage(0).withName("xkeys-e9-false"));

    xkeys
        .e10()
        .onTrue(
            V2_DeltaCompositeCommands.unClimbPostAuto(intake, climber).withName("xkeys-e10-true"));

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
                .setGoal(() -> V2_DeltaShooterConstants.ShooterGoal.STOW)
                .withName("xkeys-g4-true"))
        .onFalse(
            shooter
                .setGoal(() -> V2_DeltaShooterConstants.ShooterGoal.IDLE)
                .withName("xkeys-g4-false"));
    xkeys
        .g5()
        .whileTrue(
            V2_DeltaCompositeCommands.toggleHold(clopper, shooter)
                .withName("xkeys-g5-toggle")); // TODO: Confirm behavior

    xkeys.g6().onTrue(shooter.incrementTurretZero().withName("xkeys-g6-true"));
    xkeys.g7().onTrue(shooter.decrementTurretZero().withName("xkeys-g7-true"));

    xkeys.g9().onTrue(climber.resetClimberZero().withName("xkeys-g9-true"));
    xkeys.g10().onTrue(shooter.resetTurretZero().withName("xkeys-g10-true"));

    xkeys
        .h1()
        .or(xkeys.h2().or(xkeys.h3()))
        .whileTrue(
            shooter
                .setGoal(V2_DeltaShooterConstants.ShooterGoal.SCORE)
                .alongWith(clopper.intake())
                .withName("xkeys-h1-h2-h3-while"))
        .onFalse(
            V2_DeltaCompositeCommands.scoreOrFeedCommand(shooter, clopper)
                .withName("xkeys-h1-h2-h3-false"));

    // xkeys.h4().onTrue(); SLOW WRAP MODE
    // xkeys.h5().onTrue(); FAST WRAP MODE

    xkeys.h6().whileTrue(shooter.clockwiseSlow().withName("xkeys-h6-while"));
    xkeys.h7().whileTrue(shooter.counterClockwiseSlow().withName("xkeys-h7-while"));

    xkeys.h9().onTrue(intake.resetIntakeZero().withName("xkeys-h9-true"));

    xkeys
        .h10()
        .whileTrue(shooter.resetHoodZero().withName("xkeys-h10-while"))
        .onFalse(
            shooter
                .setGoal(() -> V2_DeltaShooterConstants.ShooterGoal.IDLE)
                .withName("xkeys-h10-false"));
  }

  @Override
  public void robotPeriodic() {
    V2_DeltaRobotState.periodic(
        drive.getRawGyroRotation(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        shooter.getTurretRotation(),
        shooter.isTurretWrapping(),
        drive.getMeasuredChassisSpeeds());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
