package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.tunable.TunableUpdaterRegistry;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.commands.shared.DriveCommands;
import frc.robot.commands.shared.SharedCompositeCommands;
import frc.robot.commands.v1_DoomSpiral.V1_DoomSpiralCompositeCommands;
import frc.robot.commands.v2_Delta.autonomous.V2_TurretTestAuto;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIO;
import frc.robot.subsystems.shared.fourbarlinkage.FourBarLinkageIOSim;
import frc.robot.subsystems.shared.hood.HoodIO;
import frc.robot.subsystems.shared.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.shared.intake.Intake;
import frc.robot.subsystems.shared.intake.IntakeConstants;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.shared.turret.TurretIOSim;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralConstants;
import frc.robot.subsystems.v1_DoomSpiral.V1_DoomSpiralRobotState;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;
import frc.robot.subsystems.v2_Delta.leds.V2_DeltaCANdle;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.subsystems.v2_Delta.V2_DeltaConstants;
import frc.robot.commands.v2_Delta.V2_DeltaCompositeCommands;

import frc.robot.util.BetterAutoChooser;
import frc.robot.util.input.XboxElite2Input;
import java.util.List;

public class V2_DeltaRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private Climber climber;
  private Intake intake;
  private V2_DeltaClopper clopper;
  private Vision vision;
  private V2_DeltaCANdle leds;
  private V2_DeltaShooter shooter;

  private final BetterAutoChooser autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);

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
          //          climber =
          //              new Climber(
          //                  new ArmIOTalonFX(ClimberConstants.CLIMBER_CONSTANTS),
          //                  gyroIO.getRoll().asSupplier());
          //          intake =
          //              new Intake(
          //                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS),
          //                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS));
          //          clopper =
          //              new V2_DeltaClopper(
          //                  new
          // GenericRollerIOTalonFX(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
          //                  new
          // GenericRollerIOTalonFX(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          //          shooter =
          //              new V2_DeltaShooter(
          //                  new TurretIOTalonFX(V2_DeltaShooterConstants.TURRET_CONSTANTS),
          //                  new HoodIOTalonFX(V2_DeltaShooterConstants.HOOD_CONSTANTS),
          //                  new
          // GenericFlywheelIOTalonFX(V2_DeltaShooterConstants.SHOOT_CONSTANTS));
          //          leds = new V2_DeltaCANdle();

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG,
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
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS));
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          leds = new V2_DeltaCANdle();

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V2_DeltaConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V2_DeltaRobotState::getHeading,
                      drive::getMeasuredChassisSpeeds,
                      V2_DeltaRobotState::getHeadingUpdateTimestamp,
                      List.of(V2_DeltaRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          shooter =
              new V2_DeltaShooter(
                  new TurretIOSim(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFXSim(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFXSim(V2_DeltaShooterConstants.SHOOT_CONSTANTS));
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
      intake = new Intake(new GenericRollerIO() {}, new FourBarLinkageIO() {});
    }
    if (clopper == null) {
      clopper = new V2_DeltaClopper(new GenericRollerIO() {}, new GenericRollerIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }
    if (shooter == null) {
      shooter = new V2_DeltaShooter(new TurretIO() {}, new HoodIO() {}, new GenericFlywheelIO() {});
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
  }


  private void configureAutos() {
    autoChooser.addRoutineConfig("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive, shooter));
    autoChooser.addCmd(
        "Drive Feedforward Characterization",
        () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive, V2_DeltaConstants.DRIVE_CONSTANTS));
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    V2_DeltaRobotState.periodic(
        drive.getRawGyroRotation(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        shooter.getTurretRotation(),
        shooter.isTurretWrapping());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  @Trace
  private void configureButtonBindings() {
      drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            V2_DeltaConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            V2_DeltaRobotState::getHeading));

    driver
        .povDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V2_DeltaRobotState::resetPose,
                () -> V2_DeltaRobotState.getGlobalPose().getTranslation()));
                
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
                driver.leftTrigger().or(driver.x()),
                driver.x(),
                0.5)
            .withName("joystickDriveRotationLock"));

    driver.leftTrigger()
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

    driver.leftBumper().onTrue(intake.collect().withName("driver-leftBumper-true"));


    driver.rightBumper().whileTrue(V2_DeltaCompositeCommands.hold(clopper, shooter).withName("driver-rightBumper-while"));

    driver
        .x()
        .onTrue(
            V2_DeltaCompositeCommands.deployClimber(intake, climber)
                .withName("driver-x-true"));

    driver
        .y()
        .whileTrue(climber.climbSequenceL3().withName("driver-y-while"))
        .onFalse(climber.stop().withName("driver-y-false"));

    driver
        .rightTrigger()
        .whileTrue(
            V2_DeltaCompositeCommands.scoreCommand(shooter, intake, clopper)
                .withName("driver-rightTrigger-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-rightTrigger-false"));

    driver
        .topLeftPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.LEFT_TRENCH.getParameters())
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .topRightPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.RIGHT_TRENCH.getParameters())
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .bottomLeftPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.LEFT_CORNER.getParameters())
                .withName("driver-topLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-topLeftPaddle-false"));

    driver
        .bottomRightPaddle()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.RIGHT_CORNER.getParameters())
                .withName("driver-topRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-topRightPaddle-false"));

    driver
        .a()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.HUB.getParameters())
                .withName("driver-bottomLeftPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-bottomLeftPaddle-false"));

    driver
        .b()
        .whileTrue(
            V2_DeltaCompositeCommands.fixedShotCommand(
                    drive,
                    shooter,
                    clopper,
                    intake,
                    V2_DeltaRobotState.FixedShots.TOWER.getParameters())
                .withName("driver-bottomRightPaddle-while"))
        .onFalse(
            V2_DeltaCompositeCommands.stopShooterCommand(shooter, clopper)
                .withName("driver-bottomRightPaddle-false"));
  }
}
