package first.robot.v0_Funky;

import static org.wpilib.units.Units.Volts;

import choreo.auto.AutoChooser;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import first.robot.Constants;
import first.robot.FieldConstants;
import first.robot.RobotConfig;
import first.robot.shared.commands.DriveCommands;
import first.robot.shared.commands.SharedCompositeCommands;
import first.robot.shared.subsystems.turret.TurretIO;
import first.robot.shared.subsystems.turret.TurretIOSim;
import first.robot.shared.subsystems.turret.TurretIOTalonFX;
import first.robot.shared.util.AllianceFlipUtil;
import first.robot.shared.util.input.XKeysInput;
import first.robot.v0_Funky.subsystems.feeder.Feeder;
import first.robot.v0_Funky.subsystems.feeder.FeederConstants;
import first.robot.v0_Funky.subsystems.shooter.V0_FunkyShooter;
import first.robot.v0_Funky.subsystems.shooter.V0_FunkyShooterConstants;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;

public class V0_FunkyRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V0_FunkyShooter shooter;
  private Feeder feeder;
  private Vision vision;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  private final XKeysInput xkeys = new XKeysInput(1);

  public V0_FunkyRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V0_FUNKY:
          drive =
              new SwerveDrive(
                  V0_FunkyConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyRobotState::setNetworktablesTimestamp),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V0_FunkyRobotState::getGlobalPose,
                  V0_FunkyRobotState::resetPose);
          shooter =
              new V0_FunkyShooter(
                  new GenericFlywheelIOTalonFXSim(V0_FunkyShooterConstants.SHOOT_CONSTANTS),
                  new TurretIOTalonFX(V0_FunkyShooterConstants.TURRET_CONSTANTS));
          // feeder = new Feeder(new GenericRollerIOTalonFX(V0_FunkyConstants.FEED_CONSTANTS));
          vision =
              new Vision(
                  () -> Field.loadField(Fields.FRC_2026_REBUILT_ANDY_MARK),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V0_FunkyConstants.LIMELIGHT_CONFIG),
                      V0_FunkyConstants.LIMELIGHT_CONFIG,
                      V0_FunkyRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      V0_FunkyRobotState::getNetworktablesTimestamp,
                      List.of(V0_FunkyRobotState::addFieldLocalizerVisionMeasurement),
                      List.of()));
          break;

        case V0_FUNKY_SIM:
          drive =
              new SwerveDrive(
                  V0_FunkyConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V0_FunkyConstants.DRIVE_CONSTANTS,
                      V0_FunkyConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  () -> Pose2d.ZERO,
                  V0_FunkyRobotState::resetPose);
          shooter =
              new V0_FunkyShooter(
                  new GenericFlywheelIOSim(V0_FunkyShooterConstants.SHOOT_CONSTANTS),
                  new TurretIOSim(V0_FunkyShooterConstants.TURRET_CONSTANTS));
          feeder = new Feeder(new GenericRollerIOSim(FeederConstants.FEEDER_CONSTANTS));
          vision = new Vision(() -> Field.loadField(Fields.FRC_2025_REEFSCAPE_ANDY_MARK));

          break;

        default:
          break;
      }
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              V0_FunkyConstants.DRIVE_CONSTANTS,
              new GyroIOPigeon2(
                  V0_FunkyConstants.DRIVE_CONSTANTS, V0_FunkyRobotState::setNetworktablesTimestamp),
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V0_FunkyRobotState::getGlobalPose,
              V0_FunkyRobotState::resetPose);
    }

    if (shooter == null) {
      shooter = new V0_FunkyShooter(new GenericFlywheelIO() {}, new TurretIO() {});
    }

    if (feeder == null) {
      feeder = new Feeder(new GenericRollerIO() {});
    }

    if (vision == null) {
      new Vision(() -> Field.loadField(Fields.FRC_2025_REEFSCAPE_ANDY_MARK));
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            V0_FunkyConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            V0_FunkyRobotState::getHeading));

    driver
        .dpadDown()
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));
    driver
        .x()
        .onTrue(shooter.setTurretGoal(() -> V0_FunkyRobotState.getHeading()))
        .onFalse(shooter.setTurretVoltage(Volts.zero()));

    driver
        .y()
        .whileTrue(
            shooter.setTurretGoalPose(
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())))
        .onFalse(shooter.setTurretVoltage(Volts.zero()));

    xkeys
        .a1()
        .or(xkeys.a2())
        .or(xkeys.a3())
        .or(xkeys.a4())
        .onTrue(
            SharedCompositeCommands.resetHeading(
                drive,
                V0_FunkyRobotState::resetPose,
                () -> V0_FunkyRobotState.getGlobalPose().getTranslation()));

    // xkeys
    //     .a5()
    //     .or(xkeys.a6())
    //     .or(xkeys.a7())
    //     .whileTrue(Commands.run(() -> shooter.setVoltage(V0_FunkyConstants.SHOOTER_VOLTAGE)))
    //     .whileFalse(Commands.run(() -> shooter.setVoltage(0)));

    xkeys
        .c8()
        .or(xkeys.c9())
        .or(xkeys.c10())
        .whileTrue(Commands.run(() -> feeder.setVoltage(V0_FunkyConstants.FEEDER_VOLTAGE)))
        .whileFalse(Commands.runOnce(() -> feeder.setVoltage(0)));
  }

  private void configureAutos() {
    // Autos here
  }

  @Override
  public void robotPeriodic() {
    V0_FunkyRobotState.periodic(
        drive.getRawGyroRotation(),
        drive.getModulePositions(),
        drive.getMeasuredChassisVelocities());
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
