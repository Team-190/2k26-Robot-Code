package frc.robot.subsystems.v2_Delta;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFXSim;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
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
import frc.robot.subsystems.shared.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shared.turret.TurretIO;
import frc.robot.subsystems.shared.turret.TurretIOSim;
import frc.robot.subsystems.shared.turret.TurretIOTalonFX;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopper;
import frc.robot.subsystems.v2_Delta.clopper.V2_DeltaClopperConstants;
import frc.robot.subsystems.v2_Delta.leds.V2_DeltaCANdle;
import frc.robot.subsystems.v2_Delta.shooter.FuelSimulator;
import frc.robot.subsystems.v2_Delta.shooter.SimFuelCount;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooter;
import frc.robot.subsystems.v2_Delta.shooter.V2_DeltaShooterConstants;
import frc.robot.util.BetterAutoChooser;
import java.util.List;

public class V2_DeltaRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private SwerveDrive drive;
  private Climber climber;
  private V2_DeltaClopper clopper;
  private Vision vision;
  private V2_DeltaCANdle leds;
  private V2_DeltaShooter shooter;
  private Intake intake;
  private FuelSimulator fuelSimulator;
  private SimFuelCount simFuelCount;

  private final BetterAutoChooser autoChooser;

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
          climber =
              new Climber(
                  new ArmIOTalonFX(ClimberConstants.CLIMBER_CONSTANTS),
                  gyroIO.getRoll().asSupplier());
          intake =
              new Intake(
                  new GenericRollerIOTalonFX(IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new FourBarLinkageIOTalonFX(IntakeConstants.LINKAGE_CONSTANTS));
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFX(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));
          shooter =
              new V2_DeltaShooter(
                  new TurretIOTalonFX(V2_DeltaShooterConstants.TURRET_CONSTANTS),
                  new HoodIOTalonFX(V2_DeltaShooterConstants.HOOD_CONSTANTS),
                  new GenericFlywheelIOTalonFX(V2_DeltaShooterConstants.SHOOT_CONSTANTS));
          leds = new V2_DeltaCANdle();

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraLimelight(
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
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS));
          clopper =
              new V2_DeltaClopper(
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.ROLLER_FLOOR_CONSTANTS),
                  new GenericRollerIOTalonFXSim(V2_DeltaClopperConstants.BALL_TUNNEL_CONSTANTS));

          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraLimelight(
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
                  new CameraLimelight(
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
          intake =
              new Intake(
                  new GenericRollerIOSim(IntakeConstants.INTAKE_ROLLER_CONSTANTS_TOP),
                  new FourBarLinkageIOSim(IntakeConstants.LINKAGE_CONSTANTS));
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
    if (leds == null) {
      leds = new V2_DeltaCANdle();
    }
    if (shooter == null) {
      shooter = new V2_DeltaShooter(new TurretIO() {}, new HoodIO() {}, new GenericFlywheelIO() {});
    }
    if (intake == null) {
      intake = new Intake(new GenericRollerIO() {}, new FourBarLinkageIO() {});
    }

    autoChooser = new BetterAutoChooser(V2_DeltaRobotState::resetPose);
    configureAutos();
    configureButtonBindings();
    configureFuelSim();
  }

  private void configureButtonBindings() {
    //

  }

  private void configureFuelSim() {

    fuelSimulator = new FuelSimulator("FuelSim");
    simFuelCount = new SimFuelCount(8);

    fuelSimulator.registerRobot(
        V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth(),
        V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.bumperLength(),
        Units.inchesToMeters(6.0),
        V2_DeltaRobotState::getGlobalPose,
        () ->
            new ChassisSpeeds(
                drive.getFieldRelativeVelocity().getX(),
                drive.getFieldRelativeVelocity().getY(),
                drive.getMeasuredChassisSpeeds().omegaRadiansPerSecond));

    fuelSimulator.registerIntake(
        IntakeConstants.LINKAGE_OFFSET.getX() - Units.inchesToMeters(4),
        IntakeConstants.LINKAGE_OFFSET.getX(),
        -V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
        V2_DeltaConstants.DRIVE_CONSTANTS.driveConfig.bumperWidth() / 2,
        () ->
            intake.getIntakeState().equals(IntakeState.INTAKE)
                && intake.atGoal()
                && simFuelCount.getFuelStored() < SimFuelCount.getCapacity(),
        () ->
            simFuelCount.setFuelStored(
                Math.min(simFuelCount.getFuelStored() + 1, SimFuelCount.getCapacity())));

    fuelSimulator.registerShooter(
        () -> false,
        () -> {},
        shooter.getHoodAngle()::getMeasure,
        shooter.getTurretRotation()::getMeasure,
        shooter::getFlywheelVelocity,
        V2_DeltaConstants.ROBOT_TO_SHOOTER_TRANSFORM.getMeasureZ());

    fuelSimulator.setSubticks(1);
    fuelSimulator.start();
    fuelSimulator.spawnStartingFuel();
    fuelSimulator.enableAirResistance();

    if (RobotBase.isSimulation()) {
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

  private void configureAutos() {
    autoChooser.addRoutineConfig("Turret Test", V2_TurretTestAuto.getAutoRoutine(drive, shooter));
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

    fuelSimulator.updateSim();
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
