package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;

/**
 * Physics sim implementation of module IO. Simulation is not vendor-specific, but the sim models
 * are configured using a set of module constants from Phoenix.
 *
 * <p>Simulation is always based on voltage control.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop;
  private boolean turnClosedLoop;

  private final PIDController driveController;
  private final PIDController turnController;

  private double driveFFVolts;
  private double driveAppliedVolts;
  private double turnAppliedVolts;

  public SwerveModuleIOSim(
      SwerveDriveConstants driveConstants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                driveConstants.driveConfig.driveModel(),
                constants.DriveInertia,
                constants.DriveMotorGearRatio),
            driveConstants.driveConfig.driveModel());
    turnSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                driveConstants.driveConfig.turnModel(),
                constants.SteerInertia,
                constants.SteerMotorGearRatio),
            driveConstants.driveConfig.turnModel());

    driveClosedLoop = false;
    turnClosedLoop = false;

    driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    turnController = new PIDController(TURN_KP, 0, TURN_KD);

    driveFFVolts = 0.0;
    driveAppliedVolts = 0.0;
    turnAppliedVolts = 0.0;

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(GompeiLib.getLoopPeriod());
    turnSim.update(GompeiLib.getLoopPeriod());

    inputs.drivePositionRadians = driveSim.getAngularPosition();
    inputs.driveVelocityRadiansPerSecond = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDraw());
    inputs.driveVelocitySetpointRadiansPerSecond = driveController.getSetpoint();
    inputs.driveVelocityErrorRadiansPerSecond = driveController.getError();

    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadiansPerSecond = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDraw());
    inputs.turnPositionGoal = Rotation2d.fromRadians(turnController.getSetpoint());
    inputs.turnPositionSetpoint = Rotation2d.fromRadians(turnController.getSetpoint());
    inputs.turnPositionError = Rotation2d.fromRadians(turnController.getError());

    inputs.driveConnected = true;
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;

    inputs.odometryTimestamps = new double[] {Timer.getTimestamp()};
    inputs.odometryDrivePositionsRadians = new double[] {inputs.drivePositionRadians};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveAmps(double currentAmps) {
    driveClosedLoop = false;
    driveAppliedVolts = currentAmps;
  }

  @Override
  public void setTurnAmps(double currentAmps) {
    turnClosedLoop = false;
    turnAppliedVolts = currentAmps;
  }

  @Override
  public void setDriveVelocity(double velocityRadiansPerSecond, double currentFeedforward) {
    driveClosedLoop = true;
    driveFFVolts =
        DRIVE_KS * Math.signum(velocityRadiansPerSecond) + DRIVE_KV * velocityRadiansPerSecond;
    driveController.setSetpoint(velocityRadiansPerSecond);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnClosedLoop = true;
    turnController.setSetpoint(position.getRadians());
  }
}
