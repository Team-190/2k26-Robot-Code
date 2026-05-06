package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class SwerveModuleIOTalonFXSim extends SwerveModuleIOTalonFX {

  private final DCMotorSim steerMotorSim;
  private final DCMotorSim driveMotorSim;

  private final TalonFXSimState steerController;
  private final TalonFXSimState driveController;
  private final CANcoderSimState encoderController;
  private final double offset;

  public SwerveModuleIOTalonFXSim(
      SwerveDriveConstants driveConstants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {

    super(driveConstants, constants);
    driveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveConstants.driveConfig.driveModel(),
                constants.DriveInertia,
                constants.DriveMotorGearRatio),
            driveConstants.driveConfig.driveModel());
    steerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveConstants.driveConfig.turnModel(),
                constants.SteerInertia,
                constants.SteerMotorGearRatio),
            driveConstants.driveConfig.turnModel());

    steerController = super.turnTalonFX.getSimState();
    driveController = super.driveTalonFX.getSimState();
    encoderController = super.cancoder.getSimState();

    offset = constants.EncoderOffset;
  }

  @Override
  @Trace
  public void updateInputs(ModuleIOInputs inputs) {
    driveController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltageDrive = driveController.getMotorVoltage();

    driveMotorSim.setInputVoltage(motorVoltageDrive);

    driveMotorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotationsDrive =
        driveMotorSim.getAngularPositionRotations() * driveMotorSim.getGearing();
    double rotorVelocityRotationsPerSecondDrive =
        driveMotorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * driveMotorSim.getGearing();
    driveController.setRawRotorPosition(rotorPositionRotationsDrive);
    driveController.setRotorVelocity(rotorVelocityRotationsPerSecondDrive);

    steerController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltageSteer = steerController.getMotorVoltage();

    steerMotorSim.setInputVoltage(motorVoltageSteer);

    steerMotorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotationsSteer =
        steerMotorSim.getAngularPositionRotations() * steerMotorSim.getGearing();
    double rotorVelocityRotationsPerSecondSteer =
        steerMotorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * steerMotorSim.getGearing();
    steerController.setRawRotorPosition(rotorPositionRotationsSteer);
    steerController.setRotorVelocity(rotorVelocityRotationsPerSecondSteer);

    encoderController.setRawPosition(steerMotorSim.getAngularPositionRotations() + offset);

    super.updateInputs(inputs);
  }
}
