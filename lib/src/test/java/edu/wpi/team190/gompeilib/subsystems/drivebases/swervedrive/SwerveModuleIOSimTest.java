package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;

public class SwerveModuleIOSimTest {

  private SwerveDriveConstants driveConstants;
  private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      moduleConstants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    CANBus canBus = new CANBus("rio");
    DCMotor driveModel = DCMotor.getKrakenX60(1);
    DCMotor turnModel = DCMotor.getKrakenX44(1);

    moduleConstants = SwerveDriveConstantsTest.createModuleConstants(1, 2, 3, 0.5, 0.5);

    SwerveDriveConstants.DriveConfig driveConfig =
        SwerveDriveConstants.DriveConfig.builder()
            .withCanBus(canBus)
            .withPigeon2Id(13)
            .withMaxLinearVelocityMetersPerSecond(5.0)
            .withWheelRadiusMeters(0.05)
            .withDriveModel(driveModel)
            .withTurnModel(turnModel)
            .withFrontLeft(moduleConstants)
            .withFrontRight(moduleConstants)
            .withBackLeft(moduleConstants)
            .withBackRight(moduleConstants)
            .withDriveClosedLoopOutputType(ClosedLoopOutputType.Voltage)
            .withSteerClosedLoopOutputType(ClosedLoopOutputType.Voltage)
            .withBumperWidth(0.8)
            .withBumperLength(0.8)
            .withRobotMassKilograms(45.0)
            .withTrackWidth(0.6)
            .withRobotMOI(2.5)
            .withModuleCurrentLimit(40.0)
            .withWheelCOF(1.2)
            .build();

    Gains driveGains = Gains.builder().withKP(1.0).withKD(0.1).build();

    Gains turnGains = Gains.builder().withKP(2.0).withKD(0.2).build();

    SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
        SwerveDriveConstants.AutoAlignConstants.builder()
            .withXGains(driveGains)
            .withXConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withYGains(driveGains)
            .withYConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withRotationGains(turnGains)
            .withRotationConstraints(
                AngularPositionConstraints.builder()
                    .withMaxVelocity(RadiansPerSecond.of(1.0))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Radians.of(0.05))
                    .build())
            .withLinearThreshold(Tunable.create(Meters.of(0.01)))
            .withAngularThreshold(Tunable.create(Radians.of(0.01)))
            .build();

    driveConstants =
        SwerveDriveConstants.builder()
            .withDriveConfig(driveConfig)
            .withDriveGains(driveGains)
            .withTurnGains(turnGains)
            .withAutoTranslationGains(driveGains)
            .withAutoRotationGains(turnGains)
            .withAutoAlignConstants(autoAlignConstants)
            .withOdometryFrequency(250.0)
            .withDriverDeadband(0.1)
            .withOperatorDeadband(0.1)
            .build();
  }

  @Test
  public void testSimIO() {
    SwerveModuleIOSim sim = new SwerveModuleIOSim(driveConstants, moduleConstants);
    SwerveModuleIO.ModuleIOInputs inputs = new SwerveModuleIO.ModuleIOInputs();

    // Initial state
    sim.updateInputs(inputs);
    assertTrue(inputs.driveConnected);
    assertTrue(inputs.turnConnected);
    assertTrue(inputs.turnEncoderConnected);
    assertEquals(0.0, inputs.drivePositionRadians, 1e-6);
    assertEquals(0.0, inputs.driveVelocityRadiansPerSecond, 1e-6);

    // Test setDriveAmps
    sim.setDriveAmps(6.0); // open loop applied volts
    sim.updateInputs(inputs);
    assertEquals(6.0, inputs.driveAppliedVolts, 1e-6);
    assertTrue(inputs.drivePositionRadians > 0.0 || inputs.driveVelocityRadiansPerSecond > 0.0);

    // Test setTurnAmps
    sim.setTurnAmps(-4.0);
    sim.updateInputs(inputs);
    assertEquals(-4.0, inputs.turnAppliedVolts, 1e-6);

    // Test setDriveVelocity
    sim.setDriveVelocity(10.0, 1.0); // closed loop velocity
    sim.updateInputs(inputs);
    assertEquals(10.0, inputs.driveVelocitySetpointRadiansPerSecond, 1e-6);

    // Test setTurnPosition
    sim.setTurnPosition(Rotation2d.fromDegrees(90));
    sim.updateInputs(inputs);
    assertEquals(Math.PI / 2.0, inputs.turnPositionGoal.getRadians(), 1e-6);
  }
}
