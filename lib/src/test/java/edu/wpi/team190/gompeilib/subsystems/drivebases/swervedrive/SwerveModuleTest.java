package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class SwerveModuleTest {

  private SwerveDriveConstants driveConstants;
  private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      moduleConstants;

  @BeforeEach
  public void setUp() {
    edu.wpi.first.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    try {
      java.lang.reflect.Field instanceField =
          PhoenixOdometryThread.class.getDeclaredField("instance");
      instanceField.setAccessible(true);
      instanceField.set(null, null);
    } catch (Exception e) {
    }

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

    Gains driveGains =
        Gains.builder()
            .withKP(new LoggedTunableNumber("kp_drive", 1.0))
            .withKD(new LoggedTunableNumber("kd_drive", 0.1))
            .build();

    Gains turnGains =
        Gains.builder()
            .withKP(new LoggedTunableNumber("kp_turn", 2.0))
            .withKD(new LoggedTunableNumber("kd_turn", 0.2))
            .build();

    SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
        SwerveDriveConstants.AutoAlignConstants.builder()
            .withXGains(driveGains)
            .withXConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(new LoggedTunableMeasure<>("max_v_x", MetersPerSecond.of(1.0)))
                    .withMaxAcceleration(
                        new LoggedTunableMeasure<>("max_a_x", MetersPerSecondPerSecond.of(1.0)))
                    .withGoalTolerance(new LoggedTunableMeasure<>("tol_x", Meters.of(0.05)))
                    .build())
            .withYGains(driveGains)
            .withYConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(new LoggedTunableMeasure<>("max_v_y", MetersPerSecond.of(1.0)))
                    .withMaxAcceleration(
                        new LoggedTunableMeasure<>("max_a_y", MetersPerSecondPerSecond.of(1.0)))
                    .withGoalTolerance(new LoggedTunableMeasure<>("tol_y", Meters.of(0.05)))
                    .build())
            .withRotationGains(turnGains)
            .withRotationConstraints(
                AngularPositionConstraints.builder()
                    .withMaxVelocity(
                        new LoggedTunableMeasure<>("max_v_rot", RadiansPerSecond.of(1.0)))
                    .withMaxAcceleration(
                        new LoggedTunableMeasure<>("max_a_rot", RadiansPerSecondPerSecond.of(1.0)))
                    .withGoalTolerance(new LoggedTunableMeasure<>("tol_rot", Radians.of(0.05)))
                    .build())
            .withLinearThreshold(new LoggedTunableMeasure<>("lin_thresh", Meters.of(0.01)))
            .withAngularThreshold(new LoggedTunableMeasure<>("ang_thresh", Radians.of(0.01)))
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
  public void testSwerveModule() {
    SwerveModuleIO io = mock(SwerveModuleIO.class);

    // Stub updateInputs to populate the ModuleIOInputs fields
    doAnswer(
            invocation -> {
              SwerveModuleIO.ModuleIOInputs in = invocation.getArgument(0);
              in.driveConnected = true;
              in.turnConnected = true;
              in.turnEncoderConnected = true;
              in.odometryTimestamps = new double[] {1.5};
              in.odometryDrivePositionsRadians = new double[] {20.0};
              in.odometryTurnPositions = new Rotation2d[] {Rotation2d.fromDegrees(30.0)};
              in.drivePositionRadians = 20.0;
              in.driveVelocityRadiansPerSecond = 5.0;
              in.turnPosition = Rotation2d.fromDegrees(30.0);
              return null;
            })
        .when(io)
        .updateInputs(any());

    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {
      SwerveModule module = new SwerveModule(driveConstants, io, 0);

      module.updateInputs();
      verify(io).updateInputs(any());

      module.periodic();

      // Check odometry calculations
      SwerveModulePosition[] positions = module.getOdometryPositions();
      assertEquals(1, positions.length);
      assertEquals(20.0 * 0.05, positions[0].distanceMeters, 1e-6);
      assertEquals(Rotation2d.fromDegrees(30.0), positions[0].angle);

      // Verify getters
      assertEquals(Rotation2d.fromDegrees(30.0), module.getAngle());
      assertEquals(20.0 * 0.05, module.getPositionMeters(), 1e-6);
      assertEquals(5.0 * 0.05, module.getVelocityMetersPerSec(), 1e-6);
      assertEquals(20.0, module.getWheelRadiusCharacterizationPosition(), 1e-6);
      assertEquals(5.0 / (2 * Math.PI), module.getFFCharacterizationVelocity(), 1e-6);
      assertArrayEquals(new double[] {1.5}, module.getOdometryTimestamps());

      SwerveModuleState state = module.getState();
      assertEquals(5.0 * 0.05, state.speedMetersPerSecond, 1e-6);
      assertEquals(Rotation2d.fromDegrees(30.0), state.angle);

      SwerveModulePosition position = module.getPosition();
      assertEquals(20.0 * 0.05, position.distanceMeters, 1e-6);
      assertEquals(Rotation2d.fromDegrees(30.0), position.angle);

      // Verify commands
      SwerveModuleState setpoint = new SwerveModuleState(2.0, Rotation2d.fromDegrees(60.0));
      SwerveModuleState feedforward = new SwerveModuleState(0.5, Rotation2d.fromDegrees(60.0));
      module.runSetpoint(setpoint, feedforward);
      verify(io).setDriveVelocity(eq(34.64101615137755), anyDouble());
      verify(io).setTurnPosition(any(Rotation2d.class));

      module.runCharacterization(12.0);
      verify(io).setDriveAmps(12.0);

      module.stop();
      verify(io).setDriveAmps(0.0);
      verify(io).setTurnAmps(0.0);

      module.setPID(1.0, 0.1, 2.0, 0.2);
      verify(io).setPID(1.0, 0.1, 2.0, 0.2);

      module.setFF(0.05, 0.1);
      verify(io).setFeedforward(0.05, 0.1);

      module.updateCurrentLimits(30.0, 20.0);
      verify(io).updateCurrentLimits(30.0, 20.0);
    }
  }

  @Test
  public void testSwerveModuleDisconnectedAlerts() {
    SwerveModuleIO io = mock(SwerveModuleIO.class);

    // Stub to report disconnected
    doAnswer(
            invocation -> {
              SwerveModuleIO.ModuleIOInputs in = invocation.getArgument(0);
              in.driveConnected = false;
              in.turnConnected = false;
              in.turnEncoderConnected = false;
              in.odometryTimestamps = new double[0];
              in.odometryDrivePositionsRadians = new double[0];
              in.odometryTurnPositions = new Rotation2d[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {
      SwerveModule module = new SwerveModule(driveConstants, io, 1);
      module.updateInputs();
      module.periodic();

      assertEquals(0, module.getOdometryPositions().length);
    }
  }
}
