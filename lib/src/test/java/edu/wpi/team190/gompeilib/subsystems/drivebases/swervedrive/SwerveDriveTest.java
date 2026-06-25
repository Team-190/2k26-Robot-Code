package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class SwerveDriveTest {

  private SwerveDriveConstants driveConstants;
  private SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      moduleConstants;

  private SwerveModuleIO flModuleIO;
  private SwerveModuleIO frModuleIO;
  private SwerveModuleIO blModuleIO;
  private SwerveModuleIO brModuleIO;

  private GyroIO lowFreqGyroIO;
  private GyroIOPigeon2 highFreqGyroIO;

  private Supplier<Pose2d> robotPoseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;

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

    flModuleIO = mock(SwerveModuleIO.class);
    frModuleIO = mock(SwerveModuleIO.class);
    blModuleIO = mock(SwerveModuleIO.class);
    brModuleIO = mock(SwerveModuleIO.class);

    setupModuleIO(flModuleIO);
    setupModuleIO(frModuleIO);
    setupModuleIO(blModuleIO);
    setupModuleIO(brModuleIO);

    lowFreqGyroIO = mock(GyroIO.class);
    highFreqGyroIO = mock(GyroIOPigeon2.class);

    setupGyroIO(lowFreqGyroIO);
    setupGyroIO(highFreqGyroIO);

    robotPoseSupplier = () -> new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    resetPoseConsumer = pose -> {};

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

  private void setupModuleIO(SwerveModuleIO io) {
    doAnswer(
            invocation -> {
              SwerveModuleIO.ModuleIOInputs in = invocation.getArgument(0);
              in.driveConnected = true;
              in.turnConnected = true;
              in.turnEncoderConnected = true;
              in.odometryTimestamps = new double[] {1.0};
              in.odometryDrivePositionsRadians = new double[] {10.0};
              in.odometryTurnPositions = new Rotation2d[] {Rotation2d.fromDegrees(30.0)};
              in.drivePositionRadians = 10.0;
              in.turnPosition = Rotation2d.fromDegrees(30.0);
              in.driveVelocityRadiansPerSecond = 5.0;
              return null;
            })
        .when(io)
        .updateInputs(any());
  }

  private void setupGyroIO(GyroIO io) {
    doAnswer(
            invocation -> {
              GyroIO.GyroIOInputs in = invocation.getArgument(0);
              in.connected = true;
              in.yawPosition = Rotation2d.fromDegrees(15.0);
              in.yawVelocityRadPerSec = 0.5;
              in.odometryYawTimestamps = new double[] {1.0};
              in.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(15.0)};
              return null;
            })
        .when(io)
        .updateInputs(any());

    doAnswer(
            invocation -> {
              GyroIO.GyroIOInputs in = invocation.getArgument(0);
              in.connected = true;
              in.yawPosition = Rotation2d.fromDegrees(15.0);
              in.yawVelocityRadPerSec = 0.5;
              in.odometryYawTimestamps = new double[] {1.0};
              in.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(15.0)};
              return null;
            })
        .when(io)
        .updateInputs(any(), any(), any());

    when(io.getYaw()).thenReturn(mock(StatusSignal.class));
  }

  @Test
  public void testSwerveDriveLowFrequencyGyro() {
    try (MockedStatic<AutoBuilder> mockAutoBuilder = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> mockLogger = mockStatic(Logger.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      mockDS.when(DriverStation::isDisabled).thenReturn(false);
      mockDS.when(DriverStation::getAlliance).thenReturn(Optional.of(DriverStation.Alliance.Red));

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      // Verify simple getters
      assertEquals(5.0, drive.getMaxLinearSpeedMetersPerSec(), 1e-6);
      assertEquals(5.0 / Math.hypot(0.5, 0.5), drive.getMaxAngularSpeedRadPerSec(), 1e-6);

      // Periodic execution
      drive.periodic();

      assertEquals(0.5, drive.getYawVelocity(), 1e-6);

      // Verify modules read
      verify(flModuleIO, atLeastOnce()).updateInputs(any());

      // runVelocity
      ChassisSpeeds targetSpeeds = new ChassisSpeeds(1.0, -1.0, 0.5);
      drive.runVelocity(targetSpeeds);
      verify(flModuleIO).setDriveVelocity(anyDouble(), anyDouble());

      // stop
      drive.stop();

      // stopWithX
      drive.stopWithX();

      // runVelocityTorque
      List<Vector<N2>> forces = new ArrayList<>();
      for (int i = 0; i < 4; i++) {
        forces.add(VecBuilder.fill(10.0, 5.0));
      }
      drive.runVelocityTorque(targetSpeeds, forces);

      // runCharacterization
      drive.runCharacterization(6.0);
      verify(flModuleIO, atLeastOnce()).setDriveAmps(6.0);

      // getModulePositions
      SwerveModulePosition[] positions = drive.getModulePositions();
      assertEquals(4, positions.length);
      assertEquals(10.0 * 0.05, positions[0].distanceMeters, 1e-6);

      // getWheelRadiusCharacterizationPositions
      double[] wheelRadPos = drive.getWheelRadiusCharacterizationPositions();
      assertEquals(4, wheelRadPos.length);
      assertEquals(10.0, wheelRadPos[0], 1e-6);

      // getFFCharacterizationVelocity
      assertEquals(5.0 / (2 * Math.PI), drive.getFFCharacterizationVelocity(), 1e-6);

      // setPIDGains
      drive.setPIDGains(2.0, 0.2, 3.0, 0.3);
      verify(flModuleIO).setPID(2.0, 0.2, 3.0, 0.3);

      // setFFGains
      drive.setFFGains(0.1, 0.15);
      verify(flModuleIO).setFeedforward(0.1, 0.15);

      // updateCurrentLimits
      drive.updateCurrentLimits(40.0, 30.0);
      verify(flModuleIO).updateCurrentLimits(40.0, 30.0);

      // setAutoControllers
      drive.setAutoControllers(driveConstants.driveGains, driveConstants.turnGains);

      // choreoDrive
      SwerveSample sample =
          new SwerveSample(
              0.0, 1.0, 2.0, 0.5, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0, new double[4], new double[4]);
      drive.choreoDrive(sample);

      // getFieldRelativeVelocity
      Translation2d fieldRelVel = drive.getFieldRelativeVelocity();
      assertNotNull(fieldRelVel);
    }
  }

  @Test
  public void testSwerveDriveHighFrequencyGyro() {
    try (MockedStatic<AutoBuilder> mockAutoBuilder = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> mockLogger = mockStatic(Logger.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class);
        MockedStatic<PhoenixOdometryThread> mockOdomThread =
            mockStatic(PhoenixOdometryThread.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      mockDS.when(DriverStation::isDisabled).thenReturn(true); // Test isDisabled branch

      PhoenixOdometryThread mockThread = mock(PhoenixOdometryThread.class);
      mockOdomThread.when(() -> PhoenixOdometryThread.getInstance(any())).thenReturn(mockThread);

      Queue<Double> tsQueue = new ArrayBlockingQueue<>(10);
      Queue<Double> yawQueue = new ArrayBlockingQueue<>(10);
      when(mockThread.makeTimestampQueue()).thenReturn(tsQueue);
      when(mockThread.registerSignal(any(StatusSignal.class))).thenReturn(yawQueue);

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              highFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      verify(mockThread).start();

      tsQueue.add(1.1);
      yawQueue.add(0.2);

      // Periodic call should consume high frequency inputs
      drive.periodic();

      assertEquals(0, tsQueue.size());
      assertEquals(0, yawQueue.size());
    }
  }

  @Test
  public void testSwerveDriveDisconnectedGyroFallback() {
    try (MockedStatic<AutoBuilder> mockAutoBuilder = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> mockLogger = mockStatic(Logger.class);
        MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));

      // Make gyro report disconnected to test kinematics fallback in periodic
      doAnswer(
              invocation -> {
                GyroIO.GyroIOInputs in = invocation.getArgument(0);
                in.connected = false;
                in.yawPosition = new Rotation2d();
                in.yawVelocityRadPerSec = 0.0;
                in.odometryYawTimestamps = new double[] {1.0};
                in.odometryYawPositions = new Rotation2d[] {new Rotation2d()};
                return null;
              })
          .when(lowFreqGyroIO)
          .updateInputs(any());

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      drive.periodic();
      assertNotNull(drive.getRawGyroRotation());
    }
  }
}
