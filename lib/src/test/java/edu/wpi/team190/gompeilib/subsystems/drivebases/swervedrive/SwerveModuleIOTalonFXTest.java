package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
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
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class SwerveModuleIOTalonFXTest {

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

    // Reset PhoenixOdometryThread instance via reflection to ensure clean state
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
  public void testTalonFXIO() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    CANcoderConfigurator cancoderConfigurator = mock(CANcoderConfigurator.class);
    when(cancoderConfigurator.apply(any(CANcoderConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    StatusSignal<Angle> drivePositionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> driveVelocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Voltage> driveAppliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> driveSupplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> driveTorqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> driveTemperatureCelcius = mock(StatusSignal.class);
    StatusSignal<Double> driveVelocitySetpointRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Double> driveVelocityErrorRotationsPerSecond = mock(StatusSignal.class);

    StatusSignal<Angle> turnAbsolutePositionRotations = mock(StatusSignal.class);
    StatusSignal<Angle> turnPositionRotations = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> turnVelocityRotationsPerSecond = mock(StatusSignal.class);
    StatusSignal<Voltage> turnAppliedVolts = mock(StatusSignal.class);
    StatusSignal<Current> turnSupplyCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Current> turnTorqueCurrentAmps = mock(StatusSignal.class);
    StatusSignal<Temperature> turnTemperatureCelcius = mock(StatusSignal.class);
    StatusSignal<Double> turnPositionSetpointRotations = mock(StatusSignal.class);
    StatusSignal<Double> turnPositionErrorRotations = mock(StatusSignal.class);

    // Mock returns
    when(drivePositionRotations.getValueAsDouble()).thenReturn(10.0);
    when(driveVelocityRotationsPerSecond.getValueAsDouble()).thenReturn(2.5);
    when(driveAppliedVolts.getValueAsDouble()).thenReturn(5.5);
    when(driveSupplyCurrentAmps.getValueAsDouble()).thenReturn(12.0);
    when(driveTorqueCurrentAmps.getValueAsDouble()).thenReturn(11.5);
    when(driveTemperatureCelcius.getValueAsDouble()).thenReturn(35.0);
    when(driveVelocitySetpointRotationsPerSecond.getValueAsDouble()).thenReturn(3.0);
    when(driveVelocityErrorRotationsPerSecond.getValueAsDouble()).thenReturn(0.5);

    when(turnAbsolutePositionRotations.getValueAsDouble()).thenReturn(0.25);
    when(turnPositionRotations.getValueAsDouble()).thenReturn(0.125);
    when(turnVelocityRotationsPerSecond.getValueAsDouble()).thenReturn(1.2);
    when(turnAppliedVolts.getValueAsDouble()).thenReturn(4.2);
    when(turnSupplyCurrentAmps.getValueAsDouble()).thenReturn(8.0);
    when(turnTorqueCurrentAmps.getValueAsDouble()).thenReturn(7.8);
    when(turnTemperatureCelcius.getValueAsDouble()).thenReturn(40.0);
    when(turnPositionSetpointRotations.getValueAsDouble()).thenReturn(0.13);
    when(turnPositionErrorRotations.getValueAsDouble()).thenReturn(0.005);

    try (MockedConstruction<TalonFX> mockTalon =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  if (context.getCount() == 1) {
                    when(mock.getPosition()).thenReturn(drivePositionRotations);
                    when(mock.getVelocity()).thenReturn(driveVelocityRotationsPerSecond);
                    when(mock.getMotorVoltage()).thenReturn(driveAppliedVolts);
                    when(mock.getSupplyCurrent()).thenReturn(driveSupplyCurrentAmps);
                    when(mock.getTorqueCurrent()).thenReturn(driveTorqueCurrentAmps);
                    when(mock.getDeviceTemp()).thenReturn(driveTemperatureCelcius);
                    when(mock.getClosedLoopReference())
                        .thenReturn(driveVelocitySetpointRotationsPerSecond);
                    when(mock.getClosedLoopError())
                        .thenReturn(driveVelocityErrorRotationsPerSecond);
                  } else {
                    when(mock.getPosition()).thenReturn(turnPositionRotations);
                    when(mock.getVelocity()).thenReturn(turnVelocityRotationsPerSecond);
                    when(mock.getMotorVoltage()).thenReturn(turnAppliedVolts);
                    when(mock.getSupplyCurrent()).thenReturn(turnSupplyCurrentAmps);
                    when(mock.getTorqueCurrent()).thenReturn(turnTorqueCurrentAmps);
                    when(mock.getDeviceTemp()).thenReturn(turnTemperatureCelcius);
                    when(mock.getClosedLoopReference()).thenReturn(turnPositionSetpointRotations);
                    when(mock.getClosedLoopError()).thenReturn(turnPositionErrorRotations);
                  }
                  when(mock.setPosition(anyDouble(), anyDouble())).thenReturn(StatusCode.OK);
                  when(mock.setControl(any(com.ctre.phoenix6.controls.ControlRequest.class)))
                      .thenReturn(StatusCode.OK);
                });
        MockedConstruction<CANcoder> mockCANcoder =
            mockConstruction(
                CANcoder.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(cancoderConfigurator);
                  when(mock.getAbsolutePosition()).thenReturn(turnAbsolutePositionRotations);
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(StatusCode.OK);

      // We need separate setups for steer talon which returns turnPositionRotations,
      // turnVelocityRotationsPerSecond, etc.
      // So let's customize based on mock construction indexing if needed, or simply let the default
      // mock configuration handle it:
      // In constructor:
      // driveTalonFX = new TalonFX(constants.DriveMotorId, driveConstants.driveConfig.canBus()); ->
      // index 0
      // turnTalonFX = new TalonFX(constants.SteerMotorId, driveConstants.driveConfig.canBus()); ->
      // index 1
      // Since they are construction mocked, we can set index-specific stubs.
      SwerveModuleIOTalonFX io = new SwerveModuleIOTalonFX(driveConstants, moduleConstants);

      // Now stub the constructed objects specifically:
      TalonFX driveTalon = mockTalon.constructed().get(0);
      TalonFX turnTalon = mockTalon.constructed().get(1);

      when(driveTalon.getPosition()).thenReturn(drivePositionRotations);
      when(driveTalon.getVelocity()).thenReturn(driveVelocityRotationsPerSecond);
      when(driveTalon.getMotorVoltage()).thenReturn(driveAppliedVolts);
      when(driveTalon.getSupplyCurrent()).thenReturn(driveSupplyCurrentAmps);
      when(driveTalon.getTorqueCurrent()).thenReturn(driveTorqueCurrentAmps);
      when(driveTalon.getDeviceTemp()).thenReturn(driveTemperatureCelcius);
      when(driveTalon.getClosedLoopReference()).thenReturn(driveVelocitySetpointRotationsPerSecond);
      when(driveTalon.getClosedLoopError()).thenReturn(driveVelocityErrorRotationsPerSecond);

      when(turnTalon.getPosition()).thenReturn(turnPositionRotations);
      when(turnTalon.getVelocity()).thenReturn(turnVelocityRotationsPerSecond);
      when(turnTalon.getMotorVoltage()).thenReturn(turnAppliedVolts);
      when(turnTalon.getSupplyCurrent()).thenReturn(turnSupplyCurrentAmps);
      when(turnTalon.getTorqueCurrent()).thenReturn(turnTorqueCurrentAmps);
      when(turnTalon.getDeviceTemp()).thenReturn(turnTemperatureCelcius);
      when(turnTalon.getClosedLoopReference()).thenReturn(turnPositionSetpointRotations);
      when(turnTalon.getClosedLoopError()).thenReturn(turnPositionErrorRotations);

      SwerveModuleIO.ModuleIOInputs inputs = new SwerveModuleIO.ModuleIOInputs();
      io.updateInputs(inputs);

      // Verify inputs mapped correctly
      assertEquals(10.0 * 2 * Math.PI, inputs.drivePositionRadians, 1e-4);
      assertEquals(2.5 * 2 * Math.PI, inputs.driveVelocityRadiansPerSecond, 1e-4);
      assertEquals(5.5, inputs.driveAppliedVolts, 1e-4);
      assertEquals(12.0, inputs.driveSupplyCurrentAmps, 1e-4);
      assertEquals(11.5, inputs.driveTorqueCurrentAmps, 1e-4);
      assertEquals(35.0, inputs.driveTemperatureCelcius, 1e-4);
      assertEquals(3.0 * 2 * Math.PI, inputs.driveVelocitySetpointRadiansPerSecond, 1e-4);
      assertEquals(0.5 * 2 * Math.PI, inputs.driveVelocityErrorRadiansPerSecond, 1e-4);

      assertEquals(0.25, inputs.turnAbsolutePosition.getRotations(), 1e-4);
      assertEquals(0.125, inputs.turnPosition.getRotations(), 1e-4);
      assertEquals(1.2 * 2 * Math.PI, inputs.turnVelocityRadiansPerSecond, 1e-4);
      assertEquals(4.2, inputs.turnAppliedVolts, 1e-4);
      assertEquals(8.0, inputs.turnSupplyCurrentAmps, 1e-4);
      assertEquals(7.8, inputs.turnTorqueCurrentAmps, 1e-4);
      assertEquals(40.0, inputs.turnTemperatureCelcius, 1e-4);
      assertEquals(0.13, inputs.turnPositionSetpoint.getRotations(), 1e-4);
      assertEquals(0.005, inputs.turnPositionError.getRotations(), 1e-4);

      // Verify commands
      io.setDriveAmps(10.0);
      verify(driveTalon).setControl(any(TorqueCurrentFOC.class));

      io.setTurnAmps(8.0);
      verify(turnTalon).setControl(any(TorqueCurrentFOC.class));

      // Drive velocity closed loop (Voltage branch)
      io.setDriveVelocity(1.5, 0.2);
      verify(driveTalon).setControl(any(VelocityVoltage.class));

      // Turn position closed loop (Voltage branch)
      io.setTurnPosition(Rotation2d.fromRotations(0.2));
      verify(turnTalon).setControl(any(MotionMagicVoltage.class));

      // PID set
      io.setPID(1.0, 0.1, 2.0, 0.2);
      // Feedforward set
      io.setFeedforward(0.05, 0.12);
      // Current limit update
      io.updateCurrentLimits(30.0, 20.0);
    }
  }

  @Test
  public void testTalonFXIOTorqueCurrentFOCBranches() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class))).thenReturn(StatusCode.OK);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    CANcoderConfigurator cancoderConfigurator = mock(CANcoderConfigurator.class);
    when(cancoderConfigurator.apply(any(CANcoderConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);

    // Setup moduleConstants with TorqueCurrentFOC outputs
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        focConstants = SwerveDriveConstantsTest.createModuleConstants(1, 2, 3, 0.5, 0.5);
    focConstants.DriveMotorClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    focConstants.SteerMotorClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    try (MockedConstruction<TalonFX> mockTalon =
            mockConstruction(
                TalonFX.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getPosition()).thenReturn(mock(StatusSignal.class));
                  when(mock.getVelocity()).thenReturn(mock(StatusSignal.class));
                  when(mock.getMotorVoltage()).thenReturn(mock(StatusSignal.class));
                  when(mock.getSupplyCurrent()).thenReturn(mock(StatusSignal.class));
                  when(mock.getTorqueCurrent()).thenReturn(mock(StatusSignal.class));
                  when(mock.getDeviceTemp()).thenReturn(mock(StatusSignal.class));
                  when(mock.getClosedLoopReference()).thenReturn(mock(StatusSignal.class));
                  when(mock.getClosedLoopError()).thenReturn(mock(StatusSignal.class));
                  when(mock.setPosition(anyDouble(), anyDouble())).thenReturn(StatusCode.OK);
                  when(mock.setControl(any(com.ctre.phoenix6.controls.ControlRequest.class)))
                      .thenReturn(StatusCode.OK);
                });
        MockedConstruction<CANcoder> mockCANcoder =
            mockConstruction(
                CANcoder.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(cancoderConfigurator);
                  when(mock.getAbsolutePosition()).thenReturn(mock(StatusSignal.class));
                });
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(StatusCode.OK);

      SwerveModuleIOTalonFX io = new SwerveModuleIOTalonFX(driveConstants, focConstants);

      TalonFX driveTalon = mockTalon.constructed().get(0);
      TalonFX turnTalon = mockTalon.constructed().get(1);

      // Drive velocity closed loop (TorqueCurrentFOC branch)
      io.setDriveVelocity(1.5, 0.2);
      verify(driveTalon).setControl(any(VelocityTorqueCurrentFOC.class));

      // Turn position closed loop (TorqueCurrentFOC branch)
      io.setTurnPosition(Rotation2d.fromRotations(0.2));
      verify(turnTalon).setControl(any(MotionMagicTorqueCurrentFOC.class));
    }
  }
}
