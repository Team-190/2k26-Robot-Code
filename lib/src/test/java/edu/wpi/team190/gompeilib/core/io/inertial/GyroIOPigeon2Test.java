package edu.wpi.team190.gompeilib.core.io.inertial;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;

public class GyroIOPigeon2Test {
  @BeforeEach
  public void setUp() {
    GompeiLib.deinit();
    GompeiLib.init(RobotMode.SIM, false, 0.02);
  }

  @Test
  public void testGyroIOPigeon2() {
    SwerveDriveConstants driveConstants = mock(SwerveDriveConstants.class);
    CANBus canBus = mock(CANBus.class);
    when(canBus.isNetworkFD()).thenReturn(false);

    DCMotor motor = mock(DCMotor.class);
    SwerveModuleConstants moduleConst = mock(SwerveModuleConstants.class);
    SwerveModuleConstants.ClosedLoopOutputType outputType =
        SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    SwerveDriveConstants.DriveConfig driveConfig =
        new SwerveDriveConstants.DriveConfig(
            canBus,
            1, // pigeon2Id
            4.0, // maxLinearVelocityMetersPerSecond
            0.05, // wheelRadiusMeters
            motor,
            motor,
            moduleConst,
            moduleConst,
            moduleConst,
            moduleConst,
            outputType,
            outputType,
            0.8, // bumperWidth
            0.8, // bumperLength
            40.0, // robotMassKilograms
            0.6, // trackWidth
            2.0, // robotMOI
            40.0, // moduleCurrentLimit
            1.0 // wheelCOF
            );

    try {
      java.lang.reflect.Field configField =
          SwerveDriveConstants.class.getDeclaredField("driveConfig");
      configField.setAccessible(true);
      configField.set(driveConstants, driveConfig);

      java.lang.reflect.Field freqField =
          SwerveDriveConstants.class.getDeclaredField("odometryFrequency");
      freqField.setAccessible(true);
      freqField.set(driveConstants, 100.0);
    } catch (Exception e) {
      fail(e);
    }

    Pigeon2Configurator configurator = mock(Pigeon2Configurator.class);
    StatusSignal<Angle> yaw = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> yawVelocity = mock(StatusSignal.class);
    StatusSignal<Angle> pitch = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> pitchVelocity = mock(StatusSignal.class);
    StatusSignal<Angle> roll = mock(StatusSignal.class);
    StatusSignal<AngularVelocity> rollVelocity = mock(StatusSignal.class);

    when(yaw.getValueAsDouble()).thenReturn(10.0);
    when(yawVelocity.getValueAsDouble()).thenReturn(1.5);
    when(pitch.getValue()).thenReturn(edu.wpi.first.units.Units.Degrees.of(5.0));
    when(pitchVelocity.getValue()).thenReturn(edu.wpi.first.units.Units.DegreesPerSecond.of(0.2));
    when(roll.getValue()).thenReturn(edu.wpi.first.units.Units.Degrees.of(-3.0));
    when(rollVelocity.getValue()).thenReturn(edu.wpi.first.units.Units.DegreesPerSecond.of(-0.1));

    AtomicLong timestampInput = new AtomicLong(0);

    try (MockedConstruction<Pigeon2> mockPigeon =
            mockConstruction(
                Pigeon2.class,
                (mock, context) -> {
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getYaw()).thenReturn(yaw);
                  when(mock.getAngularVelocityZWorld()).thenReturn(yawVelocity);
                  when(mock.getPitch()).thenReturn(pitch);
                  when(mock.getAngularVelocityXWorld()).thenReturn(pitchVelocity);
                  when(mock.getRoll()).thenReturn(roll);
                  when(mock.getAngularVelocityYWorld()).thenReturn(rollVelocity);
                });
        // MockedStatic<NetworkTablesJNI> mockNt = mockStatic(NetworkTablesJNI.class);
        MockedStatic<BaseStatusSignal> mockBss = mockStatic(BaseStatusSignal.class)) {

      // mockNt.when(() -> NetworkTablesJNI.now()).thenReturn(123456789L);
      mockBss
          .when(() -> BaseStatusSignal.isAllGood(any(BaseStatusSignal[].class)))
          .thenReturn(true);
      mockBss
          .when(
              () ->
                  BaseStatusSignal.setUpdateFrequencyForAll(
                      anyDouble(), any(BaseStatusSignal[].class)))
          .thenReturn(null);

      GyroIOPigeon2 gyroIO = new GyroIOPigeon2(driveConstants, timestampInput::set);

      assertEquals(yaw, gyroIO.getYaw());
      assertEquals(pitch, gyroIO.getPitch());
      assertEquals(roll, gyroIO.getRoll());

      GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();
      Queue<Double> yawTimestampQueue = new ArrayDeque<>();
      Queue<Double> yawPositionQueue = new ArrayDeque<>();

      yawTimestampQueue.add(1.0);
      yawPositionQueue.add(15.0);

      gyroIO.updateInputs(inputs, yawTimestampQueue, yawPositionQueue);

      assertTrue(timestampInput.get() > 0);
      assertEquals(Rotation2d.fromDegrees(10.0), inputs.yawPosition);
      assertEquals(1, inputs.odometryYawTimestamps.length);
      assertEquals(1.0, inputs.odometryYawTimestamps[0]);
      assertEquals(1, inputs.odometryYawPositions.length);
      assertEquals(Rotation2d.fromDegrees(15.0), inputs.odometryYawPositions[0]);
    }
  }
}
