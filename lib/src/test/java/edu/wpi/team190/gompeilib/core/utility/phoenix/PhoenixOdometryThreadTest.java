package edu.wpi.team190.gompeilib.core.utility.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;
import org.junit.jupiter.api.Test;

public class PhoenixOdometryThreadTest {
  @Test
  public void testPhoenixOdometryThread() {
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

      java.lang.reflect.Field lockField =
          SwerveDriveConstants.class.getDeclaredField("reentrantLock");
      lockField.setAccessible(true);
      lockField.set(driveConstants, new ReentrantLock());

      java.lang.reflect.Field freqField =
          SwerveDriveConstants.class.getDeclaredField("odometryFrequency");
      freqField.setAccessible(true);
      freqField.set(driveConstants, 100.0);
    } catch (Exception e) {
      fail(e);
    }

    PhoenixOdometryThread thread = PhoenixOdometryThread.getInstance(driveConstants);
    assertNotNull(thread);

    // Test register signal
    StatusSignal<Angle> mockSignal = mock(StatusSignal.class);
    Queue<Double> signalQueue = thread.registerSignal(mockSignal);
    assertNotNull(signalQueue);

    // Test register generic signal
    Queue<Double> genericQueue = thread.registerSignal(() -> 42.0);
    assertNotNull(genericQueue);

    // Test makeTimestampQueue
    Queue<Double> tsQueue = thread.makeTimestampQueue();
    assertNotNull(tsQueue);
  }
}
