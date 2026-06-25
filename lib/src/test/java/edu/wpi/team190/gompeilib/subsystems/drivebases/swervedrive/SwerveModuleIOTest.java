package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class SwerveModuleIOTest {
  @Test
  public void testDefaultInterfaceMethods() {
    SwerveModuleIO io = new SwerveModuleIO() {};

    // Test calling default methods (they are no-ops, should not throw)
    assertDoesNotThrow(() -> io.updateInputs(new SwerveModuleIO.ModuleIOInputs()));
    assertDoesNotThrow(() -> io.setDriveAmps(1.0));
    assertDoesNotThrow(() -> io.setTurnAmps(2.0));
    assertDoesNotThrow(() -> io.setDriveVelocity(3.0, 0.5));
    assertDoesNotThrow(() -> io.setTurnPosition(new Rotation2d()));
    assertDoesNotThrow(() -> io.setPID(1.0, 0.1, 2.0, 0.2));
    assertDoesNotThrow(() -> io.setFeedforward(0.1, 0.2));
    assertDoesNotThrow(() -> io.updateCurrentLimits(30.0, 20.0));
  }

  @Test
  public void testModuleIOInputsDefaults() {
    SwerveModuleIO.ModuleIOInputs inputs = new SwerveModuleIO.ModuleIOInputs();
    assertEquals(0.0, inputs.drivePositionRadians);
    assertEquals(0.0, inputs.driveVelocityRadiansPerSecond);
    assertEquals(0.0, inputs.driveAppliedVolts);
    assertEquals(0.0, inputs.driveSupplyCurrentAmps);
    assertEquals(0.0, inputs.driveTorqueCurrentAmps);
    assertEquals(0.0, inputs.driveTemperatureCelcius);
    assertEquals(0.0, inputs.driveVelocitySetpointRadiansPerSecond);
    assertEquals(0.0, inputs.driveVelocityErrorRadiansPerSecond);

    assertEquals(0.0, inputs.turnAbsolutePosition.getRadians());
    assertEquals(0.0, inputs.turnPosition.getRadians());
    assertEquals(0.0, inputs.turnVelocityRadiansPerSecond);
    assertEquals(0.0, inputs.turnAppliedVolts);
    assertEquals(0.0, inputs.turnSupplyCurrentAmps);
    assertEquals(0.0, inputs.turnTorqueCurrentAmps);
    assertEquals(0.0, inputs.turnTemperatureCelcius);
    assertEquals(0.0, inputs.turnPositionGoal.getRadians());
    assertEquals(0.0, inputs.turnPositionSetpoint.getRadians());
    assertEquals(0.0, inputs.turnPositionError.getRadians());

    assertFalse(inputs.driveConnected);
    assertFalse(inputs.turnConnected);
    assertFalse(inputs.turnEncoderConnected);

    assertEquals(0, inputs.odometryTimestamps.length);
    assertEquals(0, inputs.odometryDrivePositionsRadians.length);
    assertEquals(0, inputs.odometryTurnPositions.length);
  }
}
