package edu.wpi.team190.gompeilib.core.io.inertial;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import org.junit.jupiter.api.Test;

public class GyroIOTest {
  @Test
  public void testGyroIODefaults() {
    GyroIO gyro = new GyroIO() {}; // Anonymous implementation
    GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

    gyro.updateInputs(inputs);
    gyro.updateInputs(inputs, null, null);

    assertNull(gyro.getYaw());
    assertNull(gyro.getRoll());
    assertNull(gyro.getPitch());

    // Inputs defaults
    assertFalse(inputs.connected);
    assertNotNull(inputs.yawPosition);
    assertEquals(0, inputs.odometryYawTimestamps.length);
    assertEquals(0, inputs.odometryYawPositions.length);
    assertEquals(0.0, inputs.yawVelocityRadPerSec);
    assertNotNull(inputs.pitchPosition);
    assertNotNull(inputs.pitchVelocity);
    assertNotNull(inputs.rollPosition);
    assertNotNull(inputs.rollVelocity);
  }
}
