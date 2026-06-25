package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class GenericFlywheelStateTest {
  @Test
  public void testEnum() {
    assertEquals(5, GenericFlywheelState.values().length);
    assertEquals(GenericFlywheelState.IDLE, GenericFlywheelState.valueOf("IDLE"));
    assertEquals(GenericFlywheelState.STOP, GenericFlywheelState.valueOf("STOP"));
    assertEquals(
        GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL,
        GenericFlywheelState.valueOf("VELOCITY_VOLTAGE_CONTROL"));
    assertEquals(
        GenericFlywheelState.VELOCITY_TORQUE_CONTROL,
        GenericFlywheelState.valueOf("VELOCITY_TORQUE_CONTROL"));
    assertEquals(
        GenericFlywheelState.VOLTAGE_CONTROL, GenericFlywheelState.valueOf("VOLTAGE_CONTROL"));
  }
}
