package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ExtensionStateTest {
  @Test
  public void testEnum() {
    assertEquals(3, ExtensionState.values().length);
    assertEquals(ExtensionState.IDLE, ExtensionState.valueOf("IDLE"));
    assertEquals(
        ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL,
        ExtensionState.valueOf("OPEN_LOOP_VOLTAGE_CONTROL"));
    assertEquals(
        ExtensionState.CLOSED_LOOP_POSITION_CONTROL,
        ExtensionState.valueOf("CLOSED_LOOP_POSITION_CONTROL"));
  }
}
