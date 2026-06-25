package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ArmStateTest {
  @Test
  public void testEnum() {
    assertEquals(3, ArmState.values().length);
    assertEquals(ArmState.IDLE, ArmState.valueOf("IDLE"));
    assertEquals(ArmState.OPEN_LOOP_VOLTAGE_CONTROL, ArmState.valueOf("OPEN_LOOP_VOLTAGE_CONTROL"));
    assertEquals(
        ArmState.CLOSED_LOOP_POSITION_CONTROL, ArmState.valueOf("CLOSED_LOOP_POSITION_CONTROL"));
  }
}
