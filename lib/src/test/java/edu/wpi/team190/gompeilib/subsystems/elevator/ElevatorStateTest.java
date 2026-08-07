package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ElevatorStateTest {
  @Test
  public void testEnum() {
    assertEquals(3, ElevatorState.values().length);
    assertEquals(ElevatorState.IDLE, ElevatorState.valueOf("IDLE"));
    assertEquals(
        ElevatorState.OPEN_LOOP_VOLTAGE_CONTROL,
        ElevatorState.valueOf("OPEN_LOOP_VOLTAGE_CONTROL"));
    assertEquals(
        ElevatorState.CLOSED_LOOP_POSITION_CONTROL,
        ElevatorState.valueOf("CLOSED_LOOP_POSITION_CONTROL"));
  }
}
