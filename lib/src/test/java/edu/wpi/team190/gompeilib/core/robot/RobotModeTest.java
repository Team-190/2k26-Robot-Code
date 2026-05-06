package edu.wpi.team190.gompeilib.core.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RobotModeTest {

  @Test
  void testEnumValues() {
    // Ensure all enum constants exist
    RobotMode[] modes = RobotMode.values();

    assertEquals(3, modes.length, "There should be exactly 3 modes");

    assertEquals(RobotMode.REAL, RobotMode.valueOf("REAL"));
    assertEquals(RobotMode.SIM, RobotMode.valueOf("SIM"));
    assertEquals(RobotMode.REPLAY, RobotMode.valueOf("REPLAY"));
  }
}
