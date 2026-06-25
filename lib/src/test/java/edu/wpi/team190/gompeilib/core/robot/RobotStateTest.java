package edu.wpi.team190.gompeilib.core.robot;

import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class RobotStateTest {

  /** Minimal concrete implementation of RobotState for testing. */
  static class TestState implements RobotState {}

  @Test
  void testPeriodic() {
    // Use Mockito to safely call default method
    RobotState state = Mockito.mock(TestState.class, Mockito.CALLS_REAL_METHODS);

    // Call the default method
    state.periodic(); // just runs, should not throw
  }
}
