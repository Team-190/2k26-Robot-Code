package edu.wpi.team190.gompeilib.core.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

class RobotContainerTest {

  /** Minimal concrete implementation of RobotContainer for testing. */
  static class TestContainer implements RobotContainer {}

  @Test
  void testGetAutonomousCommand() {
    RobotContainer container = Mockito.mock(TestContainer.class, Mockito.CALLS_REAL_METHODS);

    // Call the default method
    Command cmd = container.getAutonomousCommand();

    // Ensure it returns something (mocked default Commands.none() safe)
    assertNotNull(cmd);
  }

  @Test
  void testRobotPeriodic() {
    RobotContainer container = Mockito.mock(TestContainer.class, Mockito.CALLS_REAL_METHODS);

    // Call the default method
    container.robotPeriodic(); // just runs, should not throw
  }
}
