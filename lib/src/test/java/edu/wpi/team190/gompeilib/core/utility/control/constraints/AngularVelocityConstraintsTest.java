package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.units.*;

public class AngularVelocityConstraintsTest {
  @BeforeEach
  public void setUp() {
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, true, 0.02);
  }

  @Test
  public void testAngularVelocityConstraints() {
    AngularVelocityConstraints c =
        AngularVelocityConstraints.builder()
            .withPrefix("Test")
            .withGoalTolerance(Units.DegreesPerSecond.of(1.0))
            .withMaxVelocity(Units.DegreesPerSecond.of(180.0))
            .withMaxAcceleration(Units.DegreesPerSecond.per(Units.Second).of(360.0))
            .build();

    assertNotNull(c.goalTolerance());
    assertNotNull(c.maxVelocity());
    assertNotNull(c.maxAcceleration());

    // No prefix means unpublished; unset values default to zero
    AngularVelocityConstraints defaults = AngularVelocityConstraints.builder().build();
    assertEquals(0.0, defaults.goalTolerance().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxVelocity().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxAcceleration().get().baseUnitMagnitude());
  }
}
