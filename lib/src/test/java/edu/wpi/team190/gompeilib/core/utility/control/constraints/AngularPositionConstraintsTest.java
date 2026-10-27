package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.units.*;

public class AngularPositionConstraintsTest {
  @BeforeEach
  public void setUp() {
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, true, 0.02);
  }

  @Test
  public void testAngularPositionConstraints() {
    AngularPositionConstraints c =
        AngularPositionConstraints.builder()
            .withPrefix("Test")
            .withGoalTolerance(Units.Degrees.of(1.0))
            .withMaxVelocity(Units.DegreesPerSecond.of(180.0))
            .withMaxAcceleration(Units.DegreesPerSecond.per(Units.Second).of(360.0))
            .build();

    assertEquals(1.0, c.getGoalTolerance(Units.Degrees));
    assertEquals(180.0, c.getMaxVelocity(Units.DegreesPerSecond));
    assertEquals(360.0, c.getMaxAcceleration(Units.DegreesPerSecond.per(Units.Second)));

    // No prefix means unpublished; unset values default to zero
    AngularPositionConstraints defaults = AngularPositionConstraints.builder().build();
    assertEquals(0.0, defaults.goalTolerance().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxVelocity().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxAcceleration().get().baseUnitMagnitude());
  }
}
