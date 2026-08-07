package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class AngularVelocityConstraintsTest {
  @Test
  public void testAngularVelocityConstraints() {
    AngularVelocityConstraints c =
        AngularVelocityConstraints.fromMeasures()
            .withPrefix("Test")
            .withGoalTolerance(Units.DegreesPerSecond.of(1.0))
            .withMaxVelocity(Units.DegreesPerSecond.of(180.0))
            .withMaxAcceleration(Units.DegreesPerSecond.per(Units.Second).of(360.0))
            .build();

    assertNotNull(c.goalTolerance());
    assertNotNull(c.maxVelocity());
    assertNotNull(c.maxAcceleration());

    AtomicInteger counter = new AtomicInteger(0);
    c.update(1, x -> counter.incrementAndGet());
    assertEquals(1, counter.get());
  }
}
