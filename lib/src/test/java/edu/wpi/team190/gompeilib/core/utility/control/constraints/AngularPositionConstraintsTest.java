package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class AngularPositionConstraintsTest {
  @Test
  public void testAngularPositionConstraints() {
    AngularPositionConstraints c =
        AngularPositionConstraints.fromMeasures()
            .withPrefix("Test")
            .withGoalTolerance(Units.Degrees.of(1.0))
            .withMaxVelocity(Units.DegreesPerSecond.of(180.0))
            .withMaxAcceleration(Units.DegreesPerSecond.per(Units.Second).of(360.0))
            .build();

    assertEquals(1.0, c.getGoalTolerance(Units.Degrees));
    assertEquals(180.0, c.getMaxVelocity(Units.DegreesPerSecond));
    assertEquals(360.0, c.getMaxAcceleration(Units.DegreesPerSecond.per(Units.Second)));

    AtomicInteger counter = new AtomicInteger(0);
    c.update(1, x -> counter.incrementAndGet());
    assertEquals(1, counter.get());
  }
}
