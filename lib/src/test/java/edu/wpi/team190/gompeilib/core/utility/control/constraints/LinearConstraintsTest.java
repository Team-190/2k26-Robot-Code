package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class LinearConstraintsTest {
  @Test
  public void testLinearConstraints() {
    LinearConstraints c =
        LinearConstraints.fromMeasures()
            .withPrefix("Test")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .withMaxAcceleration(Units.MetersPerSecond.per(Units.Second).of(6.0))
            .build();

    assertEquals(0.05, c.getGoalToleranceMeters(Units.Meters));
    assertEquals(3.0, c.getMaxVelocityMetersPerSecond(Units.MetersPerSecond));
    assertEquals(
        6.0, c.getMaxAccelerationMetersPerSecondSquared(Units.MetersPerSecond.per(Units.Second)));

    AtomicInteger counter = new AtomicInteger(0);
    c.update(1, x -> counter.incrementAndGet());
    assertEquals(1, counter.get());
  }
}
