package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.*;

public class LinearConstraintsTest {
  private MockTunableBackend backend;

  @BeforeEach
  public void setUp() {
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, true, 0.02);
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  @Test
  public void testLinearConstraints() {
    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Test")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .withMaxAcceleration(Units.MetersPerSecond.per(Units.Second).of(6.0))
            .build();

    assertEquals(0.05, c.getGoalToleranceMeters(Units.Meters));
    assertEquals(3.0, c.getMaxVelocityMetersPerSecond(Units.MetersPerSecond));
    assertEquals(
        6.0, c.getMaxAccelerationMetersPerSecondSquared(Units.MetersPerSecond.per(Units.Second)));

    // No prefix means unpublished; unset values default to zero
    LinearConstraints defaults = LinearConstraints.builder().build();
    assertEquals(0.0, defaults.goalTolerance().get().in(Units.Meters));
    assertEquals(0.0, defaults.maxVelocity().get().in(Units.MetersPerSecond));
    assertEquals(0.0, defaults.maxAcceleration().get().in(Units.MetersPerSecondPerSecond));
  }

  @Test
  public void testRemoteEditNotifies() {
    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Tuned")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .build();
    AtomicInteger counter = new AtomicInteger(0);
    c.onChange(x -> counter.incrementAndGet());

    backend.setValue("/Tuned/Goal Tolerance", Units.Meters.of(0.1));
    TunableRegistry.update();

    assertEquals(1, counter.get());
    assertEquals(0.1, c.goalTolerance().get().in(Units.Meters), 1e-9);
  }
}
