package edu.wpi.team190.gompeilib.core.utility.control;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

public class GainsTest {
  private MockTunableBackend backend;

  private void setUp(boolean isTuning) {
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, isTuning, 0.02);
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  @Test
  public void testGains() {
    setUp(true);
    Gains gains1 =
        Gains.builder()
            .withPrefix("Test")
            .withKP(1.0)
            .withKI(2.0)
            .withKD(3.0)
            .withKS(4.0)
            .withKV(5.0)
            .withKA(6.0)
            .withKG(7.0)
            .build();

    assertEquals(1.0, gains1.getKP());
    assertEquals(2.0, gains1.getKI());
    assertEquals(3.0, gains1.getKD());
    assertEquals(4.0, gains1.getKS());
    assertEquals(5.0, gains1.getKV());
    assertEquals(6.0, gains1.getKA());
    assertEquals(7.0, gains1.getKG());

    // Defaults; no prefix means unpublished
    Gains gains2 = Gains.builder().build();
    assertEquals(0.0, gains2.getKP());
    assertEquals(0.0, gains2.getKI());
    assertEquals(0.0, gains2.getKD());
    assertEquals(0.0, gains2.getKS());
    assertEquals(0.0, gains2.getKV());
    assertEquals(0.0, gains2.getKA());
    assertEquals(0.0, gains2.getKG());
  }

  @Test
  public void testRemoteEditNotifiesOncePerBatch() {
    setUp(true);
    Gains gains = Gains.builder().withPrefix("Tuned").withKP(1.0).withKD(0.1).build();
    AtomicInteger counter = new AtomicInteger(0);
    gains.onChange(g -> counter.incrementAndGet());

    // No edits, no notification
    TunableRegistry.update();
    assertEquals(0, counter.get());

    // Two gains edited in the same loop produce one notification with both new values
    backend.setDouble("/Tuned/kP", 2.0);
    backend.setDouble("/Tuned/kD", 0.2);
    TunableRegistry.update();
    assertEquals(1, counter.get());
    assertEquals(2.0, gains.getKP());
    assertEquals(0.2, gains.getKD());

    // Setting the same value again is not a change
    backend.setDouble("/Tuned/kP", 2.0);
    TunableRegistry.update();
    assertEquals(1, counter.get());
  }

  @Test
  public void testImmutableWhenNotTuning() {
    setUp(false);
    Gains gains = Gains.builder().withPrefix("Locked").withKP(1.0).build();
    AtomicInteger counter = new AtomicInteger(0);
    gains.onChange(g -> counter.incrementAndGet());

    backend.setDouble("/Locked/kP", 5.0);
    TunableRegistry.update();
    assertEquals(1.0, gains.getKP());
    assertEquals(0, counter.get());
  }
}
