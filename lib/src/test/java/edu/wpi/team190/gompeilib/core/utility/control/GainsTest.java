package edu.wpi.team190.gompeilib.core.utility.control;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class GainsTest {
  @Test
  public void testGains() {
    Gains gains1 =
        Gains.fromDoubles()
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

    // Null checks / defaults
    Gains gains2 = Gains.builder().build();
    assertEquals(0.0, gains2.getKP());
    assertEquals(0.0, gains2.getKI());
    assertEquals(0.0, gains2.getKD());
    assertEquals(0.0, gains2.getKS());
    assertEquals(0.0, gains2.getKV());
    assertEquals(0.0, gains2.getKA());
    assertEquals(0.0, gains2.getKG());

    // Update check
    AtomicInteger counter = new AtomicInteger(0);
    gains1.update(1, g -> counter.incrementAndGet());
    // Since kP is a LoggedTunableNumber, first check to see if it changed will trigger
    assertEquals(1, counter.get());
  }
}
