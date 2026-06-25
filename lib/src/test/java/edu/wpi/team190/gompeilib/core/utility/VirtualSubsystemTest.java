package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class VirtualSubsystemTest {
  @Test
  public void testVirtualSubsystem() {
    AtomicInteger counter = new AtomicInteger(0);
    VirtualSubsystem sub =
        new VirtualSubsystem() {
          @Override
          public void periodic() {
            counter.incrementAndGet();
          }
        };

    VirtualSubsystem.periodicAll();
    assertEquals(1, counter.get());
  }
}
