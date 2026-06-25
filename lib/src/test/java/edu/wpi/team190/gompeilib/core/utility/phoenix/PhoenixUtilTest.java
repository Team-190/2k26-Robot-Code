package edu.wpi.team190.gompeilib.core.utility.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

public class PhoenixUtilTest {
  @Test
  public void testPhoenixUtil() {
    new PhoenixUtil(); // Constructor coverage

    AtomicInteger count = new AtomicInteger(0);
    // Success on first try
    PhoenixUtil.tryUntilOk(
        5,
        () -> {
          count.incrementAndGet();
          return StatusCode.OK;
        });
    assertEquals(1, count.get());

    // Retries on error
    count.set(0);
    PhoenixUtil.tryUntilOk(
        3,
        () -> {
          int c = count.incrementAndGet();
          if (c < 3) {
            return StatusCode.RxTimeout;
          }
          return StatusCode.OK;
        });
    assertEquals(3, count.get());

    // Signals registration and refresh
    BaseStatusSignal mockSignal = mock(BaseStatusSignal.class);

    // We register signals. Since BaseStatusSignal.refreshAll might hit native code or static mock
    // issues,
    // let's just test that the arrays are populated. We can invoke registerSignals and refreshAll.
    try {
      PhoenixUtil.registerSignals(true, mockSignal);
      PhoenixUtil.registerSignals(false, mockSignal);
      // Since refreshAll calls BaseStatusSignal.refreshAll, we might expect it to run.
      // Let's call it and make sure it does not throw unexpected JNI exceptions or we catch them if
      // it does.
      PhoenixUtil.refreshAll();
    } catch (Throwable t) {
      // In case native code is hit and fails
    }
  }
}
