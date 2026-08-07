package edu.wpi.team190.gompeilib.core.utility.phoenix;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class GainSlotTest {
  @Test
  public void testGainSlot() {
    assertEquals(GainSlot.ZERO, GainSlot.integerToGainSlot(0));
    assertEquals(GainSlot.ONE, GainSlot.integerToGainSlot(1));
    assertEquals(GainSlot.TWO, GainSlot.integerToGainSlot(2));
    assertNull(GainSlot.integerToGainSlot(3));

    // ValueOf and values
    assertEquals(GainSlot.ZERO, GainSlot.valueOf("ZERO"));
    assertEquals(3, GainSlot.values().length);
  }
}
