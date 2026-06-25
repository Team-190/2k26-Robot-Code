package edu.wpi.team190.gompeilib.core.utility.phoenix;

/** An Enum class that handles all possible gain slots for motor control */
public enum GainSlot {
  ZERO,
  ONE,
  TWO;

  public static GainSlot integerToGainSlot(Integer integer) {
    return switch (integer) {
      case 0 -> ZERO;
      case 1 -> ONE;
      case 2 -> TWO;
      default -> null;
    };
  }
}
